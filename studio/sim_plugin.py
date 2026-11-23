"""Simulation-side MuJoCo Studio plugin for robot-demo (hot-reloaded on save).

Streams all 6 robot-demo arenas into MuJoCo Studio AND exposes the standard
robot_ctl.py HTTP bridge on http://127.0.0.1:8765 so Mode A / Mode B agents
and telemetry can drive the live Studio simulation directly.
"""

from collections.abc import Callable
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import queue
import sys
import threading
import time
from typing import Any
from urllib.parse import parse_qs, urlsplit

import messages as user_messages
import mujoco
from mujoco.studio import messages
from mujoco.studio import plugin
import numpy as np
from PIL import Image

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
  sys.path.insert(0, str(PROJECT_ROOT))

from robot_sim.arenas import (
    ARENAS,
    MODELS_DIR,
    ROBOT_INCLUDE_FILENAME,
    identify_arena_id,
    list_arenas,
    resolve_arena_model,
)
from robot_sim.core import (
    DEFAULT_CAMERA_HEIGHT,
    DEFAULT_CAMERA_WIDTH,
    DEFAULT_CAPTURES_DIR,
    DEFAULT_JPEG_QUALITY,
    MAX_CAMERA_HEIGHT,
    MAX_CAMERA_WIDTH,
    MAX_COMMAND_DURATION,
    MAX_WHEEL_RAD_S,
    RANGEFINDER_MAX,
    RobotSimulation,
    _apply_spawn_pose,
    _discover_model_bindings,
    _discover_sensors,
    _init_moving_obstacles,
    _require_finite,
    compute_move_command,
    compute_turn_command,
)
from robot_sim.integrity import verify_arena_integrity
from robot_sim.topo import DEFAULT_TOPO_STATE_PATH, read_topo_studio_summary


def _read_rf(data: mujoco.MjData, sensor_name: str) -> float:
  try:
    raw = float(data.sensor(sensor_name).data[0])
  except KeyError:
    return RANGEFINDER_MAX
  if raw < 0.0 or raw >= RANGEFINDER_MAX - 1e-6:
    return RANGEFINDER_MAX
  return raw


class _BridgeHolder:
  """Persists the 127.0.0.1:8765 HTTP server across Studio plugin hot-reloads."""

  def __init__(self) -> None:
    self.plugin: Any = None
    self.server: ThreadingHTTPServer | None = None
    self.thread: threading.Thread | None = None

  def ensure_started(self, target: Any, host: str = '127.0.0.1', port: int = 8765) -> None:
    self.plugin = target
    if self.server is not None:
      return
    holder = self

    class RequestHandler(BaseHTTPRequestHandler):
      protocol_version = 'HTTP/1.1'

      def log_message(self, format: str, *args: Any) -> None:
        return

      def _send_json(self, status_code: int, payload: dict) -> None:
        raw = json.dumps(payload, ensure_ascii=False).encode('utf-8')
        self.send_response(status_code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)

      def _read_json_body(self) -> dict:
        length = int(self.headers.get('Content-Length', '0'))
        if length <= 0:
          return {}
        if length > 65536:
          raise ValueError('Kích thước gói tin vượt quá giới hạn cho phép.')
        raw = self.rfile.read(length).decode('utf-8')
        if not raw.strip():
          return {}
        parsed = json.loads(raw)
        if not isinstance(parsed, dict):
          raise ValueError('Nội dung JSON phải là một đối tượng (object).')
        return parsed

      def _dispatch(self, fn: Callable[[Any], dict], *, is_reload: bool = False) -> None:
        p = holder.plugin
        if p is None:
          self._send_json(503, {'ok': False, 'error': 'Studio SimPlugin chưa sẵn sàng.'})
          return
        code, payload = p.submit_http(fn, is_reload=is_reload)
        self._send_json(code, payload)

      def do_GET(self) -> None:
        parsed_url = urlsplit(self.path)
        query = parse_qs(parsed_url.query)
        if parsed_url.path == '/status':
          inc_vis = query.get('vision', ['0'])[0].strip().lower() in ('1', 'true', 'yes')
          self._dispatch(lambda p: p.api_get_status(include_vision=inc_vis))
        elif parsed_url.path == '/vision':
          cam_name = query.get('camera', ['front_cam'])[0]
          self._dispatch(lambda p: p.api_analyze_vision(camera_name=cam_name))
        elif parsed_url.path == '/arenas':
          inc_dbg = query.get('debug', ['0'])[0].strip().lower() in ('1', 'true', 'yes')
          self._dispatch(lambda p: p.api_list_arenas(include_debug=inc_dbg))
        else:
          self._send_json(404, {'ok': False, 'error': f"Không hỗ trợ đường dẫn GET '{self.path}'."})

      def do_POST(self) -> None:
        try:
          body = self._read_json_body()
        except (ValueError, json.JSONDecodeError) as exc:
          self._send_json(400, {'ok': False, 'error': f'JSON không hợp lệ: {exc}'})
          return

        path = self.path
        if path == '/wheels':
          if 'left' not in body or 'right' not in body:
            self._send_json(400, {'ok': False, 'error': "Cần truyền đủ 'left' và 'right' (rad/s)."})
            return
          self._dispatch(
              lambda p: p.api_set_wheels(body['left'], body['right'], duration=body.get('duration', 1.0))
          )
        elif path == '/move':
          if 'meters' not in body:
            self._send_json(400, {'ok': False, 'error': "Cần truyền tham số 'meters'."})
            return
          self._dispatch(lambda p: p.api_move(meters=body['meters'], speed_m_s=body.get('speed', 0.25)))
        elif path == '/turn':
          if 'degrees' not in body:
            self._send_json(400, {'ok': False, 'error': "Cần truyền tham số 'degrees'."})
            return
          self._dispatch(
              lambda p: p.api_turn(degrees=body['degrees'], angular_deg_s=body.get('angular_speed', 60.0))
          )
        elif path == '/stop':
          self._dispatch(lambda p: p.api_stop())
        elif path == '/reset':
          self._dispatch(lambda p: p.api_reset(target_xy=body.get('target_xy')))
        elif path == '/reload':
          self._dispatch(
              lambda p: p.api_reload(
                  arena=body.get('arena'),
                  model=body.get('model'),
                  preserve_target=body.get('preserve_target'),
                  target_xy=body.get('target_xy'),
              ),
              is_reload=True,
          )
        elif path == '/camera':
          self._dispatch(
              lambda p: p.api_capture_camera(
                  camera_name=body.get('camera', 'front_cam'),
                  out_path=body.get('path'),
                  width=body.get('width', DEFAULT_CAMERA_WIDTH),
                  height=body.get('height', DEFAULT_CAMERA_HEIGHT),
                  img_format=body.get('format'),
                  quality=body.get('quality', DEFAULT_JPEG_QUALITY),
              )
          )
        else:
          self._send_json(404, {'ok': False, 'error': f"Không hỗ trợ đường dẫn POST '{path}'."})

    class _KeepAliveHTTPServer(ThreadingHTTPServer):
      daemon_threads = True
      allow_reuse_address = True

    try:
      self.server = _KeepAliveHTTPServer((host, port), RequestHandler)
      self.thread = threading.Thread(
          target=self.server.serve_forever,
          name='StudioRobotBridgeHTTP',
          daemon=True,
      )
      self.thread.start()
    except OSError:
      self.server = None

  def shutdown(self) -> None:
    if self.server is not None:
      try:
        self.server.shutdown()
        self.server.server_close()
      except Exception:
        pass
      self.server = None
      self.thread = None


def _get_bridge_holder() -> _BridgeHolder:
  holder = getattr(sys, '_robot_demo_studio_bridge', None)
  if holder is None:
    holder = _BridgeHolder()
    setattr(sys, '_robot_demo_studio_bridge', holder)
  return holder


class SimPlugin(plugin.SimPlugin):
  """Loads any robot-demo arena, runs physics, and bridges robot_ctl.py HTTP API."""

  arena_id: str = '2wheel'
  mode: str = 'manual'
  left_rad_s: float = 0.0
  right_rad_s: float = 0.0
  auto_speed_rad_s: float = 7.0

  reset_requested: bool = True
  _model: mujoco.MjModel | None = None
  _data: mujoco.MjData | None = None
  _model_path: Path | None = None
  _bindings: Any = None
  _sensor_adr: dict[str, int] | None = None
  _rangefinders: dict[str, tuple[int, float]] | None = None
  _default_target_pos: np.ndarray | None = None
  _command_deadline: float | None = None
  _gyro_yaw_rad: float = 0.0
  _odom_dist_m: float = 0.0
  _odom_x_m: float = 0.0
  _odom_y_m: float = 0.0
  _last_odom_time: float = 0.0
  _contact_count: int = 0
  _contact_steps: int = 0
  _in_hazard_contact: bool = False
  _state_epoch: int = 0
  _step_counter: int = 0
  _renderer: mujoco.Renderer | None = None
  _renderer_size: tuple[int, int] | None = None
  _clean_vis_opt: mujoco.MjvOption | None = None
  _vision_cache: dict[str, tuple[tuple, dict]] | None = None
  _cmd_queue: queue.Queue | None = None
  _pending_reload_resp: tuple[queue.Queue, tuple[float, float] | None, bool, np.ndarray | None] | None = None
  _topo_poll_wall: float = 0.0
  _topo_mtime_ns: int = -1
  _topo_summary: tuple[int, int, int] = (0, -1, 0)

  def _ensure_runtime_fields(self) -> None:
    if self._cmd_queue is None:
      self._cmd_queue = queue.Queue()
    if self._vision_cache is None:
      self._vision_cache = {}
    if self._clean_vis_opt is None:
      opt = mujoco.MjvOption()
      opt.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = 0
      self._clean_vis_opt = opt
    _get_bridge_holder().ensure_started(self)

  def _arena_path(self) -> Path:
    spec = ARENAS.get(self.arena_id, ARENAS['2wheel'])
    return (MODELS_DIR / spec.filename).resolve()

  def _close_renderer(self) -> None:
    if self._renderer is not None:
      try:
        self._renderer.close()
      except Exception:
        pass
      self._renderer = None
      self._renderer_size = None

  def _get_renderer(self, width: int, height: int) -> mujoco.Renderer:
    assert self._model is not None
    if self._renderer is not None and self._renderer_size != (width, height):
      self._close_renderer()
    if self._renderer is None:
      self._renderer = mujoco.Renderer(self._model, height=height, width=width)
      self._renderer_size = (width, height)
    return self._renderer

  def _reset_odom_and_contacts(self) -> None:
    self._command_deadline = None
    self._gyro_yaw_rad = 0.0
    self._odom_dist_m = 0.0
    self._odom_x_m = 0.0
    self._odom_y_m = 0.0
    self._last_odom_time = float(self._data.time) if self._data is not None else 0.0
    self._contact_count = 0
    self._contact_steps = 0
    self._in_hazard_contact = False
    self._state_epoch += 1
    try:
      self._topo_mtime_ns = (
          DEFAULT_TOPO_STATE_PATH.stat().st_mtime_ns
          if DEFAULT_TOPO_STATE_PATH.is_file()
          else -1
      )
    except OSError:
      self._topo_mtime_ns = -1
    self._topo_summary = (0, -1, 0)
    if self._vision_cache is not None:
      self._vision_cache.clear()

  # --- Studio lifecycle handlers ---

  @messages.handler
  def on_exit(self, event: messages.ExitEvent) -> None:
    del event
    self._close_renderer()
    _get_bridge_holder().shutdown()

  @messages.handler
  def on_load_model(self, event: messages.LoadModelEvent) -> bool:
    self._ensure_runtime_fields()
    if event.path:
      return False
    arena_path = self._arena_path()
    robot_path = (MODELS_DIR / ROBOT_INCLUDE_FILENAME).resolve()
    spec = mujoco.MjSpec.from_file(str(arena_path))
    self._model_path = arena_path
    self.handle.set_model(spec, watch=[str(arena_path), str(robot_path)])
    return True

  @messages.handler
  def on_post_model(self, event: messages.PostModelEvent) -> None:
    self._ensure_runtime_fields()
    self._close_renderer()
    self._model = event.model
    self._data = event.data
    if self._model_path is None:
      self._model_path = self._arena_path()
    self._bindings = _discover_model_bindings(self._model, self._model_path)
    self._sensor_adr, self._rangefinders = _discover_sensors(self._model)
    self._default_target_pos = self._data.mocap_pos[self._bindings.target_mocap_id].copy()

    pending = self._pending_reload_resp
    self._pending_reload_resp = None
    if self.reset_requested or pending is not None:
      self.reset_requested = False
      mujoco.mj_resetData(self._model, self._data)
      _apply_spawn_pose(self._model, self._data)
      _init_moving_obstacles(self._data, self._bindings.moving_obs_drivers)
      if pending is not None:
        _, validated_target, keep_prev, prev_target_pos = pending
        mocap_id = self._bindings.target_mocap_id
        if validated_target is not None:
          self._data.mocap_pos[mocap_id] = [
              validated_target[0],
              validated_target[1],
              float(self._default_target_pos[2]),
          ]
        elif keep_prev and prev_target_pos is not None:
          self._data.mocap_pos[mocap_id] = [
              float(prev_target_pos[0]),
              float(prev_target_pos[1]),
              float(self._default_target_pos[2]),
          ]
      self._reset_odom_and_contacts()
      mujoco.mj_forward(self._model, self._data)

    self._publish_telemetry(self._data)
    if pending is not None:
      resp_q = pending[0]
      try:
        resp_q.put((200, {'ok': True, 'data': self.api_get_status(include_vision=False)}))
      except Exception as exc:
        resp_q.put((500, {'ok': False, 'error': str(exc)}))

  @messages.handler
  def on_hot_reload(self, event: messages.HotReloadEvent) -> None:
    self._ensure_runtime_fields()
    if self in event.plugins:
      self.handle.reload()

  # --- Viewer UI message handlers ---

  @messages.handler
  def on_arena_select(self, event: user_messages.ArenaSelectEvent) -> None:
    if event.arena_id in ARENAS:
      self.arena_id = event.arena_id
      self.reset_requested = True
      self.left_rad_s = 0.0
      self.right_rad_s = 0.0
      self._command_deadline = None
      self.handle.reload()

  @messages.handler
  def on_reset_robot(self, event: user_messages.ResetRobotEvent) -> None:
    del event
    self.reset_requested = True
    self.left_rad_s = 0.0
    self.right_rad_s = 0.0
    self._command_deadline = None
    self.handle.reload()

  @messages.handler
  def on_drive_command(self, snapshot: user_messages.DriveCommand) -> None:
    self.mode = snapshot.mode if snapshot.mode in ('manual', 'auto') else 'manual'
    self.left_rad_s = max(-MAX_WHEEL_RAD_S, min(MAX_WHEEL_RAD_S, float(snapshot.left_rad_s)))
    self.right_rad_s = max(-MAX_WHEEL_RAD_S, min(MAX_WHEEL_RAD_S, float(snapshot.right_rad_s)))
    self.auto_speed_rad_s = max(1.0, min(MAX_WHEEL_RAD_S, float(snapshot.auto_speed_rad_s)))
    self._command_deadline = None

  # --- HTTP Bridge queue processing on sim thread ---

  def submit_http(
      self,
      fn: Callable[[Any], dict],
      *,
      timeout: float = 8.0,
      is_reload: bool = False,
  ) -> tuple[int, dict]:
    self._ensure_runtime_fields()
    assert self._cmd_queue is not None
    resp_q: queue.Queue[tuple[int, dict]] = queue.Queue(maxsize=1)
    self._cmd_queue.put((fn, resp_q, is_reload))
    try:
      return resp_q.get(timeout=timeout)
    except queue.Empty:
      return 504, {'ok': False, 'error': 'Quá thời gian chờ luồng mô phỏng MuJoCo Studio xử lý lệnh.'}

  def _drain_http_queue(self, max_items: int = 2) -> None:
    if self._cmd_queue is None:
      return
    processed = 0
    while processed < max_items:
      try:
        fn, resp_q, is_reload = self._cmd_queue.get_nowait()
      except queue.Empty:
        break
      processed += 1
      try:
        result = fn(self)
        if is_reload and self._pending_reload_resp is not None:
          # Reload will respond inside on_post_model once the new arena is loaded.
          _, v_target, keep_prev, prev_pos = self._pending_reload_resp
          self._pending_reload_resp = (resp_q, v_target, keep_prev, prev_pos)
          break
        resp_q.put((200, {'ok': True, 'data': result}))
      except (ValueError, TypeError, KeyError) as exc:
        resp_q.put((400, {'ok': False, 'error': str(exc)}))
      except Exception as exc:
        resp_q.put((500, {'ok': False, 'error': f'Lỗi nội bộ: {exc}'}))

  # --- HTTP API implementations (run on sim thread) ---

  def api_list_arenas(self, include_debug: bool = False) -> dict:
    res: dict[str, Any] = {'arenas': list_arenas(include_debug=include_debug)}
    if include_debug and self._model_path is not None:
      res['active_arena'] = identify_arena_id(self._model_path)
      res['active_model'] = self._model_path.name
    return res

  def api_set_wheels(self, left_rad_s: float, right_rad_s: float, duration: float = 1.0) -> dict:
    assert self._data is not None and self._bindings is not None
    _require_finite(left_rad_s=left_rad_s, right_rad_s=right_rad_s, duration=duration)
    if duration <= 0.0:
      raise ValueError("Thời gian thực thi 'duration' phải lớn hơn 0 giây.")
    clamped_left = max(-MAX_WHEEL_RAD_S, min(MAX_WHEEL_RAD_S, float(left_rad_s)))
    clamped_right = max(-MAX_WHEEL_RAD_S, min(MAX_WHEEL_RAD_S, float(right_rad_s)))
    capped_duration = min(float(duration), MAX_COMMAND_DURATION)
    self.mode = 'manual'
    self.left_rad_s = clamped_left
    self.right_rad_s = clamped_right
    self._data.ctrl[self._bindings.left_motor_id] = clamped_left
    self._data.ctrl[self._bindings.right_motor_id] = clamped_right
    self._command_deadline = float(self._data.time) + capped_duration
    return {
        'left_rad_s': round(clamped_left, 4),
        'right_rad_s': round(clamped_right, 4),
        'duration_s': round(capped_duration, 4),
    }

  def api_move(self, meters: float, speed_m_s: float = 0.25) -> dict:
    left, right, duration = compute_move_command(meters=meters, speed_m_s=speed_m_s)
    cmd = self.api_set_wheels(left, right, duration=duration)
    cmd['requested_meters'] = round(float(meters), 4)
    return cmd

  def api_turn(self, degrees: float, angular_deg_s: float = 60.0) -> dict:
    left, right, duration = compute_turn_command(degrees=degrees, angular_deg_s=angular_deg_s)
    cmd = self.api_set_wheels(left, right, duration=duration)
    cmd['requested_degrees'] = round(float(degrees), 4)
    return cmd

  def api_stop(self) -> dict:
    assert self._data is not None and self._bindings is not None
    self.mode = 'manual'
    self.left_rad_s = 0.0
    self.right_rad_s = 0.0
    self._data.ctrl[self._bindings.left_motor_id] = 0.0
    self._data.ctrl[self._bindings.right_motor_id] = 0.0
    self._command_deadline = None
    return {'stopped': True, 'sim_time': round(float(self._data.time), 4)}

  def _validate_target_xy(
      self,
      target_xy: tuple[float, float] | list[float],
      *,
      model: mujoco.MjModel | None = None,
      model_path: Path | None = None,
  ) -> tuple[float, float]:
    dummy = object.__new__(RobotSimulation)
    dummy.model = model if model is not None else self._model
    dummy.model_path = model_path if model_path is not None else self._model_path
    return RobotSimulation.validate_target_xy(dummy, target_xy, model=model, model_path=model_path)

  def api_reset(self, target_xy: tuple[float, float] | list[float] | None = None) -> dict:
    assert self._model is not None and self._data is not None
    assert self._bindings is not None and self._default_target_pos is not None
    if target_xy is not None:
      tx, ty = self._validate_target_xy(target_xy)
    else:
      tx, ty = float(self._default_target_pos[0]), float(self._default_target_pos[1])
    mujoco.mj_resetData(self._model, self._data)
    _apply_spawn_pose(self._model, self._data)
    _init_moving_obstacles(self._data, self._bindings.moving_obs_drivers)
    self._data.mocap_pos[self._bindings.target_mocap_id] = [tx, ty, float(self._default_target_pos[2])]
    self.left_rad_s = 0.0
    self.right_rad_s = 0.0
    self._reset_odom_and_contacts()
    mujoco.mj_forward(self._model, self._data)
    return self.api_get_status(include_vision=False)

  def api_reload(
      self,
      arena: str | None = None,
      model: str | None = None,
      preserve_target: bool | None = None,
      target_xy: tuple[float, float] | list[float] | None = None,
  ) -> dict:
    assert self._model_path is not None and self._data is not None and self._bindings is not None
    if preserve_target is not None and not isinstance(preserve_target, bool):
      raise ValueError("Tham số 'preserve_target' phải là kiểu boolean (true/false).")
    if arena is not None and (not isinstance(arena, str) or not arena.strip()):
      raise ValueError("Tham số 'arena' phải là chuỗi tên địa hình không rỗng.")
    if model is not None and (not isinstance(model, str) or not model.strip()):
      raise ValueError("Tham số 'model' phải là chuỗi đường dẫn không rỗng.")
    if arena is not None and model is not None:
      raise ValueError("Chỉ được truyền một trong hai tham số 'arena' hoặc 'model', không truyền đồng thời.")

    arena_or_model = arena if arena is not None else model
    resolved = (
        resolve_arena_model(arena_or_model, restrict_to_models_dir=True)
        if arena_or_model is not None
        else self._model_path
    )
    verify_arena_integrity(resolved)

    new_model = mujoco.MjModel.from_xml_path(str(resolved))
    keep_prev = (resolved == self._model_path) if preserve_target is None else preserve_target
    prev_target_pos = self._data.mocap_pos[self._bindings.target_mocap_id].copy()
    validated_target: tuple[float, float] | None = None
    if target_xy is not None:
      validated_target = self._validate_target_xy(target_xy, model=new_model, model_path=resolved)
    elif keep_prev and resolved != self._model_path:
      validated_target = self._validate_target_xy(
          (float(prev_target_pos[0]), float(prev_target_pos[1])),
          model=new_model,
          model_path=resolved,
      )

    self.arena_id = identify_arena_id(resolved)
    self._model_path = resolved
    self.mode = 'manual'
    self.left_rad_s = 0.0
    self.right_rad_s = 0.0
    self._command_deadline = None
    self.reset_requested = True
    # Placeholder queue; replaced in _drain_http_queue with the caller's resp_q.
    self._pending_reload_resp = (queue.Queue(), validated_target, keep_prev, prev_target_pos)
    self.handle.reload()
    return {}

  def _read_rf_with_hit(self, key: str) -> tuple[float, bool]:
    assert self._data is not None and self._rangefinders is not None
    adr, cutoff = self._rangefinders[key]
    raw = float(self._data.sensordata[adr])
    if raw < 0.0 or raw >= cutoff - 1e-6:
      return round(cutoff, 4), False
    return round(raw, 4), True

  def api_get_status(self, include_vision: bool = False) -> dict:
    assert self._model is not None and self._data is not None
    assert self._bindings is not None and self._sensor_adr is not None and self._rangefinders is not None
    assert self._model_path is not None
    if self._step_counter == 0:
      mujoco.mj_forward(self._model, self._data)

    rangefinders_m: dict[str, float] = {}
    rangefinder_hits: dict[str, bool] = {}
    for key in self._rangefinders:
      dist_m, hit = self._read_rf_with_hit(key)
      rangefinders_m[key] = dist_m
      rangefinder_hits[key] = hit

    left_vel = float(self._data.sensordata[self._sensor_adr['left_wheel_vel']])
    right_vel = float(self._data.sensordata[self._sensor_adr['right_wheel_vel']])
    forward_speed = float(self._data.sensordata[self._sensor_adr['chassis_vel']])
    yaw_rate = float(self._data.sensordata[self._sensor_adr['chassis_gyro'] + 2])

    qadr = self._bindings.robot_qpos_adr
    mocap_id = self._bindings.target_mocap_id
    quat = self._data.qpos[qadr + 3 : qadr + 7]
    yaw_rad = float(
        np.arctan2(
            2.0 * (quat[0] * quat[3] + quat[1] * quat[2]),
            1.0 - 2.0 * (quat[2] ** 2 + quat[3] ** 2),
        )
    )
    remaining = (
        max(0.0, float(self._command_deadline - self._data.time))
        if self._command_deadline is not None
        else 0.0
    )

    result: dict[str, Any] = {
        'sim_time': round(float(self._data.time), 4),
        'moving': remaining > 0.0,
        'command_remaining_s': round(remaining, 4),
        'sensors': {
            'rangefinders_m': rangefinders_m,
            'rangefinder_hits': rangefinder_hits,
            'wheel_vel_rad_s': {'left': round(left_vel, 4), 'right': round(right_vel, 4)},
            'forward_speed_m_s': round(forward_speed, 4),
            'odom_dist_m': round(self._odom_dist_m, 4),
            'odom_x_m': round(self._odom_x_m, 4),
            'odom_y_m': round(self._odom_y_m, 4),
            'yaw_rate_rad_s': round(yaw_rate, 4),
            'gyro_yaw_deg': round(math.degrees(self._gyro_yaw_rad), 3),
        },
        'debug': {
            'arena': identify_arena_id(self._model_path),
            'model': self._model_path.name,
            'robot_xy': [round(float(self._data.qpos[qadr]), 4), round(float(self._data.qpos[qadr + 1]), 4)],
            'robot_yaw_deg': round(math.degrees(yaw_rad), 2),
            'target_xy': [
                round(float(self._data.mocap_pos[mocap_id][0]), 4),
                round(float(self._data.mocap_pos[mocap_id][1]), 4),
            ],
            'in_contact': bool(self._in_hazard_contact),
            'contact_count': int(self._contact_count),
            'contact_steps': int(self._contact_steps),
        },
    }
    if include_vision:
      result['vision'] = self.api_analyze_vision('front_cam', _forward=False)
    return result

  def api_analyze_vision(self, camera_name: str = 'front_cam', *, _forward: bool = True) -> dict:
    assert self._model is not None and self._data is not None and self._bindings is not None
    assert self._vision_cache is not None and self._clean_vis_opt is not None
    cam_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
    if cam_id < 0:
      raise ValueError(f"Không tìm thấy camera '{camera_name}'.")

    pose_sig = (
        tuple(round(float(x), 3) for x in self._data.qpos),
        tuple(round(float(x), 3) for x in self._data.mocap_pos[self._bindings.target_mocap_id]),
    )
    cached = self._vision_cache.get(camera_name)
    if cached is not None and cached[0] == pose_sig:
      data = cached[1]
      return {
          **data,
          'target': dict(data['target']) if data['target'] is not None else None,
          'obstacles': [dict(obs) for obs in data['obstacles']],
      }

    width, height = DEFAULT_CAMERA_WIDTH, DEFAULT_CAMERA_HEIGHT
    if _forward:
      mujoco.mj_forward(self._model, self._data)
    renderer = self._get_renderer(width=width, height=height)
    renderer.update_scene(self._data, camera=camera_name, scene_option=self._clean_vis_opt)
    arr = renderer.render().astype(np.int16)

    r, g, b = arr[:, :, 0], arr[:, :, 1], arr[:, :, 2]
    green_mask = (g > 110) & ((g - r) > 50) & ((g - b) > 45)
    red_mask = (r > 110) & ((r - g) > 45) & ((r - b) > 45)
    blue_mask = (b > 95) & (r < 120) & ((b - r) > 50) & ((b - g) > 25)
    yellow_mask = (r > 115) & (g > 90) & (b < 85) & ((r - b) > 60) & ((g - b) > 45)

    target_info = RobotSimulation._summarize_mask(green_mask)
    obstacles: list[dict] = []
    for color_name, c_mask in (('yellow', yellow_mask), ('red', red_mask), ('blue', blue_mask)):
      summary = RobotSimulation._summarize_mask(c_mask)
      if summary is not None:
        obstacles.append({'color': color_name, **summary})
    obstacles.sort(key=lambda item: item['area_pct'], reverse=True)

    result = {
        'camera': camera_name,
        'width': width,
        'height': height,
        'target_visible': target_info is not None,
        'target': target_info,
        'obstacles': obstacles,
    }
    self._vision_cache[camera_name] = (
        pose_sig,
        {
            **result,
            'target': dict(target_info) if target_info is not None else None,
            'obstacles': [dict(obs) for obs in obstacles],
        },
    )
    return result

  def api_capture_camera(
      self,
      camera_name: str = 'front_cam',
      out_path: Path | str | None = None,
      width: int = DEFAULT_CAMERA_WIDTH,
      height: int = DEFAULT_CAMERA_HEIGHT,
      img_format: str | None = None,
      quality: int = DEFAULT_JPEG_QUALITY,
  ) -> dict:
    assert self._model is not None and self._data is not None
    cam_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
    if cam_id < 0:
      raise ValueError(f"Không tìm thấy camera '{camera_name}'.")
    if isinstance(width, bool) or not isinstance(width, int) or not (64 <= width <= MAX_CAMERA_WIDTH):
      raise ValueError(f"Chiều rộng ảnh 'width' phải là số nguyên trong [64, {MAX_CAMERA_WIDTH}].")
    if isinstance(height, bool) or not isinstance(height, int) or not (48 <= height <= MAX_CAMERA_HEIGHT):
      raise ValueError(f"Chiều cao ảnh 'height' phải là số nguyên trong [48, {MAX_CAMERA_HEIGHT}].")
    if isinstance(quality, bool) or not isinstance(quality, int) or not (10 <= quality <= 95):
      raise ValueError("Chất lượng nén JPEG 'quality' phải là số nguyên trong [10, 95].")

    fmt = 'PNG' if (img_format and img_format.strip().lower() == 'png') else 'JPEG'
    ext = 'png' if fmt == 'PNG' else 'jpg'
    target_path = (
        Path(out_path).resolve()
        if out_path is not None
        else (DEFAULT_CAPTURES_DIR / f'{camera_name}.{ext}').resolve()
    )
    target_path.parent.mkdir(parents=True, exist_ok=True)

    mujoco.mj_forward(self._model, self._data)
    renderer = self._get_renderer(width=width, height=height)
    renderer.update_scene(self._data, camera=camera_name)
    rgb = renderer.render()
    pil_img = Image.fromarray(rgb)
    if fmt == 'JPEG':
      pil_img.save(target_path, format='JPEG', quality=quality, optimize=True)
    else:
      pil_img.save(target_path, format='PNG')
    return {
        'camera': camera_name,
        'path': str(target_path),
        'width': width,
        'height': height,
        'format': fmt,
        'bytes': int(target_path.stat().st_size),
    }

  # --- Step handlers ---

  def _compute_auto_wheels(self, data: mujoco.MjData) -> tuple[float, float]:
    rf_c = _read_rf(data, 'rf_center')
    rf_l = _read_rf(data, 'rf_left')
    rf_r = _read_rf(data, 'rf_right')
    rf_l90 = _read_rf(data, 'rf_left90')
    rf_r90 = _read_rf(data, 'rf_right90')
    base = self.auto_speed_rad_s
    if rf_c < 0.32 or min(rf_l, rf_r) < 0.24:
      turn = base * 0.85
      if (rf_l + 0.5 * rf_l90) >= (rf_r + 0.5 * rf_r90):
        return -turn, turn
      return turn, -turn
    steer = 0.0
    if rf_l < 0.70 or rf_r < 0.70:
      steer = max(-1.0, min(1.0, (rf_l - rf_r) * 1.4))
    return (
        max(-MAX_WHEEL_RAD_S, min(MAX_WHEEL_RAD_S, base * (1.0 - 0.55 * steer))),
        max(-MAX_WHEEL_RAD_S, min(MAX_WHEEL_RAD_S, base * (1.0 + 0.55 * steer))),
    )

  @messages.handler
  def on_pre_step(self, event: messages.PreStepEvent) -> None:
    self._ensure_runtime_fields()
    self._model = event.model
    self._data = event.data
    self._drain_http_queue()
    if self._pending_reload_resp is not None:
      return

    if self._command_deadline is not None and event.data.time >= self._command_deadline:
      self.api_stop()

    if self._bindings is not None and self._bindings.moving_obs_drivers:
      t_cur = float(event.data.time)
      for drv in self._bindings.moving_obs_drivers:
        s_des = drv.amp * math.sin(drv.omega * t_cur + drv.phase)
        v_des = drv.amp * drv.omega * math.cos(drv.omega * t_cur + drv.phase)
        a_ff = -drv.amp * (drv.omega * drv.omega) * math.sin(drv.omega * t_cur + drv.phase)
        s_cur = float(event.data.qpos[drv.qpos_adr])
        v_cur = float(event.data.qvel[drv.dof_adr])
        event.data.qfrc_applied[drv.dof_adr] = (
            drv.mass * a_ff + 1600.0 * (s_des - s_cur) + 220.0 * (v_des - v_cur)
        )

    if self.mode == 'auto':
      left_cmd, right_cmd = self._compute_auto_wheels(event.data)
    else:
      left_cmd, right_cmd = self.left_rad_s, self.right_rad_s

    if self._bindings is not None:
      event.data.ctrl[self._bindings.left_motor_id] = left_cmd
      event.data.ctrl[self._bindings.right_motor_id] = right_cmd

  @messages.handler
  def on_post_step(self, event: messages.PostStepEvent) -> None:
    model = event.model
    data = event.data
    self._state_epoch += 1

    if self._bindings is not None and self._bindings.hazard_geom_ids:
      robot_geoms = self._bindings.robot_geom_ids
      hazard_geoms = self._bindings.hazard_geom_ids
      touching_now = any(
          (int(c.geom1) in robot_geoms and int(c.geom2) in hazard_geoms)
          or (int(c.geom2) in robot_geoms and int(c.geom1) in hazard_geoms)
          for c in data.contact[: data.ncon]
      )
      if touching_now:
        self._contact_steps += 1
        if not self._in_hazard_contact:
          self._contact_count += 1
      self._in_hazard_contact = touching_now

    if self._sensor_adr is not None:
      t_now = float(data.time)
      dt = t_now - self._last_odom_time
      if dt < 0.0:
        self._last_odom_time = t_now
      elif dt > 1e-9:
        self._last_odom_time = t_now
        gyro_z = float(data.sensordata[self._sensor_adr['chassis_gyro'] + 2])
        vel_adr = self._sensor_adr['chassis_vel']
        fwd_v = float(data.sensordata[vel_adr])
        lat_v = float(data.sensordata[vel_adr + 1])
        prev_yaw = self._gyro_yaw_rad
        self._gyro_yaw_rad += gyro_z * dt
        self._odom_dist_m += fwd_v * dt
        mid_yaw = 0.5 * (prev_yaw + self._gyro_yaw_rad)
        self._odom_x_m += (fwd_v * math.cos(mid_yaw) - lat_v * math.sin(mid_yaw)) * dt
        self._odom_y_m += (fwd_v * math.sin(mid_yaw) + lat_v * math.cos(mid_yaw)) * dt

    if self._command_deadline is not None and data.time >= self._command_deadline:
      self.api_stop()

    self._step_counter += 1
    if self._step_counter % 40 == 0:
      self._publish_telemetry(data)

  def _poll_topo_summary(self) -> tuple[int, int, int]:
    now = time.monotonic()
    if (now - self._topo_poll_wall) < 0.5:
      return self._topo_summary
    self._topo_poll_wall = now
    try:
      if not DEFAULT_TOPO_STATE_PATH.is_file():
        self._topo_mtime_ns = -1
        self._topo_summary = (0, -1, 0)
        return self._topo_summary
      mtime_ns = DEFAULT_TOPO_STATE_PATH.stat().st_mtime_ns
      if mtime_ns != self._topo_mtime_ns:
        summary = read_topo_studio_summary()
        self._topo_summary = summary
        self._topo_mtime_ns = mtime_ns
    except OSError:
      pass
    return self._topo_summary

  def _publish_telemetry(self, data: mujoco.MjData) -> None:
    try:
      l_cmd = float(data.actuator('left_motor').ctrl[0])
      r_cmd = float(data.actuator('right_motor').ctrl[0])
      l_vel = float(data.sensor('left_wheel_vel').data[0])
      r_vel = float(data.sensor('right_wheel_vel').data[0])
      fwd_v = float(data.sensor('chassis_vel').data[0])
    except KeyError:
      l_cmd = r_cmd = l_vel = r_vel = fwd_v = 0.0

    topo_nodes, topo_cur, topo_open = self._poll_topo_summary()
    self.handle.send_to_viewer(
        user_messages.RobotTelemetry(
            arena_id=self.arena_id,
            sim_time=float(data.time),
            mode=self.mode,
            left_cmd=l_cmd,
            right_cmd=r_cmd,
            left_vel=l_vel,
            right_vel=r_vel,
            forward_speed=fwd_v,
            gyro_yaw_deg=math.degrees(self._gyro_yaw_rad),
            odom_dist_m=self._odom_dist_m,
            rf_left120=_read_rf(data, 'rf_left120'),
            rf_left90=_read_rf(data, 'rf_left90'),
            rf_left=_read_rf(data, 'rf_left'),
            rf_center=_read_rf(data, 'rf_center'),
            rf_right=_read_rf(data, 'rf_right'),
            rf_right90=_read_rf(data, 'rf_right90'),
            rf_right120=_read_rf(data, 'rf_right120'),
            contact_count=self._contact_count,
            in_contact=self._in_hazard_contact,
            topo_node_count=topo_nodes,
            topo_current_node=topo_cur,
            topo_open_exits=topo_open,
        )
    )
