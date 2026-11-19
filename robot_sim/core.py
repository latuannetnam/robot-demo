from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import mujoco
import numpy as np
from PIL import Image

from robot_sim.arenas import ARENAS, identify_arena_id, resolve_arena_model
from robot_sim.integrity import verify_arena_integrity

WHEEL_RADIUS = 0.05
TRACK_WIDTH = 0.23
MAX_WHEEL_RAD_S = 15.0
MAX_COMMAND_DURATION = 10.0
RANGEFINDER_MAX = 3.0
ARENA_SAFE_LIMIT = 1.7
DEFAULT_CAMERA_WIDTH = 320
DEFAULT_CAMERA_HEIGHT = 240
MAX_CAMERA_WIDTH = 640
MAX_CAMERA_HEIGHT = 480
DEFAULT_JPEG_QUALITY = 75

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL_PATH = PROJECT_ROOT / "models" / "arena_2wheel.xml"
DEFAULT_CAPTURES_DIR = PROJECT_ROOT / "captures"


def _require_finite(**kwargs: float) -> None:
    for name, val in kwargs.items():
        if isinstance(val, bool) or not isinstance(val, (int, float)) or not math.isfinite(float(val)):
            raise ValueError(f"Tham số '{name}' phải là số hữu hạn hợp lệ (nhận được: {val!r}).")


def compute_move_command(meters: float, speed_m_s: float = 0.25) -> tuple[float, float, float]:
    """Tính vận tốc góc 2 bánh (rad/s) và thời gian (s) để tiến/lùi một quãng đường ước lượng."""
    _require_finite(meters=meters, speed_m_s=speed_m_s)
    if speed_m_s <= 0.0:
        raise ValueError("Tốc độ di chuyển 'speed_m_s' phải lớn hơn 0.")
    if abs(meters) < 1e-6:
        raise ValueError("Quãng đường 'meters' phải khác 0.")

    omega = min(abs(speed_m_s) / WHEEL_RADIUS, MAX_WHEEL_RAD_S)
    effective_speed = omega * WHEEL_RADIUS
    duration = min(abs(meters) / effective_speed, MAX_COMMAND_DURATION)
    direction = 1.0 if meters > 0.0 else -1.0
    return direction * omega, direction * omega, duration


def compute_turn_command(degrees: float, angular_deg_s: float = 60.0) -> tuple[float, float, float]:
    """Tính vận tốc góc 2 bánh (rad/s) và thời gian (s) để xoay tại chỗ (+degrees = quay trái ngược chiều kim đồng hồ)."""
    _require_finite(degrees=degrees, angular_deg_s=angular_deg_s)
    if angular_deg_s <= 0.0:
        raise ValueError("Tốc độ góc 'angular_deg_s' phải lớn hơn 0.")
    if abs(degrees) < 1e-6:
        raise ValueError("Góc xoay 'degrees' phải khác 0.")

    target_yaw_rate = math.radians(abs(angular_deg_s))
    omega = min((target_yaw_rate * TRACK_WIDTH) / (2.0 * WHEEL_RADIUS), MAX_WHEEL_RAD_S)
    effective_yaw_rate = (2.0 * WHEEL_RADIUS * omega) / TRACK_WIDTH
    duration = min(math.radians(abs(degrees)) / effective_yaw_rate, MAX_COMMAND_DURATION)

    if degrees > 0.0:
        # Quay trái (CCW): bánh phải tiến (+), bánh trái lùi (-)
        return -omega, omega, duration
    return omega, -omega, duration


def _discover_sensors(model: mujoco.MjModel) -> tuple[dict[str, int], dict[str, tuple[int, float]]]:
    fixed_adr: dict[str, int] = {}
    for name in ("left_wheel_vel", "right_wheel_vel", "chassis_vel", "chassis_gyro"):
        sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
        if sid < 0:
            raise ValueError(f"Mô hình thiếu cảm biến bắt buộc '{name}'.")
        fixed_adr[name] = int(model.sensor_adr[sid])

    rangefinders: dict[str, tuple[int, float]] = {}
    rf_type = int(mujoco.mjtSensor.mjSENS_RANGEFINDER)
    for sid in range(model.nsensor):
        if int(model.sensor_type[sid]) == rf_type:
            sname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, sid) or f"rf_{sid}"
            key = sname[3:] if sname.startswith("rf_") else sname
            if key in rangefinders:
                raise ValueError(f"Trùng khóa cảm biến khoảng cách '{key}'.")
            raw_cutoff = float(model.sensor_cutoff[sid])
            cutoff = raw_cutoff if raw_cutoff > 0.0 else RANGEFINDER_MAX
            rangefinders[key] = (int(model.sensor_adr[sid]), cutoff)

    return fixed_adr, rangefinders


def _read_spawn_pose(model: mujoco.MjModel) -> tuple[float, float, float]:
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "robot_spawn")
    if site_id < 0:
        return 0.0, 0.0, 0.0
    pos = model.site_pos[site_id]
    quat = model.site_quat[site_id]
    w, x, y, z = float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])
    yaw_rad = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return float(pos[0]), float(pos[1]), yaw_rad


def _apply_spawn_pose(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    sx, sy, syaw = _read_spawn_pose(model)
    qadr = 0
    rf_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "robot_free")
    if rf_jid >= 0:
        qadr = int(model.jnt_qposadr[rf_jid])
    data.qpos[qadr] = sx
    data.qpos[qadr + 1] = sy
    data.qpos[qadr + 3 : qadr + 7] = [math.cos(syaw / 2.0), 0.0, 0.0, math.sin(syaw / 2.0)]


@dataclass(frozen=True)
class _MovingObsDriver:
    body_name: str
    qpos_adr: int
    dof_adr: int
    amp: float
    omega: float
    phase: float
    mass: float
    base_xy: tuple[float, float]
    axis_xy: tuple[float, float]
    clearance_radius: float


@dataclass(frozen=True)
class _ModelBindings:
    robot_qpos_adr: int
    target_mocap_id: int
    left_motor_id: int
    right_motor_id: int
    robot_geom_ids: frozenset[int]
    hazard_geom_ids: frozenset[int]
    moving_obs_drivers: tuple[_MovingObsDriver, ...]


def _discover_model_bindings(model: mujoco.MjModel, model_path: Path) -> _ModelBindings:
    rf_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "robot_free")
    robot_qpos_adr = int(model.jnt_qposadr[rf_jid]) if rf_jid >= 0 else 0

    target_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "target")
    target_mocap_id = int(model.body_mocapid[target_bid]) if (target_bid >= 0 and int(model.body_mocapid[target_bid]) >= 0) else 0

    left_act = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "left_motor")
    right_act = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "right_motor")
    left_motor_id = int(left_act) if left_act >= 0 else 0
    right_motor_id = int(right_act) if right_act >= 0 else 1

    robot_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "robot")
    robot_bodies: set[int] = {int(robot_bid)} if robot_bid >= 0 else set()
    changed = True
    while changed and robot_bodies:
        changed = False
        for bid in range(model.nbody):
            if bid not in robot_bodies and int(model.body_parentid[bid]) in robot_bodies:
                robot_bodies.add(bid)
                changed = True

    robot_geoms: set[int] = set()
    hazard_geoms: set[int] = set()
    for gid in range(model.ngeom):
        bid = int(model.geom_bodyid[gid])
        if bid in robot_bodies:
            if int(model.geom_contype[gid]) != 0 and int(model.geom_conaffinity[gid]) != 0:
                robot_geoms.add(gid)
            continue
        gname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid) or ""
        bname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, bid) or ""
        if gname.startswith(("wall_", "maze_w", "obs_")) or bname.startswith("obs_"):
            hazard_geoms.add(gid)

    drivers: list[_MovingObsDriver] = []
    arena_id = identify_arena_id(model_path)
    spec = ARENAS.get(arena_id)
    if spec is not None and spec.moving_obstacles:
        for mspec in spec.moving_obstacles.values():
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, mspec.joint_name)
            if jid < 0:
                raise ValueError(f"Mô hình '{model_path.name}' thiếu khớp trượt '{mspec.joint_name}'.")
            rad = (
                float(mspec.geom_size[0])
                if mspec.geom_type == "cylinder"
                else math.hypot(float(mspec.geom_size[0]), float(mspec.geom_size[1]))
            )
            drivers.append(
                _MovingObsDriver(
                    body_name=mspec.body_name,
                    qpos_adr=int(model.jnt_qposadr[jid]),
                    dof_adr=int(model.jnt_dofadr[jid]),
                    amp=float(mspec.amplitude_m),
                    omega=2.0 * math.pi * float(mspec.freq_hz),
                    phase=float(mspec.phase_rad),
                    mass=float(mspec.mass_kg),
                    base_xy=(float(mspec.base_pos[0]), float(mspec.base_pos[1])),
                    axis_xy=(float(mspec.slide_axis[0]), float(mspec.slide_axis[1])),
                    clearance_radius=rad,
                )
            )

    return _ModelBindings(
        robot_qpos_adr=robot_qpos_adr,
        target_mocap_id=target_mocap_id,
        left_motor_id=left_motor_id,
        right_motor_id=right_motor_id,
        robot_geom_ids=frozenset(robot_geoms),
        hazard_geom_ids=frozenset(hazard_geoms),
        moving_obs_drivers=tuple(drivers),
    )


def _init_moving_obstacles(data: mujoco.MjData, drivers: tuple[_MovingObsDriver, ...]) -> None:
    t = float(data.time)
    for drv in drivers:
        data.qpos[drv.qpos_adr] = drv.amp * math.sin(drv.omega * t + drv.phase)
        data.qvel[drv.dof_adr] = drv.amp * drv.omega * math.cos(drv.omega * t + drv.phase)


class RobotSimulation:
    """Lõi mô phỏng vật lý MuJoCo cho xe robot 2 bánh vi sai."""

    def __init__(self, model_path: Path | str | None = None) -> None:
        self.model_path = resolve_arena_model(model_path)
        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.data = mujoco.MjData(self.model)
        self._bindings = _discover_model_bindings(self.model, self.model_path)
        self._default_target_pos = self.data.mocap_pos[self._bindings.target_mocap_id].copy()
        _apply_spawn_pose(self.model, self.data)
        _init_moving_obstacles(self.data, self._bindings.moving_obs_drivers)
        self._command_deadline: float | None = None
        self._gyro_yaw_rad: float = 0.0
        self._odom_dist_m: float = 0.0
        self._odom_x_m: float = 0.0
        self._odom_y_m: float = 0.0
        self._contact_count: int = 0
        self._contact_steps: int = 0
        self._in_hazard_contact: bool = False
        self._renderer: mujoco.Renderer | None = None
        self._renderer_size: tuple[int, int] | None = None
        self._clean_vis_opt = mujoco.MjvOption()
        self._clean_vis_opt.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = 0

        self._sensor_adr, self._rangefinders = _discover_sensors(self.model)
        self._state_epoch: int = 0
        self._vision_cache: dict[str, tuple[tuple, dict]] = {}
        mujoco.mj_forward(self.model, self.data)

    def close(self) -> None:
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
            self._renderer_size = None

    def reload(
        self,
        model_path: Path | str | None = None,
        preserve_target: bool | None = None,
        target_xy: tuple[float, float] | list[float] | None = None,
    ) -> dict:
        """Kiểm định tính toàn vẹn sa bàn và nạp lại mô hình vật lý an toàn (nguyên tử)."""
        if preserve_target is not None and not isinstance(preserve_target, bool):
            raise ValueError("Tham số 'preserve_target' phải là kiểu boolean (true/false).")
        resolved = resolve_arena_model(model_path) if model_path is not None else self.model_path
        verify_arena_integrity(resolved)

        new_model = mujoco.MjModel.from_xml_path(str(resolved))
        new_bindings = _discover_model_bindings(new_model, resolved)
        new_sensor_adr, new_rangefinders = _discover_sensors(new_model)
        keep_prev = (resolved == self.model_path) if preserve_target is None else preserve_target
        prev_target_pos = self.data.mocap_pos[self._bindings.target_mocap_id].copy()
        validated_target: tuple[float, float] | None = None
        if target_xy is not None:
            validated_target = self.validate_target_xy(target_xy, model=new_model, model_path=resolved)
        elif keep_prev and resolved != self.model_path:
            validated_target = self.validate_target_xy(
                (float(prev_target_pos[0]), float(prev_target_pos[1])),
                model=new_model,
                model_path=resolved,
            )

        new_data = mujoco.MjData(new_model)
        new_mocap_id = new_bindings.target_mocap_id
        new_default_target_pos = new_data.mocap_pos[new_mocap_id].copy()
        _apply_spawn_pose(new_model, new_data)
        _init_moving_obstacles(new_data, new_bindings.moving_obs_drivers)
        if validated_target is not None:
            new_data.mocap_pos[new_mocap_id] = [
                validated_target[0],
                validated_target[1],
                float(new_default_target_pos[2]),
            ]
        elif keep_prev:
            new_data.mocap_pos[new_mocap_id] = [
                float(prev_target_pos[0]),
                float(prev_target_pos[1]),
                float(new_default_target_pos[2]),
            ]
        else:
            new_data.mocap_pos[new_mocap_id] = new_default_target_pos.copy()
        mujoco.mj_forward(new_model, new_data)

        self.close()
        self.model_path = resolved
        self.model = new_model
        self.data = new_data
        self._bindings = new_bindings
        self._default_target_pos = new_default_target_pos
        self._sensor_adr = new_sensor_adr
        self._rangefinders = new_rangefinders
        self._command_deadline = None
        self._gyro_yaw_rad = 0.0
        self._odom_dist_m = 0.0
        self._odom_x_m = 0.0
        self._odom_y_m = 0.0
        self._contact_count = 0
        self._contact_steps = 0
        self._in_hazard_contact = False
        self._state_epoch += 1
        self._vision_cache.clear()
        return self.get_status()

    def _get_renderer(self, width: int, height: int) -> mujoco.Renderer:
        if self._renderer is not None and self._renderer_size != (width, height):
            self._renderer.close()
            self._renderer = None
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, height=height, width=width)
            self._renderer_size = (width, height)
        return self._renderer

    def set_wheels(self, left_rad_s: float, right_rad_s: float, duration: float = 1.0) -> dict:
        _require_finite(left_rad_s=left_rad_s, right_rad_s=right_rad_s, duration=duration)
        if duration <= 0.0:
            raise ValueError("Thời gian thực thi 'duration' phải lớn hơn 0 giây.")

        clamped_left = max(-MAX_WHEEL_RAD_S, min(MAX_WHEEL_RAD_S, float(left_rad_s)))
        clamped_right = max(-MAX_WHEEL_RAD_S, min(MAX_WHEEL_RAD_S, float(right_rad_s)))
        capped_duration = min(float(duration), MAX_COMMAND_DURATION)

        self.data.ctrl[self._bindings.left_motor_id] = clamped_left
        self.data.ctrl[self._bindings.right_motor_id] = clamped_right
        self._command_deadline = float(self.data.time) + capped_duration
        return {
            "left_rad_s": round(clamped_left, 4),
            "right_rad_s": round(clamped_right, 4),
            "duration_s": round(capped_duration, 4),
        }

    def move(self, meters: float, speed_m_s: float = 0.25) -> dict:
        left, right, duration = compute_move_command(meters=meters, speed_m_s=speed_m_s)
        cmd = self.set_wheels(left, right, duration=duration)
        cmd["requested_meters"] = round(float(meters), 4)
        return cmd

    def turn(self, degrees: float, angular_deg_s: float = 60.0) -> dict:
        left, right, duration = compute_turn_command(degrees=degrees, angular_deg_s=angular_deg_s)
        cmd = self.set_wheels(left, right, duration=duration)
        cmd["requested_degrees"] = round(float(degrees), 4)
        return cmd

    def stop(self) -> dict:
        self.data.ctrl[self._bindings.left_motor_id] = 0.0
        self.data.ctrl[self._bindings.right_motor_id] = 0.0
        self._command_deadline = None
        return {"stopped": True, "sim_time": round(float(self.data.time), 4)}

    @staticmethod
    def _point_to_segment_dist(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
        dx, dy = bx - ax, by - ay
        len_sq = dx * dx + dy * dy
        if len_sq <= 1e-12:
            return math.hypot(px - ax, py - ay)
        t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / len_sq))
        return math.hypot(px - (ax + t * dx), py - (ay + t * dy))

    def validate_target_xy(
        self,
        target_xy: tuple[float, float] | list[float],
        *,
        model: mujoco.MjModel | None = None,
        model_path: Path | None = None,
    ) -> tuple[float, float]:
        if not isinstance(target_xy, (list, tuple)) or len(target_xy) != 2:
            raise ValueError("'target_xy' phải là danh sách gồm đúng 2 số [x, y].")
        _require_finite(target_x=target_xy[0], target_y=target_xy[1])
        tx, ty = float(target_xy[0]), float(target_xy[1])
        if abs(tx) > ARENA_SAFE_LIMIT or abs(ty) > ARENA_SAFE_LIMIT:
            raise ValueError(
                f"Tọa độ mục tiêu ({tx}, {ty}) nằm ngoài vùng an toàn của sân [-{ARENA_SAFE_LIMIT}, {ARENA_SAFE_LIMIT}]m."
            )

        target_model = model if model is not None else self.model
        target_radius = 0.10
        box_type = int(mujoco.mjtGeom.mjGEOM_BOX)
        cyl_type = int(mujoco.mjtGeom.mjGEOM_CYLINDER)
        moving_names: set[str] = set()

        check_path = model_path if model_path is not None else self.model_path
        arena_id = identify_arena_id(check_path)
        spec = ARENAS.get(arena_id)
        if spec is not None and spec.moving_obstacles:
            moving_names = set(spec.moving_obstacles.keys())
            for mspec in spec.moving_obstacles.values():
                bx, by = float(mspec.base_pos[0]), float(mspec.base_pos[1])
                ax_dir, ay_dir = float(mspec.slide_axis[0]), float(mspec.slide_axis[1])
                amp = float(mspec.amplitude_m)
                p1x, p1y = bx - amp * ax_dir, by - amp * ay_dir
                p2x, p2y = bx + amp * ax_dir, by + amp * ay_dir
                obs_rad = (
                    float(mspec.geom_size[0])
                    if mspec.geom_type == "cylinder"
                    else max(float(mspec.geom_size[0]), float(mspec.geom_size[1]))
                )
                if self._point_to_segment_dist(tx, ty, p1x, p1y, p2x, p2y) < (obs_rad + target_radius - 1e-6):
                    raise ValueError(
                        f"Tọa độ mục tiêu ({tx}, {ty}) nằm trên hành lang tuần tra của vật cản di động '{mspec.body_name}'."
                    )

        for gid in range(target_model.ngeom):
            gname = mujoco.mj_id2name(target_model, mujoco.mjtObj.mjOBJ_GEOM, gid) or ""
            gtype = int(target_model.geom_type[gid])
            bid = int(target_model.geom_bodyid[gid])
            bname = mujoco.mj_id2name(target_model, mujoco.mjtObj.mjOBJ_BODY, bid) or ""

            if gname.startswith(("wall_", "maze_w")) and gtype == box_type:
                cx = float(target_model.body_pos[bid][0] + target_model.geom_pos[gid][0])
                cy = float(target_model.body_pos[bid][1] + target_model.geom_pos[gid][1])
                sx = float(target_model.geom_size[gid][0])
                sy = float(target_model.geom_size[gid][1])
                dx = max(abs(tx - cx) - sx, 0.0)
                dy = max(abs(ty - cy) - sy, 0.0)
                if math.hypot(dx, dy) < target_radius - 1e-6:
                    raise ValueError(
                        f"Tọa độ mục tiêu ({tx}, {ty}) chồng lấn lên tường '{gname}'."
                    )
            elif bname.startswith("obs_") and bname not in moving_names:
                ox = float(target_model.body_pos[bid][0] + target_model.geom_pos[gid][0])
                oy = float(target_model.body_pos[bid][1] + target_model.geom_pos[gid][1])
                if gtype == box_type:
                    sx = float(target_model.geom_size[gid][0])
                    sy = float(target_model.geom_size[gid][1])
                    if abs(tx - ox) <= sx + 1e-6 and abs(ty - oy) <= sy + 1e-6:
                        raise ValueError(
                            f"Tọa độ mục tiêu ({tx}, {ty}) nằm bên trong vật cản '{bname}'."
                        )
                elif gtype == cyl_type:
                    rad = float(target_model.geom_size[gid][0])
                    if math.hypot(tx - ox, ty - oy) <= rad + 1e-6:
                        raise ValueError(
                            f"Tọa độ mục tiêu ({tx}, {ty}) nằm bên trong vật cản '{bname}'."
                        )

        return tx, ty

    def reset(self, target_xy: tuple[float, float] | list[float] | None = None) -> dict:
        if target_xy is not None:
            tx, ty = self.validate_target_xy(target_xy)
        else:
            tx, ty = float(self._default_target_pos[0]), float(self._default_target_pos[1])

        mujoco.mj_resetData(self.model, self.data)
        _apply_spawn_pose(self.model, self.data)
        _init_moving_obstacles(self.data, self._bindings.moving_obs_drivers)
        self.data.mocap_pos[self._bindings.target_mocap_id] = [tx, ty, float(self._default_target_pos[2])]
        self._command_deadline = None
        self._gyro_yaw_rad = 0.0
        self._odom_dist_m = 0.0
        self._odom_x_m = 0.0
        self._odom_y_m = 0.0
        self._contact_count = 0
        self._contact_steps = 0
        self._in_hazard_contact = False
        self._state_epoch += 1
        self._vision_cache.clear()
        mujoco.mj_forward(self.model, self.data)
        return self.get_status()

    def step_once(self) -> None:
        if self._command_deadline is not None and self.data.time >= self._command_deadline:
            self.stop()
        if self._bindings.moving_obs_drivers:
            t_cur = float(self.data.time)
            for drv in self._bindings.moving_obs_drivers:
                s_des = drv.amp * math.sin(drv.omega * t_cur + drv.phase)
                v_des = drv.amp * drv.omega * math.cos(drv.omega * t_cur + drv.phase)
                a_ff = -drv.amp * (drv.omega * drv.omega) * math.sin(drv.omega * t_cur + drv.phase)
                s_cur = float(self.data.qpos[drv.qpos_adr])
                v_cur = float(self.data.qvel[drv.dof_adr])
                self.data.qfrc_applied[drv.dof_adr] = (
                    drv.mass * a_ff + 1600.0 * (s_des - s_cur) + 220.0 * (v_des - v_cur)
                )
        mujoco.mj_step(self.model, self.data)
        self._state_epoch += 1
        if self._bindings.hazard_geom_ids:
            robot_geoms = self._bindings.robot_geom_ids
            hazard_geoms = self._bindings.hazard_geom_ids
            touching_now = any(
                (int(c.geom1) in robot_geoms and int(c.geom2) in hazard_geoms)
                or (int(c.geom2) in robot_geoms and int(c.geom1) in hazard_geoms)
                for c in self.data.contact[: self.data.ncon]
            )
            if touching_now:
                self._contact_steps += 1
                if not self._in_hazard_contact:
                    self._contact_count += 1
            self._in_hazard_contact = touching_now
        gyro_z = float(self.data.sensordata[self._sensor_adr["chassis_gyro"] + 2])
        vel_adr = self._sensor_adr["chassis_vel"]
        fwd_v = float(self.data.sensordata[vel_adr])
        lat_v = float(self.data.sensordata[vel_adr + 1])
        dt = float(self.model.opt.timestep)
        prev_yaw = self._gyro_yaw_rad
        self._gyro_yaw_rad += gyro_z * dt
        self._odom_dist_m += fwd_v * dt
        mid_yaw = 0.5 * (prev_yaw + self._gyro_yaw_rad)
        self._odom_x_m += (fwd_v * math.cos(mid_yaw) - lat_v * math.sin(mid_yaw)) * dt
        self._odom_y_m += (fwd_v * math.sin(mid_yaw) + lat_v * math.cos(mid_yaw)) * dt
        if self._command_deadline is not None and self.data.time >= self._command_deadline:
            self.stop()

    def step_seconds(self, seconds: float) -> None:
        _require_finite(seconds=seconds)
        if seconds < 0.0:
            raise ValueError("Số giây mô phỏng không được âm.")
        steps = int(round(seconds / self.model.opt.timestep))
        for _ in range(steps):
            self.step_once()

    def _read_rangefinder(self, key: str) -> tuple[float, bool]:
        adr, cutoff = self._rangefinders[key]
        raw = float(self.data.sensordata[adr])
        if raw < 0.0 or raw >= cutoff - 1e-6:
            return round(cutoff, 4), False
        return round(raw, 4), True

    def get_status(self, include_vision: bool = False) -> dict:
        mujoco.mj_forward(self.model, self.data)
        rangefinders_m: dict[str, float] = {}
        rangefinder_hits: dict[str, bool] = {}
        for key in self._rangefinders:
            dist_m, hit = self._read_rangefinder(key)
            rangefinders_m[key] = dist_m
            rangefinder_hits[key] = hit

        left_vel = float(self.data.sensordata[self._sensor_adr["left_wheel_vel"]])
        right_vel = float(self.data.sensordata[self._sensor_adr["right_wheel_vel"]])
        forward_speed = float(self.data.sensordata[self._sensor_adr["chassis_vel"]])
        yaw_rate = float(self.data.sensordata[self._sensor_adr["chassis_gyro"] + 2])

        qadr = self._bindings.robot_qpos_adr
        mocap_id = self._bindings.target_mocap_id
        quat = self.data.qpos[qadr + 3 : qadr + 7]
        yaw_rad = float(
            np.arctan2(
                2.0 * (quat[0] * quat[3] + quat[1] * quat[2]),
                1.0 - 2.0 * (quat[2] ** 2 + quat[3] ** 2),
            )
        )
        remaining = (
            max(0.0, float(self._command_deadline - self.data.time))
            if self._command_deadline is not None
            else 0.0
        )

        result = {
            "sim_time": round(float(self.data.time), 4),
            "moving": remaining > 0.0,
            "command_remaining_s": round(remaining, 4),
            "sensors": {
                "rangefinders_m": rangefinders_m,
                "rangefinder_hits": rangefinder_hits,
                "wheel_vel_rad_s": {"left": round(left_vel, 4), "right": round(right_vel, 4)},
                "forward_speed_m_s": round(forward_speed, 4),
                "odom_dist_m": round(self._odom_dist_m, 4),
                "odom_x_m": round(self._odom_x_m, 4),
                "odom_y_m": round(self._odom_y_m, 4),
                "yaw_rate_rad_s": round(yaw_rate, 4),
                "gyro_yaw_deg": round(math.degrees(self._gyro_yaw_rad), 3),
            },
            "debug": {
                "arena": identify_arena_id(self.model_path),
                "model": self.model_path.name,
                "robot_xy": [round(float(self.data.qpos[qadr]), 4), round(float(self.data.qpos[qadr + 1]), 4)],
                "robot_yaw_deg": round(math.degrees(yaw_rad), 2),
                "target_xy": [
                    round(float(self.data.mocap_pos[mocap_id][0]), 4),
                    round(float(self.data.mocap_pos[mocap_id][1]), 4),
                ],
                "in_contact": bool(self._in_hazard_contact),
                "contact_count": int(self._contact_count),
                "contact_steps": int(self._contact_steps),
            },
        }
        if include_vision:
            result["vision"] = self.analyze_vision("front_cam", _forward=False)
        return result

    def _validate_camera_name(self, camera_name: str) -> None:
        if not isinstance(camera_name, str) or not camera_name.strip():
            raise ValueError("Tên camera phải là chuỗi hợp lệ ('front_cam' hoặc 'overview_cam').")
        cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
        if cam_id < 0:
            raise ValueError(f"Không tìm thấy camera '{camera_name}'. Chọn 'front_cam' hoặc 'overview_cam'.")

    @staticmethod
    def _summarize_mask(mask: np.ndarray, min_area_pct: float = 0.12) -> dict | None:
        total = mask.shape[0] * mask.shape[1]
        count = int(mask.sum())
        area_pct = (100.0 * count) / float(total)
        if area_pct < min_area_pct:
            return None
        ys, xs = np.where(mask)
        half_w = mask.shape[1] / 2.0
        offset_x = (float(xs.mean()) - half_w) / half_w
        v_span = float(ys.max() - ys.min() + 1) / float(mask.shape[0])
        if offset_x < -0.20:
            side = "left"
        elif offset_x > 0.20:
            side = "right"
        else:
            side = "center"
        return {
            "offset_x": round(offset_x, 3),
            "side": side,
            "area_pct": round(area_pct, 2),
            "v_span": round(v_span, 3),
        }

    def analyze_vision(self, camera_name: str = "front_cam", *, _forward: bool = True) -> dict:
        """Phân tích trực tiếp khung hình camera trong RAM (không ghi đĩa) để trích xuất đích xanh & vật cản."""
        self._validate_camera_name(camera_name)
        pose_sig = (
            self._state_epoch,
            tuple(round(float(x), 6) for x in self.data.qpos),
            tuple(round(float(x), 6) for x in self.data.mocap_pos[self._bindings.target_mocap_id]),
        )
        cached = self._vision_cache.get(camera_name)
        if cached is not None and cached[0] == pose_sig:
            data = cached[1]
            return {
                **data,
                "target": dict(data["target"]) if data["target"] is not None else None,
                "obstacles": [dict(obs) for obs in data["obstacles"]],
            }

        width, height = DEFAULT_CAMERA_WIDTH, DEFAULT_CAMERA_HEIGHT
        if _forward:
            mujoco.mj_forward(self.model, self.data)
        renderer = self._get_renderer(width=width, height=height)
        renderer.update_scene(self.data, camera=camera_name, scene_option=self._clean_vis_opt)
        arr = renderer.render().astype(np.int16)

        r, g, b = arr[:, :, 0], arr[:, :, 1], arr[:, :, 2]
        green_mask = (g > 110) & ((g - r) > 50) & ((g - b) > 45)
        red_mask = (r > 110) & ((r - g) > 45) & ((r - b) > 45)
        blue_mask = (b > 95) & (r < 120) & ((b - r) > 50) & ((b - g) > 25)
        yellow_mask = (r > 115) & (g > 90) & (b < 85) & ((r - b) > 60) & ((g - b) > 45)

        target_info = self._summarize_mask(green_mask)
        obstacles: list[dict] = []
        for color_name, c_mask in (("yellow", yellow_mask), ("red", red_mask), ("blue", blue_mask)):
            summary = self._summarize_mask(c_mask)
            if summary is not None:
                obstacles.append({"color": color_name, **summary})
        obstacles.sort(key=lambda item: item["area_pct"], reverse=True)

        result = {
            "camera": camera_name,
            "width": width,
            "height": height,
            "target_visible": target_info is not None,
            "target": target_info,
            "obstacles": obstacles,
        }
        self._vision_cache[camera_name] = (
            pose_sig,
            {
                **result,
                "target": dict(target_info) if target_info is not None else None,
                "obstacles": [dict(obs) for obs in obstacles],
            },
        )
        return result

    def capture_camera(
        self,
        camera_name: str = "front_cam",
        out_path: Path | str | None = None,
        width: int = DEFAULT_CAMERA_WIDTH,
        height: int = DEFAULT_CAMERA_HEIGHT,
        img_format: str | None = None,
        quality: int = DEFAULT_JPEG_QUALITY,
    ) -> dict:
        self._validate_camera_name(camera_name)
        if isinstance(width, bool) or not isinstance(width, int) or not (64 <= width <= MAX_CAMERA_WIDTH):
            raise ValueError(f"Chiều rộng ảnh 'width' phải là số nguyên trong [64, {MAX_CAMERA_WIDTH}].")
        if isinstance(height, bool) or not isinstance(height, int) or not (48 <= height <= MAX_CAMERA_HEIGHT):
            raise ValueError(f"Chiều cao ảnh 'height' phải là số nguyên trong [48, {MAX_CAMERA_HEIGHT}].")
        if isinstance(quality, bool) or not isinstance(quality, int) or not (10 <= quality <= 95):
            raise ValueError("Chất lượng nén JPEG 'quality' phải là số nguyên trong [10, 95].")

        fmt: str
        if img_format is not None:
            if not isinstance(img_format, str):
                raise ValueError("Định dạng 'format' phải là 'jpg' hoặc 'png'.")
            norm_fmt = img_format.strip().lower()
            if norm_fmt in ("jpg", "jpeg"):
                fmt = "JPEG"
            elif norm_fmt == "png":
                fmt = "PNG"
            else:
                raise ValueError(f"Định dạng ảnh '{img_format}' không hỗ trợ (chọn 'jpg' hoặc 'png').")
        elif out_path is not None and str(out_path).strip().lower().endswith(".png"):
            fmt = "PNG"
        else:
            fmt = "JPEG"

        ext = "png" if fmt == "PNG" else "jpg"
        if out_path is not None:
            if not isinstance(out_path, (str, Path)) or (isinstance(out_path, str) and not out_path.strip()):
                raise ValueError("Đường dẫn lưu ảnh 'path' phải là chuỗi tên file hợp lệ.")
            target_path = Path(out_path).resolve()
        else:
            target_path = (DEFAULT_CAPTURES_DIR / f"{camera_name}.{ext}").resolve()

        if target_path.exists() and target_path.is_dir():
            raise ValueError(f"Đường dẫn '{target_path}' là thư mục; vui lòng truyền đường dẫn tới file .{ext}.")
        target_path.parent.mkdir(parents=True, exist_ok=True)

        mujoco.mj_forward(self.model, self.data)
        renderer = self._get_renderer(width=width, height=height)
        renderer.update_scene(self.data, camera=camera_name)
        rgb = renderer.render()

        pil_img = Image.fromarray(rgb)
        if fmt == "JPEG":
            pil_img.save(target_path, format="JPEG", quality=quality, optimize=True)
        else:
            pil_img.save(target_path, format="PNG")

        return {
            "camera": camera_name,
            "path": str(target_path),
            "width": width,
            "height": height,
            "format": fmt,
            "bytes": int(target_path.stat().st_size),
        }
