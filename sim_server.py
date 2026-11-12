from __future__ import annotations

import argparse
from collections.abc import Callable
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import queue
import threading
import time
from typing import Any
from urllib.parse import parse_qs, urlsplit

from robot_sim.core import DEFAULT_CAMERA_HEIGHT, DEFAULT_CAMERA_WIDTH, DEFAULT_JPEG_QUALITY, RobotSimulation


class SimulationApp:
    """Quản lý vòng lặp vật lý MuJoCo trên luồng chính và nhận lệnh qua hàng đợi từ HTTP server."""

    def __init__(self, model_path: str | None = None) -> None:
        self.sim = RobotSimulation(model_path=model_path)
        self._cmd_queue: queue.Queue[tuple[Callable[[RobotSimulation], dict], queue.Queue]] = queue.Queue()
        self._http_server: ThreadingHTTPServer | None = None
        self._http_thread: threading.Thread | None = None
        self._viewer_reload_pending = False
        self._closed = False

    def submit_command(self, fn: Callable[[RobotSimulation], dict], timeout: float = 5.0) -> tuple[int, dict]:
        if self._closed:
            return 503, {"ok": False, "error": "Máy chủ mô phỏng đã dừng."}
        resp_q: queue.Queue[tuple[int, dict]] = queue.Queue(maxsize=1)
        self._cmd_queue.put((fn, resp_q))
        try:
            return resp_q.get(timeout=timeout)
        except queue.Empty:
            return 504, {"ok": False, "error": "Quá thời gian chờ luồng mô phỏng xử lý lệnh."}

    def process_pending_commands(self) -> None:
        while True:
            try:
                fn, resp_q = self._cmd_queue.get_nowait()
            except queue.Empty:
                break
            try:
                result = fn(self.sim)
                resp_q.put((200, {"ok": True, "data": result}))
            except (ValueError, TypeError, KeyError) as exc:
                resp_q.put((400, {"ok": False, "error": str(exc)}))
            except Exception as exc:  # pragma: no cover - defensive guard keeps main loop alive
                resp_q.put((500, {"ok": False, "error": f"Lỗi nội bộ: {exc}"}))

    def tick_headless(self, step_batch: int = 10) -> None:
        self.process_pending_commands()
        for _ in range(step_batch):
            self.sim.step_once()

    def start_http_server(self, host: str = "127.0.0.1", port: int = 8765) -> tuple[str, int]:
        app = self

        class RequestHandler(BaseHTTPRequestHandler):
            def log_message(self, format: str, *args: Any) -> None:
                return  # Giữ terminal gọn gàng khi gọi liên tục

            def _send_json(self, status_code: int, payload: dict) -> None:
                raw = json.dumps(payload, ensure_ascii=False).encode("utf-8")
                self.send_response(status_code)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(raw)))
                self.end_headers()
                self.wfile.write(raw)

            def _read_json_body(self) -> dict:
                length = int(self.headers.get("Content-Length", "0"))
                if length <= 0:
                    return {}
                if length > 65536:
                    raise ValueError("Kích thước gói tin vượt quá giới hạn cho phép.")
                raw = self.rfile.read(length).decode("utf-8")
                if not raw.strip():
                    return {}
                parsed = json.loads(raw)
                if not isinstance(parsed, dict):
                    raise ValueError("Nội dung JSON phải là một đối tượng (object).")
                return parsed

            def do_GET(self) -> None:
                parsed_url = urlsplit(self.path)
                query = parse_qs(parsed_url.query)
                if parsed_url.path == "/status":
                    inc_vis = query.get("vision", ["0"])[0] in ("1", "true", "yes")
                    code, payload = app.submit_command(lambda sim: sim.get_status(include_vision=inc_vis))
                    self._send_json(code, payload)
                elif parsed_url.path == "/vision":
                    cam_name = query.get("camera", ["front_cam"])[0]
                    code, payload = app.submit_command(lambda sim: sim.analyze_vision(camera_name=cam_name))
                    self._send_json(code, payload)
                else:
                    self._send_json(404, {"ok": False, "error": f"Không hỗ trợ đường dẫn GET '{self.path}'."})

            def do_POST(self) -> None:
                try:
                    body = self._read_json_body()
                except (ValueError, json.JSONDecodeError) as exc:
                    self._send_json(400, {"ok": False, "error": f"JSON không hợp lệ: {exc}"})
                    return

                path = self.path
                if path == "/wheels":
                    if "left" not in body or "right" not in body:
                        self._send_json(400, {"ok": False, "error": "Cần truyền đủ 'left' và 'right' (rad/s)."})
                        return
                    left = body["left"]
                    right = body["right"]
                    duration = body.get("duration", 1.0)
                    code, payload = app.submit_command(
                        lambda sim: sim.set_wheels(left, right, duration=duration)
                    )
                elif path == "/move":
                    if "meters" not in body:
                        self._send_json(400, {"ok": False, "error": "Cần truyền tham số 'meters'."})
                        return
                    meters = body["meters"]
                    speed = body.get("speed", 0.25)
                    code, payload = app.submit_command(
                        lambda sim: sim.move(meters=meters, speed_m_s=speed)
                    )
                elif path == "/turn":
                    if "degrees" not in body:
                        self._send_json(400, {"ok": False, "error": "Cần truyền tham số 'degrees'."})
                        return
                    degrees = body["degrees"]
                    angular_speed = body.get("angular_speed", 60.0)
                    code, payload = app.submit_command(
                        lambda sim: sim.turn(degrees=degrees, angular_deg_s=angular_speed)
                    )
                elif path == "/stop":
                    code, payload = app.submit_command(lambda sim: sim.stop())
                elif path == "/reset":
                    target_xy = body.get("target_xy")
                    code, payload = app.submit_command(lambda sim: sim.reset(target_xy=target_xy))
                elif path == "/reload":
                    preserve_target = body.get("preserve_target", True)
                    target_xy = body.get("target_xy")

                    def _do_reload(sim: RobotSimulation) -> dict:
                        if target_xy is not None:
                            sim.validate_target_xy(target_xy)
                        st = sim.reload(preserve_target=preserve_target)
                        app._viewer_reload_pending = True
                        if target_xy is not None:
                            st = sim.reset(target_xy=target_xy)
                        return st


                    code, payload = app.submit_command(_do_reload)
                elif path == "/camera":
                    camera = body.get("camera", "front_cam")
                    out_path = body.get("path")
                    width = body.get("width", DEFAULT_CAMERA_WIDTH)
                    height = body.get("height", DEFAULT_CAMERA_HEIGHT)
                    img_format = body.get("format")
                    quality = body.get("quality", DEFAULT_JPEG_QUALITY)
                    code, payload = app.submit_command(
                        lambda sim: sim.capture_camera(
                            camera_name=camera,
                            out_path=out_path,
                            width=width,
                            height=height,
                            img_format=img_format,
                            quality=quality,
                        )
                    )
                else:
                    self._send_json(404, {"ok": False, "error": f"Không hỗ trợ đường dẫn POST '{path}'."})
                    return

                self._send_json(code, payload)

        self._http_server = ThreadingHTTPServer((host, port), RequestHandler)
        bound_host, bound_port = self._http_server.server_address[:2]
        self._http_thread = threading.Thread(
            target=self._http_server.serve_forever,
            name="SimHTTPThread",
            daemon=True,
        )
        self._http_thread.start()
        return str(bound_host), int(bound_port)

    def run_loop(self, headless: bool = False, duration: float | None = None) -> None:
        dt = float(self.sim.model.opt.timestep)
        frame_dt = 8 * dt
        start_wall = time.perf_counter()
        last_wall = start_wall

        if headless:
            while not self._closed:
                now = time.perf_counter()
                if duration is not None and (now - start_wall) >= duration:
                    break
                elapsed = min(max(0.0, now - last_wall), 0.10)
                last_wall = now
                steps = max(1, int(round(elapsed / dt))) if elapsed > 0 else 8
                t0 = time.perf_counter()
                self.tick_headless(step_batch=steps)
                sleep_s = frame_dt - (time.perf_counter() - t0)
                if sleep_s > 0:
                    time.sleep(sleep_s)
            return

        import mujoco.viewer

        with mujoco.viewer.launch_passive(self.sim.model, self.sim.data) as viewer:
            viewer.cam.lookat[:] = [0.25, 0.0, 0.08]
            viewer.cam.distance = 3.1
            viewer.cam.elevation = -32.0
            viewer.cam.azimuth = 135.0
            last_wall = time.perf_counter()

            while viewer.is_running() and not self._closed:
                now = time.perf_counter()
                if duration is not None and (now - start_wall) >= duration:
                    break
                elapsed = min(max(0.0, now - last_wall), 0.10)
                last_wall = now
                steps = max(1, int(round(elapsed / dt))) if elapsed > 0 else 8
                t0 = time.perf_counter()
                with viewer.lock():
                    self.process_pending_commands()
                    for _ in range(steps):
                        self.sim.step_once()
                if self._viewer_reload_pending:
                    self._viewer_reload_pending = False
                    raw_sim = viewer._get_sim()
                    if raw_sim is not None:
                        raw_sim.load(self.sim.model, self.sim.data, str(self.sim.model_path))
                viewer.sync()
                sleep_s = frame_dt - (time.perf_counter() - t0)
                if sleep_s > 0:
                    time.sleep(sleep_s)

    def shutdown(self) -> None:
        self._closed = True
        while True:
            try:
                _, resp_q = self._cmd_queue.get_nowait()
                resp_q.put((503, {"ok": False, "error": "Máy chủ mô phỏng đang tắt."}))
            except queue.Empty:
                break
        if self._http_server is not None:
            self._http_server.shutdown()
            self._http_server.server_close()
            self._http_server = None
        if self._http_thread is not None:
            self._http_thread.join(timeout=2.0)
            self._http_thread = None
        self.sim.close()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Khởi chạy mô phỏng xe robot 2 bánh MuJoCo kèm cổng điều khiển HTTP cục bộ."
    )
    parser.add_argument("--host", default="127.0.0.1", help="Địa chỉ lắng nghe HTTP (mặc định: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=8765, help="Cổng HTTP (mặc định: 8765)")
    parser.add_argument("--headless", action="store_true", help="Chạy không mở cửa sổ 3D (dùng khi kiểm thử tự động)")
    parser.add_argument("--duration", type=float, default=None, help="Tự động thoát sau N giây (tùy chọn)")
    args = parser.parse_args(argv)

    app = SimulationApp()
    try:
        host, port = app.start_http_server(host=args.host, port=args.port)
        mode_str = "Headless (không giao diện)" if args.headless else "Cửa sổ 3D trực tiếp"
        print(f"=== Mô phỏng Robot 2 Bánh MuJoCo đã sẵn sàng ({mode_str}) ===", flush=True)
        print(f"Cổng điều khiển HTTP: http://{host}:{port}", flush=True)
        print("Thử điều khiển ở cửa sổ terminal khác bằng:", flush=True)
        print("  uv run python robot_ctl.py status", flush=True)
        print("  uv run python robot_ctl.py move 0.5 --wait", flush=True)
        print("  uv run python robot_ctl.py camera", flush=True)
        app.run_loop(headless=args.headless, duration=args.duration)
    except KeyboardInterrupt:
        print("\nĐang dừng mô phỏng...", flush=True)
    finally:
        app.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
