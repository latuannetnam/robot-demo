from __future__ import annotations

import argparse
import http.client
import json
from pathlib import Path
import sys
import threading
import time
from urllib.parse import urlsplit

from robot_sim.skills import (
    advance_segment,
    approach_target,
    closed_loop_move,
    closed_loop_turn,
    compute_open_branches,
    cross_dynamic_gate,
    dock_target,
    explore_maze,
    go_to_decision,
    scan_for_target,
    wait_until_settled,
)
from robot_sim.telemetry import (
    DEFAULT_STOP_FLAG_PATH,
    DEFAULT_TELEMETRY_PATH,
    record_telemetry,
    stop_telemetry,
)
from robot_sim.topo import (
    DEFAULT_TOPO_STATE_PATH,
    TopologicalOdometer,
    branches_to_cardinals,
    is_cardinal_aligned,
)

DEFAULT_URL = "http://127.0.0.1:8765"


class BridgeOfflineError(RuntimeError):
    pass


class BridgeRequestError(RuntimeError):
    pass


_BRIDGE_CONNS = threading.local()


def _get_bridge_pool() -> dict[tuple[str, str, int], http.client.HTTPConnection]:
    pool = getattr(_BRIDGE_CONNS, "pool", None)
    if pool is None:
        pool = {}
        _BRIDGE_CONNS.pool = pool
    return pool


def _call_bridge(base_url: str, path: str, method: str = "GET", payload: dict | None = None) -> dict:
    if not isinstance(base_url, str) or not base_url.startswith(("http://", "https://")):
        raise BridgeRequestError(
            f"Địa chỉ --url không hợp lệ ('{base_url}'). Cần bắt đầu bằng http:// hoặc https:// (ví dụ: {DEFAULT_URL})."
        )
    parsed_url = urlsplit(base_url)
    scheme = parsed_url.scheme.lower()
    host = parsed_url.hostname or "127.0.0.1"
    port = parsed_url.port or (443 if scheme == "https" else 80)
    base_path = parsed_url.path.rstrip("/")
    req_path = f"{base_path}{path}"
    raw_body = None if payload is None else json.dumps(payload).encode("utf-8")
    headers = {"Content-Type": "application/json", "Connection": "keep-alive"}
    pool = _get_bridge_pool()
    key = (scheme, host, port)

    for attempt in range(2):
        conn = pool.get(key)
        was_reused = conn is not None
        if conn is None:
            cls = http.client.HTTPSConnection if scheme == "https" else http.client.HTTPConnection
            conn = cls(host, port, timeout=6.0)
            pool[key] = conn
        try:
            conn.request(method, req_path, body=raw_body, headers=headers)
            resp = conn.getresponse()
            resp_bytes = resp.read()
            if resp.will_close:
                conn.close()
                pool.pop(key, None)
            if resp.status >= 400:
                try:
                    err_json = json.loads(resp_bytes.decode("utf-8"))
                    msg = err_json.get("error", f"HTTP {resp.status} {resp.reason}")
                except Exception:
                    msg = f"HTTP {resp.status} {resp.reason}"
                raise BridgeRequestError(msg)
            parsed = json.loads(resp_bytes.decode("utf-8"))
            return parsed.get("data", parsed)
        except BridgeRequestError:
            raise
        except (http.client.HTTPException, TimeoutError, ConnectionError, OSError) as exc:
            pool.pop(key, None)
            try:
                conn.close()
            except Exception:
                pass
            if was_reused and attempt == 0:
                continue
            raise BridgeOfflineError(
                f"Không kết nối được hoặc mất kết nối tới mô phỏng tại {base_url}.\n"
                "Hãy mở một cửa sổ terminal khác và khởi chạy cửa sổ 3D MuJoCo trước bằng lệnh:\n"
                "  uv run python sim_server.py"
            ) from exc
    raise BridgeOfflineError(f"Không kết nối được tới mô phỏng tại {base_url}.")


def _strip_debug(status_data: dict, keep_debug: bool) -> dict:
    if keep_debug:
        return status_data
    return {k: v for k, v in status_data.items() if k != "debug"}


def _wait_for_completion(base_url: str, duration_s: float, include_vision: bool = False) -> dict:
    """Chờ cho tới khi mô phỏng hoàn tất lệnh chuyển động và thân xe tắt hẳn dao động."""
    call_fn = lambda path, method="GET", payload=None: _call_bridge(base_url, path, method, payload)
    return wait_until_settled(call_fn, duration_s=duration_s, include_vision=include_vision)


def _run_reactive_demo(base_url: str, steps: int, step_dt: float) -> dict:
    """Kịch bản tự hành tốc độ cao dùng cảm biến khoảng cách + Edge Vision trong RAM (không ghi ảnh ra đĩa)."""
    history: list[str] = []
    reached = False

    for step_idx in range(steps):
        st = _call_bridge(base_url, "/status?vision=1", "GET")
        rf = st["sensors"]["rangefinders_m"]
        vis = st["vision"]
        seen = bool(vis["target_visible"])
        offset_x = float(vis["target"]["offset_x"]) if seen else 0.0
        area_pct = float(vis["target"]["area_pct"]) if seen else 0.0

        if seen and abs(offset_x) < 0.25 and rf["center"] < 0.30:
            _call_bridge(base_url, "/stop", "POST", {})
            history.append(f"Bước {step_idx + 1}: Đã tiếp cận mục tiêu màu xanh (cách {rf['center']:.2f}m)!")
            reached = True
            break

        target_ahead = seen and abs(offset_x) < 0.25 and area_pct > 1.0
        if not target_ahead and (rf["center"] < 0.45 or rf["left"] < 0.35 or rf["right"] < 0.35):
            if rf["left"] >= rf["right"]:
                _call_bridge(base_url, "/wheels", "POST", {"left": -4.5, "right": 4.5, "duration": step_dt})
                action = "né vật cản -> quay trái"
            else:
                _call_bridge(base_url, "/wheels", "POST", {"left": 4.5, "right": -4.5, "duration": step_dt})
                action = "né vật cản -> quay phải"
        elif seen:
            steer = 3.2 * offset_x
            left_w = 5.5 + steer
            right_w = 5.5 - steer
            _call_bridge(base_url, "/wheels", "POST", {"left": left_w, "right": right_w, "duration": step_dt})
            action = f"bám mục tiêu xanh (lệch ngang={offset_x:+.2f})"
        else:
            if rf["center"] > 0.9:
                _call_bridge(base_url, "/wheels", "POST", {"left": 2.5, "right": 5.5, "duration": step_dt})
                action = "quét vòng tìm mục tiêu"
            else:
                _call_bridge(base_url, "/wheels", "POST", {"left": -4.0, "right": 4.0, "duration": step_dt})
                action = "xoay tại chỗ tìm mục tiêu"

        history.append(f"Bước {step_idx + 1}: {action}")
        time.sleep(step_dt)

    _call_bridge(base_url, "/stop", "POST", {})
    final_status = _strip_debug(_call_bridge(base_url, "/status?vision=1", "GET"), keep_debug=False)
    return {"reached_target": reached, "steps_run": len(history), "log": history[-5:], "status": final_status}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="robot_ctl.py",
        allow_abbrev=False,
        description="Công cụ điều khiển và đọc cảm biến xe robot 2 bánh MuJoCo (cho người dùng & Antigravity).",
    )
    parser.add_argument("--url", default=DEFAULT_URL, help=f"Địa chỉ HTTP của sim_server (mặc định: {DEFAULT_URL})")
    sub = parser.add_subparsers(dest="command", required=True)

    p_status = sub.add_parser("status", help="Đọc dữ liệu cảm biến trên xe")
    p_status.add_argument(
        "--vision",
        action="store_true",
        help="Đính kèm kết quả phân tích thị giác tại biên (Edge Vision JSON) từ camera mũi xe",
    )
    p_status.add_argument(
        "--debug",
        action="store_true",
        help="Hiển thị thêm tọa độ thực trong sân (chỉ dùng kiểm tra, không dùng khi đóng vai bộ não robot)",
    )

    p_vision = sub.add_parser("vision", help="Phân tích nhanh vật thể trước mặt qua camera trong RAM (trả JSON nhẹ)")
    p_vision.add_argument("--name", default="front_cam", choices=["front_cam", "overview_cam"])

    p_cam = sub.add_parser("camera", help="Chụp ảnh từ camera mũi xe hoặc toàn cảnh (mặc định JPEG 320x240 siêu nhẹ)")
    p_cam.add_argument("--name", default="front_cam", choices=["front_cam", "overview_cam"])
    p_cam.add_argument("--out", default=None, help="Đường dẫn file ảnh đầu ra (tùy chọn)")
    p_cam.add_argument("--format", default=None, choices=["jpg", "jpeg", "png"], help="Định dạng ảnh (mặc định: jpg)")
    p_cam.add_argument("--width", type=int, default=320, help="Chiều rộng ảnh (64..640, mặc định: 320)")
    p_cam.add_argument("--height", type=int, default=240, help="Chiều cao ảnh (48..480, mặc định: 240)")
    p_cam.add_argument("--quality", type=int, default=75, help="Chất lượng nén JPEG (10..95, mặc định: 75)")

    p_wheels = sub.add_parser("wheels", help="Đặt tốc độ góc 2 bánh trái/phải (rad/s)")
    p_wheels.add_argument("left", type=float, help="Vận tốc góc bánh trái (rad/s, -15..15)")
    p_wheels.add_argument("right", type=float, help="Vận tốc góc bánh phải (rad/s, -15..15)")
    p_wheels.add_argument("--duration", type=float, default=1.0, help="Thời gian duy trì lệnh (giây)")
    p_wheels.add_argument("--wait", action="store_true", help="Chờ xe thực hiện xong lệnh rồi trả trạng thái cảm biến")
    p_wheels.add_argument("--vision", action="store_true", help="Kèm tóm tắt Edge Vision trong after_status khi dùng --wait")

    p_move = sub.add_parser("move", help="Tiến (+) hoặc lùi (-) một quãng đường ước lượng (mét)")
    p_move.add_argument("meters", type=float, nargs="?", default=None, help="Quãng đường cần đi (m, âm để lùi)")
    p_move.add_argument("--meters", dest="meters_opt", type=float, default=None, help="Quãng đường cần đi (m)")
    p_move.add_argument("--speed", type=float, default=0.25, help="Tốc độ dài ước lượng (m/s, mặc định: 0.25)")
    p_move.add_argument("--wait", action="store_true", help="Chờ xe chạy xong (mặc định giữ thẳng hướng bằng Gyro khép kín)")
    p_move.add_argument("--vision", action="store_true", help="Kèm tóm tắt Edge Vision trong after_status khi dùng --wait")
    p_move.add_argument("--open-loop", action="store_true", help="Di chuyển vòng hở theo thời gian cố định thay vì Gyro")

    p_turn = sub.add_parser("turn", help="Xoay tại chỗ một góc (độ: + quay trái, - quay phải)")
    p_turn.add_argument("degrees", type=float, nargs="?", default=None, help="Góc xoay (độ, + trái / - phải)")
    p_turn.add_argument("--degrees", dest="degrees_opt", type=float, default=None, help="Góc xoay (độ)")
    p_turn.add_argument("--speed", type=float, default=60.0, help="Tốc độ xoay (độ/s, mặc định: 60)")
    p_turn.add_argument("--wait", action="store_true", help="Chờ xe xoay xong (mặc định dùng hồi tiếp Gyro khép kín)")
    p_turn.add_argument("--vision", action="store_true", help="Kèm tóm tắt Edge Vision trong after_status khi dùng --wait")
    p_turn.add_argument("--open-loop", action="store_true", help="Xoay vòng hở theo thời gian cố định thay vì Gyro")

    p_scan = sub.add_parser("scan", help="Quét vòng tròn 20Hz tìm mục tiêu xanh lá và tự căn tâm (chuẩn Sim-to-Real)")
    p_scan.add_argument("--max-degrees", type=float, default=360.0, help="Góc quét tối đa (0..720, mặc định: 360)")
    p_scan.add_argument("--direction", default="left", choices=["left", "right"], help="Chiều quét (left hoặc right)")
    p_scan.add_argument("--speed", type=float, default=75.0, help="Tốc độ góc khi quét (độ/s, mặc định: 75)")
    p_scan.add_argument(
        "--recover-occlusion",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Tự động tiến 0.85m vào hành lang thoáng tạo thị sai và quét lại nếu 360° đầu bị khuất (mặc định: bật)",
    )

    p_approach = sub.add_parser("approach", help="Tiếp cận mục tiêu xanh khép kín 20Hz kết hợp né hành lang vật cản")
    p_approach.add_argument("--stop-distance", type=float, default=0.25, help="Khoảng cách dừng trước đích (0.12..1.5m)")
    p_approach.add_argument("--max-steps", type=int, default=160, help="Số nhịp điều khiển tối đa (mặc định: 160)")
    p_approach.add_argument("--step-dt", type=float, default=0.15, help="Chu kỳ mỗi nhịp điều khiển (giây)")

    p_explore = sub.add_parser(
        "explore",
        help="Tự hành dò mê cung khép kín theo quy tắc bám tường kèm Odometry 2D tự phát hiện và thoát đảo vòng lặp (chuẩn Sim-to-Real)",
    )
    p_explore.add_argument("--rule", default="right", choices=["right", "left"], help="Quy tắc bám tường (right hoặc left)")
    p_explore.add_argument("--max-steps", type=int, default=850, help="Số nhịp điều khiển tối đa (mặc định: 850)")
    p_explore.add_argument("--speed", type=float, default=0.64, help="Tốc độ di chuyển hành lang (m/s, mặc định: 0.64)")
    p_explore.add_argument("--step-dt", type=float, default=0.10, help="Chu kỳ mỗi nhịp điều khiển (giây)")

    p_advance = sub.add_parser(
        "advance",
        help="Tiến thẳng vô trạng thái 1 đoạn hành lang/sân mở và dừng ngay khi gặp ngã rẽ, vật cản hoặc gần đích (agent-brain)",
    )
    p_advance.add_argument("--max-dist", type=float, default=1.8, help="Quãng đường tối đa một lần gọi (mét, mặc định: 1.8)")
    p_advance.add_argument("--speed", type=float, default=0.55, help="Tốc độ tiến thẳng (m/s, mặc định: 0.55)")
    p_advance.add_argument("--step-dt", type=float, default=0.10, help="Chu kỳ mỗi nhịp điều khiển (giây)")

    p_cross_gate = sub.add_parser(
        "cross-gate",
        help="Chờ một cổng/làn tuần tra động mở khe an toàn rồi băng qua đúng 1 nhịp (agent-brain)",
    )
    p_cross_gate.add_argument("--timeout", type=float, default=12.0, help="Thời gian chờ mở cổng tối đa theo sim_time (giây)")

    p_dock = sub.add_parser(
        "dock",
        help="Căn thẳng tâm và tiến sát dừng êm trước mục tiêu ở cự ly gần (agent-brain)",
    )
    p_dock.add_argument("--stop-distance", type=float, default=0.25, help="Khoảng cách dừng trước mục tiêu (mét, mặc định: 0.25)")
    p_dock.add_argument("--max-steps", type=int, default=80, help="Số nhịp tối đa (mặc định: 80)")
    p_dock.add_argument("--step-dt", type=float, default=0.12, help="Chu kỳ mỗi nhịp điều khiển (giây)")

    p_go = sub.add_parser(
        "go",
        help="Kỹ năng gộp System 1 (Mode C hybrid-brain): xoay khóa trục, tiến giữ tâm, lướt góc chữ L 1 lối đi và đồng bộ đồ thị Topo",
    )
    go_dir = p_go.add_mutually_exclusive_group()
    go_dir.add_argument(
        "--cardinal",
        type=lambda s: str(s).strip().upper(),
        choices=["E", "N", "W", "S"],
        default=None,
        help="Hướng hệ quy chiếu khởi hành cần khóa trước khi chạy (E=0°, N=+90°, W=180°, S=-90°)",
    )
    go_dir.add_argument(
        "--heading-deg",
        type=float,
        default=None,
        help="Góc hướng mục tiêu tùy chọn theo độ trong hệ quy chiếu khởi hành",
    )
    go_dir.add_argument(
        "--route",
        type=str,
        default=None,
        help="Chuỗi hướng chính do System 2 lập kế hoạch, phân tách bằng dấu phẩy (ví dụ: W,S,N,N)",
    )
    p_go.add_argument(
        "--stop-at-node",
        type=int,
        default=None,
        help="ID nút đích trên đồ thị Topo cần căn dừng êm ở cuối chặng (ví dụ: 0 cho Return-to-Home)",
    )
    p_go.add_argument("--max-dist", type=float, default=6.0, help="Tổng quãng đường tối đa của một lệnh go (mét, mặc định: 6.0)")
    p_go.add_argument("--speed", type=float, default=0.55, help="Tốc độ tiến hành lang (m/s, mặc định: 0.55)")
    p_go.add_argument("--step-dt", type=float, default=0.10, help="Chu kỳ điều khiển mỗi nhịp (giây, mặc định: 0.10)")
    p_go.add_argument("--max-chain", type=int, default=6, help="Số đoạn tối đa nối liên tiếp qua góc chữ L (mặc định: 6)")
    p_go.add_argument("--no-chain", action="store_true", help="Tắt tự động lướt góc chữ L (chỉ chạy đúng 1 chặng)")

    p_topo = sub.add_parser(
        "topo",
        help="Đọc hoặc xóa sổ cái định vị đồ thị Odometry khách quan (Mode C hybrid-brain)",
    )
    topo_sub = p_topo.add_subparsers(dest="topo_action", required=True)
    topo_sub.add_parser("show", help="Đọc cảm biến hiện tại, cập nhật nút đứng và in bản đồ Topo JSON")
    topo_sub.add_parser("reset", help="Xóa phiên đồ thị Topo của địa chỉ --url hiện tại")

    sub.add_parser("stop", help="Dừng khẩn cấp hai động cơ bánh xe")

    p_reset = sub.add_parser("reset", help="Đưa robot và mục tiêu về trạng thái ban đầu")
    p_reset.add_argument("--target-x", type=float, default=None, help="Tọa độ X mới của mục tiêu (-1.7..1.7)")
    p_reset.add_argument("--target-y", type=float, default=None, help="Tọa độ Y mới của mục tiêu (-1.7..1.7)")
    p_reset.add_argument("--vision", action="store_true", help="Kèm tóm tắt Edge Vision ngay sau khi reset")
    p_reset.add_argument(
        "--keep-topo",
        action="store_true",
        help="Giữ lại cấu trúc đồ thị Topo đã học cho Lượt 2 (chỉ xóa bộ đếm visits về 0)",
    )

    p_arenas = sub.add_parser("arenas", help="Liệt kê danh sách các bản đồ địa hình chuẩn có sẵn")
    p_arenas.add_argument(
        "--debug",
        action="store_true",
        help="Hiển thị thêm địa hình đang nạp, điểm xuất phát và tọa độ đích mặc định (chỉ dành cho Main Agent / Engineer)",
    )

    p_reload = sub.add_parser(
        "reload",
        help="Kiểm định chống gian lận và nạp nóng lại mô hình hoặc chuyển đổi bản đồ địa hình",
    )
    reload_group = p_reload.add_mutually_exclusive_group()
    reload_group.add_argument(
        "--arena",
        default=None,
        help="Chuyển sang địa hình chuẩn (2wheel, maze_a / maze, maze_b, maze_c, dynamic_a / dynamic, dynamic_b)",
    )
    reload_group.add_argument(
        "--model",
        default=None,
        help="Đường dẫn file XML mô hình trong thư mục models/ (loại trừ --arena)",
    )
    p_reload.add_argument("--no-preserve-target", action="store_true", help="Đặt lại vị trí mục tiêu mặc định khi reload")
    p_reload.add_argument("--target-x", type=float, default=None, help="Tọa độ X mới của mục tiêu (-1.7..1.7)")
    p_reload.add_argument("--target-y", type=float, default=None, help="Tọa độ Y mới của mục tiêu (-1.7..1.7)")
    p_reload.add_argument("--vision", action="store_true", help="Kèm tóm tắt Edge Vision ngay sau khi reload")

    p_demo = sub.add_parser("demo", help="Chạy thử vòng lặp tự hành cổ điển (dùng Edge Vision + cảm biến khoảng cách)")
    p_demo.add_argument("--steps", type=int, default=25, help="Số nhịp điều khiển tối đa (mặc định: 25)")
    p_demo.add_argument("--step-dt", type=float, default=0.20, help="Chu kỳ mỗi nhịp điều khiển (giây)")

    p_tel = sub.add_parser("telemetry", help="Quản lý Hộp đen giám sát nền 10Hz (record / stop)")
    tel_sub = p_tel.add_subparsers(dest="telemetry_action", required=True)

    p_tel_rec = tel_sub.add_parser("record", help="Chạy nền thu thập telemetry 10Hz ra file JSON tóm tắt")
    p_tel_rec.add_argument("--out", default=str(DEFAULT_TELEMETRY_PATH), help="Đường dẫn file JSON báo cáo")
    p_tel_rec.add_argument("--stop-file", default=str(DEFAULT_STOP_FLAG_PATH), help="Đường dẫn file cờ dừng")
    p_tel_rec.add_argument("--hz", type=float, default=10.0, help="Tần số lấy mẫu Hz (mặc định: 10)")
    p_tel_rec.add_argument("--max-duration", type=float, default=300.0, help="Thời gian chạy tối đa (giây)")

    p_tel_stop = tel_sub.add_parser("stop", help="Gửi cờ dừng Hộp đen và in ngay báo cáo JSON")
    p_tel_stop.add_argument("--out", default=str(DEFAULT_TELEMETRY_PATH), help="Đường dẫn file JSON báo cáo")
    p_tel_stop.add_argument("--stop-file", default=str(DEFAULT_STOP_FLAG_PATH), help="Đường dẫn file cờ dừng")
    p_tel_stop.add_argument("--wait-timeout", type=float, default=3.0, help="Thời gian chờ Hộp đen ghi file (giây)")

    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    try:
        if args.command == "status":
            status_path = "/status?vision=1" if args.vision else "/status"
            res = _call_bridge(args.url, status_path, "GET")
            output = _strip_debug(res, keep_debug=args.debug)
        elif args.command == "vision":
            output = _call_bridge(args.url, f"/vision?camera={args.name}", "GET")
        elif args.command == "camera":
            out_path = str(Path(args.out).resolve()) if args.out else None
            output = _call_bridge(
                args.url,
                "/camera",
                "POST",
                {
                    "camera": args.name,
                    "path": out_path,
                    "format": args.format,
                    "width": args.width,
                    "height": args.height,
                    "quality": args.quality,
                },
            )
        elif args.command == "wheels":
            output = _call_bridge(
                args.url,
                "/wheels",
                "POST",
                {"left": args.left, "right": args.right, "duration": args.duration},
            )
            if args.wait:
                output["after_status"] = _wait_for_completion(
                    args.url, output["duration_s"], include_vision=args.vision
                )
            if TopologicalOdometer.has_active_session(args.url, state_path=DEFAULT_TOPO_STATE_PATH):
                TopologicalOdometer(args.url, state_path=DEFAULT_TOPO_STATE_PATH).cancel_pending_leg(
                    output.get("after_status")
                )
        elif args.command == "move":
            meters = args.meters_opt if args.meters_opt is not None else args.meters
            if meters is None:
                raise BridgeRequestError("Vui lòng truyền quãng đường (ví dụ: move 0.5 hoặc move -0.3).")
            if args.wait and not args.open_loop:
                call_fn = lambda path, method="GET", payload=None: _call_bridge(args.url, path, method, payload)
                output = closed_loop_move(
                    call_fn,
                    meters=meters,
                    speed_m_s=args.speed,
                    include_vision=args.vision,
                )
            else:
                output = _call_bridge(args.url, "/move", "POST", {"meters": meters, "speed": args.speed})
                if args.wait:
                    output["after_status"] = _wait_for_completion(
                        args.url, output["duration_s"], include_vision=args.vision
                    )
            if TopologicalOdometer.has_active_session(args.url, state_path=DEFAULT_TOPO_STATE_PATH):
                TopologicalOdometer(args.url, state_path=DEFAULT_TOPO_STATE_PATH).cancel_pending_leg(
                    output.get("after_status")
                )
        elif args.command == "turn":
            degrees = args.degrees_opt if args.degrees_opt is not None else args.degrees
            if degrees is None:
                raise BridgeRequestError("Vui lòng truyền góc xoay (ví dụ: turn 45 hoặc turn -45).")
            if args.wait and not args.open_loop:
                call_fn = lambda path, method="GET", payload=None: _call_bridge(args.url, path, method, payload)
                output = closed_loop_turn(
                    call_fn,
                    degrees=degrees,
                    max_speed_deg_s=args.speed,
                    include_vision=args.vision,
                )
            else:
                output = _call_bridge(
                    args.url,
                    "/turn",
                    "POST",
                    {"degrees": degrees, "angular_speed": args.speed},
                )
                if args.wait:
                    output["after_status"] = _wait_for_completion(
                        args.url, output["duration_s"], include_vision=args.vision
                    )
            if (
                args.wait
                and "after_status" in output
                and TopologicalOdometer.has_active_session(args.url, state_path=DEFAULT_TOPO_STATE_PATH)
            ):
                topo_inst = TopologicalOdometer(args.url, state_path=DEFAULT_TOPO_STATE_PATH)
                after_st = output["after_status"]
                yaw_after = float(after_st["sensors"]["gyro_yaw_deg"])
                if topo_inst.current_node is None and not is_cardinal_aligned(yaw_after, tol_deg=20.0):
                    topo_inst.cancel_pending_leg(after_st)
                else:
                    topo_inst.observe_status(after_st)
        elif args.command == "scan":
            call_fn = lambda path, method="GET", payload=None: _call_bridge(args.url, path, method, payload)
            output = scan_for_target(
                call_fn,
                max_degrees=args.max_degrees,
                direction=args.direction,
                angular_speed_deg_s=args.speed,
                recover_occlusion=args.recover_occlusion,
            )
            if (
                "after_status" in output
                and TopologicalOdometer.has_active_session(args.url, state_path=DEFAULT_TOPO_STATE_PATH)
            ):
                TopologicalOdometer(args.url, state_path=DEFAULT_TOPO_STATE_PATH).observe_status(
                    output["after_status"]
                )
        elif args.command == "approach":
            call_fn = lambda path, method="GET", payload=None: _call_bridge(args.url, path, method, payload)
            output = approach_target(
                call_fn,
                stop_distance=args.stop_distance,
                max_steps=args.max_steps,
                step_dt=args.step_dt,
            )
        elif args.command == "explore":
            call_fn = lambda path, method="GET", payload=None: _call_bridge(args.url, path, method, payload)
            output = explore_maze(
                call_fn,
                rule=args.rule,
                max_steps=args.max_steps,
                speed_m_s=args.speed,
                step_dt=args.step_dt,
            )
        elif args.command == "advance":
            call_fn = lambda path, method="GET", payload=None: _call_bridge(args.url, path, method, payload)
            output = advance_segment(
                call_fn,
                max_dist=args.max_dist,
                speed_m_s=args.speed,
                step_dt=args.step_dt,
            )
        elif args.command == "cross-gate":
            call_fn = lambda path, method="GET", payload=None: _call_bridge(args.url, path, method, payload)
            output = cross_dynamic_gate(
                call_fn,
                timeout_s=args.timeout,
            )
            yaw_after = float(output["after_status"]["sensors"]["gyro_yaw_deg"])
            output["open_cardinals"] = branches_to_cardinals(output["open_branches"], yaw_after)
            if TopologicalOdometer.has_active_session(args.url, state_path=DEFAULT_TOPO_STATE_PATH):
                topo_inst = TopologicalOdometer(args.url, state_path=DEFAULT_TOPO_STATE_PATH)
                output["topo"] = topo_inst.record_progress(output["moved_m"], output["after_status"])
        elif args.command == "dock":
            call_fn = lambda path, method="GET", payload=None: _call_bridge(args.url, path, method, payload)
            output = dock_target(
                call_fn,
                stop_distance=args.stop_distance,
                max_steps=args.max_steps,
                step_dt=args.step_dt,
            )
            if (
                "after_status" in output
                and TopologicalOdometer.has_active_session(args.url, state_path=DEFAULT_TOPO_STATE_PATH)
            ):
                topo_inst = TopologicalOdometer(args.url, state_path=DEFAULT_TOPO_STATE_PATH)
                output["topo"] = topo_inst.record_dock(
                    output.get("moved_m", 0.0),
                    output["after_status"],
                    reached=bool(output.get("reached", False)),
                )
        elif args.command == "go":
            call_fn = lambda path, method="GET", payload=None: _call_bridge(args.url, path, method, payload)
            topo_inst = TopologicalOdometer(args.url, state_path=DEFAULT_TOPO_STATE_PATH)
            output = go_to_decision(
                call_fn,
                cardinal=args.cardinal,
                heading_deg=args.heading_deg,
                max_dist=args.max_dist,
                speed_m_s=args.speed,
                step_dt=args.step_dt,
                max_chain=args.max_chain,
                auto_chain=not args.no_chain,
                topo=topo_inst,
                route=args.route,
                stop_at_node=args.stop_at_node,
            )
        elif args.command == "topo":
            if args.topo_action == "reset":
                TopologicalOdometer.reset_session(args.url, state_path=DEFAULT_TOPO_STATE_PATH)
                output = {"reset": True}
            else:
                res = _call_bridge(args.url, "/status?vision=1", "GET")
                clean_st = _strip_debug(res, keep_debug=False)
                topo_inst = TopologicalOdometer(args.url, state_path=DEFAULT_TOPO_STATE_PATH)
                topo_snap = topo_inst.observe_status(clean_st)
                rf = clean_st["sensors"]["rangefinders_m"]
                yaw_deg = float(clean_st["sensors"]["gyro_yaw_deg"])
                open_branches = compute_open_branches(rf)
                output = {
                    "topo": topo_snap,
                    "open_branches": open_branches,
                    "open_cardinals": branches_to_cardinals(open_branches, yaw_deg),
                    "after_status": clean_st,
                }
        elif args.command == "stop":
            output = _call_bridge(args.url, "/stop", "POST", {})
        elif args.command == "reset":
            if (args.target_x is None) != (args.target_y is None):
                raise BridgeRequestError("Cần truyền cả --target-x lẫn --target-y khi đổi vị trí mục tiêu.")
            target_xy = [args.target_x, args.target_y] if args.target_x is not None else None
            res = _call_bridge(args.url, "/reset", "POST", {"target_xy": target_xy})
            output = _strip_debug(res, keep_debug=False)
            if TopologicalOdometer.has_active_session(args.url, state_path=DEFAULT_TOPO_STATE_PATH):
                if args.keep_topo:
                    topo_inst = TopologicalOdometer(args.url, state_path=DEFAULT_TOPO_STATE_PATH)
                    output["topo"] = topo_inst.reset_for_repass(
                        output,
                        target_changed=(target_xy is not None),
                    )
                else:
                    TopologicalOdometer.reset_session(args.url, state_path=DEFAULT_TOPO_STATE_PATH)
            if args.vision:
                output["vision"] = _call_bridge(args.url, "/vision?camera=front_cam", "GET")
        elif args.command == "arenas":
            arenas_path = "/arenas?debug=1" if args.debug else "/arenas"
            output = _call_bridge(args.url, arenas_path, "GET")
        elif args.command == "reload":
            if (args.target_x is None) != (args.target_y is None):
                raise BridgeRequestError("Cần truyền cả --target-x lẫn --target-y khi đổi vị trí mục tiêu.")
            target_xy = [args.target_x, args.target_y] if args.target_x is not None else None
            preserve_target = False if args.no_preserve_target else None
            res = _call_bridge(
                args.url,
                "/reload",
                "POST",
                {
                    "arena": args.arena,
                    "model": args.model,
                    "preserve_target": preserve_target,
                    "target_xy": target_xy,
                },
            )
            if TopologicalOdometer.has_active_session(args.url, state_path=DEFAULT_TOPO_STATE_PATH):
                TopologicalOdometer.reset_session(args.url, state_path=DEFAULT_TOPO_STATE_PATH)
            output = _strip_debug(res, keep_debug=False)
            if args.vision:
                output["vision"] = _call_bridge(args.url, "/vision?camera=front_cam", "GET")
        elif args.command == "demo":
            if args.steps <= 0 or args.step_dt <= 0:
                raise BridgeRequestError("'--steps' và '--step-dt' phải lớn hơn 0.")
            output = _run_reactive_demo(args.url, steps=args.steps, step_dt=args.step_dt)
        elif args.command == "telemetry":
            if args.telemetry_action == "record":
                output = record_telemetry(
                    args.url,
                    out_path=args.out,
                    stop_flag_path=args.stop_file,
                    hz=args.hz,
                    max_duration_s=args.max_duration,
                )
            else:
                output = stop_telemetry(
                    out_path=args.out,
                    stop_flag_path=args.stop_file,
                    wait_timeout_s=args.wait_timeout,
                )
        else:  # pragma: no cover
            parser.print_help()
            return 2

        print(json.dumps(output, ensure_ascii=False, indent=2))
        return 0
    except (BridgeOfflineError, BridgeRequestError, ValueError, RuntimeError) as exc:
        print(f"[Lỗi điều khiển robot] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
