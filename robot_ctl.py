from __future__ import annotations

import argparse
import http.client
import json
from pathlib import Path
import sys
import time
import urllib.error
import urllib.request

from robot_sim.skills import approach_target, closed_loop_turn, scan_for_target, wait_until_settled

DEFAULT_URL = "http://127.0.0.1:8765"


class BridgeOfflineError(RuntimeError):
    pass


class BridgeRequestError(RuntimeError):
    pass


def _call_bridge(base_url: str, path: str, method: str = "GET", payload: dict | None = None) -> dict:
    if not isinstance(base_url, str) or not base_url.startswith(("http://", "https://")):
        raise BridgeRequestError(
            f"Địa chỉ --url không hợp lệ ('{base_url}'). Cần bắt đầu bằng http:// hoặc https:// (ví dụ: {DEFAULT_URL})."
        )
    url = f"{base_url.rstrip('/')}{path}"
    raw_body = None if payload is None else json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(
        url,
        data=raw_body,
        method=method,
        headers={"Content-Type": "application/json"},
    )
    try:
        with urllib.request.urlopen(req, timeout=6.0) as resp:
            parsed = json.loads(resp.read().decode("utf-8"))
            return parsed.get("data", parsed)
    except urllib.error.HTTPError as exc:
        try:
            err_json = json.loads(exc.read().decode("utf-8"))
            msg = err_json.get("error", str(exc))
        except Exception:
            msg = str(exc)
        raise BridgeRequestError(msg) from exc
    except (urllib.error.URLError, http.client.HTTPException, TimeoutError, ConnectionError, OSError) as exc:
        raise BridgeOfflineError(
            f"Không kết nối được hoặc mất kết nối tới mô phỏng tại {base_url}.\n"
            "Hãy mở một cửa sổ terminal khác và khởi chạy cửa sổ 3D MuJoCo trước bằng lệnh:\n"
            "  uv run python sim_server.py"
        ) from exc


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
    p_move.add_argument("--wait", action="store_true", help="Chờ xe chạy xong rồi in cảm biến mới")
    p_move.add_argument("--vision", action="store_true", help="Kèm tóm tắt Edge Vision trong after_status khi dùng --wait")

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

    p_approach = sub.add_parser("approach", help="Tiếp cận mục tiêu xanh khép kín 20Hz kết hợp né vật cản gần")
    p_approach.add_argument("--stop-distance", type=float, default=0.25, help="Khoảng cách dừng trước đích (0.12..1.5m)")
    p_approach.add_argument("--max-steps", type=int, default=65, help="Số nhịp điều khiển tối đa (mặc định: 65)")
    p_approach.add_argument("--step-dt", type=float, default=0.18, help="Chu kỳ mỗi nhịp điều khiển (giây)")

    sub.add_parser("stop", help="Dừng khẩn cấp hai động cơ bánh xe")

    p_reset = sub.add_parser("reset", help="Đưa robot và mục tiêu về trạng thái ban đầu")
    p_reset.add_argument("--target-x", type=float, default=None, help="Tọa độ X mới của mục tiêu (-1.7..1.7)")
    p_reset.add_argument("--target-y", type=float, default=None, help="Tọa độ Y mới của mục tiêu (-1.7..1.7)")
    p_reset.add_argument("--vision", action="store_true", help="Kèm tóm tắt Edge Vision ngay sau khi reset")

    p_reload = sub.add_parser("reload", help="Kiểm định chống gian lận và nạp nóng lại models/arena_2wheel.xml")
    p_reload.add_argument("--no-preserve-target", action="store_true", help="Đặt lại vị trí mục tiêu mặc định khi reload")
    p_reload.add_argument("--target-x", type=float, default=None, help="Tọa độ X mới của mục tiêu (-1.7..1.7)")
    p_reload.add_argument("--target-y", type=float, default=None, help="Tọa độ Y mới của mục tiêu (-1.7..1.7)")
    p_reload.add_argument("--vision", action="store_true", help="Kèm tóm tắt Edge Vision ngay sau khi reload")

    p_demo = sub.add_parser("demo", help="Chạy thử vòng lặp tự hành cổ điển (dùng Edge Vision + cảm biến khoảng cách)")
    p_demo.add_argument("--steps", type=int, default=25, help="Số nhịp điều khiển tối đa (mặc định: 25)")
    p_demo.add_argument("--step-dt", type=float, default=0.20, help="Chu kỳ mỗi nhịp điều khiển (giây)")

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
        elif args.command == "move":
            meters = args.meters_opt if args.meters_opt is not None else args.meters
            if meters is None:
                raise BridgeRequestError("Vui lòng truyền quãng đường (ví dụ: move 0.5 hoặc move -0.3).")
            output = _call_bridge(args.url, "/move", "POST", {"meters": meters, "speed": args.speed})
            if args.wait:
                output["after_status"] = _wait_for_completion(
                    args.url, output["duration_s"], include_vision=args.vision
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
        elif args.command == "scan":
            call_fn = lambda path, method="GET", payload=None: _call_bridge(args.url, path, method, payload)
            output = scan_for_target(
                call_fn,
                max_degrees=args.max_degrees,
                direction=args.direction,
                angular_speed_deg_s=args.speed,
            )
        elif args.command == "approach":
            call_fn = lambda path, method="GET", payload=None: _call_bridge(args.url, path, method, payload)
            output = approach_target(
                call_fn,
                stop_distance=args.stop_distance,
                max_steps=args.max_steps,
                step_dt=args.step_dt,
            )
        elif args.command == "stop":
            output = _call_bridge(args.url, "/stop", "POST", {})
        elif args.command == "reset":
            if (args.target_x is None) != (args.target_y is None):
                raise BridgeRequestError("Cần truyền cả --target-x lẫn --target-y khi đổi vị trí mục tiêu.")
            target_xy = [args.target_x, args.target_y] if args.target_x is not None else None
            res = _call_bridge(args.url, "/reset", "POST", {"target_xy": target_xy})
            output = _strip_debug(res, keep_debug=False)
            if args.vision:
                output["vision"] = _call_bridge(args.url, "/vision?camera=front_cam", "GET")
        elif args.command == "reload":
            if (args.target_x is None) != (args.target_y is None):
                raise BridgeRequestError("Cần truyền cả --target-x lẫn --target-y khi đổi vị trí mục tiêu.")
            target_xy = [args.target_x, args.target_y] if args.target_x is not None else None
            res = _call_bridge(
                args.url,
                "/reload",
                "POST",
                {"preserve_target": not args.no_preserve_target, "target_xy": target_xy},
            )
            output = _strip_debug(res, keep_debug=False)
            if args.vision:
                output["vision"] = _call_bridge(args.url, "/vision?camera=front_cam", "GET")
        elif args.command == "demo":
            if args.steps <= 0 or args.step_dt <= 0:
                raise BridgeRequestError("'--steps' và '--step-dt' phải lớn hơn 0.")
            output = _run_reactive_demo(args.url, steps=args.steps, step_dt=args.step_dt)
        else:  # pragma: no cover
            parser.print_help()
            return 2

        print(json.dumps(output, ensure_ascii=False, indent=2))
        return 0
    except (BridgeOfflineError, BridgeRequestError, ValueError) as exc:
        print(f"[Lỗi điều khiển robot] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
