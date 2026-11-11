from __future__ import annotations

import argparse
import http.client
import json
from pathlib import Path
import sys
import time
import urllib.error
import urllib.request
import numpy as np
from PIL import Image

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


def _wait_for_completion(base_url: str, duration_s: float) -> dict:
    """Chờ cho tới khi mô phỏng hoàn tất lệnh chuyển động (moving == False)."""
    deadline = time.monotonic() + max(3.0, duration_s * 4.0)
    time.sleep(min(max(0.0, duration_s * 0.5), 0.25))
    status = _call_bridge(base_url, "/status", "GET")
    while status.get("moving", False) and time.monotonic() < deadline:
        time.sleep(0.05)
        status = _call_bridge(base_url, "/status", "GET")
    return _strip_debug(status, keep_debug=False)


def _detect_green_target_in_image(image_path: str) -> tuple[bool, float, float]:
    """Phân tích ảnh RGB từ camera mũi xe để tìm vùng màu xanh lá của mục tiêu.

    Trả về: (có_thấy_đích, độ_lệch_ngang [-1..+1, âm=bên trái, dương=bên phải], tỷ_lệ_điểm_ảnh).
    """
    arr = np.asarray(Image.open(image_path).convert("RGB"), dtype=np.int16)
    r, g, b = arr[:, :, 0], arr[:, :, 1], arr[:, :, 2]
    mask = (g > 130) & ((g - r) > 55) & ((g - b) > 55)
    count = int(mask.sum())
    total = mask.shape[0] * mask.shape[1]
    ratio = count / float(total)
    if count < 80:
        return False, 0.0, ratio
    xs = np.where(mask)[1]
    width = mask.shape[1]
    offset_norm = (float(xs.mean()) - (width / 2.0)) / (width / 2.0)
    return True, offset_norm, ratio


def _run_reactive_demo(base_url: str, steps: int, step_dt: float) -> dict:
    """Kịch bản tự hành cổ điển chỉ dùng cảm biến khoảng cách + camera mũi xe (không đọc tọa độ debug)."""
    history: list[str] = []
    reached = False

    for step_idx in range(steps):
        st = _call_bridge(base_url, "/status", "GET")
        rf = st["sensors"]["rangefinders_m"]
        cam = _call_bridge(base_url, "/camera", "POST", {"camera": "front_cam"})
        seen, offset_x, green_ratio = _detect_green_target_in_image(cam["path"])

        if seen and abs(offset_x) < 0.25 and rf["center"] < 0.30:
            _call_bridge(base_url, "/stop", "POST", {})
            history.append(f"Bước {step_idx + 1}: Đã tiếp cận mục tiêu màu xanh (cách {rf['center']:.2f}m)!")
            reached = True
            break

        # Phản xạ tránh vật cản gần (khi vật trước mặt KHÔNG phải mục tiêu xanh ở chính giữa)
        target_ahead = seen and abs(offset_x) < 0.25 and green_ratio > 0.01
        if not target_ahead and (rf["center"] < 0.45 or rf["left"] < 0.35 or rf["right"] < 0.35):
            if rf["left"] >= rf["right"]:
                _call_bridge(base_url, "/wheels", "POST", {"left": -4.5, "right": 4.5, "duration": step_dt})
                action = "né vật cản -> quay trái"
            else:
                _call_bridge(base_url, "/wheels", "POST", {"left": 4.5, "right": -4.5, "duration": step_dt})
                action = "né vật cản -> quay phải"
        elif seen:
            # Hướng đầu xe về phía khối màu xanh lá nhìn thấy qua camera mũi xe
            steer = 3.2 * offset_x
            left_w = 5.5 + steer
            right_w = 5.5 - steer
            _call_bridge(base_url, "/wheels", "POST", {"left": left_w, "right": right_w, "duration": step_dt})
            action = f"bám mục tiêu xanh (lệch ngang={offset_x:+.2f})"
        else:
            # Chưa thấy mục tiêu: vừa quét tìm vừa tiến nhẹ nếu đường thoáng
            if rf["center"] > 0.9:
                _call_bridge(base_url, "/wheels", "POST", {"left": 2.5, "right": 5.5, "duration": step_dt})
                action = "quét vòng tìm mục tiêu"
            else:
                _call_bridge(base_url, "/wheels", "POST", {"left": -4.0, "right": 4.0, "duration": step_dt})
                action = "xoay tại chỗ tìm mục tiêu"

        history.append(f"Bước {step_idx + 1}: {action}")
        time.sleep(step_dt)

    _call_bridge(base_url, "/stop", "POST", {})
    final_status = _strip_debug(_call_bridge(base_url, "/status", "GET"), keep_debug=False)
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
        "--debug",
        action="store_true",
        help="Hiển thị thêm tọa độ thực trong sân (chỉ dùng kiểm tra, không dùng khi đóng vai bộ não robot)",
    )

    p_cam = sub.add_parser("camera", help="Chụp ảnh PNG từ camera mũi xe hoặc camera toàn cảnh")
    p_cam.add_argument("--name", default="front_cam", choices=["front_cam", "overview_cam"])
    p_cam.add_argument("--out", default=None, help="Đường dẫn file ảnh PNG đầu ra (tùy chọn)")

    p_wheels = sub.add_parser("wheels", help="Đặt tốc độ góc 2 bánh trái/phải (rad/s)")
    p_wheels.add_argument("left", type=float, help="Vận tốc góc bánh trái (rad/s, -15..15)")
    p_wheels.add_argument("right", type=float, help="Vận tốc góc bánh phải (rad/s, -15..15)")
    p_wheels.add_argument("--duration", type=float, default=1.0, help="Thời gian duy trì lệnh (giây)")
    p_wheels.add_argument("--wait", action="store_true", help="Chờ xe thực hiện xong lệnh rồi trả trạng thái cảm biến")

    p_move = sub.add_parser("move", help="Tiến (+) hoặc lùi (-) một quãng đường ước lượng (mét)")
    p_move.add_argument("meters", type=float, nargs="?", default=None, help="Quãng đường cần đi (m, âm để lùi)")
    p_move.add_argument("--meters", dest="meters_opt", type=float, default=None, help="Quãng đường cần đi (m)")
    p_move.add_argument("--speed", type=float, default=0.25, help="Tốc độ dài ước lượng (m/s, mặc định: 0.25)")
    p_move.add_argument("--wait", action="store_true", help="Chờ xe chạy xong rồi in cảm biến mới")

    p_turn = sub.add_parser("turn", help="Xoay tại chỗ một góc (độ: + quay trái, - quay phải)")
    p_turn.add_argument("degrees", type=float, nargs="?", default=None, help="Góc xoay (độ, + trái / - phải)")
    p_turn.add_argument("--degrees", dest="degrees_opt", type=float, default=None, help="Góc xoay (độ)")
    p_turn.add_argument("--speed", type=float, default=60.0, help="Tốc độ xoay (độ/s, mặc định: 60)")
    p_turn.add_argument("--wait", action="store_true", help="Chờ xe xoay xong rồi in cảm biến mới")

    sub.add_parser("stop", help="Dừng khẩn cấp hai động cơ bánh xe")

    p_reset = sub.add_parser("reset", help="Đưa robot và mục tiêu về trạng thái ban đầu")
    p_reset.add_argument("--target-x", type=float, default=None, help="Tọa độ X mới của mục tiêu (-1.7..1.7)")
    p_reset.add_argument("--target-y", type=float, default=None, help="Tọa độ Y mới của mục tiêu (-1.7..1.7)")

    p_demo = sub.add_parser("demo", help="Chạy thử vòng lặp tự hành cổ điển (dùng camera + cảm biến khoảng cách)")
    p_demo.add_argument("--steps", type=int, default=25, help="Số nhịp điều khiển tối đa (mặc định: 25)")
    p_demo.add_argument("--step-dt", type=float, default=0.20, help="Chu kỳ mỗi nhịp điều khiển (giây)")

    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    try:
        if args.command == "status":
            res = _call_bridge(args.url, "/status", "GET")
            output = _strip_debug(res, keep_debug=args.debug)
        elif args.command == "camera":
            out_path = str(Path(args.out).resolve()) if args.out else None
            output = _call_bridge(args.url, "/camera", "POST", {"camera": args.name, "path": out_path})
        elif args.command == "wheels":
            output = _call_bridge(
                args.url,
                "/wheels",
                "POST",
                {"left": args.left, "right": args.right, "duration": args.duration},
            )
            if args.wait:
                output["after_status"] = _wait_for_completion(args.url, output["duration_s"])
        elif args.command == "move":
            meters = args.meters_opt if args.meters_opt is not None else args.meters
            if meters is None:
                raise BridgeRequestError("Vui lòng truyền quãng đường (ví dụ: move 0.5 hoặc move -0.3).")
            output = _call_bridge(args.url, "/move", "POST", {"meters": meters, "speed": args.speed})
            if args.wait:
                output["after_status"] = _wait_for_completion(args.url, output["duration_s"])
        elif args.command == "turn":
            degrees = args.degrees_opt if args.degrees_opt is not None else args.degrees
            if degrees is None:
                raise BridgeRequestError("Vui lòng truyền góc xoay (ví dụ: turn 45 hoặc turn -45).")
            output = _call_bridge(
                args.url,
                "/turn",
                "POST",
                {"degrees": degrees, "angular_speed": args.speed},
            )
            if args.wait:
                output["after_status"] = _wait_for_completion(args.url, output["duration_s"])
        elif args.command == "stop":
            output = _call_bridge(args.url, "/stop", "POST", {})
        elif args.command == "reset":
            if (args.target_x is None) != (args.target_y is None):
                raise BridgeRequestError("Cần truyền cả --target-x lẫn --target-y khi đổi vị trí mục tiêu.")
            target_xy = [args.target_x, args.target_y] if args.target_x is not None else None
            res = _call_bridge(args.url, "/reset", "POST", {"target_xy": target_xy})
            output = _strip_debug(res, keep_debug=False)
        elif args.command == "demo":
            if args.steps <= 0 or args.step_dt <= 0:
                raise BridgeRequestError("'--steps' và '--step-dt' phải lớn hơn 0.")
            output = _run_reactive_demo(args.url, steps=args.steps, step_dt=args.step_dt)
        else:  # pragma: no cover
            parser.print_help()
            return 2

        print(json.dumps(output, ensure_ascii=False, indent=2))
        return 0
    except (BridgeOfflineError, BridgeRequestError) as exc:
        print(f"[Lỗi điều khiển robot] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
