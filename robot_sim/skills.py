from __future__ import annotations

from collections.abc import Callable
import math
import time

from robot_sim.core import MAX_WHEEL_RAD_S, TRACK_WIDTH, WHEEL_RADIUS, _require_finite

BridgeCall = Callable[[str, str, dict | None], dict]


def _clean_status(status_data: dict) -> dict:
    return {k: v for k, v in status_data.items() if k != "debug"}


def wait_until_settled(
    call_fn: BridgeCall,
    duration_s: float,
    include_vision: bool = False,
    speed_tol: float = 0.015,
    yaw_tol: float = 0.03,
    settle_samples: int = 2,
) -> dict:
    """Chờ hết thời hạn lệnh VÀ chờ thân xe tắt hẳn dao động quán tính trước khi đọc cảm biến."""
    _require_finite(duration_s=duration_s, speed_tol=speed_tol, yaw_tol=yaw_tol)
    deadline = time.monotonic() + max(3.0, duration_s * 4.0)
    if duration_s > 0.04:
        time.sleep(min(duration_s * 0.45, 0.20))

    status = call_fn("/status", "GET", None)
    while status.get("moving", False) and time.monotonic() < deadline:
        time.sleep(0.035)
        status = call_fn("/status", "GET", None)

    quiet_streak = 0
    settle_deadline = time.monotonic() + 0.65
    while time.monotonic() < settle_deadline:
        sensors = status.get("sensors", {})
        fwd = abs(float(sensors.get("forward_speed_m_s", 0.0)))
        yaw = abs(float(sensors.get("yaw_rate_rad_s", 0.0)))
        if not status.get("moving", False) and fwd <= speed_tol and yaw <= yaw_tol:
            quiet_streak += 1
            if quiet_streak >= settle_samples:
                break
        else:
            quiet_streak = 0
        time.sleep(0.025)
        status = call_fn("/status", "GET", None)

    if include_vision:
        status = call_fn("/status?vision=1", "GET", None)
    return _clean_status(status)


def closed_loop_turn(
    call_fn: BridgeCall,
    degrees: float,
    max_speed_deg_s: float = 90.0,
    tol_deg: float = 1.8,
    timeout_s: float = 8.0,
    include_vision: bool = False,
) -> dict:
    """Xoay khép kín dựa hoàn toàn trên tích phân con quay hồi chuyển gắn trên xe (chassis_gyro)."""
    _require_finite(degrees=degrees, max_speed_deg_s=max_speed_deg_s, tol_deg=tol_deg, timeout_s=timeout_s)
    if abs(degrees) < 1e-4:
        raise ValueError("Góc xoay 'degrees' phải khác 0.")
    if max_speed_deg_s <= 0.0 or tol_deg <= 0.0 or timeout_s <= 0.0:
        raise ValueError("Các tham số tốc độ, dung sai và timeout của lệnh xoay phải lớn hơn 0.")

    st = call_fn("/status", "GET", None)
    start_gyro_deg = float(st["sensors"]["gyro_yaw_deg"])
    accum_deg = 0.0
    deadline = time.monotonic() + max(timeout_s, (abs(degrees) / max(10.0, max_speed_deg_s)) * 4.0 + 2.5)

    while time.monotonic() < deadline:
        remaining = degrees - accum_deg
        if abs(remaining) <= tol_deg:
            break

        cmd_deg_s = min(max_speed_deg_s, max(8.0, 2.5 * abs(remaining)))
        omega = min((math.radians(cmd_deg_s) * TRACK_WIDTH) / (2.0 * WHEEL_RADIUS), MAX_WHEEL_RAD_S)
        if remaining > 0.0:
            left_w, right_w = -omega, omega
        else:
            left_w, right_w = omega, -omega

        call_fn("/wheels", "POST", {"left": left_w, "right": right_w, "duration": 0.25})
        time.sleep(0.02)

        st = call_fn("/status", "GET", None)
        accum_deg = float(st["sensors"]["gyro_yaw_deg"]) - start_gyro_deg

    call_fn("/stop", "POST", {})
    settled = wait_until_settled(call_fn, duration_s=0.05, include_vision=include_vision)
    accum_deg = float(settled["sensors"]["gyro_yaw_deg"]) - start_gyro_deg
    return {
        "requested_degrees": round(float(degrees), 2),
        "measured_turn_deg": round(accum_deg, 2),
        "after_status": settled,
    }


def scan_for_target(
    call_fn: BridgeCall,
    max_degrees: float = 360.0,
    direction: str = "left",
    angular_speed_deg_s: float = 75.0,
) -> dict:
    """Quét vòng tròn ở tần số cao để tìm trụ đích màu xanh lá và tự động căn vào giữa khung hình."""
    _require_finite(max_degrees=max_degrees, angular_speed_deg_s=angular_speed_deg_s)
    if max_degrees <= 0.0 or max_degrees > 720.0:
        raise ValueError("Tham số 'max_degrees' phải nằm trong khoảng (0, 720].")
    if direction not in ("left", "right"):
        raise ValueError("Hướng quét 'direction' phải là 'left' hoặc 'right'.")
    if angular_speed_deg_s <= 0.0:
        raise ValueError("Tốc độ quét 'angular_speed_deg_s' phải lớn hơn 0.")

    st = call_fn("/status?vision=1", "GET", None)
    start_gyro_deg = float(st["sensors"]["gyro_yaw_deg"])
    scanned_deg = 0.0
    omega = min((math.radians(angular_speed_deg_s) * TRACK_WIDTH) / (2.0 * WHEEL_RADIUS), MAX_WHEEL_RAD_S)
    left_w, right_w = (-omega, omega) if direction == "left" else (omega, -omega)
    deadline = time.monotonic() + max(6.0, (max_degrees / max(15.0, angular_speed_deg_s)) * 4.0 + 3.0)

    found = False
    while time.monotonic() < deadline:
        vis = st["vision"]
        if vis["target_visible"] and abs(float(vis["target"]["offset_x"])) < 0.78:
            found = True
            break
        if scanned_deg >= max_degrees:
            break

        call_fn("/wheels", "POST", {"left": left_w, "right": right_w, "duration": 0.25})
        time.sleep(0.035)
        st = call_fn("/status?vision=1", "GET", None)
        scanned_deg = abs(float(st["sensors"]["gyro_yaw_deg"]) - start_gyro_deg)

    call_fn("/stop", "POST", {})
    settled = wait_until_settled(call_fn, duration_s=0.05, include_vision=True)

    if settled["vision"]["target_visible"]:
        found = True
        offset_x = float(settled["vision"]["target"]["offset_x"])
        if abs(offset_x) > 0.14:
            center_turn = closed_loop_turn(
                call_fn,
                degrees=-offset_x * 26.0,
                max_speed_deg_s=55.0,
                include_vision=True,
            )
            settled = center_turn["after_status"]

    return {
        "target_found": bool(found and settled["vision"]["target_visible"]),
        "scanned_degrees": round(scanned_deg, 2),
        "after_status": _clean_status(settled),
    }


def approach_target(
    call_fn: BridgeCall,
    stop_distance: float = 0.25,
    max_steps: int = 65,
    step_dt: float = 0.18,
) -> dict:
    """Tiếp cận mục tiêu xanh khép kín 20Hz dựa trên Edge Vision + cảm biến khoảng cách (chuẩn Sim-to-Real)."""
    _require_finite(stop_distance=stop_distance, max_steps=max_steps, step_dt=step_dt)
    if isinstance(max_steps, bool) or int(max_steps) < 1:
        raise ValueError("Số bước 'max_steps' phải là số nguyên >= 1.")
    if not (0.12 <= stop_distance <= 1.5):
        raise ValueError("Khoảng cách dừng 'stop_distance' phải nằm trong đoạn [0.12, 1.5] mét.")
    if not (0.04 <= step_dt <= 1.0):
        raise ValueError("Chu kỳ điều khiển 'step_dt' phải nằm trong đoạn [0.04, 1.0] giây.")

    lost_streak = 0
    steps_taken = 0

    for step_idx in range(1, int(max_steps) + 1):
        steps_taken = step_idx
        st = call_fn("/status?vision=1", "GET", None)
        rf = st["sensors"]["rangefinders_m"]
        vis = st["vision"]
        seen = bool(vis["target_visible"])
        offset_x = float(vis["target"]["offset_x"]) if seen else 0.0
        area_pct = float(vis["target"]["area_pct"]) if seen else 0.0

        if seen and abs(offset_x) < 0.24 and rf["center"] <= stop_distance + 0.03:
            call_fn("/stop", "POST", {})
            settled = wait_until_settled(call_fn, duration_s=0.05, include_vision=True)
            return {
                "reached": True,
                "reason": "reached_target",
                "steps": steps_taken,
                "after_status": _clean_status(settled),
            }

        if not seen:
            lost_streak += 1
            if lost_streak >= 4:
                call_fn("/stop", "POST", {})
                settled = wait_until_settled(call_fn, duration_s=0.05, include_vision=True)
                return {
                    "reached": False,
                    "reason": "target_lost",
                    "steps": steps_taken,
                    "after_status": _clean_status(settled),
                }
            time.sleep(0.04)
            continue

        lost_streak = 0
        target_ahead = abs(offset_x) < 0.26 and (area_pct > 1.2 or len(vis["obstacles"]) == 0)

        if not target_ahead and rf["center"] < 0.23:
            call_fn("/stop", "POST", {})
            settled = wait_until_settled(call_fn, duration_s=0.05, include_vision=True)
            return {
                "reached": False,
                "reason": "obstacle_blocked",
                "steps": steps_taken,
                "after_status": _clean_status(settled),
            }

        if (not target_ahead and rf["center"] < 0.42) or rf["left"] < 0.25 or rf["right"] < 0.25:
            avoid = 3.6 if rf["left"] < rf["right"] else -3.6
            left_w = 2.4 + avoid
            right_w = 2.4 - avoid
        elif abs(offset_x) > 0.34:
            steer = 3.8 * offset_x
            left_w = 1.2 + steer
            right_w = 1.2 - steer
        else:
            dist_err = max(0.0, float(rf["center"]) - stop_distance)
            base_w = min(6.8, max(2.0, dist_err * 11.5)) if rf["center"] < stop_distance + 0.55 else 6.8
            steer = 3.4 * offset_x
            left_w = base_w + steer
            right_w = base_w - steer

        call_fn("/wheels", "POST", {"left": left_w, "right": right_w, "duration": round(step_dt * 1.35, 3)})
        time.sleep(step_dt * 0.85)

    call_fn("/stop", "POST", {})
    settled = wait_until_settled(call_fn, duration_s=0.05, include_vision=True)
    vis_end = settled["vision"]
    rf_end = settled["sensors"]["rangefinders_m"]["center"]
    goal_ok = bool(
        vis_end["target_visible"]
        and abs(float(vis_end["target"]["offset_x"])) < 0.25
        and rf_end <= stop_distance + 0.05
    )
    return {
        "reached": goal_ok,
        "reason": "reached_target" if goal_ok else "max_steps",
        "steps": steps_taken,
        "after_status": _clean_status(settled),
    }
