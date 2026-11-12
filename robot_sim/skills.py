from __future__ import annotations

from collections.abc import Callable
import math
import time

from robot_sim.core import (
    DEFAULT_CAMERA_HEIGHT,
    DEFAULT_CAMERA_WIDTH,
    MAX_WHEEL_RAD_S,
    TRACK_WIDTH,
    WHEEL_RADIUS,
    _require_finite,
)

BridgeCall = Callable[[str, str, dict | None], dict]

# Hằng số quang học nội tại của camera mũi xe (fovy=65°, khung hình 320x240 => tan(fovx/2) ≈ 0.8494)
FRONT_CAM_FOVY_DEG = 65.0
FRONT_CAM_TAN_HALF_FOVX = (DEFAULT_CAMERA_WIDTH / float(DEFAULT_CAMERA_HEIGHT)) * math.tan(
    math.radians(FRONT_CAM_FOVY_DEG / 2.0)
)


def _clean_status(status_data: dict) -> dict:
    return {k: v for k, v in status_data.items() if k != "debug"}


def _settle_at_zero_hold(
    call_fn: BridgeCall,
    include_vision: bool = False,
    speed_tol: float = 0.015,
    yaw_tol: float = 0.025,
    settle_samples: int = 2,
) -> dict:
    """Giữ phanh chủ động (0 rad/s, moving=True) cho tới khi thân xe tắt hẳn dao động quán tính."""
    call_fn("/wheels", "POST", {"left": 0.0, "right": 0.0, "duration": 0.55})
    quiet_streak = 0
    deadline = time.monotonic() + 0.55
    status = call_fn("/status", "GET", None)
    while time.monotonic() < deadline:
        sensors = status.get("sensors", {})
        fwd = abs(float(sensors.get("forward_speed_m_s", 0.0)))
        yaw = abs(float(sensors.get("yaw_rate_rad_s", 0.0)))
        if fwd <= speed_tol and yaw <= yaw_tol:
            quiet_streak += 1
            if quiet_streak >= settle_samples:
                break
        else:
            quiet_streak = 0
        call_fn("/wheels", "POST", {"left": 0.0, "right": 0.0, "duration": 0.55})
        time.sleep(0.018)
        status = call_fn("/status", "GET", None)

    if include_vision:
        status = call_fn("/status?vision=1", "GET", None)
    return _clean_status(status)


def _stop_and_settle(call_fn: BridgeCall, include_vision: bool = False) -> dict:
    """Phanh chủ động về 0 m/s trước khi ngắt /stop để triệt tiêu trôi quán tính sau lệnh dừng."""
    try:
        settled = _settle_at_zero_hold(call_fn, include_vision=False)
    finally:
        call_fn("/stop", "POST", {})
    if include_vision:
        settled = _clean_status(call_fn("/status?vision=1", "GET", None))
    else:
        settled = _clean_status(call_fn("/status", "GET", None))
    return settled


def wait_until_settled(
    call_fn: BridgeCall,
    duration_s: float,
    include_vision: bool = False,
    speed_tol: float = 0.015,
    yaw_tol: float = 0.025,
    settle_samples: int = 2,
) -> dict:
    """Chờ hết thời hạn lệnh VÀ chờ thân xe tắt hẳn dao động quán tính trước khi đọc cảm biến."""
    _require_finite(duration_s=duration_s, speed_tol=speed_tol, yaw_tol=yaw_tol)
    deadline = time.monotonic() + max(3.0, duration_s * 4.0)
    if duration_s > 0.04:
        time.sleep(min(duration_s * 0.45, 0.20))

    status = call_fn("/status", "GET", None)
    while status.get("moving", False) and time.monotonic() < deadline:
        time.sleep(0.03)
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
        time.sleep(0.022)
        status = call_fn("/status", "GET", None)

    if include_vision:
        status = call_fn("/status?vision=1", "GET", None)
    return _clean_status(status)


def closed_loop_turn(
    call_fn: BridgeCall,
    degrees: float,
    max_speed_deg_s: float = 90.0,
    tol_deg: float = 0.55,
    timeout_s: float = 8.0,
    include_vision: bool = False,
    release_stop: bool = True,
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
    watchdog_dur = 0.55
    deadline = time.monotonic() + max(timeout_s, (abs(degrees) / max(10.0, max_speed_deg_s)) * 4.5 + 3.0)

    completed_ok = False
    try:
        while time.monotonic() < deadline:
            remaining = degrees - accum_deg
            if abs(remaining) <= tol_deg:
                break

            cmd_deg_s = min(max_speed_deg_s, max(4.8, 2.4 * abs(remaining)))
            omega = min((math.radians(cmd_deg_s) * TRACK_WIDTH) / (2.0 * WHEEL_RADIUS), MAX_WHEEL_RAD_S)
            if remaining > 0.0:
                left_w, right_w = -omega, omega
            else:
                left_w, right_w = omega, -omega

            call_fn(
                "/wheels",
                "POST",
                {"left": round(left_w, 4), "right": round(right_w, 4), "duration": watchdog_dur},
            )
            time.sleep(0.015)

            st = call_fn("/status", "GET", None)
            accum_deg = float(st["sensors"]["gyro_yaw_deg"]) - start_gyro_deg

        settled = (
            _stop_and_settle(call_fn, include_vision=include_vision)
            if release_stop
            else _settle_at_zero_hold(call_fn, include_vision=include_vision)
        )
        completed_ok = True
    finally:
        if not completed_ok:
            call_fn("/stop", "POST", {})

    accum_deg = float(settled["sensors"]["gyro_yaw_deg"]) - start_gyro_deg
    return {
        "requested_degrees": round(float(degrees), 2),
        "measured_turn_deg": round(accum_deg, 2),
        "after_status": settled,
    }


def closed_loop_move(
    call_fn: BridgeCall,
    meters: float,
    speed_m_s: float = 0.25,
    kp_yaw: float = 0.55,
    ki_yaw: float = 0.35,
    kd_yaw: float = 0.02,
    safety_stop_m: float = 0.22,
    tol_m: float = 0.012,
    timeout_s: float = 12.0,
    include_vision: bool = False,
    release_stop: bool = True,
) -> dict:
    """Đi thẳng tiến/lùi khép kín giữ nguyên góc hướng IMU (gyro_yaw_deg) và tích phân vận tốc thân xe."""
    _require_finite(
        meters=meters,
        speed_m_s=speed_m_s,
        kp_yaw=kp_yaw,
        ki_yaw=ki_yaw,
        kd_yaw=kd_yaw,
        safety_stop_m=safety_stop_m,
        tol_m=tol_m,
        timeout_s=timeout_s,
    )
    if abs(meters) < 1e-4:
        raise ValueError("Quãng đường 'meters' phải khác 0.")
    if speed_m_s <= 0.0 or tol_m <= 0.0 or timeout_s <= 0.0 or safety_stop_m < 0.0:
        raise ValueError("Tốc độ, dung sai và timeout của lệnh di chuyển phải hợp lệ (> 0).")
    if kp_yaw < 0.0 or ki_yaw < 0.0 or kd_yaw < 0.0:
        raise ValueError("Hệ số điều khiển giữ hướng 'kp_yaw', 'ki_yaw', 'kd_yaw' không được âm.")

    st = call_fn("/status", "GET", None)
    start_gyro_deg = float(st["sensors"]["gyro_yaw_deg"])
    has_hw_odom = "odom_dist_m" in st["sensors"]
    start_odom_m = float(st["sensors"].get("odom_dist_m", 0.0))
    start_sim_t = float(st["sim_time"])
    prev_sim_t = start_sim_t
    prev_fwd = float(st["sensors"]["forward_speed_m_s"])

    direction = 1.0 if meters > 0.0 else -1.0
    target_dist = abs(float(meters))
    max_speed = min(abs(float(speed_m_s)), MAX_WHEEL_RAD_S * WHEEL_RADIUS)
    signed_traveled = 0.0
    prev_cmd_speed = max(0.08, abs(prev_fwd))
    yaw_int = 0.0
    step_dt = 0.02
    watchdog_dur = round(max(0.55, step_dt * 3.5), 4)
    last_dt_sim = step_dt
    last_pulse_dur = watchdog_dur
    stopped_early = False
    stop_reason = "completed"

    max_sim_duration = (target_dist / max(0.05, max_speed)) * 2.8 + 1.8
    deadline = time.monotonic() + max(timeout_s, (target_dist / max(0.05, max_speed)) * 5.0 + 3.5)

    completed_ok = False
    try:
        while time.monotonic() < deadline:
            rf_c = float(st["sensors"]["rangefinders_m"]["center"])
            if direction > 0.0 and safety_stop_m > 0.0 and rf_c < safety_stop_m:
                stopped_early = True
                stop_reason = "obstacle_ahead"
                break

            remaining_m = target_dist - signed_traveled
            if remaining_m <= tol_m:
                break

            if (float(st["sim_time"]) - start_sim_t) >= max_sim_duration:
                stopped_early = True
                stop_reason = "sim_timeout"
                break

            accel_cap = prev_cmd_speed + 1.4 * max(0.02, last_dt_sim)
            decel_cap = max(0.055, 2.8 * remaining_m)
            cmd_speed = min(max_speed, accel_cap, decel_cap)
            prev_cmd_speed = cmd_speed
            base_omega = min(cmd_speed / WHEEL_RADIUS, MAX_WHEEL_RAD_S)
            last_pulse_dur = watchdog_dur

            cur_gyro_deg = float(st["sensors"]["gyro_yaw_deg"])
            yaw_rate_deg_s = math.degrees(float(st["sensors"]["yaw_rate_rad_s"]))
            yaw_err_deg = start_gyro_deg - cur_gyro_deg
            eff_kp = min(kp_yaw, 0.65 / (25.0 * max(0.02, last_dt_sim)))
            speed_scale = cmd_speed / max(0.05, max_speed)
            trim = max(
                -2.2,
                min(2.2, eff_kp * yaw_err_deg + ki_yaw * yaw_int * speed_scale - kd_yaw * yaw_rate_deg_s),
            )

            left_w = max(-MAX_WHEEL_RAD_S, min(MAX_WHEEL_RAD_S, direction * base_omega - trim))
            right_w = max(-MAX_WHEEL_RAD_S, min(MAX_WHEEL_RAD_S, direction * base_omega + trim))

            call_fn(
                "/wheels",
                "POST",
                {"left": round(left_w, 4), "right": round(right_w, 4), "duration": watchdog_dur},
            )
            time.sleep(step_dt)

            st = call_fn("/status", "GET", None)
            cur_sim_t = float(st["sim_time"])
            cur_fwd = float(st["sensors"]["forward_speed_m_s"])
            dt_sim = max(0.0, cur_sim_t - prev_sim_t)
            if has_hw_odom and "odom_dist_m" in st["sensors"]:
                signed_traveled = direction * (float(st["sensors"]["odom_dist_m"]) - start_odom_m)
            elif dt_sim > 0.0:
                eff_dt = min(dt_sim, last_pulse_dur + 0.03)
                signed_traveled += direction * 0.5 * (prev_fwd + cur_fwd) * eff_dt
            if dt_sim > 0.0:
                yaw_err_now = start_gyro_deg - float(st["sensors"]["gyro_yaw_deg"])
                yaw_int = max(-1.8, min(1.8, yaw_int + yaw_err_now * dt_sim))
                last_dt_sim = dt_sim
                prev_sim_t = cur_sim_t
                prev_fwd = cur_fwd

        settled = _settle_at_zero_hold(call_fn, include_vision=include_vision)
        if has_hw_odom and "odom_dist_m" in settled["sensors"]:
            signed_traveled = direction * (float(settled["sensors"]["odom_dist_m"]) - start_odom_m)

        yaw_residual = start_gyro_deg - float(settled["sensors"]["gyro_yaw_deg"])
        if not stopped_early and abs(yaw_residual) > 0.45:
            settled = closed_loop_turn(
                call_fn,
                degrees=yaw_residual,
                max_speed_deg_s=45.0,
                tol_deg=0.4,
                include_vision=include_vision,
                release_stop=False,
            )["after_status"]

        if release_stop:
            settled = _stop_and_settle(call_fn, include_vision=include_vision)
        completed_ok = True
    finally:
        if not completed_ok:
            call_fn("/stop", "POST", {})

    heading_drift_deg = float(settled["sensors"]["gyro_yaw_deg"]) - start_gyro_deg
    return {
        "requested_meters": round(float(meters), 4),
        "measured_meters": round(direction * signed_traveled, 4),
        "heading_drift_deg": round(heading_drift_deg, 3),
        "stopped_early": stopped_early,
        "stop_reason": stop_reason,
        "after_status": settled,
    }


def scan_for_target(
    call_fn: BridgeCall,
    max_degrees: float = 360.0,
    direction: str = "left",
    angular_speed_deg_s: float = 75.0,
    recover_occlusion: bool = False,
) -> dict:
    """Quét vòng tròn ở tần số cao để tìm trụ đích màu xanh lá và tự động căn chính xác vào tâm khung hình."""
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

    best_open_yaw_deg: float | None = None
    best_open_score = -1.0
    best_fallback_yaw_deg = start_gyro_deg
    best_fallback_min_rf = -1.0

    def _record_open_corridor(sample: dict) -> None:
        nonlocal best_open_yaw_deg, best_open_score, best_fallback_yaw_deg, best_fallback_min_rf
        rf = sample["sensors"]["rangefinders_m"]
        rf_l, rf_c, rf_r = float(rf["left"]), float(rf["center"]), float(rf["right"])
        min_rf = min(rf_l, rf_c, rf_r)
        yaw_deg = float(sample["sensors"]["gyro_yaw_deg"])
        if min_rf > best_fallback_min_rf:
            best_fallback_min_rf = min_rf
            best_fallback_yaw_deg = yaw_deg
        if rf_c > 1.4:
            score = min_rf + 0.15 * rf_c
            if score > best_open_score:
                best_open_score = score
                best_open_yaw_deg = yaw_deg

    found = False
    last_seen_offset_x: float | None = None
    completed_ok = False
    try:
        while time.monotonic() < deadline:
            _record_open_corridor(st)
            vis = st["vision"]
            if vis["target_visible"] and abs(float(vis["target"]["offset_x"])) < 0.84:
                found = True
                last_seen_offset_x = float(vis["target"]["offset_x"])
                break
            if scanned_deg >= max_degrees:
                break

            call_fn("/wheels", "POST", {"left": round(left_w, 4), "right": round(right_w, 4), "duration": 0.55})
            time.sleep(0.03)
            st = call_fn("/status?vision=1", "GET", None)
            scanned_deg = abs(float(st["sensors"]["gyro_yaw_deg"]) - start_gyro_deg)

        settled = _settle_at_zero_hold(call_fn, include_vision=True)
        _record_open_corridor(settled)

        # Nếu mục tiêu vừa trôi nhẹ qua mép ảnh trong lúc phanh dừng, xoay bù lại theo góc nhìn cuối cùng
        if found and not settled["vision"]["target_visible"] and last_seen_offset_x is not None:
            recover_deg = -math.degrees(math.atan(last_seen_offset_x * FRONT_CAM_TAN_HALF_FOVX))
            if abs(recover_deg) >= 0.8:
                settled = closed_loop_turn(
                    call_fn,
                    degrees=recover_deg,
                    max_speed_deg_s=60.0,
                    tol_deg=0.5,
                    include_vision=True,
                    release_stop=False,
                )["after_status"]

        # Căn tâm chặt (|offset_x| <= 0.08 < 0.10) theo hình học quang học camera mũi xe
        for _ in range(3):
            if not settled["vision"]["target_visible"]:
                break
            found = True
            offset_x = float(settled["vision"]["target"]["offset_x"])
            if abs(offset_x) <= 0.08:
                break
            turn_deg = -math.degrees(math.atan(offset_x * FRONT_CAM_TAN_HALF_FOVX))
            if abs(turn_deg) < 0.55:
                break
            center_turn = closed_loop_turn(
                call_fn,
                degrees=turn_deg,
                max_speed_deg_s=60.0,
                tol_deg=0.45,
                include_vision=True,
                release_stop=False,
            )
            next_settled = center_turn["after_status"]
            if not next_settled["vision"]["target_visible"]:
                # Nếu xoay căn tâm làm mũi camera khuất sau mép vật cản, lùi góc nhẹ để giữ khóa ảnh
                settled = closed_loop_turn(
                    call_fn,
                    degrees=-0.7 * turn_deg,
                    max_speed_deg_s=45.0,
                    tol_deg=0.45,
                    include_vision=True,
                    release_stop=False,
                )["after_status"]
                break
            settled = next_settled

        target_found = bool(found and settled["vision"]["target_visible"])
        total_scanned_deg = abs(float(settled["sensors"]["gyro_yaw_deg"]) - start_gyro_deg)

        if not target_found and recover_occlusion:
            chosen_yaw = best_open_yaw_deg if best_open_yaw_deg is not None else best_fallback_yaw_deg
            cur_yaw = float(settled["sensors"]["gyro_yaw_deg"])
            turn_to_open_deg = ((chosen_yaw - cur_yaw + 180.0) % 360.0) - 180.0
            if abs(turn_to_open_deg) >= 1.0:
                closed_loop_turn(
                    call_fn,
                    degrees=turn_to_open_deg,
                    max_speed_deg_s=max(75.0, min(95.0, angular_speed_deg_s)),
                    include_vision=False,
                    release_stop=False,
                )
            closed_loop_move(call_fn, meters=0.85, speed_m_s=0.35, include_vision=False, release_stop=False)
            second_sweep = scan_for_target(
                call_fn,
                max_degrees=360.0,
                direction=direction,
                angular_speed_deg_s=angular_speed_deg_s,
                recover_occlusion=False,
            )
            completed_ok = True
            return {
                "target_found": second_sweep["target_found"],
                "scanned_degrees": round(total_scanned_deg + float(second_sweep["scanned_degrees"]), 2),
                "occlusion_recovered": True,
                "after_status": second_sweep["after_status"],
            }

        settled = _stop_and_settle(call_fn, include_vision=True)
        target_found = bool(found and settled["vision"]["target_visible"])
        completed_ok = True
    finally:
        if not completed_ok:
            call_fn("/stop", "POST", {})

    return {
        "target_found": target_found,
        "scanned_degrees": round(total_scanned_deg, 2),
        "occlusion_recovered": False,
        "after_status": _clean_status(settled),
    }


def approach_target(
    call_fn: BridgeCall,
    stop_distance: float = 0.25,
    max_steps: int = 120,
    step_dt: float = 0.15,
) -> dict:
    """Tiếp cận mục tiêu xanh khép kín tầm xa qua hành lang vật cản dựa trên Edge Vision + Rangefinder."""
    _require_finite(stop_distance=stop_distance, max_steps=max_steps, step_dt=step_dt)
    if isinstance(max_steps, bool) or int(max_steps) < 1:
        raise ValueError("Số bước 'max_steps' phải là số nguyên >= 1.")
    if not (0.12 <= stop_distance <= 1.5):
        raise ValueError("Khoảng cách dừng 'stop_distance' phải nằm trong đoạn [0.12, 1.5] mét.")
    if not (0.04 <= step_dt <= 1.0):
        raise ValueError("Chu kỳ điều khiển 'step_dt' phải nằm trong đoạn [0.04, 1.0] giây.")

    ever_seen = False
    lost_streak = 0
    last_offset_x = 0.0
    last_avoid_dir = 0.0
    steps_taken = 0
    min_goal_area = 0.8 / ((stop_distance + 0.20) ** 2)
    watchdog_dur = round(max(0.55, step_dt * 3.5), 3)

    sim_per_wall = 1.0
    has_rate_estimate = False
    prev_iter_wall: float | None = None
    prev_iter_sim: float | None = None
    stopped_cleanly = False

    try:
        for step_idx in range(1, int(max_steps) + 1):
            steps_taken = step_idx
            t_iter_wall = time.perf_counter()
            st = call_fn("/status?vision=1", "GET", None)
            cur_sim_t = float(st.get("sim_time", 0.0))
            if prev_iter_wall is not None and prev_iter_sim is not None:
                dt_w = t_iter_wall - prev_iter_wall
                dt_s = cur_sim_t - prev_iter_sim
                if dt_w > 1e-3 and dt_s > 1e-4:
                    inst_rate = max(0.8, min(10.0, dt_s / dt_w))
                    sim_per_wall = inst_rate if not has_rate_estimate else (0.5 * sim_per_wall + 0.5 * inst_rate)
                    has_rate_estimate = True
            prev_iter_wall = t_iter_wall
            prev_iter_sim = cur_sim_t

            rf = st["sensors"]["rangefinders_m"]
            rf_l, rf_c, rf_r = float(rf["left"]), float(rf["center"]), float(rf["right"])
            vis = st["vision"]
            seen = bool(vis["target_visible"])
            offset_x = float(vis["target"]["offset_x"]) if seen else last_offset_x
            area_pct = float(vis["target"]["area_pct"]) if seen else 0.0
            obstacles = vis.get("obstacles", [])

            closer_center_obs = any(
                float(o["area_pct"]) > max(1.0, area_pct * 1.2) and abs(float(o["offset_x"])) < 0.30
                for o in obstacles
            )
            if seen and not closer_center_obs and area_pct >= min_goal_area and rf_c <= stop_distance + 0.03:
                if abs(offset_x) < 0.20:
                    settled = _stop_and_settle(call_fn, include_vision=True)
                    stopped_cleanly = True
                    return {
                        "reached": True,
                        "reason": "reached_target",
                        "steps": steps_taken,
                        "after_status": settled,
                    }
                # Đã tới sát cự ly dừng nhưng hơi lệch tâm: xoay nhẹ tại chỗ để căn chuẩn tâm đích
                align_w = max(-3.2, min(3.2, 3.0 * offset_x))
                call_fn(
                    "/wheels",
                    "POST",
                    {"left": round(align_w, 4), "right": round(-align_w, 4), "duration": watchdog_dur},
                )
                target_wall_dt = min(0.030, step_dt * 0.8) if not has_rate_estimate else (step_dt * 0.8) / sim_per_wall
                time.sleep(max(0.008, target_wall_dt - (time.perf_counter() - t_iter_wall)))
                continue

            if seen:
                ever_seen = True
                lost_streak = 0
                last_offset_x = offset_x
            else:
                lost_streak += 1
                max_lost = 4 if ever_seen else 1
                if lost_streak > max_lost:
                    settled = _stop_and_settle(call_fn, include_vision=True)
                    stopped_cleanly = True
                    return {
                        "reached": False,
                        "reason": "target_lost",
                        "steps": steps_taken,
                        "after_status": settled,
                    }

            target_is_close_center = (
                seen
                and not closer_center_obs
                and abs(offset_x) < 0.30
                and area_pct >= max(3.2, 0.8 / ((rf_c + 0.15) ** 2))
            )

            if not target_is_close_center and rf_c < 0.16 and min(rf_l, rf_r) < 0.18:
                settled = _stop_and_settle(call_fn, include_vision=True)
                stopped_cleanly = True
                return {
                    "reached": False,
                    "reason": "obstacle_blocked",
                    "steps": steps_taken,
                    "after_status": settled,
                }

            attract_steer = (3.5 * offset_x) if seen else (2.0 * last_offset_x)

            vis_repel = 0.0
            max_obs_threat = 0.0
            if not (target_is_close_center and rf_c < 0.58):
                for obs in obstacles:
                    obs_x = float(obs["offset_x"])
                    obs_area = float(obs["area_pct"])
                    if seen and area_pct > 3.5 and area_pct > obs_area * 1.6:
                        continue
                    if obs_area < 0.70 or abs(obs_x) >= 0.86:
                        continue
                    if abs(obs_x) > 0.12:
                        repel_dir = -1.0 if obs_x > 0.0 else 1.0
                    elif last_avoid_dir != 0.0:
                        repel_dir = last_avoid_dir
                    elif (seen or ever_seen) and abs(offset_x - obs_x) > 0.04:
                        repel_dir = 1.0 if offset_x > obs_x else -1.0
                    else:
                        repel_dir = 1.0 if rf_r >= rf_l else -1.0

                    if abs(obs_x) < 0.55:
                        last_avoid_dir = repel_dir

                    proximity = min(1.25, obs_area / 4.8)
                    centrality = max(0.0, 1.0 - abs(obs_x) / 0.86) ** 1.25
                    threat = proximity * centrality
                    max_obs_threat = max(max_obs_threat, threat)
                    vis_repel += repel_dir * 4.4 * threat

            rf_repel = 0.0
            side_close = False
            if not (seen and offset_x < -0.42 and area_pct > 3.5) and rf_l < 0.65:
                rf_repel += 5.0 * ((0.65 - rf_l) / 0.65)
                side_close = side_close or (rf_l < 0.38)
            if not (seen and offset_x > 0.42 and area_pct > 3.5) and rf_r < 0.65:
                rf_repel -= 5.0 * ((0.65 - rf_r) / 0.65)
                side_close = side_close or (rf_r < 0.38)

            if not target_is_close_center and rf_c < 0.65:
                if abs(vis_repel) > 0.25:
                    c_dir = 1.0 if vis_repel > 0.0 else -1.0
                elif abs(rf_r - rf_l) > 0.04:
                    c_dir = 1.0 if rf_r > rf_l else -1.0
                elif last_avoid_dir != 0.0:
                    c_dir = last_avoid_dir
                else:
                    c_dir = 1.0 if offset_x >= 0.0 else -1.0
                last_avoid_dir = c_dir
                rf_repel += c_dir * 6.0 * ((0.65 - rf_c) / 0.65)

            if max_obs_threat == 0.0 and rf_c >= 0.65 and rf_l >= 0.65 and rf_r >= 0.65:
                last_avoid_dir = 0.0

            opt_dist_m = (
                (math.sqrt(2.35 / max(0.25, area_pct)) - 0.06) if (seen and not closer_center_obs) else 3.0
            )
            approach_dist_m = min(rf_c, opt_dist_m) if (seen and not closer_center_obs) else rf_c

            if not target_is_close_center and rf_c < 0.25:
                pivot_sign = 1.0 if (rf_repel + vis_repel) >= 0.0 else -1.0
                pivot_w = pivot_sign * min(6.0, max(4.2, abs(rf_repel + vis_repel)))
                left_w, right_w = pivot_w, -pivot_w
            elif seen and abs(offset_x) > 0.48 and max_obs_threat < 0.25 and not side_close:
                steer = max(-6.0, min(6.0, attract_steer + vis_repel + rf_repel))
                left_w = 1.6 + steer
                right_w = 1.6 - steer
            else:
                if (seen and not closer_center_obs) and approach_dist_m < stop_distance + 0.45:
                    ramp = max(0.0, min(1.0, (approach_dist_m - stop_distance) / 0.45))
                    desired_speed_m_s = 0.10 + 0.25 * ramp  # Giảm tốc mượt 0.35 -> 0.10 m/s trước khi dừng
                    base_w = desired_speed_m_s / WHEEL_RADIUS
                    max_steer = min(6.5, max(2.2, base_w * 0.95))
                elif not seen:
                    base_w = 4.2
                    max_steer = 6.5
                elif max_obs_threat > 0.45 or side_close or (not target_is_close_center and rf_c < 0.55):
                    base_w = 4.8
                    max_steer = 6.5
                else:
                    base_w = 7.0
                    max_steer = 6.5
                steer = max(-max_steer, min(max_steer, attract_steer + vis_repel + rf_repel))
                left_w = max(-MAX_WHEEL_RAD_S, min(MAX_WHEEL_RAD_S, base_w + steer))
                right_w = max(-MAX_WHEEL_RAD_S, min(MAX_WHEEL_RAD_S, base_w - steer))

            call_fn(
                "/wheels",
                "POST",
                {"left": round(left_w, 4), "right": round(right_w, 4), "duration": watchdog_dur},
            )
            target_wall_dt = min(0.030, step_dt * 0.9) if not has_rate_estimate else (step_dt * 0.9) / sim_per_wall
            time.sleep(max(0.008, target_wall_dt - (time.perf_counter() - t_iter_wall)))

        settled = _stop_and_settle(call_fn, include_vision=True)
        stopped_cleanly = True
    finally:
        if not stopped_cleanly:
            call_fn("/stop", "POST", {})

    vis_end = settled["vision"]
    rf_end = float(settled["sensors"]["rangefinders_m"]["center"])
    goal_ok = bool(
        vis_end["target_visible"]
        and abs(float(vis_end["target"]["offset_x"])) < 0.25
        and float(vis_end["target"]["area_pct"]) >= min_goal_area
        and rf_end <= stop_distance + 0.05
    )
    return {
        "reached": goal_ok,
        "reason": "reached_target" if goal_ok else "max_steps",
        "steps": steps_taken,
        "after_status": settled,
    }
