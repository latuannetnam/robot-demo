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
    max_wait_s: float = 1.5,
) -> dict:
    """Giữ phanh chủ động (0 rad/s, moving=True) cho tới khi thân xe tắt hẳn dao động quán tính."""
    hold_dur = round(max(0.55, min(1.5, float(max_wait_s))), 3)
    call_fn("/wheels", "POST", {"left": 0.0, "right": 0.0, "duration": hold_dur})
    quiet_streak = 0
    deadline = time.monotonic() + max_wait_s
    sleep_s = 0.004 if settle_samples <= 1 else 0.010
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
        if not status.get("moving", False):
            call_fn("/wheels", "POST", {"left": 0.0, "right": 0.0, "duration": hold_dur})
        time.sleep(sleep_s)
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
    tol_deg: float = 0.40,
    timeout_s: float = 8.0,
    include_vision: bool = False,
    release_stop: bool = True,
    fast_settle: bool = False,
) -> dict:
    """Xoay khép kín dựa hoàn toàn trên tích phân con quay hồi chuyển gắn trên xe (chassis_gyro)."""
    _require_finite(degrees=degrees, max_speed_deg_s=max_speed_deg_s, tol_deg=tol_deg, timeout_s=timeout_s)
    if abs(degrees) < 1e-4:
        raise ValueError("Góc xoay 'degrees' phải khác 0.")
    if max_speed_deg_s <= 0.0 or tol_deg <= 0.0 or timeout_s <= 0.0:
        raise ValueError("Các tham số tốc độ, dung sai và timeout của lệnh xoay phải lớn hơn 0.")

    st = call_fn("/status", "GET", None)
    start_gyro_deg = float(st["sensors"]["gyro_yaw_deg"])
    prev_sim_t = float(st.get("sim_time", 0.0))
    eff_dt_sim = 0.14
    accum_deg = 0.0
    watchdog_dur = 0.55
    sleep_s = 0.003 if fast_settle else 0.008
    deadline = time.monotonic() + max(timeout_s, (abs(degrees) / max(10.0, max_speed_deg_s)) * 4.5 + 3.0)

    completed_ok = False
    try:
        while time.monotonic() < deadline:
            remaining = degrees - accum_deg
            yaw_rate_deg_s = math.degrees(float(st["sensors"].get("yaw_rate_rad_s", 0.0)))
            max_exit_yaw_deg_s = 22.0 if fast_settle else 10.0
            if abs(remaining) <= tol_deg and abs(yaw_rate_deg_s) <= max_exit_yaw_deg_s:
                break

            min_deg_s = (14.0 if abs(remaining) > 2.4 * tol_deg else 8.0) if tol_deg >= 0.7 else 4.2
            kp_deg = 3.8 if tol_deg >= 0.7 else 2.5
            lead_rem = remaining - (0.06 + 0.25 * eff_dt_sim) * yaw_rate_deg_s
            if abs(remaining) <= tol_deg:
                # Đang nằm trong vùng dung sai góc nhưng vận tốc góc còn cao: hãm nhẹ về 0
                left_w, right_w = 0.0, 0.0
                pulse_dur = 0.12
            elif lead_rem * remaining < 0.0 and abs(yaw_rate_deg_s) > 25.0:
                # Quán tính hiện tại đủ để đưa góc tới đích: chủ động hãm 0 để tránh lố góc
                left_w, right_w = 0.0, 0.0
                pulse_dur = 0.10
            else:
                deadbeat_cap = abs(remaining) / max(0.08, 1.20 * eff_dt_sim)
                cmd_deg_s = min(max_speed_deg_s, max(min_deg_s, min(kp_deg * abs(lead_rem), deadbeat_cap)))
                omega = min((math.radians(cmd_deg_s) * TRACK_WIDTH) / (2.0 * WHEEL_RADIUS), MAX_WHEEL_RAD_S)
                if lead_rem > 0.0:
                    left_w, right_w = -omega, omega
                else:
                    left_w, right_w = omega, -omega

                pulse_dur = min(
                    watchdog_dur,
                    max(0.03, (0.85 * abs(remaining)) / max(15.0, cmd_deg_s)),
                )
            call_fn(
                "/wheels",
                "POST",
                {"left": round(left_w, 4), "right": round(right_w, 4), "duration": round(pulse_dur, 4)},
            )
            time.sleep(sleep_s)

            st = call_fn("/status", "GET", None)
            cur_sim_t = float(st.get("sim_time", prev_sim_t))
            dt_s = cur_sim_t - prev_sim_t
            if dt_s > 1e-4:
                eff_dt_sim = max(0.75 * eff_dt_sim, min(0.35, dt_s))
                prev_sim_t = cur_sim_t
            accum_deg = float(st["sensors"]["gyro_yaw_deg"]) - start_gyro_deg

        if release_stop:
            settled = _stop_and_settle(call_fn, include_vision=include_vision)
        elif fast_settle:
            settled = _settle_at_zero_hold(
                call_fn,
                include_vision=include_vision,
                speed_tol=0.07,
                yaw_tol=0.18,
                settle_samples=1,
                max_wait_s=0.18,
            )
        else:
            settled = _settle_at_zero_hold(call_fn, include_vision=include_vision)
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
    wall_avoided = False
    sim_per_wall = 1.0
    prev_wall_t = time.perf_counter()

    max_sim_duration = (target_dist / max(0.05, max_speed)) * 2.8 + 1.8
    deadline = time.monotonic() + max(timeout_s, (target_dist / max(0.05, max_speed)) * 5.0 + 3.5)

    completed_ok = False
    try:
        while time.monotonic() < deadline:
            rf = st["sensors"]["rangefinders_m"]
            rf_c = float(rf["center"])
            rf_l = float(rf["left"])
            rf_r = float(rf["right"])
            rf_l90 = float(rf.get("left90", 3.0))
            rf_r90 = float(rf.get("right90", 3.0))
            rf_l120 = float(rf.get("left120", 3.0))
            rf_r120 = float(rf.get("right120", 3.0))
            if direction > 0.0 and safety_stop_m > 0.0 and (
                rf_c < safety_stop_m or min(rf_l, rf_r) < min(safety_stop_m, 0.22)
            ):
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
            obs_cap = 0.18 if (direction > 0.0 and min(rf_c, rf_l, rf_r) < 0.40) else max_speed
            cmd_speed = min(max_speed, accel_cap, decel_cap, obs_cap)
            prev_cmd_speed = cmd_speed
            base_omega = min(cmd_speed / WHEEL_RADIUS, MAX_WHEEL_RAD_S)
            last_pulse_dur = watchdog_dur

            cur_gyro_deg = float(st["sensors"]["gyro_yaw_deg"])
            yaw_rate_deg_s = math.degrees(float(st["sensors"]["yaw_rate_rad_s"]))
            lat_trim = 0.0
            if direction > 0.0:
                if rf_l90 < 0.65 and rf_r90 < 0.65:
                    lat_trim += max(-1.2, min(1.2, 2.2 * (rf_l90 - rf_r90)))
                if rf_l90 < 0.28:
                    lat_trim -= 2.8 * ((0.28 - rf_l90) / 0.28)
                if rf_r90 < 0.28:
                    lat_trim += 2.8 * ((0.28 - rf_r90) / 0.28)
                if rf_l < 0.42:
                    lat_trim -= 2.8 * ((0.42 - rf_l) / 0.42)
                if rf_r < 0.42:
                    lat_trim += 2.8 * ((0.42 - rf_r) / 0.42)
            if abs(lat_trim) > 0.05:
                wall_avoided = True
                start_gyro_deg = cur_gyro_deg
                yaw_int = 0.0

            yaw_err_deg = start_gyro_deg - cur_gyro_deg
            eff_kp = min(kp_yaw, 0.65 / (25.0 * max(0.02, last_dt_sim)))
            speed_scale = cmd_speed / max(0.05, max_speed)
            trim = max(
                -2.6,
                min(2.6, eff_kp * yaw_err_deg + ki_yaw * yaw_int * speed_scale - kd_yaw * yaw_rate_deg_s + lat_trim),
            )

            left_w = max(-MAX_WHEEL_RAD_S, min(MAX_WHEEL_RAD_S, direction * base_omega - trim))
            right_w = max(-MAX_WHEEL_RAD_S, min(MAX_WHEEL_RAD_S, direction * base_omega + trim))

            call_fn(
                "/wheels",
                "POST",
                {"left": round(left_w, 4), "right": round(right_w, 4), "duration": watchdog_dur},
            )
            target_wall_dt = min(0.012, step_dt / max(1.0, sim_per_wall))
            time.sleep(max(0.003, target_wall_dt))

            now_wall_t = time.perf_counter()
            st = call_fn("/status", "GET", None)
            cur_sim_t = float(st["sim_time"])
            cur_fwd = float(st["sensors"]["forward_speed_m_s"])
            dt_sim = max(0.0, cur_sim_t - prev_sim_t)
            dt_wall = max(1e-3, now_wall_t - prev_wall_t)
            if dt_sim > 1e-4:
                sim_per_wall = 0.5 * sim_per_wall + 0.5 * max(1.0, min(12.0, dt_sim / dt_wall))
            prev_wall_t = now_wall_t
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
        if not stopped_early and not wall_avoided and abs(yaw_residual) > 0.45:
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
        rf_l90 = float(rf.get("left90", 3.0))
        rf_r90 = float(rf.get("right90", 3.0))
        min_fwd = min(rf_l, rf_c, rf_r)
        min_side = min(rf_l90, rf_r90)
        min_rf = min(min_fwd, min_side)
        yaw_deg = float(sample["sensors"]["gyro_yaw_deg"])
        if min_rf > best_fallback_min_rf:
            best_fallback_min_rf = min_rf
            best_fallback_yaw_deg = yaw_deg
        if rf_c > 1.4 and min_fwd > 0.45:
            obs_list = sample.get("vision", {}).get("obstacles", [])
            obs_pen = 0.55 * sum(min(1.0, float(o.get("area_pct", 0.0)) / 1.2) for o in obs_list)
            score = min(min_fwd, 0.9) + 0.30 * rf_c + 0.25 * min(min_side, 0.6) - obs_pen
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
            rem_deg = max_degrees - scanned_deg
            yaw_rate_deg_s = abs(math.degrees(float(st["sensors"].get("yaw_rate_rad_s", 0.0))))
            if rem_deg <= max(0.8, 0.045 * yaw_rate_deg_s):
                break

            eff_deg_s = angular_speed_deg_s
            pulse_dur = 0.55
            if rem_deg < 28.0:
                eff_deg_s = min(angular_speed_deg_s, max(18.0, 2.5 * rem_deg))
                pulse_dur = min(0.55, max(0.03, (0.85 * rem_deg) / max(18.0, eff_deg_s)))
            eff_omega = min((math.radians(eff_deg_s) * TRACK_WIDTH) / (2.0 * WHEEL_RADIUS), MAX_WHEEL_RAD_S)
            cur_left_w, cur_right_w = (-eff_omega, eff_omega) if direction == "left" else (eff_omega, -eff_omega)

            call_fn(
                "/wheels",
                "POST",
                {"left": round(cur_left_w, 4), "right": round(cur_right_w, 4), "duration": round(pulse_dur, 4)},
            )
            time.sleep(0.006)
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
            rec_move = closed_loop_move(
                call_fn,
                meters=0.85,
                speed_m_s=0.35,
                safety_stop_m=0.25,
                include_vision=False,
                release_stop=False,
            )
            if rec_move["stopped_early"]:
                closed_loop_move(
                    call_fn,
                    meters=-0.18,
                    speed_m_s=0.25,
                    safety_stop_m=0.0,
                    include_vision=False,
                    release_stop=False,
                )
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


def _estimate_obs_cam_dist_m(obs: dict) -> float:
    """Ước lượng khoảng cách từ front_cam tới bề mặt vật cản màu qua chiều cao ảnh chuẩn hóa (v_span)."""
    v_span = obs.get("v_span")
    if v_span is not None and float(v_span) > 0.04:
        return 0.188 / float(v_span)
    area = max(0.20, float(obs.get("area_pct", 0.20)))
    return max(0.15, math.sqrt(2.35 / area) - 0.12)


def approach_target(
    call_fn: BridgeCall,
    stop_distance: float = 0.25,
    max_steps: int = 160,
    step_dt: float = 0.15,
) -> dict:
    """Tiếp cận mục tiêu xanh khép kín tầm xa qua hành lang vật cản (tĩnh & di động) dựa trên Edge Vision + Rangefinder."""
    _require_finite(stop_distance=stop_distance, max_steps=max_steps, step_dt=step_dt)
    if isinstance(max_steps, bool) or int(max_steps) < 1:
        raise ValueError("Số bước 'max_steps' phải là số nguyên >= 1.")
    if not (0.12 <= stop_distance <= 1.5):
        raise ValueError("Khoảng cách dừng 'stop_distance' phải nằm trong đoạn [0.12, 1.5] mét.")
    if not (0.04 <= step_dt <= 1.0):
        raise ValueError("Chu kỳ điều khiển 'step_dt' phải nằm trong đoạn [0.04, 1.0] giây.")

    ever_seen = False
    lost_streak = 0
    lost_start_odom_m: float | None = None
    last_offset_x = 0.0
    last_avoid_dir = 0.0
    steps_taken = 0
    min_goal_area = 0.8 / ((stop_distance + 0.20) ** 2)
    watchdog_dur = round(max(0.55, step_dt * 3.5), 3)

    prev_obs_state: dict[str, tuple[float, float, float]] = {}
    obs_probe_start: dict[str, tuple[float, float, float]] = {}
    confirmed_dynamic: set[str] = set()
    confirmed_static: set[str] = set()
    initial_scene_checked = False
    init_probe_t0: float | None = None
    init_probe_obs: dict[str, float] = {}
    last_seen_dyn_rail: dict[str, tuple[float, float, float, float]] = {}
    blind_wing_wait_since: dict[str, float] = {}
    commit_until_odom = -1.0
    was_visual_yielding = False
    ever_narrow_corridor = False
    wide_corridor_latched = False

    sim_per_wall = 1.0
    has_rate_estimate = False
    prev_iter_wall: float | None = None
    prev_iter_sim: float | None = None
    stopped_cleanly = False

    try:
        drive_steps = 0
        yield_steps = 0
        max_yield_steps = max(480, int(max_steps) * 4)
        while drive_steps < int(max_steps) and yield_steps < max_yield_steps:
            steps_taken = max(1, drive_steps + 1)
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
            rf_l90 = float(rf.get("left90", 3.0))
            rf_r90 = float(rf.get("right90", 3.0))
            rf_l120 = float(rf.get("left120", 3.0))
            rf_r120 = float(rf.get("right120", 3.0))
            corridor_wall_now = (
                (rf_l90 + rf_r90) < 1.35
                or (rf_l90 < 0.50 and rf_l120 < 0.62)
                or (rf_r90 < 0.50 and rf_r120 < 0.62)
            )
            if corridor_wall_now:
                ever_narrow_corridor = True
            elif min(rf_l90, rf_r90) > 0.75:
                ever_narrow_corridor = False
            in_narrow_corridor = corridor_wall_now or ever_narrow_corridor
            cur_odom_m = float(st["sensors"].get("odom_dist_m", 0.0))
            vis = st["vision"]
            seen = bool(vis["target_visible"])
            offset_x = float(vis["target"]["offset_x"]) if seen else last_offset_x
            area_pct = float(vis["target"]["area_pct"]) if seen else 0.0
            obstacles = vis.get("obstacles", [])
            if not in_narrow_corridor:
                wide_corridor_latched = False
            elif (
                seen
                and abs(offset_x) < 0.28
                and rf_c > 0.80
                and min(rf_l, rf_r) > 0.50
                and (
                    min(rf_l90, rf_r90) >= 0.31
                    or (bool(confirmed_dynamic) and min(rf_l90, rf_r90) >= 0.24 and (rf_l90 + rf_r90) >= 0.72)
                )
            ):
                wide_corridor_latched = True

            closer_center_obs = any(
                float(o["area_pct"]) > max(1.0, area_pct * 1.2) and abs(float(o["offset_x"])) < 0.30
                for o in obstacles
            )
            if seen and not closer_center_obs and area_pct >= min_goal_area and rf_c <= stop_distance + 0.03:
                align_off = offset_x
                if abs(offset_x) < 0.15:
                    settled = _stop_and_settle(call_fn, include_vision=True)
                    vis_s = settled["vision"]
                    if vis_s["target_visible"] and abs(float(vis_s["target"]["offset_x"])) >= 0.16:
                        corr_deg = -math.degrees(math.atan(float(vis_s["target"]["offset_x"]) * FRONT_CAM_TAN_HALF_FOVX))
                        if abs(corr_deg) >= 0.8:
                            settled = closed_loop_turn(
                                call_fn,
                                degrees=corr_deg,
                                max_speed_deg_s=60.0,
                                tol_deg=0.6,
                                include_vision=True,
                                release_stop=True,
                            )["after_status"]
                            vis_s = settled["vision"]
                    rf_s = float(settled["sensors"]["rangefinders_m"]["center"])
                    if (
                        vis_s["target_visible"]
                        and abs(float(vis_s["target"]["offset_x"])) < 0.20
                        and float(vis_s["target"]["area_pct"]) >= min_goal_area
                        and rf_s <= stop_distance + 0.05
                    ):
                        stopped_cleanly = True
                        return {
                            "reached": True,
                            "reason": "reached_target",
                            "steps": steps_taken,
                            "after_status": settled,
                        }
                    if vis_s["target_visible"]:
                        align_off = float(vis_s["target"]["offset_x"])
                # Đã tới sát cự ly dừng nhưng hơi lệch tâm: xoay nhẹ tại chỗ để căn chuẩn tâm đích
                align_w = max(-3.2, min(3.2, 3.0 * align_off))
                call_fn(
                    "/wheels",
                    "POST",
                    {"left": round(align_w, 4), "right": round(-align_w, 4), "duration": watchdog_dur},
                )
                target_wall_dt = min(0.030, step_dt * 0.8) if not has_rate_estimate else (step_dt * 0.8) / sim_per_wall
                time.sleep(max(0.008, target_wall_dt - (time.perf_counter() - t_iter_wall)))
                drive_steps += 1
                steps_taken = drive_steps
                continue

            if seen:
                ever_seen = True
                lost_streak = 0
                lost_start_odom_m = None
                last_offset_x = offset_x
            else:
                lost_streak += 1
                if lost_start_odom_m is None:
                    lost_start_odom_m = cur_odom_m
                max_lost = (
                    75
                    if (ever_seen and (confirmed_dynamic or cur_odom_m < commit_until_odom))
                    else (
                        25
                        if (
                            not ever_seen
                            and (confirmed_dynamic or (not initial_scene_checked and bool(obstacles)))
                        )
                        else (42 if ever_seen else 1)
                    )
                )
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

            # Phân loại vật cản tĩnh/động & nhường đường an toàn trước đường ray tuần tra
            cur_obs_state: dict[str, tuple[float, float, float]] = {}
            yield_brake = False
            visual_yield_now = False
            min_yield_z = 99.0
            fwd_spd = float(st["sensors"].get("forward_speed_m_s", 0.0))
            fwd_spd_abs = abs(fwd_spd)
            yaw_rate = float(st["sensors"].get("yaw_rate_rad_s", 0.0))
            yaw_spd_abs = abs(yaw_rate)
            min_front_rf = min(rf_l, rf_c, rf_r)

            if not initial_scene_checked:
                pocket_piston_corridor = (
                    in_narrow_corridor
                    and seen
                    and abs(offset_x) < 0.25
                    and rf_c > 1.05
                    and min(rf_l, rf_r) > 0.50
                    and min(rf_l90, rf_r90) >= 0.24
                    and (rf_l90 + rf_r90) >= 0.72
                )
                if not obstacles:
                    if pocket_piston_corridor:
                        if init_probe_t0 is None:
                            init_probe_t0 = cur_sim_t
                        elif (cur_sim_t - init_probe_t0) >= 2.2:
                            initial_scene_checked = True
                        if not initial_scene_checked:
                            call_fn("/wheels", "POST", {"left": 0.0, "right": 0.0, "duration": watchdog_dur})
                            time.sleep(0.015)
                            yield_steps += 1
                            continue
                    else:
                        initial_scene_checked = True
                elif in_narrow_corridor:
                    confirmed_dynamic.update(str(o.get("color", "")) for o in obstacles)
                    initial_scene_checked = True
                else:
                    if fwd_spd_abs < 0.04 and yaw_spd_abs < 0.06:
                        cur_map = {str(o.get("color", "")): float(o["offset_x"]) for o in obstacles}
                        if init_probe_t0 is None:
                            init_probe_t0 = cur_sim_t
                            init_probe_obs = cur_map
                        else:
                            common = set(cur_map) & set(init_probe_obs)
                            max_dox = max((abs(cur_map[c] - init_probe_obs[c]) for c in common), default=0.0)
                            sym_diff = set(cur_map) ^ set(init_probe_obs)
                            if max_dox >= 0.015 or (sym_diff and (cur_sim_t - init_probe_t0) >= 0.16):
                                confirmed_dynamic.update(cur_map.keys())
                                confirmed_dynamic.update(init_probe_obs.keys())
                                initial_scene_checked = True
                                yaw0_deg = float(st["sensors"]["gyro_yaw_deg"])

                                def _glance_wing(target_yaw_deg: float) -> dict:
                                    cur_y = float(call_fn("/status", "GET", None)["sensors"]["gyro_yaw_deg"])
                                    delta_deg = _wrap_angle_deg(target_yaw_deg - cur_y)
                                    st_a = (
                                        closed_loop_turn(
                                            call_fn,
                                            degrees=delta_deg,
                                            max_speed_deg_s=220.0,
                                            tol_deg=1.8,
                                            include_vision=True,
                                            release_stop=False,
                                            fast_settle=True,
                                        )["after_status"]
                                        if abs(delta_deg) >= 1.0
                                        else call_fn("/status?vision=1", "GET", None)
                                    )
                                    t_a = float(st_a.get("sim_time", 0.0))
                                    st_b = st_a
                                    w_deadline = time.monotonic() + 1.5
                                    while time.monotonic() < w_deadline and (float(st_b.get("sim_time", t_a)) - t_a) < 0.12:
                                        call_fn("/wheels", "POST", {"left": 0.0, "right": 0.0, "duration": watchdog_dur})
                                        time.sleep(0.012)
                                        st_b = call_fn("/status?vision=1", "GET", None)
                                    d_yaw_rad = math.radians(_wrap_angle_deg(float(st_b["sensors"]["gyro_yaw_deg"]) - yaw0_deg))
                                    map_a = {str(o.get("color", "")): o for o in st_a.get("vision", {}).get("obstacles", [])}
                                    odom_b = float(st_b["sensors"].get("odom_dist_m", 0.0))
                                    t_b = float(st_b.get("sim_time", 0.0))
                                    for o_b in st_b.get("vision", {}).get("obstacles", []):
                                        cn = str(o_b.get("color", ""))
                                        if cn in confirmed_static:
                                            continue
                                        zb = _estimate_obs_cam_dist_m(o_b)
                                        ox_b = float(o_b["offset_x"])
                                        y_cam_b = -ox_b * zb * FRONT_CAM_TAN_HALF_FOVX
                                        fwd_b = zb * math.cos(d_yaw_rad) - y_cam_b * math.sin(d_yaw_rad)
                                        lat_b = zb * math.sin(d_yaw_rad) + y_cam_b * math.cos(d_yaw_rad)
                                        if not (0.18 <= fwd_b <= 1.35):
                                            continue
                                        ox_eq = -lat_b / max(0.20, fwd_b * FRONT_CAM_TAN_HALF_FOVX)
                                        dox_eq = -0.5 * (1.0 if ox_eq >= 0.0 else -1.0)
                                        if cn in map_a:
                                            za = _estimate_obs_cam_dist_m(map_a[cn])
                                            ox_a = float(map_a[cn]["offset_x"])
                                            y_cam_a = -ox_a * za * FRONT_CAM_TAN_HALF_FOVX
                                            lat_a = za * math.sin(d_yaw_rad) + y_cam_a * math.cos(d_yaw_rad)
                                            if abs(lat_b) > abs(lat_a) + 0.015:
                                                dox_eq = 0.5 * (1.0 if ox_eq >= 0.0 else -1.0)
                                        confirmed_dynamic.add(cn)
                                        last_seen_dyn_rail[cn] = (t_b, odom_b + fwd_b, ox_eq, dox_eq)
                                        blind_wing_wait_since.pop(cn, None)
                                    return st_b

                                _glance_wing(yaw0_deg + 26.0)
                                _glance_wing(yaw0_deg - 26.0)
                                cur_y_end = float(call_fn("/status", "GET", None)["sensors"]["gyro_yaw_deg"])
                                back_deg = _wrap_angle_deg(yaw0_deg - cur_y_end)
                                if abs(back_deg) >= 0.8:
                                    closed_loop_turn(
                                        call_fn,
                                        degrees=back_deg,
                                        max_speed_deg_s=200.0,
                                        tol_deg=1.2,
                                        include_vision=True,
                                        release_stop=False,
                                        fast_settle=True,
                                    )
                                call_fn("/wheels", "POST", {"left": 0.0, "right": 0.0, "duration": watchdog_dur})
                                prev_iter_wall = None
                                prev_iter_sim = None
                                if not seen:
                                    lost_streak = max(0, lost_streak - 1)
                                yield_steps += 1
                                continue
                            elif (cur_sim_t - init_probe_t0) >= 0.25:
                                confirmed_static.update(cur_map.keys())
                                initial_scene_checked = True
                    if not initial_scene_checked:
                        call_fn(
                            "/wheels",
                            "POST",
                            {"left": 0.0, "right": 0.0, "duration": watchdog_dur},
                        )
                        time.sleep(0.015)
                        if not seen:
                            lost_streak = max(0, lost_streak - 1)
                        yield_steps += 1
                        continue

            if in_narrow_corridor:
                z_lo, z_hi = 0.15, 0.58
                ox_gate = 1.01
            elif confirmed_dynamic:
                z_lo, z_hi = 0.14, 0.58
                ox_gate = 0.74
            else:
                z_lo, z_hi = 0.18, 0.43
                ox_gate = 0.72
            for obs in obstacles:
                cname = str(obs.get("color", ""))
                ox = float(obs["offset_x"])
                z_est = _estimate_obs_cam_dist_m(obs)
                dox_dt = 0.0
                if cname in prev_obs_state:
                    pt, pox, _pz = prev_obs_state[cname]
                    dt_obs = cur_sim_t - pt
                    if dt_obs > 0.02:
                        dox_dt = (
                            (ox - pox) / dt_obs
                            - (yaw_rate / FRONT_CAM_TAN_HALF_FOVX)
                            - ox * (fwd_spd / max(0.25, z_est))
                        )
                    if (
                        in_narrow_corridor
                        and cname not in confirmed_static
                        and dt_obs > 0.02
                        and yaw_spd_abs < 0.20
                        and abs(dox_dt) >= 0.32
                    ):
                        confirmed_dynamic.add(cname)
                cur_obs_state[cname] = (cur_sim_t, ox, z_est)
                if (
                    (cname in confirmed_dynamic or ((bool(confirmed_dynamic) or in_narrow_corridor) and cname not in confirmed_static))
                    and 0.22 <= z_est <= 1.35
                    and (cname not in blind_wing_wait_since or z_est <= z_hi)
                ):
                    last_seen_dyn_rail[cname] = (cur_sim_t, cur_odom_m + z_est, ox, dox_dt)
                    blind_wing_wait_since.pop(cname, None)

                treat_as_dynamic = (cname in confirmed_dynamic) or (
                    bool(confirmed_dynamic) and not in_narrow_corridor
                )
                inward_edge_obs = (
                    (not in_narrow_corridor)
                    and (0.14 <= z_est <= 0.60)
                    and (abs(ox) <= 1.01)
                    and (yaw_spd_abs < 0.10)
                    and (ox * dox_dt < -0.12)
                )
                if (
                    cname not in confirmed_static
                    and ((z_lo <= z_est <= z_hi) or inward_edge_obs)
                    and cur_odom_m >= commit_until_odom
                ):
                    if not treat_as_dynamic:
                        if abs(ox) < ox_gate or inward_edge_obs:
                            yield_brake = True
                            min_yield_z = min(min_yield_z, z_est)
                            if fwd_spd_abs < 0.04 and yaw_spd_abs < 0.08:
                                if cname not in obs_probe_start:
                                    obs_probe_start[cname] = (cur_sim_t, ox, min_front_rf)
                                else:
                                    t0, ox0, rf0 = obs_probe_start[cname]
                                    if abs(ox - ox0) >= 0.018 or abs(min_front_rf - rf0) >= 0.015:
                                        confirmed_dynamic.add(cname)
                                        visual_yield_now = True
                                    elif (cur_sim_t - t0) >= 0.45:
                                        confirmed_static.add(cname)
                                        yield_brake = False
                    else:
                        if in_narrow_corridor:
                            pred_end = ox + dox_dt * 1.15
                            receding = (
                                (ox * dox_dt > 0.08)
                                and (abs(ox) >= 0.76)
                                and (min_front_rf > 0.60)
                                and (fwd_spd_abs < 0.08)
                                and (yaw_spd_abs < 0.10)
                            )
                            if (ox < 0.0 and 0.20 <= rf_l90 < 0.48) or (ox > 0.0 and 0.20 <= rf_r90 < 0.48):
                                receding = False
                            entering_from_edge = (abs(ox) <= 1.01) and (ox * dox_dt <= 0.05)
                            blocked = (
                                entering_from_edge
                                or (abs(ox) < 0.80)
                                or (abs(pred_end) < 0.78)
                                or (ox * pred_end <= 0.0)
                            )
                        else:
                            pred_end = ox + dox_dt * 1.45
                            receding = (ox * dox_dt > 0.10) and (abs(ox) >= 0.64) and (yaw_spd_abs < 0.08)
                            entering_from_edge = (abs(ox) <= 1.01) and (ox * dox_dt <= 0.05)
                            blocked = (
                                entering_from_edge
                                or (abs(ox) < 0.74)
                                or (abs(pred_end) < 0.74)
                                or (ox * pred_end <= 0.0)
                            )
                        if blocked and not receding:
                            yield_brake = True
                            visual_yield_now = True
                            min_yield_z = min(min_yield_z, z_est)

            if not in_narrow_corridor and confirmed_dynamic and cur_odom_m >= commit_until_odom:
                for cname, (last_t, rail_odom, last_ox, last_dox) in last_seen_dyn_rail.items():
                    if cname in cur_obs_state and cur_obs_state[cname][2] <= z_hi:
                        continue
                    dist_to_rail = rail_odom - cur_odom_m
                    dt_unseen = cur_sim_t - last_t
                    just_receded_outward = (
                        dt_unseen <= 1.35
                        and abs(last_ox) <= 1.05
                        and (
                            (last_ox * last_dox > 0.08)
                            or (was_visual_yielding and abs(last_ox) >= 0.78 and (last_ox * last_dox) >= -0.05)
                        )
                    )
                    if 0.18 <= dist_to_rail <= 0.50 and not just_receded_outward and dt_unseen <= 12.0:
                        w0 = blind_wing_wait_since.setdefault(cname, cur_sim_t)
                        if (cur_sim_t - w0) <= 8.5:
                            yield_brake = True

            if (
                confirmed_dynamic
                and bool(obstacles)
                and not target_is_close_center
                and not (seen and not closer_center_obs and area_pct >= 3.5)
                and cur_odom_m >= commit_until_odom
                and min_front_rf < 0.42
            ):
                yield_brake = True
                visual_yield_now = True

            if not ever_seen and not seen and not in_narrow_corridor and confirmed_dynamic:
                yield_brake = True

            if was_visual_yielding and not yield_brake and confirmed_dynamic:
                commit_until_odom = cur_odom_m + (0.82 if in_narrow_corridor else 0.72)
            was_visual_yielding = visual_yield_now if yield_brake else False
            prev_obs_state = cur_obs_state

            if yield_brake:
                if min_front_rf < 0.28 or min_yield_z < 0.26:
                    left_w, right_w = -4.5, -4.5
                elif confirmed_dynamic:
                    left_w, right_w = 0.0, 0.0
                else:
                    if in_narrow_corridor:
                        align = max(-1.2, min(1.2, 1.8 * offset_x)) if seen else 0.0
                    else:
                        align = max(-1.8, min(1.8, 2.2 * offset_x)) if seen else 0.0
                    left_w, right_w = align, -align
                call_fn(
                    "/wheels",
                    "POST",
                    {"left": round(left_w, 4), "right": round(right_w, 4), "duration": watchdog_dur},
                )
                target_wall_dt = min(0.025, step_dt * 0.65) if not has_rate_estimate else (step_dt * 0.65) / sim_per_wall
                time.sleep(max(0.006, target_wall_dt - (time.perf_counter() - t_iter_wall)))
                if not seen and confirmed_dynamic:
                    lost_streak = max(0, lost_streak - 1)
                yield_steps += 1
                continue

            drive_steps += 1
            steps_taken = drive_steps

            if not confirmed_dynamic and not target_is_close_center and ((rf_c < 0.17 and min(rf_l, rf_r) < 0.20) or min(rf_c, rf_l, rf_r) < 0.15):
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
            if (
                not (target_is_close_center and rf_c < 0.58)
                and not confirmed_dynamic
                and cur_odom_m >= commit_until_odom
                and not in_narrow_corridor
            ):
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
            wall_corner = min(rf_l, rf_r) < 0.28 and min(rf_l90, rf_r90) < 0.32
            ignore_open_dyn_flanks = bool(confirmed_dynamic) and not in_narrow_corridor and not wall_corner
            ignore_diag_piston = in_narrow_corridor and rf_c > 0.65 and (
                bool(confirmed_dynamic) or cur_odom_m < commit_until_odom or bool(obstacles)
            )
            if ignore_diag_piston:
                if rf_l < 0.45:
                    rf_repel += 2.5 * ((0.45 - rf_l) / 0.45)
                if rf_r < 0.45:
                    rf_repel -= 2.5 * ((0.45 - rf_r) / 0.45)
            elif not ignore_open_dyn_flanks:
                if not (seen and offset_x < -0.42 and area_pct > 3.5) and rf_l < 0.65:
                    rf_repel += 5.0 * ((0.65 - rf_l) / 0.65)
                    side_close = side_close or (rf_l < 0.38)
                if not (seen and offset_x > 0.42 and area_pct > 3.5) and rf_r < 0.65:
                    rf_repel -= 5.0 * ((0.65 - rf_r) / 0.65)
                    side_close = side_close or (rf_r < 0.38)
            if not (target_is_close_center and rf_c < 0.45) and not ignore_open_dyn_flanks:
                l_flank = min(rf_l90, rf_l120 + 0.04)
                r_flank = min(rf_r90, rf_r120 + 0.04)
                if l_flank < 0.38:
                    rf_repel += 6.5 * ((0.38 - l_flank) / 0.38)
                    if l_flank < 0.23:
                        rf_repel += 6.0 * ((0.23 - l_flank) / 0.23)
                    side_close = side_close or (l_flank < 0.28)
                if r_flank < 0.38:
                    rf_repel -= 6.5 * ((0.38 - r_flank) / 0.38)
                    if r_flank < 0.23:
                        rf_repel -= 6.0 * ((0.23 - r_flank) / 0.23)
                    side_close = side_close or (r_flank < 0.28)
                l_wall_d = min(rf_l90, rf_l120 + 0.08)
                r_wall_d = min(rf_r90, rf_r120 + 0.08)
                if l_wall_d < 0.31 and attract_steer < 0.0:
                    attract_steer *= max(0.0, (l_wall_d - 0.21) / 0.10)
                    side_close = True
                if r_wall_d < 0.31 and attract_steer > 0.0:
                    attract_steer *= max(0.0, (r_wall_d - 0.21) / 0.10)
                    side_close = True

            if (
                wide_corridor_latched
                and rf_c > 0.65
                and abs(offset_x) < 0.28
                and min(rf_l, rf_r) >= 0.28
                and min(rf_l90, rf_r90) >= 0.22
            ):
                rf_repel = max(-2.4, min(2.4, rf_repel))

            if (
                not target_is_close_center
                and rf_c < 0.65
                and not (confirmed_dynamic or cur_odom_m < commit_until_odom)
            ):
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

            if not target_is_close_center and not confirmed_dynamic and (rf_c < 0.26 or min(rf_l, rf_r) < 0.21):
                pivot_sign = 1.0 if (rf_repel + vis_repel) >= 0.0 else -1.0
                pivot_w = pivot_sign * min(6.0, max(4.2, abs(rf_repel + vis_repel)))
                left_w, right_w = pivot_w, -pivot_w
            elif seen and abs(offset_x) > 0.48 and max_obs_threat < 0.25 and not side_close and cur_odom_m >= commit_until_odom:
                steer = max(-6.0, min(6.0, attract_steer + vis_repel + rf_repel))
                left_w = 1.6 + steer
                right_w = 1.6 - steer
            else:
                straddling_dyn_rail = (
                    (bool(confirmed_dynamic) or (in_narrow_corridor and bool(last_seen_dyn_rail)))
                    and any(-0.38 <= (ro - cur_odom_m) <= 0.18 for _, ro, _, _ in last_seen_dyn_rail.values())
                )
                if (
                    (seen and not closer_center_obs)
                    and cur_odom_m >= commit_until_odom
                    and (not straddling_dyn_rail or approach_dist_m < stop_distance + 0.14)
                    and approach_dist_m < stop_distance + 0.45
                ):
                    ramp = max(0.0, min(1.0, (approach_dist_m - stop_distance) / 0.45))
                    desired_speed_m_s = 0.10 + 0.25 * ramp  # Giảm tốc mượt 0.35 -> 0.10 m/s trước khi dừng
                    base_w = desired_speed_m_s / WHEEL_RADIUS
                    max_steer = min(6.5, max(2.2, base_w * 0.95))
                elif (
                    wide_corridor_latched
                    and seen
                    and not closer_center_obs
                    and approach_dist_m >= stop_distance + 0.45
                    and abs(offset_x) < 0.28
                    and rf_c > 0.55
                ):
                    base_w = 13.0
                    max_steer = 3.2
                elif cur_odom_m < commit_until_odom or (confirmed_dynamic and (seen or lost_streak <= 6)):
                    base_w = 13.5
                    max_steer = 3.5
                elif not seen:
                    odom_since_lost = cur_odom_m - (lost_start_odom_m if lost_start_odom_m is not None else cur_odom_m)
                    if last_offset_x >= 0.0:
                        target_side_clear = rf_r >= 0.48 and rf_r90 >= 0.36 and rf_r120 >= 0.34
                    else:
                        target_side_clear = rf_l >= 0.48 and rf_l90 >= 0.36 and rf_l120 >= 0.34
                    if confirmed_dynamic and not in_narrow_corridor:
                        base_w = 6.5
                        attract_steer = 1.2 * last_offset_x
                        max_steer = 2.5
                    elif lost_streak >= 4 and odom_since_lost >= 0.45 and max_obs_threat < 0.15 and target_side_clear:
                        base_w = 1.6 if rf_c > 0.50 else 0.3
                        reacq_sign = 1.0 if last_offset_x >= 0.0 else -1.0
                        attract_steer = reacq_sign * max(3.2, 4.0 * abs(last_offset_x))
                        max_steer = 6.5
                    else:
                        base_w = 4.2
                        attract_steer = 2.0 * last_offset_x
                        max_steer = 6.5
                elif not (target_is_close_center and rf_c < 0.45) and (
                    min(rf_l90, rf_r90) < 0.26 or min(rf_l120, rf_r120) < 0.21
                ):
                    base_w = 3.8
                    max_steer = 6.5
                elif max_obs_threat > 0.45 or side_close or (not target_is_close_center and rf_c < 0.55):
                    base_w = 4.8
                    max_steer = 6.5
                else:
                    base_w = 7.0
                    max_steer = 6.5
                near_uncommitted_obs = cur_odom_m >= commit_until_odom and any(
                    str(o.get("color", "")) not in confirmed_static
                    and _estimate_obs_cam_dist_m(o) <= 0.68
                    for o in obstacles
                )
                if confirmed_dynamic and near_uncommitted_obs:
                    base_w = min(base_w, 8.0)
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


def _wrap_angle_deg(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


def explore_maze(
    call_fn: BridgeCall,
    rule: str = "right",
    max_steps: int = 850,
    speed_m_s: float = 0.64,
    step_dt: float = 0.10,
) -> dict:
    """Tự hành dò mê cung khép kín 20Hz theo quy tắc bám tường phải/trái, căn giữa hành lang và dừng ngay khi thấy đích."""
    _require_finite(max_steps=max_steps, speed_m_s=speed_m_s, step_dt=step_dt)
    if rule not in ("right", "left"):
        raise ValueError("Quy tắc bám tường 'rule' phải là 'right' hoặc 'left'.")
    if isinstance(max_steps, bool) or int(max_steps) < 1:
        raise ValueError("Số bước 'max_steps' phải là số nguyên >= 1.")
    if speed_m_s <= 0.0:
        raise ValueError("Tốc độ 'speed_m_s' phải lớn hơn 0.")
    if not (0.03 <= step_dt <= 0.5):
        raise ValueError("Chu kỳ điều khiển 'step_dt' phải nằm trong đoạn [0.03, 0.5] giây.")

    pref_is_right = rule == "right"
    pref_turn_deg = -90.0 if pref_is_right else 90.0
    other_turn_deg = -pref_turn_deg
    max_speed = min(abs(float(speed_m_s)), MAX_WHEEL_RAD_S * WHEEL_RADIUS)
    watchdog_dur = round(max(0.34, step_dt * 3.2), 3)

    st = call_fn("/status?vision=1", "GET", None)
    corridor_yaw_deg = float(st["sensors"]["gyro_yaw_deg"])
    base_yaw_deg = corridor_yaw_deg
    prev_gyro_yaw_deg = corridor_yaw_deg
    has_hw_odom = "odom_dist_m" in st["sensors"]
    has_hw_odom_xy = ("odom_x_m" in st["sensors"]) and ("odom_y_m" in st["sensors"])
    odom_m = float(st["sensors"].get("odom_dist_m", 0.0))
    odom_x = float(st["sensors"].get("odom_x_m", 0.0))
    odom_y = float(st["sensors"].get("odom_y_m", 0.0))
    last_turn_odom_m = odom_m - 0.50
    saw_pref_wall = False
    wall_edge_odom_m = odom_m
    open_branch_streak = 0
    saw_other_wall = False
    other_wall_edge_odom_m = odom_m
    other_branch_streak = 0
    branch_turn_odom_m: float | None = None
    turns_taken = 0
    steps_taken = 0

    sim_per_wall = 1.0
    has_rate_estimate = False
    prev_iter_wall: float | None = None
    prev_iter_sim: float | None = None
    prev_sim_t = float(st.get("sim_time", 0.0))
    stopped_cleanly = False

    breadcrumbs: list[tuple[float, float, float, int]] = []
    last_crumb_odom = -999.0
    loop_mode = False
    gate_commit_until_odom = -1.0
    ignore_static_obs_until_odom = -1.0
    gates_yielded = 0

    def _heading_idx(yaw_deg: float) -> int:
        return int(round((yaw_deg - base_yaw_deg) / 90.0)) % 4

    def _sync_odom(cur_st: dict) -> None:
        nonlocal odom_m, odom_x, odom_y, prev_sim_t, prev_gyro_yaw_deg, last_crumb_odom, loop_mode
        cur_sim = float(cur_st.get("sim_time", prev_sim_t))
        dt_s = max(0.0, cur_sim - prev_sim_t)
        prev_sim_t = cur_sim
        sens = cur_st["sensors"]
        cur_gyro = float(sens.get("gyro_yaw_deg", corridor_yaw_deg))
        prev_odom = odom_m
        if has_hw_odom and "odom_dist_m" in sens:
            odom_m = float(sens["odom_dist_m"])
            delta_d = odom_m - prev_odom
        else:
            fwd_v = float(sens.get("forward_speed_m_s", 0.0))
            delta_d = fwd_v * dt_s
            odom_m += abs(delta_d)
        if has_hw_odom_xy and "odom_x_m" in sens and "odom_y_m" in sens:
            odom_x = float(sens["odom_x_m"])
            odom_y = float(sens["odom_y_m"])
        else:
            mid_yaw_rad = math.radians(0.5 * (prev_gyro_yaw_deg + cur_gyro))
            odom_x += delta_d * math.cos(mid_yaw_rad)
            odom_y += delta_d * math.sin(mid_yaw_rad)
        prev_gyro_yaw_deg = cur_gyro

        h_idx = _heading_idx(corridor_yaw_deg)
        if not loop_mode:
            for bx, by, bodom, bhead in breadcrumbs:
                if bhead == h_idx and (odom_m - bodom) >= 2.2 and math.hypot(odom_x - bx, odom_y - by) <= 0.28:
                    loop_mode = True
                    break
        if (odom_m - last_crumb_odom) >= 0.14:
            breadcrumbs.append((odom_x, odom_y, odom_m, h_idx))
            last_crumb_odom = odom_m

    def _probe_xy(
        delta_deg: float,
        dist_m: float = 0.76,
        origin_xy: tuple[float, float] | None = None,
    ) -> tuple[float, float]:
        ox, oy = (odom_x, odom_y) if origin_xy is None else origin_xy
        ang = math.radians(corridor_yaw_deg + delta_deg)
        return ox + dist_m * math.cos(ang), oy + dist_m * math.sin(ang)

    def _min_crumb_dist(px: float, py: float) -> float:
        if not breadcrumbs:
            return 99.0
        return min(math.hypot(px - bx, py - by) for bx, by, _, _ in breadcrumbs)

    def _dir_visit_count(px: float, py: float, delta_deg: float) -> int:
        target_h = _heading_idx(corridor_yaw_deg + delta_deg)
        cnt = 0
        last_matched_odom = -999.0
        for bx, by, bodom, bhead in breadcrumbs:
            if bhead == target_h and (bodom - last_matched_odom) >= 1.4 and math.hypot(px - bx, py - by) <= 0.36:
                cnt += 1
                last_matched_odom = bodom
        return cnt

    def _option_rank(delta_deg: float, tie_prio: int) -> tuple[int, int, int]:
        px, py = _probe_xy(delta_deg, 0.76)
        unvisited = 1 if _min_crumb_dist(px, py) >= 0.44 else 0
        d_visits = _dir_visit_count(px, py, delta_deg)
        return (unvisited, -d_visits, tie_prio)

    _sync_odom(st)

    def _fast_settle(include_vision: bool = True) -> dict:
        settled_st = _settle_at_zero_hold(
            call_fn,
            include_vision=include_vision,
            speed_tol=0.07,
            yaw_tol=0.18,
            settle_samples=1,
            max_wait_s=0.18,
        )
        _sync_odom(settled_st)
        return settled_st

    def _finish_target_seen(cur_vis: dict) -> dict | None:
        last_off = float(cur_vis["target"]["offset_x"])
        settled_end = _stop_and_settle(call_fn, include_vision=True)
        _sync_odom(settled_end)
        vis_end = settled_end["vision"]
        need_recenter = (not vis_end["target_visible"]) or (
            abs(float(vis_end["target"]["offset_x"])) > 0.55
        )
        if need_recenter:
            aim_off = float(vis_end["target"]["offset_x"]) if vis_end["target_visible"] else last_off
            rec_deg = -math.degrees(math.atan(aim_off * FRONT_CAM_TAN_HALF_FOVX))
            if abs(rec_deg) >= 0.8:
                settled_end = closed_loop_turn(
                    call_fn,
                    degrees=rec_deg,
                    max_speed_deg_s=75.0,
                    tol_deg=0.6,
                    include_vision=True,
                    release_stop=True,
                )["after_status"]
                _sync_odom(settled_end)
        if gates_yielded > 0 or bool(settled_end["vision"].get("obstacles")):
            settled_end = _apply_asymmetric_shaft_tuck(call_fn, settled_end, re_center_target=True)
            _sync_odom(settled_end)
        if settled_end["vision"]["target_visible"]:
            return {
                "target_found": True,
                "reason": "target_visible",
                "rule": rule,
                "steps": steps_taken,
                "turns": turns_taken,
                "loop_escaped": bool(loop_mode),
                "gates_yielded": int(gates_yielded),
                "after_status": _clean_status(settled_end),
            }
        return None

    def _maybe_finish_target(cur_st: dict) -> dict | None:
        cur_vis = cur_st.get("vision", {})
        if not cur_vis.get("target_visible"):
            return None
        cur_rf = cur_st["sensors"]["rangefinders_m"]
        t_off = abs(float(cur_vis["target"]["offset_x"]))
        t_area = float(cur_vis["target"]["area_pct"])
        if t_off >= 0.72 and t_area < 0.85:
            return None
        c_dist = float(cur_rf["center"])
        flank_min = min(float(cur_rf.get("left90", 3.0)), float(cur_rf.get("right90", 3.0)))
        cur_obs = cur_vis.get("obstacles", [])
        fg_obs = bool(cur_obs) and (
            c_dist < 0.85
            or any(_estimate_obs_cam_dist_m(o) < 0.85 for o in cur_obs)
        )
        # Do not mistake a foreground moving piston + distant peek in a narrow corridor for clear sight of target.
        if fg_obs and t_area < 0.85 and flank_min < 0.50:
            return None
        effective_c_ok = (c_dist < 0.80) and not (cur_obs and t_area < 0.85 and flank_min < 0.50)
        if (
            (t_off < 0.32 and t_area >= 0.32)
            or t_area >= 2.8
            or (t_area >= 0.38 and (effective_c_ok or flank_min >= 0.50))
        ):
            return _finish_target_seen(cur_vis)
        return None

    def _aligned_turn(cur_st: dict, nominal_deg: float) -> dict:
        nonlocal corridor_yaw_deg, turns_taken, last_turn_odom_m, wall_edge_odom_m, other_wall_edge_odom_m
        nonlocal saw_pref_wall, open_branch_streak, saw_other_wall, other_branch_streak, branch_turn_odom_m
        corridor_yaw_deg = corridor_yaw_deg + nominal_deg
        cur_yaw = float(cur_st["sensors"]["gyro_yaw_deg"])
        turn_delta = _wrap_angle_deg(corridor_yaw_deg - cur_yaw)
        if abs(nominal_deg) >= 170.0 and abs(turn_delta) >= 150.0 and (turn_delta * nominal_deg) < 0.0:
            turn_delta = nominal_deg
        if abs(turn_delta) >= 0.8:
            cur_st = closed_loop_turn(
                call_fn,
                degrees=turn_delta,
                max_speed_deg_s=240.0,
                tol_deg=1.4,
                include_vision=True,
                release_stop=False,
                fast_settle=True,
            )["after_status"]
        turns_taken += 1
        _sync_odom(cur_st)
        breadcrumbs.append((odom_x, odom_y, odom_m, _heading_idx(corridor_yaw_deg)))
        last_turn_odom_m = odom_m
        wall_edge_odom_m = odom_m
        other_wall_edge_odom_m = odom_m
        saw_pref_wall = False
        open_branch_streak = 0
        saw_other_wall = False
        other_branch_streak = 0
        branch_turn_odom_m = None
        return cur_st

    def _check_and_yield_dynamic_gate(
        cur_st: dict, *, already_settled: bool = False
    ) -> tuple[bool, dict | None, dict]:
        nonlocal gate_commit_until_odom, ignore_static_obs_until_odom, gates_yielded
        settled_gate = cur_st if already_settled else _fast_settle(include_vision=True)
        done_now = _maybe_finish_target(settled_gate)
        if done_now is not None:
            return True, done_now, settled_gate

        def _obs_sig(vis_dict: dict) -> dict[str, float]:
            return {
                str(o.get("color", "")): float(o["offset_x"])
                for o in vis_dict.get("obstacles", [])
            }

        t0 = float(settled_gate.get("sim_time", 0.0))
        obs0 = _obs_sig(settled_gate.get("vision", {}))
        rf0_dict = settled_gate["sensors"]["rangefinders_m"]
        rf0_min = min(float(rf0_dict["left"]), float(rf0_dict["center"]), float(rf0_dict["right"]))
        is_dynamic = False
        probe_st = settled_gate
        probe_deadline = time.monotonic() + 2.5

        while time.monotonic() < probe_deadline:
            call_fn("/wheels", "POST", {"left": 0.0, "right": 0.0, "duration": 0.35})
            time.sleep(0.015)
            probe_st = call_fn("/status?vision=1", "GET", None)
            _sync_odom(probe_st)
            done_now = _maybe_finish_target(probe_st)
            if done_now is not None:
                return True, done_now, probe_st

            obs_cur = _obs_sig(probe_st.get("vision", {}))
            rf_cur = probe_st["sensors"]["rangefinders_m"]
            rf_cur_min = min(float(rf_cur["left"]), float(rf_cur["center"]), float(rf_cur["right"]))
            if set(obs_cur.keys()) != set(obs0.keys()):
                is_dynamic = True
                break
            if any(abs(obs_cur[k] - obs0[k]) >= 0.018 for k in obs_cur):
                is_dynamic = True
                break
            if abs(rf_cur_min - rf0_min) >= 0.015:
                is_dynamic = True
                break
            if (float(probe_st.get("sim_time", t0)) - t0) >= 0.42:
                break

        if not is_dynamic:
            ignore_static_obs_until_odom = odom_m + 0.35
            return False, None, probe_st

        gates_yielded += 1
        rf_p = probe_st["sensors"]["rangefinders_m"]
        p_obs = probe_st.get("vision", {}).get("obstacles", [])
        if min(float(rf_p["left"]), float(rf_p["center"]), float(rf_p["right"])) < 0.34 or any(
            _estimate_obs_cam_dist_m(o) < 0.26 for o in p_obs
        ):
            probe_st = closed_loop_move(
                call_fn,
                meters=-0.10,
                speed_m_s=0.25,
                safety_stop_m=0.0,
                include_vision=True,
                release_stop=False,
            )["after_status"]
            _sync_odom(probe_st)
            done_now = _maybe_finish_target(probe_st)
            if done_now is not None:
                return True, done_now, probe_st

        wait_start_sim = float(probe_st.get("sim_time", 0.0))
        wait_deadline = time.monotonic() + 20.0
        clear_streak = 0
        last_clear_sim_t = -999.0
        wait_st = probe_st
        prev_w_obs = _obs_sig(probe_st.get("vision", {}))

        while time.monotonic() < wait_deadline and (float(wait_st.get("sim_time", wait_start_sim)) - wait_start_sim) < 10.5:
            call_fn("/wheels", "POST", {"left": 0.0, "right": 0.0, "duration": 0.35})
            time.sleep(0.015)
            wait_st = call_fn("/status?vision=1", "GET", None)
            _sync_odom(wait_st)
            done_now = _maybe_finish_target(wait_st)
            if done_now is not None:
                return True, done_now, wait_st

            w_rf = wait_st["sensors"]["rangefinders_m"]
            w_obs = wait_st.get("vision", {}).get("obstacles", [])
            cur_w_obs = _obs_sig(wait_st.get("vision", {}))
            obs_blocking_gate = False
            nearest_gate_z = 0.45
            for o in w_obs:
                z_o = _estimate_obs_cam_dist_m(o)
                if z_o <= 0.72:
                    nearest_gate_z = min(nearest_gate_z, z_o)
                    cn = str(o.get("color", ""))
                    ox_w = float(o["offset_x"])
                    if abs(ox_w) < 0.78:
                        obs_blocking_gate = True
                        break
                    if cn in prev_w_obs:
                        d_ox = ox_w - prev_w_obs[cn]
                        if (ox_w * d_ox) <= 0.004:
                            obs_blocking_gate = True
                            break
                    else:
                        obs_blocking_gate = True
                        break
            prev_w_obs = cur_w_obs
            gate_clear = (
                float(w_rf["center"]) > 0.78
                and min(float(w_rf["left"]), float(w_rf["right"])) > 0.38
                and not obs_blocking_gate
            )
            cur_w_sim = float(wait_st.get("sim_time", wait_start_sim))
            if gate_clear:
                if clear_streak == 0 or (cur_w_sim - last_clear_sim_t) >= 0.08:
                    clear_streak += 1
                    last_clear_sim_t = cur_w_sim
                if clear_streak >= 2:
                    gate_commit_until_odom = odom_m + min(1.05, max(0.70, nearest_gate_z + 0.48))
                    return True, None, wait_st
            else:
                clear_streak = 0

        gate_commit_until_odom = odom_m + 0.50
        return True, None, wait_st

    try:
        for step_idx in range(1, int(max_steps) + 1):
            steps_taken = step_idx
            t_iter_wall = time.perf_counter()
            if step_idx > 1:
                st = call_fn("/status?vision=1", "GET", None)
                _sync_odom(st)
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
            rf_l = float(rf["left"])
            rf_c = float(rf["center"])
            rf_r = float(rf["right"])
            rf_l90 = float(rf.get("left90", 3.0))
            rf_r90 = float(rf.get("right90", 3.0))
            rf_l120 = float(rf.get("left120", 3.0))
            rf_r120 = float(rf.get("right120", 3.0))
            has_120 = ("right120" in rf) and ("left120" in rf)

            done = _maybe_finish_target(st)
            if done is not None:
                stopped_cleanly = True
                return done

            if odom_m >= gate_commit_until_odom and odom_m >= ignore_static_obs_until_odom:
                early_dyn_obs = any(
                    0.16 <= _estimate_obs_cam_dist_m(o) <= 0.65 and abs(float(o["offset_x"])) < 1.01
                    for o in st.get("vision", {}).get("obstacles", [])
                )
                if early_dyn_obs:
                    handled_dyn, done_dyn, st = _check_and_yield_dynamic_gate(st, already_settled=False)
                    if done_dyn is not None:
                        stopped_cleanly = True
                        return done_dyn
                    if handled_dyn:
                        continue

            pref_90 = rf_r90 if pref_is_right else rf_l90
            other_90 = rf_l90 if pref_is_right else rf_r90
            pref_120 = rf_r120 if pref_is_right else rf_l120
            other_120 = rf_l120 if pref_is_right else rf_r120
            since_turn_m = odom_m - last_turn_odom_m

            if since_turn_m >= 0.18:
                if pref_90 < 0.65:
                    saw_pref_wall = True
                    wall_edge_odom_m = max(wall_edge_odom_m, odom_m + 0.10)
                if has_120 and pref_120 < 0.72:
                    saw_pref_wall = True
                    wall_edge_odom_m = max(wall_edge_odom_m, odom_m + 0.06 - 0.5 * pref_120)
                if other_90 < 0.65:
                    saw_other_wall = True
                    other_wall_edge_odom_m = max(other_wall_edge_odom_m, odom_m + 0.10)
                if has_120 and other_120 < 0.72:
                    saw_other_wall = True
                    other_wall_edge_odom_m = max(other_wall_edge_odom_m, odom_m + 0.06 - 0.5 * other_120)

            front_blocked = (rf_c < 0.42) or (min(rf_l, rf_r) < 0.30)
            if front_blocked:
                branch_turn_odom_m = None
                settled = _fast_settle(include_vision=True)
                if settled["vision"].get("target_visible"):
                    s_obs = settled["vision"].get("obstacles", [])
                    s_t_area = float(settled["vision"]["target"]["area_pct"])
                    if s_obs and s_t_area < 0.85:
                        done = _maybe_finish_target(settled)
                    else:
                        done = _finish_target_seen(settled["vision"])
                    if done is not None:
                        stopped_cleanly = True
                        return done

                if (
                    odom_m >= gate_commit_until_odom
                    and odom_m >= ignore_static_obs_until_odom
                    and settled.get("vision", {}).get("obstacles")
                ):
                    handled_dyn, done_dyn, settled = _check_and_yield_dynamic_gate(settled, already_settled=True)
                    if done_dyn is not None:
                        stopped_cleanly = True
                        return done_dyn
                    if handled_dyn:
                        st = settled
                        continue

                rf = settled["sensors"]["rangefinders_m"]
                rf_l = float(rf["left"])
                rf_c = float(rf["center"])
                rf_r = float(rf["right"])
                rf_l90 = float(rf.get("left90", 3.0))
                rf_r90 = float(rf.get("right90", 3.0))
                pref_90 = rf_r90 if pref_is_right else rf_l90
                other_90 = rf_l90 if pref_is_right else rf_r90

                if rf_c < 0.25 or min(rf_l, rf_r) < 0.20:
                    settled = closed_loop_move(
                        call_fn,
                        meters=-0.08,
                        speed_m_s=0.26,
                        safety_stop_m=0.0,
                        include_vision=True,
                        release_stop=False,
                    )["after_status"]
                    _sync_odom(settled)
                    rf = settled["sensors"]["rangefinders_m"]
                    rf_l = float(rf["left"])
                    rf_c = float(rf["center"])
                    rf_r = float(rf["right"])
                    rf_l90 = float(rf.get("left90", 3.0))
                    rf_r90 = float(rf.get("right90", 3.0))
                    pref_90 = rf_r90 if pref_is_right else rf_l90
                    other_90 = rf_l90 if pref_is_right else rf_r90

                if has_hw_odom and (pref_90 > 0.52 or other_90 > 0.52) and rf_c > 0.34 and min(rf_l, rf_r) > 0.31:
                    nudge_target_odom = float(settled["sensors"].get("odom_dist_m", odom_m)) + min(0.06, rf_c - 0.30)
                    nudge_deadline = time.monotonic() + 1.5
                    while time.monotonic() < nudge_deadline:
                        n_rf = settled["sensors"]["rangefinders_m"]
                        n_odom = float(settled["sensors"].get("odom_dist_m", odom_m))
                        if (
                            n_odom >= nudge_target_odom - 0.008
                            or float(n_rf["center"]) <= 0.29
                            or min(float(n_rf["left"]), float(n_rf["right"])) <= 0.26
                        ):
                            break
                        n_yaw_err = _wrap_angle_deg(corridor_yaw_deg - float(settled["sensors"]["gyro_yaw_deg"]))
                        n_trim = max(-1.2, min(1.2, 0.22 * n_yaw_err))
                        call_fn(
                            "/wheels",
                            "POST",
                            {"left": round(4.4 - n_trim, 4), "right": round(4.4 + n_trim, 4), "duration": 0.14},
                        )
                        time.sleep(0.015)
                        settled = call_fn("/status?vision=1", "GET", None)
                        _sync_odom(settled)
                        if settled["vision"].get("target_visible"):
                            done = _finish_target_seen(settled["vision"])
                            if done is not None:
                                stopped_cleanly = True
                                return done
                    settled = _fast_settle(include_vision=True)
                    if settled["vision"].get("target_visible"):
                        done = _finish_target_seen(settled["vision"])
                        if done is not None:
                            stopped_cleanly = True
                            return done
                    rf = settled["sensors"]["rangefinders_m"]
                    rf_l90 = float(rf.get("left90", 3.0))
                    rf_r90 = float(rf.get("right90", 3.0))
                    pref_90 = rf_r90 if pref_is_right else rf_l90
                    other_90 = rf_l90 if pref_is_right else rf_r90

                if loop_mode and pref_90 > 0.52 and other_90 > 0.52:
                    chosen_deg = (
                        pref_turn_deg
                        if _option_rank(pref_turn_deg, 2) >= _option_rank(other_turn_deg, 1)
                        else other_turn_deg
                    )
                    st = _aligned_turn(settled, chosen_deg)
                elif pref_90 > 0.52:
                    st = _aligned_turn(settled, pref_turn_deg)
                elif other_90 > 0.52:
                    st = _aligned_turn(settled, other_turn_deg)
                else:
                    # Ngõ cụt: xoay 90° hướng mũi ra xa tường hẹp trước, tiến nhẹ ra giữa hành lang rồi xoay nốt 90°
                    too_close_side = min(rf_l90, rf_r90) < 0.24
                    step_deg = -90.0 if rf_l90 < rf_r90 else 90.0
                    mid_st = _aligned_turn(settled, step_deg)
                    if mid_st["vision"].get("target_visible"):
                        done = _finish_target_seen(mid_st["vision"])
                        if done is not None:
                            stopped_cleanly = True
                            return done
                    mid_rf_c = float(mid_st["sensors"]["rangefinders_m"]["center"])
                    if too_close_side and mid_rf_c > 0.30:
                        recenter_m = min(0.10, max(0.04, 0.5 * (mid_rf_c - 0.24)))
                        mid_st = closed_loop_move(
                            call_fn,
                            meters=recenter_m,
                            speed_m_s=0.24,
                            safety_stop_m=0.22,
                            include_vision=False,
                            release_stop=False,
                        )["after_status"]
                        _sync_odom(mid_st)
                    st = _aligned_turn(mid_st, step_deg)

                done = _maybe_finish_target(st)
                if done is not None:
                    stopped_cleanly = True
                    return done
                continue

            side_open = (pref_90 > 0.78 and pref_120 > 0.75) if has_120 else (pref_90 > 0.85)
            other_open = (other_90 > 0.78 and other_120 > 0.75) if has_120 else (other_90 > 0.85)
            if branch_turn_odom_m is None:
                if saw_pref_wall and since_turn_m >= 0.28 and side_open:
                    open_branch_streak += 1
                    req_streak = 1 if has_120 else 2
                    if open_branch_streak >= req_streak:
                        rem_adv = max(0.0, min(0.32, (wall_edge_odom_m + 0.40) - odom_m)) if has_120 else 0.32
                        branch_turn_odom_m = odom_m + max(0.0, rem_adv - 0.015)
                elif not side_open:
                    open_branch_streak = 0

                if branch_turn_odom_m is None and loop_mode and saw_other_wall and since_turn_m >= 0.28 and other_open:
                    adv_other = (
                        max(0.0, min(0.36, (other_wall_edge_odom_m + 0.43) - odom_m))
                        if has_120
                        else 0.34
                    )
                    jx, jy = _probe_xy(0.0, dist_m=adv_other)
                    pox, poy = _probe_xy(other_turn_deg, dist_m=0.72, origin_xy=(jx, jy))
                    if _min_crumb_dist(pox, poy) >= 0.44:
                        other_branch_streak += 1
                        req_streak = 1 if has_120 else 2
                        if other_branch_streak >= req_streak:
                            branch_turn_odom_m = odom_m + max(0.0, adv_other - 0.015)
                    else:
                        other_branch_streak = 0
                elif not other_open:
                    other_branch_streak = 0

            if branch_turn_odom_m is not None and odom_m >= (branch_turn_odom_m - 0.025):
                branch_turn_odom_m = None
                st = _fast_settle(include_vision=True)
                done = _maybe_finish_target(st)
                if done is not None:
                    stopped_cleanly = True
                    return done
                rf_adv = st["sensors"]["rangefinders_m"]
                pref_adv = float(rf_adv.get("right90" if pref_is_right else "left90", 3.0))
                other_adv = float(rf_adv.get("left90" if pref_is_right else "right90", 3.0))
                fwd_adv_ok = (
                    float(rf_adv["center"]) > 0.68
                    and min(float(rf_adv["left"]), float(rf_adv["right"])) > 0.36
                )
                if not loop_mode:
                    if pref_adv > 0.52:
                        st = _aligned_turn(st, pref_turn_deg)
                        done = _maybe_finish_target(st)
                        if done is not None:
                            stopped_cleanly = True
                            return done
                        continue
                    open_branch_streak = 0
                else:
                    candidates: list[tuple[tuple[int, int, int], float]] = []
                    if pref_adv > 0.52:
                        candidates.append((_option_rank(pref_turn_deg, 3), pref_turn_deg))
                    if fwd_adv_ok:
                        candidates.append((_option_rank(0.0, 2), 0.0))
                    if other_adv > 0.52:
                        candidates.append((_option_rank(other_turn_deg, 1), other_turn_deg))
                    if candidates:
                        best_deg = max(candidates, key=lambda item: item[0])[1]
                        if abs(best_deg) > 1.0:
                            if float(rf_adv["center"]) < 0.24 or min(float(rf_adv["left"]), float(rf_adv["right"])) < 0.19:
                                st = closed_loop_move(
                                    call_fn,
                                    meters=-0.06,
                                    speed_m_s=0.24,
                                    safety_stop_m=0.0,
                                    include_vision=True,
                                    release_stop=False,
                                )["after_status"]
                                _sync_odom(st)
                            st = _aligned_turn(st, best_deg)
                            done = _maybe_finish_target(st)
                            if done is not None:
                                stopped_cleanly = True
                                return done
                            continue
                        last_turn_odom_m = odom_m
                        saw_pref_wall = False
                        saw_other_wall = False
                        open_branch_streak = 0
                        other_branch_streak = 0
                    else:
                        open_branch_streak = 0
                        other_branch_streak = 0

            # Lái bám tâm hành lang mượt mà không làm trôi góc chuẩn corridor_yaw_deg
            cur_yaw_deg = float(st["sensors"]["gyro_yaw_deg"])
            yaw_rate_deg_s = math.degrees(float(st["sensors"]["yaw_rate_rad_s"]))
            yaw_bias_deg = 0.0
            if rf_l90 < 0.65 and rf_r90 < 0.65:
                lat_err = rf_l90 - rf_r90
                yaw_bias_deg += max(-16.0, min(16.0, 38.0 * lat_err))
            else:
                if rf_l90 < 0.30:
                    yaw_bias_deg -= min(16.0, 75.0 * (0.30 - rf_l90))
                if rf_r90 < 0.30:
                    yaw_bias_deg += min(16.0, 75.0 * (0.30 - rf_r90))

            if rf_l < 0.36 and rf_r > rf_l + 0.05:
                yaw_bias_deg -= min(10.0, 28.0 * (0.36 - rf_l))
            if rf_r < 0.36 and rf_l > rf_r + 0.05:
                yaw_bias_deg += min(10.0, 28.0 * (0.36 - rf_r))

            desired_yaw_deg = corridor_yaw_deg + max(-18.0, min(18.0, yaw_bias_deg))
            yaw_err = _wrap_angle_deg(desired_yaw_deg - cur_yaw_deg)
            trim = max(-3.2, min(3.2, 0.26 * yaw_err - 0.022 * yaw_rate_deg_s))

            front_clear = min(rf_c, rf_l + 0.22, rf_r + 0.22)
            cruise_speed = max_speed if front_clear > 0.78 else max(0.24, max_speed * ((front_clear - 0.32) / 0.46))
            if min(rf_l90, rf_r90) < 0.24 or min(rf_l120, rf_r120) < 0.22:
                cruise_speed = min(cruise_speed, 0.38)
            cmd_dur = watchdog_dur
            if branch_turn_odom_m is not None:
                rem_branch = max(0.0, branch_turn_odom_m - odom_m)
                if rem_branch < 0.22:
                    cruise_speed = min(cruise_speed, max(0.20, 1.9 * rem_branch))
                cmd_dur = round(max(0.05, min(watchdog_dur, rem_branch / max(0.18, cruise_speed) + 0.01)), 3)
            base_w = min(cruise_speed / WHEEL_RADIUS, MAX_WHEEL_RAD_S)
            left_w = max(-MAX_WHEEL_RAD_S, min(MAX_WHEEL_RAD_S, base_w - trim))
            right_w = max(-MAX_WHEEL_RAD_S, min(MAX_WHEEL_RAD_S, base_w + trim))

            call_fn(
                "/wheels",
                "POST",
                {"left": round(left_w, 4), "right": round(right_w, 4), "duration": cmd_dur},
            )
            target_wall_dt = min(0.045, step_dt * 1.35) if not has_rate_estimate else (step_dt * 1.35) / sim_per_wall
            time.sleep(max(0.002, target_wall_dt - (time.perf_counter() - t_iter_wall)))

        settled = _stop_and_settle(call_fn, include_vision=True)
        stopped_cleanly = True
    finally:
        if not stopped_cleanly:
            call_fn("/stop", "POST", {})

    return {
        "target_found": bool(settled["vision"].get("target_visible", False)),
        "reason": "target_visible" if settled["vision"].get("target_visible", False) else "max_steps",
        "rule": rule,
        "steps": steps_taken,
        "turns": turns_taken,
        "loop_escaped": bool(loop_mode),
        "gates_yielded": int(gates_yielded),
        "after_status": _clean_status(settled),
    }


def _apply_asymmetric_shaft_tuck(
    call_fn: BridgeCall,
    settled_st: dict,
    *,
    re_center_target: bool = True,
) -> dict:
    """Nép nhẹ khỏi mép tường hẹp sang phía rộng khi dừng tại miệng hành lang bất đối xứng."""
    rf_end = settled_st["sensors"]["rangefinders_m"]
    l90_e = float(rf_end.get("left90", 3.0))
    r90_e = float(rf_end.get("right90", 3.0))
    wall_e = min(l90_e, r90_e)
    open_e = max(l90_e, r90_e)
    if not (float(rf_end["center"]) > 0.90 and open_e > 0.85 and 0.29 < wall_e < 0.65):
        return settled_st
    tuck_sign = 1.0 if r90_e > l90_e else -1.0
    tuck_ang = 30.0 if wall_e > 0.43 else 45.0
    tuck_dist = 0.18 if wall_e > 0.43 else min(0.11, max(0.055, (wall_e - 0.265) / 0.707))
    closed_loop_turn(
        call_fn,
        degrees=tuck_ang * tuck_sign,
        max_speed_deg_s=120.0,
        tol_deg=1.2,
        include_vision=False,
        release_stop=False,
        fast_settle=True,
    )
    closed_loop_move(
        call_fn,
        meters=tuck_dist,
        speed_m_s=0.20,
        safety_stop_m=0.30,
        include_vision=False,
        release_stop=False,
    )
    out_st = closed_loop_turn(
        call_fn,
        degrees=-tuck_ang * tuck_sign,
        max_speed_deg_s=120.0,
        tol_deg=1.0,
        include_vision=True,
        release_stop=True,
    )["after_status"]
    if (
        re_center_target
        and out_st["vision"].get("target_visible")
        and abs(float(out_st["vision"]["target"]["offset_x"])) > 0.22
    ):
        fix_deg = -math.degrees(
            math.atan(float(out_st["vision"]["target"]["offset_x"]) * FRONT_CAM_TAN_HALF_FOVX)
        )
        if abs(fix_deg) >= 0.8:
            out_st = closed_loop_turn(
                call_fn,
                degrees=fix_deg,
                max_speed_deg_s=75.0,
                tol_deg=0.6,
                include_vision=True,
                release_stop=True,
            )["after_status"]
    return out_st


def compute_open_branches(rf: dict[str, float]) -> list[str]:
    """Liệt kê các nhánh rẽ mở tại vị trí hiện tại theo thứ tự ['left', 'straight', 'right']."""
    rf_c = float(rf.get("center", 0.0))
    rf_l = float(rf.get("left", rf_c))
    rf_r = float(rf.get("right", rf_c))
    rf_l90 = float(rf.get("left90", 0.0))
    rf_r90 = float(rf.get("right90", 0.0))
    min_diag = min(rf_l, rf_r)
    max_diag = max(rf_l, rf_r)
    branches: list[str] = []
    if rf_l90 > 0.55:
        branches.append("left")
    if rf_c > 0.62 and (min_diag > 0.34 or (min_diag > 0.20 and max_diag > 0.55)):
        branches.append("straight")
    if rf_r90 > 0.55:
        branches.append("right")
    return branches


def _is_front_blocked(rf: dict[str, float], *, allow_corner_grazing: bool = False) -> bool:
    """Xác định phía trước thực sự bị chắn (chỉ bỏ qua tia chéo ±30° sượt mép góc lúc xuất phát khi không có vật cản)."""
    rf_c = float(rf["center"])
    rf_l = float(rf["left"])
    rf_r = float(rf["right"])
    if rf_c < 0.40:
        return True
    min_diag = min(rf_l, rf_r)
    max_diag = max(rf_l, rf_r)
    if min_diag < 0.20:
        return True
    if min_diag < 0.28:
        if allow_corner_grazing and rf_c > 0.62 and max_diag > 0.55:
            return False
        return True
    return False


def _obs_signature(vis_dict: dict) -> dict[str, float]:
    return {
        str(o.get("color", "")): float(o["offset_x"])
        for o in vis_dict.get("obstacles", [])
    }


def _corridor_heading_bias(rf: dict[str, float], in_narrow: bool) -> float:
    """Tính góc hiệu chỉnh giữ tâm hành lang và chống quẹt mũi/sườn từ các tia sonar."""
    rf_l = float(rf["left"])
    rf_r = float(rf["right"])
    rf_l90 = float(rf.get("left90", 3.0))
    rf_r90 = float(rf.get("right90", 3.0))
    bias = 0.0
    if rf_l90 < 0.65 and rf_r90 < 0.65:
        bias += max(-10.0, min(10.0, 26.0 * (rf_l90 - rf_r90)))
    else:
        if rf_l90 < 0.32:
            bias -= min(15.0, 70.0 * (0.32 - rf_l90))
        if rf_r90 < 0.32:
            bias += min(15.0, 70.0 * (0.32 - rf_r90))
    if in_narrow:
        rf_l120 = float(rf.get("left120", 3.0))
        rf_r120 = float(rf.get("right120", 3.0))
        if rf_l120 < 0.24:
            bias -= min(12.0, 55.0 * (0.24 - rf_l120))
        if rf_r120 < 0.24:
            bias += min(12.0, 55.0 * (0.24 - rf_r120))
    if rf_l < 0.36 and rf_r > rf_l + 0.05:
        bias -= min(10.0, 28.0 * (0.36 - rf_l))
    if rf_r < 0.36 and rf_l > rf_r + 0.05:
        bias += min(10.0, 28.0 * (0.36 - rf_r))
    return max(-18.0, min(18.0, bias))


def _summarize_obstacle(obs_candidates: list[dict], motion: str) -> dict | None:
    if not obs_candidates:
        return None
    best = min(obs_candidates, key=_estimate_obs_cam_dist_m)
    return {
        "color": str(best.get("color", "")),
        "offset_x": round(float(best.get("offset_x", 0.0)), 4),
        "area_pct": round(float(best.get("area_pct", 0.0)), 4),
        "est_dist_m": round(_estimate_obs_cam_dist_m(best), 3),
        "motion": motion,
    }


def _probe_front_obstacle_motion(
    call_fn: BridgeCall,
    initial_st: dict,
    z_limit: float = 0.78,
    ignored_colors: set[str] | None = None,
) -> tuple[str, dict, dict | None]:
    """Dừng ổn định thân xe rồi đứng yên <= 0.85 s sim-time để phân loại vật cản phía trước ('dynamic' | 'static' | 'none')."""
    ignored = ignored_colors or set()

    def _candidates(cur_st: dict) -> list[dict]:
        obs_l = cur_st.get("vision", {}).get("obstacles", [])
        rf_d = cur_st["sensors"]["rangefinders_m"]
        rf_m = min(float(rf_d["left"]), float(rf_d["center"]), float(rf_d["right"]))
        return [
            o
            for o in obs_l
            if str(o.get("color", "")) not in ignored
            and (
                (_estimate_obs_cam_dist_m(o) <= z_limit and abs(float(o.get("offset_x", 0.0))) < 0.78)
                or (_estimate_obs_cam_dist_m(o) <= min(0.65, z_limit) and abs(float(o.get("offset_x", 0.0))) < 0.95)
                or rf_m < 0.48
            )
        ]

    init_cands = _candidates(initial_st)
    if not init_cands:
        return "none", initial_st, None

    # Hãm đứng hẳn trước khi lấy mẫu mốc để quán tính phanh không làm nhiễu offset_x / rangefinder
    sensors0 = initial_st["sensors"]
    needs_settle = (
        bool(initial_st.get("moving", False))
        or abs(float(sensors0.get("forward_speed_m_s", 0.0))) > 0.03
        or abs(float(sensors0.get("yaw_rate_rad_s", 0.0))) > 0.05
    )
    probe_st = _stop_and_settle(call_fn, include_vision=True) if needs_settle else initial_st
    settled_cands = _candidates(probe_st)
    if not settled_cands:
        if needs_settle:
            # Vật cản đã trượt khỏi góc nhìn ngay trong lúc xe phanh dừng -> chắc chắn đang di chuyển
            return "dynamic", probe_st, _summarize_obstacle(init_cands, "dynamic")
        return "none", probe_st, None

    obs0_list = probe_st.get("vision", {}).get("obstacles", [])
    rf0 = probe_st["sensors"]["rangefinders_m"]
    rf0_min = min(float(rf0["left"]), float(rf0["center"]), float(rf0["right"]))
    obs0 = _obs_signature(probe_st.get("vision", {}))
    obs_min = dict(obs0)
    obs_max = dict(obs0)
    t0 = float(probe_st.get("sim_time", 0.0))
    motion = "static"
    deadline = time.monotonic() + 2.5
    stagnant_polls = 0
    prev_sim = t0

    for _ in range(40):
        if time.monotonic() >= deadline:
            break
        call_fn("/wheels", "POST", {"left": 0.0, "right": 0.0, "duration": 0.25})
        time.sleep(0.012)
        probe_st = call_fn("/status?vision=1", "GET", None)
        obs_cur = _obs_signature(probe_st.get("vision", {}))
        rf_cur = probe_st["sensors"]["rangefinders_m"]
        rf_cur_min = min(float(rf_cur["left"]), float(rf_cur["center"]), float(rf_cur["right"]))

        if set(obs_cur.keys()) != set(obs0.keys()):
            motion = "dynamic"
            break
        for k, val in obs_cur.items():
            obs_min[k] = min(obs_min.get(k, val), val)
            obs_max[k] = max(obs_max.get(k, val), val)
        if any((obs_max[k] - obs_min[k]) >= 0.010 for k in obs_cur):
            motion = "dynamic"
            break
        if abs(rf_cur_min - rf0_min) >= 0.015:
            motion = "dynamic"
            break

        cur_sim = float(probe_st.get("sim_time", t0))
        if (cur_sim - t0) >= 0.85:
            break
        if abs(cur_sim - prev_sim) < 1e-6:
            stagnant_polls += 1
            if stagnant_polls >= 3:
                break
        else:
            stagnant_polls = 0
        prev_sim = cur_sim

    obs_latest = probe_st.get("vision", {}).get("obstacles", [])
    src_obs = _candidates(probe_st) or obs_latest or settled_cands
    return motion, probe_st, _summarize_obstacle(src_obs, motion)


def _is_target_ready_for_dock(st: dict) -> bool:
    vis = st.get("vision", {})
    if not vis.get("target_visible") or not isinstance(vis.get("target"), dict):
        return False
    t_off = float(vis["target"]["offset_x"])
    t_area = float(vis["target"]["area_pct"])
    rf_c = float(st["sensors"]["rangefinders_m"]["center"])
    obs = vis.get("obstacles", [])
    closer_obs = any(
        float(o["area_pct"]) > max(1.0, t_area * 1.2) and abs(float(o["offset_x"])) < 0.45
        for o in obs
    )
    fg_obs = any(
        _estimate_obs_cam_dist_m(o) < min(0.85, rf_c + 0.05) and abs(float(o.get("offset_x", 0.0))) < 0.90
        for o in obs
    )
    if closer_obs:
        return False
    if t_area >= 6.0 and not fg_obs:
        return True
    if abs(t_off) >= 0.45:
        return False
    return (t_area >= 1.2 and rf_c <= 0.95 and not fg_obs) or (t_area >= 0.45 and rf_c <= 0.85 and not fg_obs)


def _glance_open_wings_for_dynamic_gate(
    call_fn: BridgeCall,
    st: dict,
    yaw0_deg: float,
) -> tuple[bool, dict, dict | None, bool]:
    """Liếc nhanh ±32° trong không gian mở để phát hiện vật cản động đang ở ngoài góc nhìn chính diện."""
    found_dyn: dict | None = None
    approaching_center = False
    cur_st = st

    def _lat_map(obs_list: list[dict], yaw_deg: float) -> dict[str, float]:
        d_rad = math.radians(_wrap_angle_deg(yaw_deg - yaw0_deg))
        out: dict[str, float] = {}
        for o in obs_list:
            z = _estimate_obs_cam_dist_m(o)
            y_c = -float(o["offset_x"]) * z * FRONT_CAM_TAN_HALF_FOVX
            out[str(o.get("color", ""))] = z * math.sin(d_rad) + y_c * math.cos(d_rad)
        return out

    for sign in (1.0, -1.0):
        cur_y = float(cur_st["sensors"]["gyro_yaw_deg"])
        delta = _wrap_angle_deg((yaw0_deg + 32.0 * sign) - cur_y)
        st_a = (
            closed_loop_turn(
                call_fn,
                degrees=delta,
                max_speed_deg_s=220.0,
                tol_deg=1.8,
                include_vision=True,
                release_stop=False,
                fast_settle=True,
            )["after_status"]
            if abs(delta) >= 1.0
            else call_fn("/status?vision=1", "GET", None)
        )
        t_a = float(st_a.get("sim_time", 0.0))
        st_b = st_a
        obs_a = _obs_signature(st_a.get("vision", {}))
        obs_a_list = st_a.get("vision", {}).get("obstacles", [])
        lat_a_map = _lat_map(obs_a_list, float(st_a["sensors"]["gyro_yaw_deg"]))
        dwell_s = 0.55 if obs_a_list else 0.16
        w_deadline = time.monotonic() + 1.5
        for _ in range(25):
            if time.monotonic() >= w_deadline or (float(st_b.get("sim_time", t_a)) - t_a) >= dwell_s:
                break
            call_fn("/wheels", "POST", {"left": 0.0, "right": 0.0, "duration": 0.25})
            time.sleep(0.012)
            st_b = call_fn("/status?vision=1", "GET", None)
            if obs_a_list:
                obs_cur_map = _obs_signature(st_b.get("vision", {}))
                if any(
                    cn not in obs_a or abs(obs_cur_map[cn] - obs_a[cn]) >= 0.010
                    for cn in obs_cur_map
                ):
                    break
        cur_st = st_b
        obs_b_list = st_b.get("vision", {}).get("obstacles", [])
        obs_b = _obs_signature(st_b.get("vision", {}))
        d_yaw_rad = math.radians(_wrap_angle_deg(float(st_b["sensors"]["gyro_yaw_deg"]) - yaw0_deg))
        for o_b in obs_b_list:
            zb = _estimate_obs_cam_dist_m(o_b)
            ox_b = float(o_b["offset_x"])
            y_cam_b = -ox_b * zb * FRONT_CAM_TAN_HALF_FOVX
            fwd_b = zb * math.cos(d_yaw_rad) - y_cam_b * math.sin(d_yaw_rad)
            lat_b = zb * math.sin(d_yaw_rad) + y_cam_b * math.cos(d_yaw_rad)
            cn = str(o_b.get("color", ""))
            moved_sideways = (cn not in obs_a) or abs(obs_b.get(cn, ox_b) - obs_a.get(cn, ox_b)) >= 0.010
            if moved_sideways and 0.14 <= fwd_b <= 1.05 and abs(lat_b) <= 1.40:
                found_dyn = _summarize_obstacle([o_b], "dynamic")
                if cn not in lat_a_map or abs(lat_b) < abs(lat_a_map[cn]) - 0.008 or abs(lat_b) <= 0.65:
                    approaching_center = True
                break
        if found_dyn is not None:
            break

    cur_y_end = float(call_fn("/status", "GET", None)["sensors"]["gyro_yaw_deg"])
    back_deg = _wrap_angle_deg(yaw0_deg - cur_y_end)
    if abs(back_deg) >= 0.8:
        cur_st = closed_loop_turn(
            call_fn,
            degrees=back_deg,
            max_speed_deg_s=220.0,
            tol_deg=1.4,
            include_vision=True,
            release_stop=False,
            fast_settle=True,
        )["after_status"]
    else:
        cur_st = call_fn("/status?vision=1", "GET", None)
    return found_dyn is not None, cur_st, found_dyn, approaching_center


def advance_segment(
    call_fn: BridgeCall,
    max_dist: float = 1.8,
    speed_m_s: float = 0.55,
    step_dt: float = 0.10,
    *,
    _out_meta: dict | None = None,
    _target_odom_xy: tuple[float, float] | None = None,
    _ignore_target_ready: bool = False,
) -> dict:
    """Tiến thẳng một đoạn hành lang/sân mở hoàn toàn vô trạng thái và dừng ngay khi gặp sự kiện."""
    _require_finite(max_dist=max_dist, speed_m_s=speed_m_s, step_dt=step_dt)
    max_wheel_linear = MAX_WHEEL_RAD_S * WHEEL_RADIUS
    if max_dist <= 0.0:
        raise ValueError("Tham số 'max_dist' phải lớn hơn 0.")
    if speed_m_s < 0.05 or speed_m_s > max_wheel_linear:
        raise ValueError(f"Tốc độ 'speed_m_s' phải nằm trong khoảng [0.05, {max_wheel_linear:.2f}] m/s.")
    if step_dt < 0.04 or step_dt > 1.0:
        raise ValueError("Chu kỳ điều khiển 'step_dt' phải nằm trong đoạn [0.04, 1.0] giây.")

    watchdog_dur = round(max(0.32, step_dt * 3.0), 3)
    st = call_fn("/status?vision=1", "GET", None)
    rf0 = st["sensors"]["rangefinders_m"]
    odom0 = float(st["sensors"].get("odom_dist_m", 0.0))
    odom_m = odom0
    has_hw_odom = "odom_dist_m" in st["sensors"]
    prev_sim_t = float(st.get("sim_time", 0.0))
    yaw0_deg = float(st["sensors"]["gyro_yaw_deg"])
    l90_0 = float(rf0.get("left90", 3.0))
    r90_0 = float(rf0.get("right90", 3.0))
    in_narrow_0 = (l90_0 + r90_0) < 1.35 or l90_0 < 0.52 or r90_0 < 0.52
    corridor_yaw_deg = yaw0_deg
    if in_narrow_0:
        nearest_90 = round(yaw0_deg / 90.0) * 90.0
        if abs(_wrap_angle_deg(nearest_90 - yaw0_deg)) <= 15.0:
            corridor_yaw_deg = nearest_90

    def _sync_local_odom(cur_st: dict) -> float:
        nonlocal odom_m, prev_sim_t
        cur_sim = float(cur_st.get("sim_time", prev_sim_t))
        dt_s = max(0.0, cur_sim - prev_sim_t)
        prev_sim_t = cur_sim
        sens = cur_st["sensors"]
        if has_hw_odom:
            odom_m = float(sens["odom_dist_m"])
        else:
            odom_m += abs(float(sens.get("forward_speed_m_s", 0.0))) * dt_s
        return odom_m - odom0

    saw_left_wall = l90_0 <= 0.55
    saw_right_wall = r90_0 <= 0.55
    saw_corridor_wall = min(l90_0, r90_0) <= 0.60

    def _finish_result(
        event: str,
        end_st: dict,
        *,
        obstacle: dict | None = None,
        settle: bool = True,
    ) -> dict:
        if settle and (
            end_st.get("moving", False)
            or abs(float(end_st["sensors"].get("forward_speed_m_s", 0.0))) > 0.02
            or abs(float(end_st["sensors"].get("yaw_rate_rad_s", 0.0))) > 0.04
        ):
            end_st = _stop_and_settle(call_fn, include_vision=True)
        moved = round(max(0.0, _sync_local_odom(end_st)), 4)
        rf_e = end_st["sensors"]["rangefinders_m"]
        if _out_meta is not None:
            _out_meta["saw_corridor_wall"] = bool(
                saw_corridor_wall or saw_left_wall or saw_right_wall
            )
        return {
            "event": event,
            "moved_m": moved,
            "open_branches": compute_open_branches(rf_e),
            "obstacle": obstacle,
            "flank_clearance_m": {
                "left": round(float(rf_e.get("left90", 3.0)), 3),
                "right": round(float(rf_e.get("right90", 3.0)), 3),
            },
            "after_status": _clean_status(end_st),
        }

    stopped_cleanly = False
    ignored_static_colors: set[str] = set()

    try:
        def _has_probe_candidate(vis_d: dict) -> bool:
            return any(
                (
                    _estimate_obs_cam_dist_m(o) <= 0.78
                    and abs(float(o.get("offset_x", 0.0))) < 0.78
                )
                or (
                    _estimate_obs_cam_dist_m(o) <= 0.65
                    and abs(float(o.get("offset_x", 0.0))) < 0.95
                )
                for o in vis_d.get("obstacles", [])
            )

        # Chờ đầu nhánh rộng có piston nép hốc (<= 4.5s sim-time) chỉ khi thấy đích từ xa dọc hành lang hẹp
        vis0 = st.get("vision", {})
        if (
            in_narrow_0
            and vis0.get("target_visible")
            and not _has_probe_candidate(vis0)
            and abs(float(vis0["target"]["offset_x"])) < 0.25
            and float(rf0["center"]) > 1.05
            and min(float(rf0["left"]), float(rf0["right"])) > 0.30
            and min(l90_0, r90_0) >= 0.24
            and (l90_0 + r90_0) >= 0.72
        ):
            t_p0 = float(st.get("sim_time", 0.0))
            prev_p_sim = t_p0
            stagnant = 0
            p_deadline = time.monotonic() + 7.0
            while time.monotonic() < p_deadline:
                call_fn("/wheels", "POST", {"left": 0.0, "right": 0.0, "duration": watchdog_dur})
                time.sleep(0.015)
                st = call_fn("/status?vision=1", "GET", None)
                _sync_local_odom(st)
                if _has_probe_candidate(st.get("vision", {})):
                    break
                cur_p_sim = float(st.get("sim_time", t_p0))
                if (cur_p_sim - t_p0) >= 4.5:
                    break
                if abs(cur_p_sim - prev_p_sim) < 1e-6:
                    stagnant += 1
                    if stagnant >= 4:
                        break
                else:
                    stagnant = 0
                prev_p_sim = cur_p_sim

        def _check_obstacle_event(cur_st: dict) -> dict | None:
            motion, probe_st, obs_sum = _probe_front_obstacle_motion(
                call_fn, cur_st, z_limit=0.78, ignored_colors=ignored_static_colors
            )
            if motion == "none" or obs_sum is None:
                return None
            p_rf = probe_st["sensors"]["rangefinders_m"]
            min_f = min(float(p_rf["left"]), float(p_rf["center"]), float(p_rf["right"]))
            if motion == "dynamic":
                if min_f < 0.34 or obs_sum["est_dist_m"] < 0.26:
                    p_l90 = float(p_rf.get("left90", 3.0))
                    p_r90 = float(p_rf.get("right90", 3.0))
                    back_m = -0.05 if min(p_l90, p_r90) > 0.75 else -0.10
                    probe_st = closed_loop_move(
                        call_fn,
                        meters=back_m,
                        speed_m_s=0.25,
                        safety_stop_m=0.0,
                        include_vision=True,
                        release_stop=True,
                    )["after_status"]
                return _finish_result("dynamic_gate_ahead", probe_st, obstacle=obs_sum, settle=True)
            # Vật cản tĩnh: chỉ báo khi nằm ngay trước đường tiến hoặc cận sườn mũi; ghi nhớ tạm trong lần gọi này nếu đứng ngoài mép
            if min_f < 0.65 or (obs_sum["est_dist_m"] <= 0.78 and abs(obs_sum["offset_x"]) < 0.78):
                return _finish_result("static_obstacle_ahead", probe_st, obstacle=obs_sum, settle=True)
            if obs_sum["color"]:
                ignored_static_colors.add(obs_sum["color"])
            return None

        def _finish_target_ready_checked(cur_st: dict, *, settle: bool = True) -> dict:
            chk_st = _stop_and_settle(call_fn, include_vision=True) if (settle or cur_st.get("moving", False)) else cur_st
            rf_chk = chk_st["sensors"]["rangefinders_m"]
            vis_chk = chk_st.get("vision", {})
            t_info = vis_chk.get("target") if isinstance(vis_chk.get("target"), dict) else None
            t_area_chk = float(t_info["area_pct"]) if t_info is not None else 0.0
            if (
                min(float(rf_chk.get("left90", 3.0)), float(rf_chk.get("right90", 3.0))) > 0.75
                and float(rf_chk["center"]) > 0.50
                and t_area_chk < 8.0
            ):
                obs_chk = _check_obstacle_event(chk_st)
                if obs_chk is not None:
                    return obs_chk
                found_w, chk_st, w_obs, _ = _glance_open_wings_for_dynamic_gate(call_fn, chk_st, yaw0_deg)
                _sync_local_odom(chk_st)
                if found_w and w_obs is not None:
                    return _finish_result("dynamic_gate_ahead", chk_st, obstacle=w_obs, settle=True)
            return _finish_result("target_ready", chk_st, settle=False)

        # Kiểm tra trước khi xuất phát (ưu tiên target_ready trước khi chưa có vật cản gần hơn)
        if not _ignore_target_ready and _is_target_ready_for_dock(st):
            stopped_cleanly = True
            return _finish_target_ready_checked(st, settle=True)

        if _target_odom_xy is not None:
            init_tx, init_ty = float(_target_odom_xy[0]), float(_target_odom_xy[1])
            d0_target_node = math.hypot(
                float(st["sensors"].get("odom_x_m", 0.0)) - init_tx,
                float(st["sensors"].get("odom_y_m", 0.0)) - init_ty,
            )
            if d0_target_node <= 0.10:
                stopped_cleanly = True
                return _finish_result("arrived_node", st, settle=True)
        else:
            d0_target_node = float("inf")
        best_d_target_node = d0_target_node

        obs_res = _check_obstacle_event(st)
        if obs_res is not None:
            stopped_cleanly = True
            return obs_res

        # Trong không gian mở không có tường hai bên, liếc nhanh ±32° nếu chưa thấy vật cản cự ly gần (<= 0.90m) để tránh quẹt piston lệch góc
        vis_cur = st.get("vision", {})
        rf_init = st["sensors"]["rangefinders_m"]
        has_near_front_obs = any(
            _estimate_obs_cam_dist_m(o) <= 0.90 for o in vis_cur.get("obstacles", [])
        )
        if (
            min(float(rf_init.get("left90", 3.0)), float(rf_init.get("right90", 3.0))) > 0.75
            and float(rf_init["center"]) > 0.85
            and not has_near_front_obs
            and (
                not vis_cur.get("target_visible")
                or float(vis_cur["target"]["area_pct"]) < 2.2
            )
        ):
            found_wing, st, wing_obs, _ = _glance_open_wings_for_dynamic_gate(call_fn, st, yaw0_deg)
            _sync_local_odom(st)
            if found_wing and wing_obs is not None:
                stopped_cleanly = True
                return _finish_result("dynamic_gate_ahead", st, obstacle=wing_obs, settle=True)

        if not _ignore_target_ready and _is_target_ready_for_dock(st):
            stopped_cleanly = True
            return _finish_result("target_ready", st, settle=True)

        def _resolve_front_block(block_st: dict, *, do_settle: bool) -> dict:
            settled = (
                _stop_and_settle(call_fn, include_vision=True)
                if (do_settle or block_st.get("moving", False))
                else block_st
            )
            if not _ignore_target_ready and _is_target_ready_for_dock(settled):
                return _finish_result("target_ready", settled, settle=False)
            obs_after = _check_obstacle_event(settled)
            if obs_after is not None:
                return obs_after
            rf_s = settled["sensors"]["rangefinders_m"]
            if float(rf_s["center"]) < 0.26 or min(float(rf_s["left"]), float(rf_s["right"])) < 0.21:
                settled = closed_loop_move(
                    call_fn,
                    meters=-0.08,
                    speed_m_s=0.24,
                    safety_stop_m=0.0,
                    include_vision=True,
                    release_stop=True,
                )["after_status"]
                rf_s = settled["sensors"]["rangefinders_m"]
            if (
                float(rf_s.get("left90", 3.0)) <= 0.52
                and float(rf_s.get("right90", 3.0)) <= 0.52
                and (
                    ("left120" in rf_s and float(rf_s["left120"]) > 0.72)
                    or ("right120" in rf_s and float(rf_s["right120"]) > 0.72)
                )
            ):
                # Chờ tới 5.2s sim-time nếu left120/right120 mở (piston có thể đang che góc ±90° tại ngã rẽ)
                t_c0 = float(settled.get("sim_time", 0.0))
                prev_c_sim = t_c0
                stagnant_c = 0
                c_deadline = time.monotonic() + 7.5
                for _ in range(180):
                    if time.monotonic() >= c_deadline or (float(settled.get("sim_time", t_c0)) - t_c0) >= 5.2:
                        break
                    call_fn("/wheels", "POST", {"left": 0.0, "right": 0.0, "duration": 0.25})
                    time.sleep(0.012)
                    settled = call_fn("/status?vision=1", "GET", None)
                    rf_s = settled["sensors"]["rangefinders_m"]
                    if float(rf_s.get("left90", 3.0)) > 0.55 or float(rf_s.get("right90", 3.0)) > 0.55:
                        break
                    cur_c_sim = float(settled.get("sim_time", t_c0))
                    if abs(cur_c_sim - prev_c_sim) < 1e-6:
                        stagnant_c += 1
                        if stagnant_c >= 4:
                            break
                    else:
                        stagnant_c = 0
                    prev_c_sim = cur_c_sim
            ev_block = (
                "junction"
                if (float(rf_s.get("left90", 3.0)) > 0.52 or float(rf_s.get("right90", 3.0)) > 0.52)
                else "dead_end"
            )
            return _finish_result(ev_block, settled, settle=False)

        rf_now = st["sensors"]["rangefinders_m"]
        if _is_front_blocked(rf_now, allow_corner_grazing=not st.get("vision", {}).get("obstacles")):
            stopped_cleanly = True
            return _resolve_front_block(st, do_settle=False)

        left_edge_odom = odom0 + 0.10 if saw_left_wall else odom0
        right_edge_odom = odom0 + 0.10 if saw_right_wall else odom0
        left_open_streak = 0
        right_open_streak = 0
        branch_center_odom_m: float | None = None

        sim_per_wall = 1.0
        has_rate_est = False
        prev_iter_wall: float | None = None
        prev_iter_sim: float | None = None
        max_iters = max(40, int(max_dist / max(0.05, speed_m_s * step_dt) * 4) + 30)

        for step_idx in range(1, max_iters + 1):
            t_iter_wall = time.perf_counter()
            if step_idx > 1:
                st = call_fn("/status?vision=1", "GET", None)
            moved = _sync_local_odom(st)
            cur_sim_t = float(st.get("sim_time", 0.0))
            if prev_iter_wall is not None and prev_iter_sim is not None:
                dt_w = t_iter_wall - prev_iter_wall
                dt_s = cur_sim_t - prev_iter_sim
                if dt_w > 1e-3 and dt_s > 1e-4:
                    inst_rate = max(0.8, min(10.0, dt_s / dt_w))
                    sim_per_wall = inst_rate if not has_rate_est else (0.5 * sim_per_wall + 0.5 * inst_rate)
                    has_rate_est = True
            prev_iter_wall = t_iter_wall
            prev_iter_sim = cur_sim_t

            rf = st["sensors"]["rangefinders_m"]
            rf_l = float(rf["left"])
            rf_c = float(rf["center"])
            rf_r = float(rf["right"])
            rf_l90 = float(rf.get("left90", 3.0))
            rf_r90 = float(rf.get("right90", 3.0))
            rf_l120 = float(rf.get("left120", 3.0))
            rf_r120 = float(rf.get("right120", 3.0))
            has_120 = ("left120" in rf) and ("right120" in rf)
            in_narrow_now = (rf_l90 + rf_r90) < 1.35 or rf_l90 < 0.52 or rf_r90 < 0.52
            if min(rf_l90, rf_r90) <= 0.60:
                saw_corridor_wall = True

            # Kiểm tra vật cản & target_ready trước
            obs_res = _check_obstacle_event(st)
            if obs_res is not None:
                stopped_cleanly = True
                return obs_res

            if not _ignore_target_ready and _is_target_ready_for_dock(st):
                stopped_cleanly = True
                return _finish_target_ready_checked(st, settle=True)

            cur_d_target_node = float("inf")
            if _target_odom_xy is not None:
                cur_d_target_node = math.hypot(
                    float(st["sensors"].get("odom_x_m", 0.0)) - float(_target_odom_xy[0]),
                    float(st["sensors"].get("odom_y_m", 0.0)) - float(_target_odom_xy[1]),
                )
                if (moved >= 0.12 or d0_target_node <= 0.25) and (
                    cur_d_target_node <= 0.08
                    or (cur_d_target_node <= 0.28 and cur_d_target_node > best_d_target_node + 0.012)
                ):
                    stopped_cleanly = True
                    return _finish_result("arrived_node", st, settle=True)
                if cur_d_target_node < best_d_target_node:
                    best_d_target_node = cur_d_target_node

            # Kiểm tra tường phía trước -> dừng và trả về junction hoặc dead_end
            allow_graze_now = moved < 0.22 and not st.get("vision", {}).get("obstacles")
            if _is_front_blocked(rf, allow_corner_grazing=allow_graze_now):
                stopped_cleanly = True
                return _resolve_front_block(st, do_settle=True)

            # Cập nhật lịch sử tường hông trong phạm vi lần gọi này (vô trạng thái giữa các lần gọi)
            if moved >= 0.08:
                if rf_l90 <= 0.55:
                    saw_left_wall = True
                    left_edge_odom = max(left_edge_odom, odom_m + 0.08)
                if saw_left_wall and has_120 and rf_l120 < 0.72:
                    left_edge_odom = max(left_edge_odom, odom_m + 0.05 - 0.5 * rf_l120)
                if rf_r90 <= 0.55:
                    saw_right_wall = True
                    right_edge_odom = max(right_edge_odom, odom_m + 0.08)
                if saw_right_wall and has_120 and rf_r120 < 0.72:
                    right_edge_odom = max(right_edge_odom, odom_m + 0.05 - 0.5 * rf_r120)

            l_open_now = (rf_l90 > 0.75 and rf_l120 > 0.72) if has_120 else (rf_l90 > 0.72)
            r_open_now = (rf_r90 > 0.75 and rf_r120 > 0.72) if has_120 else (rf_r90 > 0.72)
            if branch_center_odom_m is None and moved >= 0.22:
                if saw_left_wall and l_open_now:
                    left_open_streak += 1
                    rem_l = max(0.12, min(0.30, (left_edge_odom + 0.34) - odom_m)) if has_120 else 0.28
                    branch_center_odom_m = odom_m + rem_l
                elif not l_open_now:
                    left_open_streak = 0

                if branch_center_odom_m is None and saw_right_wall and r_open_now:
                    right_open_streak += 1
                    rem_r = max(0.12, min(0.30, (right_edge_odom + 0.34) - odom_m)) if has_120 else 0.28
                    branch_center_odom_m = odom_m + rem_r
                elif not r_open_now:
                    right_open_streak = 0

            if branch_center_odom_m is not None and (odom_m >= branch_center_odom_m - 0.02 or rf_c <= 0.36):
                settled = _stop_and_settle(call_fn, include_vision=True)
                branches = compute_open_branches(settled["sensors"]["rangefinders_m"])
                if not _ignore_target_ready and _is_target_ready_for_dock(settled):
                    stopped_cleanly = True
                    return _finish_result("target_ready", settled, settle=False)
                if "left" in branches or "right" in branches:
                    stopped_cleanly = True
                    return _finish_result("junction", settled, settle=False)
                branch_center_odom_m = None

            if moved >= max_dist - 0.015:
                stopped_cleanly = True
                return _finish_result("max_dist", st, settle=True)

            cur_yaw_deg = float(st["sensors"]["gyro_yaw_deg"])
            yaw_rate_deg_s = math.degrees(float(st["sensors"].get("yaw_rate_rad_s", 0.0)))
            if in_narrow_now:
                yaw_bias = _corridor_heading_bias(rf, True)
            elif allow_graze_now and min(rf_l, rf_r) < 0.36:
                yaw_bias = _corridor_heading_bias(rf, False)
            else:
                yaw_bias = 0.0
            yaw_err = _wrap_angle_deg((corridor_yaw_deg + yaw_bias) - cur_yaw_deg)
            trim = max(-3.2, min(3.2, 0.26 * yaw_err - 0.022 * yaw_rate_deg_s))

            front_clear = min(rf_c, rf_l + 0.22, rf_r + 0.22)
            cruise = speed_m_s if front_clear > 0.78 else max(0.22, speed_m_s * ((front_clear - 0.32) / 0.46))
            rem_limit = max_dist - moved
            if _target_odom_xy is not None:
                rem_limit = min(rem_limit, max(0.06, cur_d_target_node))
            if branch_center_odom_m is not None:
                rem_limit = min(rem_limit, max(0.02, branch_center_odom_m - odom_m))
            if rem_limit < 0.24:
                cruise = min(cruise, max(0.18, 1.8 * rem_limit))
            if min(rf_l90, rf_r90) < 0.24:
                cruise = min(cruise, 0.36)

            cmd_dur = round(max(0.05, min(watchdog_dur, rem_limit / max(0.18, cruise) + 0.02)), 3)
            base_w = min(cruise / WHEEL_RADIUS, MAX_WHEEL_RAD_S)
            lw = max(-MAX_WHEEL_RAD_S, min(MAX_WHEEL_RAD_S, base_w - trim))
            rw = max(-MAX_WHEEL_RAD_S, min(MAX_WHEEL_RAD_S, base_w + trim))
            call_fn("/wheels", "POST", {"left": round(lw, 4), "right": round(rw, 4), "duration": cmd_dur})

            target_wall_dt = min(0.045, step_dt * 1.25) if not has_rate_est else (step_dt * 1.25) / sim_per_wall
            time.sleep(max(0.002, target_wall_dt - (time.perf_counter() - t_iter_wall)))

        settled = _stop_and_settle(call_fn, include_vision=True)
        stopped_cleanly = True
        return _finish_result("max_dist", settled, settle=False)
    finally:
        if not stopped_cleanly:
            call_fn("/stop", "POST", {})


def cross_dynamic_gate(call_fn: BridgeCall, timeout_s: float = 12.0) -> dict:
    """Chờ đúng 1 làn/cổng tuần tra động rút hẳn khỏi tâm đường rồi băng qua dứt điểm 1 nhịp."""
    _require_finite(timeout_s=timeout_s)
    if timeout_s <= 0.0:
        raise ValueError("Tham số 'timeout_s' phải lớn hơn 0.")

    st = call_fn("/status?vision=1", "GET", None)
    odom0 = float(st["sensors"].get("odom_dist_m", 0.0))
    odom_m = odom0
    has_hw_odom = "odom_dist_m" in st["sensors"]
    prev_sim_t = float(st.get("sim_time", 0.0))
    yaw0_deg = float(st["sensors"]["gyro_yaw_deg"])
    rf0 = st["sensors"]["rangefinders_m"]
    l90_0 = float(rf0.get("left90", 3.0))
    r90_0 = float(rf0.get("right90", 3.0))
    if (l90_0 + r90_0) < 1.35 or l90_0 < 0.52 or r90_0 < 0.52:
        nearest_90 = round(yaw0_deg / 90.0) * 90.0
        if abs(_wrap_angle_deg(nearest_90 - yaw0_deg)) <= 15.0:
            yaw0_deg = nearest_90

    def _sync_local_odom(cur_st: dict) -> float:
        nonlocal odom_m, prev_sim_t
        cur_sim = float(cur_st.get("sim_time", prev_sim_t))
        dt_s = max(0.0, cur_sim - prev_sim_t)
        prev_sim_t = cur_sim
        sens = cur_st["sensors"]
        if has_hw_odom:
            odom_m = float(sens["odom_dist_m"])
        else:
            odom_m += abs(float(sens.get("forward_speed_m_s", 0.0))) * dt_s
        return odom_m - odom0

    def _result(crossed: bool, reason: str, end_st: dict) -> dict:
        rf_e = end_st["sensors"]["rangefinders_m"]
        return {
            "crossed": crossed,
            "reason": reason,
            "moved_m": round(max(0.0, _sync_local_odom(end_st)), 4),
            "open_branches": compute_open_branches(rf_e),
            "after_status": _clean_status(end_st),
        }

    # Trong không gian mở chưa thấy vật cản cự ly gần (<= 0.90m), liếc ±32° để ghi nhận pha piston biên
    stopped_cleanly = False
    try:
        wing_approaching = False
        has_near_gate_obs = any(
            _estimate_obs_cam_dist_m(o) <= 0.90 for o in st.get("vision", {}).get("obstacles", [])
        )
        if min(l90_0, r90_0) > 0.75 and not has_near_gate_obs:
            _, st, _, wing_approaching = _glance_open_wings_for_dynamic_gate(call_fn, st, yaw0_deg)
            _sync_local_odom(st)

        nearest_gate_z = 0.50
        if st.get("vision", {}).get("obstacles"):
            motion, st, obs_sum = _probe_front_obstacle_motion(call_fn, st, z_limit=0.95)
            _sync_local_odom(st)
            rf_p = st["sensors"]["rangefinders_m"]
            min_f = min(float(rf_p["left"]), float(rf_p["center"]), float(rf_p["right"]))
            if motion == "static" and (min_f < 0.72 or (obs_sum is not None and obs_sum["est_dist_m"] <= 0.85)):
                if st.get("moving", False):
                    st = _stop_and_settle(call_fn, include_vision=True)
                stopped_cleanly = True
                return _result(False, "static_obstacle", st)
            if obs_sum is not None and obs_sum["est_dist_m"] <= 0.90:
                nearest_gate_z = max(0.32, obs_sum["est_dist_m"])

        rf_now = st["sensors"]["rangefinders_m"]
        cur_obs = st.get("vision", {}).get("obstacles", [])
        if min(float(rf_now["left"]), float(rf_now["center"]), float(rf_now["right"])) < 0.34 or any(
            _estimate_obs_cam_dist_m(o) < 0.26 for o in cur_obs
        ):
            n_l90 = float(rf_now.get("left90", 3.0))
            n_r90 = float(rf_now.get("right90", 3.0))
            back_m = -0.05 if min(n_l90, n_r90) > 0.75 else -0.10
            st = closed_loop_move(
                call_fn,
                meters=back_m,
                speed_m_s=0.25,
                safety_stop_m=0.0,
                include_vision=True,
                release_stop=False,
            )["after_status"]
            _sync_local_odom(st)

        # Nếu cánh ngoài đang lao vào tâm mà camera thẳng chưa thấy vật cản, chờ tối đa 5.0s sim-time cho nó quét qua
        if wing_approaching and not any(
            _estimate_obs_cam_dist_m(o) <= 0.90 for o in st.get("vision", {}).get("obstacles", [])
        ):
            t_app0 = float(st.get("sim_time", 0.0))
            app_wait_s = min(5.0, timeout_s)
            prev_app_sim = t_app0
            stagnant_app = 0
            for _ in range(120):
                if (float(st.get("sim_time", t_app0)) - t_app0) >= app_wait_s:
                    break
                call_fn("/wheels", "POST", {"left": 0.0, "right": 0.0, "duration": 0.25})
                time.sleep(0.015)
                st = call_fn("/status?vision=1", "GET", None)
                _sync_local_odom(st)
                w_rf_a = st["sensors"]["rangefinders_m"]
                if (
                    any(_estimate_obs_cam_dist_m(o) <= 0.90 for o in st.get("vision", {}).get("obstacles", []))
                    or min(float(w_rf_a["left"]), float(w_rf_a["center"]), float(w_rf_a["right"])) < 0.78
                ):
                    break
                cur_app_sim = float(st.get("sim_time", t_app0))
                if abs(cur_app_sim - prev_app_sim) < 1e-6:
                    stagnant_app += 1
                    if stagnant_app >= 4:
                        break
                else:
                    stagnant_app = 0
                prev_app_sim = cur_app_sim

        # Chờ khe hở 2 nhịp liên tiếp theo hướng rút lui
        wait_start_sim = float(st.get("sim_time", 0.0))
        wait_deadline = time.monotonic() + max(18.0, timeout_s * 2.5)
        clear_streak = 0
        last_clear_sim_t = -999.0
        prev_obs = _obs_signature(st.get("vision", {}))
        wait_st = st
        gate_opened = False
        prev_w_sim = wait_start_sim
        stagnant_w = 0

        while time.monotonic() < wait_deadline and (float(wait_st.get("sim_time", wait_start_sim)) - wait_start_sim) < timeout_s:
            call_fn("/wheels", "POST", {"left": 0.0, "right": 0.0, "duration": 0.30})
            time.sleep(0.015)
            wait_st = call_fn("/status?vision=1", "GET", None)
            _sync_local_odom(wait_st)
            w_rf = wait_st["sensors"]["rangefinders_m"]
            w_obs = wait_st.get("vision", {}).get("obstacles", [])
            cur_map = _obs_signature(wait_st.get("vision", {}))
            obs_blocking = False
            for o in w_obs:
                z_o = _estimate_obs_cam_dist_m(o)
                if z_o <= 0.78:
                    nearest_gate_z = min(nearest_gate_z, max(0.30, z_o))
                    cn = str(o.get("color", ""))
                    ox_w = float(o["offset_x"])
                    if abs(ox_w) < 0.78:
                        obs_blocking = True
                        break
                    if cn in prev_obs:
                        d_ox = ox_w - prev_obs[cn]
                        if (ox_w * d_ox) <= 0.004 and not (
                            abs(ox_w) >= 0.85
                            and (ox_w * d_ox) >= -0.004
                            and float(w_rf["center"]) > 0.90
                            and min(float(w_rf["left"]), float(w_rf["right"])) > 0.42
                        ):
                            obs_blocking = True
                            break
                    else:
                        if not (
                            abs(ox_w) >= 0.85
                            and float(w_rf["center"]) > 0.90
                            and min(float(w_rf["left"]), float(w_rf["right"])) > 0.42
                        ):
                            obs_blocking = True
                            break
            prev_obs = cur_map
            center_ok = (float(w_rf["center"]) > 0.78) or (
                not w_obs and not wing_approaching and float(w_rf["center"]) > 0.45
            )
            gate_clear = (
                center_ok
                and min(float(w_rf["left"]), float(w_rf["right"])) > 0.38
                and not obs_blocking
            )
            cur_w_sim = float(wait_st.get("sim_time", wait_start_sim))
            if gate_clear:
                if clear_streak == 0 or (cur_w_sim - last_clear_sim_t) >= 0.08:
                    clear_streak += 1
                    last_clear_sim_t = cur_w_sim
                if clear_streak >= 2:
                    gate_opened = True
                    break
            else:
                clear_streak = 0
            if abs(cur_w_sim - prev_w_sim) < 1e-6:
                stagnant_w += 1
                if stagnant_w >= 4:
                    break
            else:
                stagnant_w = 0
            prev_w_sim = cur_w_sim

        if not gate_opened:
            settled = _stop_and_settle(call_fn, include_vision=True)
            stopped_cleanly = True
            return _result(False, "timeout", settled)

        # Băng qua dứt điểm tối đa 1 cổng
        cross_start_odom = _sync_local_odom(wait_st)
        commit_dist = min(1.42, max(0.76, nearest_gate_z + 0.64))
        cur_st = wait_st
        stop_reason = "timeout"
        sprint_done = False
        sim_per_wall = 1.0
        has_rate_est = False
        prev_iter_wall: float | None = None
        prev_iter_sim: float | None = None

        for step_idx in range(1, 140):
            t_iter_wall = time.perf_counter()
            if step_idx > 1:
                cur_st = call_fn("/status?vision=1", "GET", None)
            moved_total = _sync_local_odom(cur_st)
            moved_cross = moved_total - cross_start_odom
            cur_sim_t = float(cur_st.get("sim_time", 0.0))
            if prev_iter_wall is not None and prev_iter_sim is not None:
                dt_w = t_iter_wall - prev_iter_wall
                dt_s = cur_sim_t - prev_iter_sim
                if dt_w > 1e-3 and dt_s > 1e-4:
                    inst_rate = max(0.8, min(10.0, dt_s / dt_w))
                    sim_per_wall = inst_rate if not has_rate_est else (0.5 * sim_per_wall + 0.5 * inst_rate)
                    has_rate_est = True
            prev_iter_wall = t_iter_wall
            prev_iter_sim = cur_sim_t

            rf_c_dict = cur_st["sensors"]["rangefinders_m"]
            rf_c = float(rf_c_dict["center"])
            rf_l = float(rf_c_dict["left"])
            rf_r = float(rf_c_dict["right"])
            rf_l90 = float(rf_c_dict.get("left90", 3.0))
            rf_r90 = float(rf_c_dict.get("right90", 3.0))
            in_narrow = (rf_l90 + rf_r90) < 1.35 or rf_l90 < 0.52 or rf_r90 < 0.52
            c_obs = cur_st.get("vision", {}).get("obstacles", [])

            if rf_c <= 0.34 or min(rf_l, rf_r) <= 0.26:
                stop_reason = "wall_ahead"
                sprint_done = True
                break
            if moved_cross >= nearest_gate_z + 0.54 and (
                any(_estimate_obs_cam_dist_m(o) <= 0.56 and abs(float(o["offset_x"])) < 0.90 for o in c_obs)
                or min(rf_l, rf_c, rf_r) < 0.44
            ):
                stop_reason = "next_obstacle_ahead"
                sprint_done = True
                break
            if moved_cross >= commit_dist - 0.015:
                stop_reason = "cleared"
                sprint_done = True
                break

            cur_yaw = float(cur_st["sensors"]["gyro_yaw_deg"])
            yaw_rate = math.degrees(float(cur_st["sensors"].get("yaw_rate_rad_s", 0.0)))
            yaw_bias = _corridor_heading_bias(rf_c_dict, in_narrow) if in_narrow else 0.0
            yaw_err = _wrap_angle_deg((yaw0_deg + yaw_bias) - cur_yaw)
            trim = max(-3.2, min(3.2, 0.26 * yaw_err - 0.022 * yaw_rate))

            rem = max(0.02, commit_dist - moved_cross)
            spd = 0.58 if rem > 0.20 else max(0.22, 2.0 * rem)
            base_w = min(spd / WHEEL_RADIUS, MAX_WHEEL_RAD_S)
            lw = max(-MAX_WHEEL_RAD_S, min(MAX_WHEEL_RAD_S, base_w - trim))
            rw = max(-MAX_WHEEL_RAD_S, min(MAX_WHEEL_RAD_S, base_w + trim))
            call_fn("/wheels", "POST", {"left": round(lw, 4), "right": round(rw, 4), "duration": 0.30})

            target_wall_dt = 0.045 if not has_rate_est else 0.12 / sim_per_wall
            time.sleep(max(0.002, target_wall_dt - (time.perf_counter() - t_iter_wall)))

        settled = _stop_and_settle(call_fn, include_vision=True)
        if not sprint_done:
            stopped_cleanly = True
            return _result(False, "timeout", settled)
        settled = _apply_asymmetric_shaft_tuck(call_fn, settled, re_center_target=False)
        stopped_cleanly = True
        return _result(True, stop_reason, settled)
    finally:
        if not stopped_cleanly:
            call_fn("/stop", "POST", {})


def dock_target(
    call_fn: BridgeCall,
    stop_distance: float = 0.25,
    max_steps: int = 80,
    step_dt: float = 0.12,
) -> dict:
    """Tiếp cận và căn thẳng tâm mục tiêu ở cự ly gần khi không còn vật cản chắn trước mặt."""
    _require_finite(stop_distance=stop_distance, max_steps=max_steps, step_dt=step_dt)
    if stop_distance < 0.12 or stop_distance > 1.5:
        raise ValueError("Khoảng cách dừng 'stop_distance' phải nằm trong đoạn [0.12, 1.5] mét.")
    if isinstance(max_steps, bool) or float(max_steps) != int(float(max_steps)) or int(max_steps) < 1:
        raise ValueError("Số nhịp 'max_steps' phải là số nguyên lớn hơn hoặc bằng 1.")
    if step_dt < 0.04 or step_dt > 1.0:
        raise ValueError("Chu kỳ điều khiển 'step_dt' phải nằm trong đoạn [0.04, 1.0] giây.")

    watchdog_dur = round(max(0.35, step_dt * 3.0), 3)
    st = call_fn("/status?vision=1", "GET", None)
    odom0 = float(st["sensors"].get("odom_dist_m", 0.0))

    def _dock_moved(end_st: dict) -> float:
        cur_odom = float(end_st["sensors"].get("odom_dist_m", odom0))
        return round(max(0.0, cur_odom - odom0), 4)

    def _refusal(cur_st: dict) -> str | None:
        vis = cur_st.get("vision", {})
        rf = cur_st["sensors"]["rangefinders_m"]
        rf_c = float(rf["center"])
        min_f = min(float(rf["left"]), rf_c, float(rf["right"]))
        if not vis.get("target_visible") or not isinstance(vis.get("target"), dict):
            return "not_visible"
        t_obj = vis["target"]
        t_off = float(t_obj["offset_x"])
        t_area = float(t_obj["area_pct"])
        obs = vis.get("obstacles", [])
        closer_obs = any(
            float(o["area_pct"]) > max(1.0, t_area * 1.2) and abs(float(o["offset_x"])) < 0.35
            for o in obs
        )
        close_to_target = (
            abs(t_off) < 0.35 and (rf_c <= stop_distance + 0.35 or t_area >= 4.0)
        ) or (not obs and t_area >= 3.5)
        blocking_rf = (min_f < 0.38) if obs else (rf_c < 0.38)
        if closer_obs or (blocking_rf and not close_to_target):
            return "obstacle_blocking"
        if rf_c > 1.25 and t_area < 0.90:
            return "too_far"
        return None

    init_err = _refusal(st)
    if init_err is not None:
        return {
            "reached": False,
            "reason": init_err,
            "steps": 0,
            "moved_m": 0.0,
            "after_status": _clean_status(st),
        }

    sim_per_wall = 1.0
    has_rate_est = False
    prev_iter_wall: float | None = None
    prev_iter_sim: float | None = None
    stopped_cleanly = False
    steps_taken = 0
    lost_streak = 0
    last_offset_x = float(st["vision"]["target"]["offset_x"])

    try:
        for step_idx in range(1, int(max_steps) + 1):
            steps_taken = step_idx
            t_iter_wall = time.perf_counter()
            if step_idx > 1:
                st = call_fn("/status?vision=1", "GET", None)
            cur_sim_t = float(st.get("sim_time", 0.0))
            if prev_iter_wall is not None and prev_iter_sim is not None:
                dt_w = t_iter_wall - prev_iter_wall
                dt_s = cur_sim_t - prev_iter_sim
                if dt_w > 1e-3 and dt_s > 1e-4:
                    inst_rate = max(0.8, min(10.0, dt_s / dt_w))
                    sim_per_wall = inst_rate if not has_rate_est else (0.5 * sim_per_wall + 0.5 * inst_rate)
                    has_rate_est = True
            prev_iter_wall = t_iter_wall
            prev_iter_sim = cur_sim_t

            vis = st.get("vision", {})
            rf = st["sensors"]["rangefinders_m"]
            rf_c = float(rf["center"])
            rf_l = float(rf["left"])
            rf_r = float(rf["right"])
            min_f = min(rf_l, rf_c, rf_r)

            seen = bool(vis.get("target_visible")) and isinstance(vis.get("target"), dict)
            if seen:
                lost_streak = 0
                offset_x = float(vis["target"]["offset_x"])
                area_pct = float(vis["target"]["area_pct"])
                last_offset_x = offset_x
                obs = vis.get("obstacles", [])
                closer_obs = any(
                    float(o["area_pct"]) > max(1.0, area_pct * 1.2) and abs(float(o["offset_x"])) < 0.35
                    for o in obs
                )
                blocking_rf_loop = (min_f < 0.34) if obs else (rf_c < 0.34)
                if closer_obs or (blocking_rf_loop and rf_c > stop_distance + 0.14 and area_pct < 3.5):
                    settled = _stop_and_settle(call_fn, include_vision=True)
                    stopped_cleanly = True
                    return {
                        "reached": False,
                        "reason": "obstacle_blocking",
                        "steps": steps_taken,
                        "moved_m": _dock_moved(settled),
                        "after_status": _clean_status(settled),
                    }
            else:
                lost_streak += 1
                if lost_streak > 4:
                    settled = _stop_and_settle(call_fn, include_vision=True)
                    stopped_cleanly = True
                    return {
                        "reached": False,
                        "reason": "not_visible",
                        "steps": steps_taken,
                        "moved_m": _dock_moved(settled),
                        "after_status": _clean_status(settled),
                    }
                w_spin = 2.8 if last_offset_x >= 0.0 else -2.8
                call_fn("/wheels", "POST", {"left": w_spin, "right": -w_spin, "duration": watchdog_dur})
                target_wall_dt = min(0.04, step_dt) if not has_rate_est else step_dt / sim_per_wall
                time.sleep(max(0.002, target_wall_dt - (time.perf_counter() - t_iter_wall)))
                continue

            if rf_c <= stop_distance + 0.04 and (area_pct >= 3.0 or rf_c <= stop_distance + 0.03):
                settled = _stop_and_settle(call_fn, include_vision=True)
                vis_s = settled.get("vision", {})
                if vis_s.get("target_visible") and abs(float(vis_s["target"]["offset_x"])) >= 0.14:
                    corr_deg = -math.degrees(math.atan(float(vis_s["target"]["offset_x"]) * FRONT_CAM_TAN_HALF_FOVX))
                    if abs(corr_deg) >= 0.8:
                        settled = closed_loop_turn(
                            call_fn,
                            degrees=corr_deg,
                            max_speed_deg_s=65.0,
                            tol_deg=0.6,
                            include_vision=True,
                            release_stop=True,
                        )["after_status"]
                        vis_s = settled.get("vision", {})
                rf_s = float(settled["sensors"]["rangefinders_m"]["center"])
                if (
                    vis_s.get("target_visible")
                    and abs(float(vis_s["target"]["offset_x"])) < 0.22
                    and 0.15 <= rf_s <= max(0.35, stop_distance + 0.08)
                ):
                    stopped_cleanly = True
                    return {
                        "reached": True,
                        "reason": "reached_target",
                        "steps": steps_taken,
                        "moved_m": _dock_moved(settled),
                        "after_status": _clean_status(settled),
                    }

            if abs(offset_x) > 0.36:
                spin_w = max(-3.6, min(3.6, 4.2 * offset_x))
                lw, rw = spin_w, -spin_w
            else:
                ramp = max(0.0, min(1.0, (rf_c - stop_distance) / 0.55))
                if rf_c < min(0.14, stop_distance - 0.03):
                    spd = -0.10
                elif rf_c > stop_distance + 0.01:
                    spd = 0.08 + 0.34 * ramp
                else:
                    spd = 0.0
                base_w = spd / WHEEL_RADIUS
                steer = max(-3.8, min(3.8, 3.6 * offset_x))
                lw = max(-MAX_WHEEL_RAD_S, min(MAX_WHEEL_RAD_S, base_w + steer))
                rw = max(-MAX_WHEEL_RAD_S, min(MAX_WHEEL_RAD_S, base_w - steer))

            call_fn("/wheels", "POST", {"left": round(lw, 4), "right": round(rw, 4), "duration": watchdog_dur})
            target_wall_dt = min(0.04, step_dt) if not has_rate_est else step_dt / sim_per_wall
            time.sleep(max(0.002, target_wall_dt - (time.perf_counter() - t_iter_wall)))

        settled = _stop_and_settle(call_fn, include_vision=True)
        stopped_cleanly = True
        vis_f = settled.get("vision", {})
        rf_f = float(settled["sensors"]["rangefinders_m"]["center"])
        ok = (
            bool(vis_f.get("target_visible"))
            and abs(float(vis_f["target"]["offset_x"])) < 0.22
            and 0.15 <= rf_f <= max(0.35, stop_distance + 0.08)
        )
        return {
            "reached": ok,
            "reason": "reached_target" if ok else "max_steps",
            "steps": steps_taken,
            "moved_m": _dock_moved(settled),
            "after_status": _clean_status(settled),
        }
    finally:
        if not stopped_cleanly:
            call_fn("/stop", "POST", {})


def go_to_decision(
    call_fn: BridgeCall,
    *,
    cardinal: str | None = None,
    heading_deg: float | None = None,
    max_dist: float = 6.0,
    speed_m_s: float = 0.55,
    step_dt: float = 0.10,
    max_chain: int = 6,
    auto_chain: bool = True,
    topo=None,
    route: list[str] | tuple[str, ...] | str | None = None,
    stop_at_node: int | None = None,
) -> dict:
    """System 1 gộp (Mode C): xoay khóa hướng, tiến phản xạ 20 Hz, tự lướt góc chữ L, thực thi chuỗi route và dừng đúng nút."""
    from robot_sim.topo import (
        NODE_MATCH_RADIUS_M,
        TopologicalOdometer,
        branches_to_cardinals,
        cardinal_to_deg,
    )

    dir_args_count = sum(x is not None for x in (cardinal, heading_deg, route))
    if dir_args_count > 1:
        raise ValueError("Chỉ được truyền tối đa một trong ba tham số 'cardinal', 'heading_deg', hoặc 'route'.")
    _require_finite(max_dist=max_dist, speed_m_s=speed_m_s, step_dt=step_dt)
    if heading_deg is not None:
        _require_finite(heading_deg=heading_deg)
    max_wheel_linear = MAX_WHEEL_RAD_S * WHEEL_RADIUS
    if max_dist <= 0.0:
        raise ValueError("Tham số 'max_dist' phải lớn hơn 0.")
    if speed_m_s < 0.05 or speed_m_s > max_wheel_linear:
        raise ValueError(f"Tốc độ 'speed_m_s' phải nằm trong khoảng [0.05, {max_wheel_linear:.2f}] m/s.")
    if step_dt < 0.04 or step_dt > 1.0:
        raise ValueError("Chu kỳ điều khiển 'step_dt' phải nằm trong đoạn [0.04, 1.0] giây.")
    if isinstance(max_chain, bool) or not isinstance(max_chain, int) or max_chain < 1:
        raise ValueError("Tham số 'max_chain' phải là số nguyên dương (>= 1).")
    if stop_at_node is not None and (isinstance(stop_at_node, bool) or not isinstance(stop_at_node, int) or stop_at_node < 0):
        raise ValueError("Tham số 'stop_at_node' phải là số nguyên không âm (>= 0).")

    route_steps: list[str] | None = None
    if route is not None:
        if isinstance(route, str):
            raw_tokens = [tok.strip().upper() for tok in route.split(",") if tok.strip()]
        elif isinstance(route, (list, tuple)):
            raw_tokens = [str(tok).strip().upper() for tok in route if str(tok).strip()]
        else:
            raise ValueError("Tham số 'route' phải là chuỗi phân tách dấu phẩy hoặc danh sách hướng E/N/W/S.")
        if not raw_tokens or len(raw_tokens) > 16:
            raise ValueError("Tham số 'route' phải chứa từ 1 đến 16 hướng E/N/W/S.")
        for tok in raw_tokens:
            cardinal_to_deg(tok)
        route_steps = raw_tokens

    single_target_deg: float | None = None
    if cardinal is not None:
        single_target_deg = cardinal_to_deg(cardinal)
    elif heading_deg is not None:
        single_target_deg = float(heading_deg)

    active_topo = topo if topo is not None else TopologicalOdometer(auto_persist=False)

    st = call_fn("/status?vision=1", "GET", None)
    topo_snap = active_topo.observe_status(st)

    stop_node_xy: tuple[float, float] | None = None
    ignore_target_ready = False
    if stop_at_node is not None:
        if stop_at_node >= len(active_topo.nodes):
            raise ValueError(
                f"Nút stop_at_node={stop_at_node} chưa tồn tại trong đồ thị Topo (hiện có {len(active_topo.nodes)} nút)."
            )
        n_xy = active_topo.nodes[stop_at_node]["xy"]
        stop_node_xy = (float(n_xy[0]), float(n_xy[1]))
        ignore_target_ready = active_topo.nodes[stop_at_node].get("kind") != "goal"

    def _run_single_hop(
        cur_st: dict,
        *,
        hop_target_deg: float | None,
        hop_max_dist: float,
        target_odom_xy: tuple[float, float] | None,
        check_blocked_before_drive: bool,
    ) -> tuple[dict, dict, dict, float, int, list[str]]:
        hop_st = cur_st
        if hop_target_deg is not None:
            cur_yaw = float(hop_st["sensors"]["gyro_yaw_deg"])
            turn_deg = _wrap_angle_deg(hop_target_deg - cur_yaw)
            if abs(turn_deg) > 3.0:
                hop_st = closed_loop_turn(
                    call_fn,
                    degrees=turn_deg,
                    max_speed_deg_s=200.0,
                    tol_deg=1.2,
                    include_vision=True,
                    release_stop=False,
                    fast_settle=True,
                )["after_status"]
                active_topo.observe_status(hop_st)

        if check_blocked_before_drive:
            rf_chk = hop_st["sensors"]["rangefinders_m"]
            if (
                _is_front_blocked(rf_chk)
                and float(rf_chk["center"]) < 0.42
                and (ignore_target_ready or not _is_target_ready_for_dock(hop_st))
            ):
                blocked_res = {
                    "event": "route_blocked",
                    "moved_m": 0.0,
                    "open_branches": compute_open_branches(rf_chk),
                    "obstacle": None,
                    "flank_clearance_m": {
                        "left": round(float(rf_chk.get("left90", 3.0)), 3),
                        "right": round(float(rf_chk.get("right90", 3.0)), 3),
                    },
                    "after_status": _clean_status(hop_st),
                }
                return blocked_res, active_topo.observe_status(hop_st), hop_st, 0.0, 0, []

        active_topo.begin_leg(hop_st)
        hop_start_node = active_topo.current_node

        rem = float(hop_max_dist)
        hop_moved = 0.0
        hop_segs = 0
        hop_l_turns: list[str] = []
        any_wall = False
        max_segs = max_chain if auto_chain else 1
        seg_res: dict | None = None

        while hop_segs < max_segs and (hop_segs == 0 or rem > 0.02):
            rf_cur = hop_st["sensors"]["rangefinders_m"]
            l90 = float(rf_cur.get("left90", 3.0))
            r90 = float(rf_cur.get("right90", 3.0))
            l120 = float(rf_cur.get("left120", 3.0))
            r120 = float(rf_cur.get("right120", 3.0))
            truly_open_field = (min(l90, r90, l120, r120) > 1.35) or (
                active_topo.is_open_space_start() and l90 > 0.85 and r90 > 0.85
            )
            seg_max = min(1.8 if truly_open_field else 3.5, rem)

            seg_meta: dict[str, bool] = {}
            seg_res = advance_segment(
                call_fn,
                max_dist=seg_max,
                speed_m_s=speed_m_s,
                step_dt=step_dt,
                _out_meta=seg_meta,
                _target_odom_xy=target_odom_xy,
                _ignore_target_ready=ignore_target_ready,
            )
            hop_segs += 1
            seg_moved = float(seg_res["moved_m"])
            hop_moved += seg_moved
            rem = max(0.0, float(hop_max_dist) - hop_moved)
            seg_saw_wall = bool(seg_meta.get("saw_corridor_wall", False))
            any_wall = any_wall or seg_saw_wall
            hop_st = seg_res["after_status"]

            if not auto_chain or hop_segs >= max_segs or rem <= 0.15:
                break
            if hop_segs >= 2 and seg_moved < 0.20:
                break
            if seg_res["event"] != "junction":
                break

            end_sens = hop_st["sensors"]
            end_x = float(end_sens.get("odom_x_m", 0.0))
            end_y = float(end_sens.get("odom_y_m", 0.0))
            matched_l = active_topo.nearest_existing_node(end_x, end_y, radius_m=NODE_MATCH_RADIUS_M)
            if matched_l is not None:
                is_self_near = matched_l == hop_start_node and hop_moved < 0.80
                is_unseen_goal_corner = (
                    not ignore_target_ready
                    and active_topo.nodes[matched_l].get("kind") == "goal"
                    and not hop_st.get("vision", {}).get("target_visible")
                )
                if not is_self_near and not is_unseen_goal_corner:
                    break

            branches = list(seg_res["open_branches"])
            if branches not in (["left"], ["right"]):
                break

            vis = hop_st.get("vision", {})
            if (not ignore_target_ready and vis.get("target_visible")) or vis.get("obstacles"):
                break

            rf_end = end_sens["rangefinders_m"]
            end_l90 = float(rf_end.get("left90", 3.0))
            end_r90 = float(rf_end.get("right90", 3.0))
            end_l120 = float(rf_end.get("left120", 3.0))
            end_r120 = float(rf_end.get("right120", 3.0))
            blocked_side_rf = end_r90 if branches == ["left"] else end_l90
            corridor_context = seg_saw_wall or (
                blocked_side_rf <= 0.55 and min(end_l120, end_r120) < 1.35
            )
            if not corridor_context:
                break

            branch = branches[0]
            cur_yaw = float(end_sens["gyro_yaw_deg"])
            next_yaw = round((cur_yaw + (90.0 if branch == "left" else -90.0)) / 90.0) * 90.0
            delta_deg = _wrap_angle_deg(next_yaw - cur_yaw)
            hop_st = closed_loop_turn(
                call_fn,
                degrees=delta_deg,
                max_speed_deg_s=200.0,
                tol_deg=1.2,
                include_vision=True,
                release_stop=False,
                fast_settle=True,
            )["after_status"]
            hop_l_turns.append(branch)

        assert seg_res is not None
        final_hop_st = seg_res["after_status"]
        rf_final = final_hop_st["sensors"]["rangefinders_m"]
        end_in_corridor = any_wall or (
            min(float(rf_final.get("left90", 3.0)), float(rf_final.get("right90", 3.0))) <= 0.60
            or float(rf_final.get("center", 3.0)) <= 0.55
        )
        if active_topo.is_open_space_start() and not any_wall:
            end_in_corridor = False

        hop_topo = active_topo.finish_go(
            seg_res["event"],
            hop_moved,
            final_hop_st,
            in_corridor=end_in_corridor,
        )
        return seg_res, hop_topo, final_hop_st, hop_moved, hop_segs, hop_l_turns

    total_moved_m = 0.0
    total_segments_chained = 0
    all_auto_l_turns: list[str] = []
    route_completed = 0
    last_seg_res: dict | None = None
    final_st = st

    if route_steps is None:
        hop_deg = single_target_deg
        if hop_deg is None and stop_node_xy is not None and active_topo.is_open_space_start():
            cx = float(st["sensors"].get("odom_x_m", 0.0))
            cy = float(st["sensors"].get("odom_y_m", 0.0))
            dx, dy = stop_node_xy[0] - cx, stop_node_xy[1] - cy
            if math.hypot(dx, dy) > 0.18:
                hop_deg = math.degrees(math.atan2(dy, dx))
        last_seg_res, topo_snap, final_st, total_moved_m, total_segments_chained, all_auto_l_turns = _run_single_hop(
            st,
            hop_target_deg=hop_deg,
            hop_max_dist=float(max_dist),
            target_odom_xy=stop_node_xy,
            check_blocked_before_drive=False,
        )
    else:
        num_steps = len(route_steps)
        total_budget = (
            max(float(max_dist), 6.0 * num_steps)
            if (num_steps > 1 and abs(float(max_dist) - 6.0) < 1e-9)
            else float(max_dist)
        )
        cur_st = st
        for idx, step_card in enumerate(route_steps):
            is_last = idx == (num_steps - 1)
            rem_budget = max(0.0, total_budget - total_moved_m)
            if idx > 0 and rem_budget <= 0.05:
                break
            hop_budget = min(6.0, rem_budget) if num_steps > 1 else rem_budget

            hop_target_xy: tuple[float, float] | None = None
            is_intermediate_wp = False
            if is_last and stop_node_xy is not None:
                hop_target_xy = stop_node_xy
            elif not is_last and active_topo.current_node is not None:
                exits_cur = active_topo.nodes[active_topo.current_node].get("exits", {})
                e_info = exits_cur.get(step_card)
                if (
                    isinstance(e_info, dict)
                    and e_info.get("state") == "explored"
                    and e_info.get("to_node") is not None
                    and route_steps[idx + 1] != step_card
                ):
                    nid = int(e_info["to_node"])
                    if 0 <= nid < len(active_topo.nodes):
                        nx, ny = active_topo.nodes[nid]["xy"]
                        hop_target_xy = (float(nx), float(ny))
                        is_intermediate_wp = True

            hop_start_id = active_topo.current_node
            hop_res, topo_snap, cur_st, h_moved, h_segs, h_turns = _run_single_hop(
                cur_st,
                hop_target_deg=cardinal_to_deg(step_card),
                hop_max_dist=hop_budget,
                target_odom_xy=hop_target_xy,
                check_blocked_before_drive=(idx > 0),
            )
            last_seg_res = hop_res
            final_st = cur_st
            total_moved_m += h_moved
            total_segments_chained += h_segs
            all_auto_l_turns.extend(h_turns)

            h_event = hop_res["event"]
            if h_event == "route_blocked":
                break
            if is_intermediate_wp and h_event == "arrived_node":
                h_event = "junction"
                last_seg_res = {**hop_res, "event": "junction"}

            stalled_at_start = (
                hop_start_id is not None
                and topo_snap["current_node"] == hop_start_id
                and h_moved < 0.80
            )
            if is_last:
                route_completed = (
                    num_steps
                    if (not stalled_at_start and (topo_snap["current_node"] is not None or h_event in ("target_ready", "arrived_node")))
                    else idx
                )
                break

            if (
                stalled_at_start
                or h_event in ("dynamic_gate_ahead", "static_obstacle_ahead", "target_ready", "arrived_node", "dead_end", "max_dist")
                or topo_snap["current_node"] is None
            ):
                route_completed = idx if (stalled_at_start or topo_snap["current_node"] is None) else (idx + 1)
                break
            route_completed = idx + 1

    assert last_seg_res is not None
    final_event = str(last_seg_res["event"])

    # Tinh chỉnh nhẹ & gắn nhãn arrived_node nếu đã về đúng nút stop_at_node
    if stop_at_node is not None and stop_node_xy is not None and final_event in ("junction", "dead_end", "arrived_node", "max_dist"):
        fx = float(final_st["sensors"].get("odom_x_m", 0.0))
        fy = float(final_st["sensors"].get("odom_y_m", 0.0))
        d_final = math.hypot(fx - stop_node_xy[0], fy - stop_node_xy[1])
        if d_final <= NODE_MATCH_RADIUS_M or topo_snap["current_node"] == stop_at_node:
            if 0.20 <= d_final <= NODE_MATCH_RADIUS_M:
                fyaw_rad = math.radians(float(final_st["sensors"]["gyro_yaw_deg"]))
                proj = (stop_node_xy[0] - fx) * math.cos(fyaw_rad) + (stop_node_xy[1] - fy) * math.sin(fyaw_rad)
                rf_trim = final_st["sensors"]["rangefinders_m"]
                if proj > 0.12 and float(rf_trim["center"]) > proj + 0.25:
                    trim_m = min(0.22, proj)
                    trim_res = closed_loop_move(
                        call_fn,
                        meters=trim_m,
                        speed_m_s=0.25,
                        include_vision=True,
                        release_stop=True,
                    )
                    final_st = trim_res["after_status"]
                    total_moved_m += abs(float(trim_res.get("measured_meters", trim_m)))
                    topo_snap = active_topo.observe_status(final_st)
            final_event = "arrived_node"

    final_yaw = float(final_st["sensors"]["gyro_yaw_deg"])
    final_branches = compute_open_branches(final_st["sensors"]["rangefinders_m"])
    result_dict: dict[str, Any] = {
        "event": final_event,
        "moved_m": round(total_moved_m, 4),
        "segments_chained": total_segments_chained,
        "auto_l_turns": all_auto_l_turns,
        "open_branches": final_branches,
        "open_cardinals": branches_to_cardinals(final_branches, final_yaw),
        "obstacle": last_seg_res["obstacle"],
        "flank_clearance_m": last_seg_res["flank_clearance_m"],
        "topo": topo_snap,
        "after_status": _clean_status(final_st),
    }
    if route_steps is not None or stop_at_node is not None:
        req = list(route_steps) if route_steps is not None else ([cardinal.strip().upper()] if cardinal is not None else [])
        comp = route_completed if route_steps is not None else (1 if (req and final_event in ("arrived_node", "junction", "target_ready")) else 0)
        result_dict["route_progress"] = {
            "requested": req,
            "completed": comp,
            "remaining": list(req[comp:]) if comp < len(req) else [],
            "stop_at_node": stop_at_node,
        }
    return result_dict
