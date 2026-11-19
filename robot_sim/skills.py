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
        rf_end_s = settled_end["sensors"]["rangefinders_m"]
        l90_e = float(rf_end_s.get("left90", 3.0))
        r90_e = float(rf_end_s.get("right90", 3.0))
        wall_e = min(l90_e, r90_e)
        open_e = max(l90_e, r90_e)
        if (
            (gates_yielded > 0 or bool(settled_end["vision"].get("obstacles")))
            and float(rf_end_s["center"]) > 0.90
            and open_e > 0.85
            and wall_e > 0.29
        ):
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
            settled_end = closed_loop_turn(
                call_fn,
                degrees=-tuck_ang * tuck_sign,
                max_speed_deg_s=120.0,
                tol_deg=1.0,
                include_vision=True,
                release_stop=True,
            )["after_status"]
            _sync_odom(settled_end)
            if (
                settled_end["vision"]["target_visible"]
                and abs(float(settled_end["vision"]["target"]["offset_x"])) > 0.22
            ):
                fix_deg = -math.degrees(
                    math.atan(float(settled_end["vision"]["target"]["offset_x"]) * FRONT_CAM_TAN_HALF_FOVX)
                )
                if abs(fix_deg) >= 0.8:
                    settled_end = closed_loop_turn(
                        call_fn,
                        degrees=fix_deg,
                        max_speed_deg_s=75.0,
                        tol_deg=0.6,
                        include_vision=True,
                        release_stop=True,
                    )["after_status"]
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
