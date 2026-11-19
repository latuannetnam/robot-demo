from __future__ import annotations

from datetime import datetime, timezone
import http.client
import json
import math
from pathlib import Path
import threading
import time
import urllib.error
from urllib.parse import urlsplit

from robot_sim.arenas import ARENAS
from robot_sim.core import PROJECT_ROOT, _require_finite

DEFAULT_TELEMETRY_PATH = PROJECT_ROOT / "captures" / "telemetry_latest.json"
DEFAULT_STOP_FLAG_PATH = PROJECT_ROOT / "captures" / ".telemetry_stop"
_THREAD_CONNS = threading.local()


def _get_conn_pool() -> dict[tuple[str, str, int, float], http.client.HTTPConnection]:
    pool = getattr(_THREAD_CONNS, "pool", None)
    if pool is None:
        pool = {}
        _THREAD_CONNS.pool = pool
    return pool


def _fetch_raw_status(base_url: str, timeout: float = 4.0) -> dict:
    parsed_url = urlsplit(base_url)
    scheme = parsed_url.scheme.lower()
    host = parsed_url.hostname or "127.0.0.1"
    port = parsed_url.port or (443 if scheme == "https" else 80)
    base_path = parsed_url.path.rstrip("/")
    req_path = f"{base_path}/status?vision=1"
    key = (scheme, host, port, float(timeout))
    pool = _get_conn_pool()

    for attempt in range(2):
        conn = pool.get(key)
        was_reused = conn is not None
        if conn is None:
            cls = http.client.HTTPSConnection if scheme == "https" else http.client.HTTPConnection
            conn = cls(host, port, timeout=timeout)
            pool[key] = conn
        try:
            conn.request("GET", req_path, headers={"Content-Type": "application/json", "Connection": "keep-alive"})
            resp = conn.getresponse()
            raw = resp.read()
            if resp.status >= 400:
                raise urllib.error.HTTPError(
                    f"{base_url.rstrip('/')}/status?vision=1",
                    resp.status,
                    resp.reason,
                    resp.headers,
                    None,
                )
            if resp.will_close:
                conn.close()
                pool.pop(key, None)
            parsed = json.loads(raw.decode("utf-8"))
            return parsed.get("data", parsed)
        except (http.client.HTTPException, TimeoutError, ConnectionError, OSError):
            pool.pop(key, None)
            try:
                conn.close()
            except Exception:
                pass
            if was_reused and attempt == 0:
                continue
            raise
    raise RuntimeError("Unexpected retry exit in _fetch_raw_status")


def record_telemetry(
    base_url: str,
    out_path: Path | str = DEFAULT_TELEMETRY_PATH,
    stop_flag_path: Path | str = DEFAULT_STOP_FLAG_PATH,
    hz: float = 10.0,
    max_duration_s: float = 300.0,
) -> dict:
    """Ghi hộp đen 10Hz chạy nền (0 token LLM) và xuất báo cáo JSON gọn nhẹ (< 2 KB)."""
    if not isinstance(base_url, str) or not base_url.startswith(("http://", "https://")):
        raise ValueError(f"Địa chỉ base_url không hợp lệ: {base_url!r}")
    _require_finite(hz=hz, max_duration_s=max_duration_s)
    if hz <= 0.0 or hz > 100.0:
        raise ValueError("Tần số lấy mẫu 'hz' phải nằm trong khoảng (0, 100].")
    if max_duration_s <= 0.0:
        raise ValueError("Thời lượng tối đa 'max_duration_s' phải lớn hơn 0.")

    resolved_out = Path(out_path).resolve()
    resolved_stop = Path(stop_flag_path).resolve()
    resolved_out.parent.mkdir(parents=True, exist_ok=True)
    resolved_stop.parent.mkdir(parents=True, exist_ok=True)
    resolved_stop.unlink(missing_ok=True)

    started_iso = datetime.now(timezone.utc).isoformat()
    t_wall_start = time.perf_counter()
    period = 1.0 / float(hz)

    sample_count = 0
    error_count = 0
    last_error: str | None = None
    latencies_ms: list[float] = []
    active_samples = 0
    active_drive_time_s = 0.0
    agent_think_time_s = 0.0
    prev_poll_wall: float | None = None
    last_is_active = False
    command_burst_count = 0
    last_cell: tuple[int, int] | None = None
    last_dir_cell: tuple[tuple[int, int], int] | None = None
    cell_last_path: dict[tuple[int, int], float] = {}
    dir_cell_last_path: dict[tuple[tuple[int, int], int], float] = {}
    backtrack_revisit_count = 0
    redundant_revisit_count = 0

    first_sim_t: float | None = None
    last_sim_t: float | None = None
    arena_id: str | None = None
    start_xy: list[float] | None = None
    prev_xy: list[float] | None = None
    end_xy: list[float] | None = None
    end_yaw_deg: float | None = None
    target_xy: list[float] | None = None
    path_length_m = 0.0

    min_rf_m = 3.0
    near_collision_count = 0
    base_contact_count: int | None = None
    latest_contact_count = 0
    max_post_stop_speed = 0.0
    braking_rf_drop_m = 0.0

    prev_moving = False
    prev_rf_center: float | None = None
    prev_yaw_deg: float | None = None
    post_stop_window = 0
    rf_at_stop_onset: float | None = None
    xy_at_stop_onset: list[float] | None = None
    yaw_at_stop_onset: float | None = None

    final_rf_center: float | None = None
    final_target_visible = False
    final_target_offset_x: float | None = None

    try:
        while not resolved_stop.exists():
            now_wall = time.perf_counter()
            if (now_wall - t_wall_start) >= max_duration_s:
                break

            t0 = time.perf_counter()
            try:
                st = _fetch_raw_status(base_url)
                lat_ms = (time.perf_counter() - t0) * 1000.0
                latencies_ms.append(lat_ms)
                sample_count += 1
                dt_sample = (t0 - prev_poll_wall) if prev_poll_wall is not None else max(0.0, t0 - t_wall_start)
                prev_poll_wall = t0

                sim_t = float(st["sim_time"])
                if first_sim_t is None:
                    first_sim_t = sim_t
                last_sim_t = sim_t

                moving = bool(st.get("moving", False))
                sensors = st.get("sensors", {})
                rf = sensors.get("rangefinders_m", {})
                rf_c = float(rf.get("center", 3.0))
                fwd_spd = abs(float(sensors.get("forward_speed_m_s", 0.0)))
                yaw_spd = abs(float(sensors.get("yaw_rate_rad_s", 0.0)))

                is_active = bool(moving or fwd_spd > 0.02 or yaw_spd > 0.04)
                if is_active:
                    active_samples += 1
                    active_drive_time_s += dt_sample
                else:
                    agent_think_time_s += dt_sample
                last_is_active = is_active
                if moving and not prev_moving:
                    command_burst_count += 1

                dbg = st.get("debug", {})
                if isinstance(dbg.get("arena"), str) and dbg["arena"]:
                    arena_id = dbg["arena"]
                reset_jumped = False
                curr_xy = dbg.get("robot_xy")
                if isinstance(curr_xy, list) and len(curr_xy) == 2:
                    if start_xy is None:
                        start_xy = [float(curr_xy[0]), float(curr_xy[1])]
                    if prev_xy is not None:
                        step_dist = math.hypot(float(curr_xy[0]) - prev_xy[0], float(curr_xy[1]) - prev_xy[1])
                        # Ignore discontinuous reset jumps (> 0.45m between consecutive polls)
                        if step_dist <= 0.45:
                            path_length_m += step_dist
                        else:
                            start_xy = [float(curr_xy[0]), float(curr_xy[1])]
                            path_length_m = 0.0
                            reset_jumped = True
                            cell_last_path.clear()
                            dir_cell_last_path.clear()
                            last_cell = None
                            last_dir_cell = None
                            backtrack_revisit_count = 0
                            redundant_revisit_count = 0
                    prev_xy = [float(curr_xy[0]), float(curr_xy[1])]
                    end_xy = prev_xy

                if "contact_count" in dbg:
                    cur_cc = int(dbg["contact_count"])
                    if base_contact_count is None or reset_jumped or cur_cc < latest_contact_count:
                        base_contact_count = cur_cc
                    latest_contact_count = cur_cc

                if "robot_yaw_deg" in dbg:
                    end_yaw_deg = float(dbg["robot_yaw_deg"])
                if "target_xy" in dbg:
                    target_xy = [float(dbg["target_xy"][0]), float(dbg["target_xy"][1])]

                if isinstance(curr_xy, list) and len(curr_xy) == 2:
                    cell = (round(float(curr_xy[0]) / 0.50), round(float(curr_xy[1]) / 0.50))
                    if cell != last_cell:
                        if cell in cell_last_path and (path_length_m - cell_last_path[cell]) >= 0.80:
                            backtrack_revisit_count += 1
                        last_cell = cell
                        if end_yaw_deg is not None:
                            dir_q = int(((end_yaw_deg + 45.0) % 360.0) // 90.0)
                            dcell = (cell, dir_q)
                            if dcell != last_dir_cell:
                                if dcell in dir_cell_last_path and (path_length_m - dir_cell_last_path[dcell]) >= 0.80:
                                    redundant_revisit_count += 1
                                last_dir_cell = dcell
                            dir_cell_last_path[dcell] = path_length_m
                    cell_last_path[cell] = path_length_m

                vis = st.get("vision", {})
                t_vis = bool(vis.get("target_visible", False))
                t_obj = vis.get("target")
                t_off = float(t_obj["offset_x"]) if (t_vis and isinstance(t_obj, dict)) else None

                final_rf_center = round(rf_c, 4)
                final_target_visible = t_vis
                final_target_offset_x = round(t_off, 3) if t_off is not None else None

                cur_min_rf = min((float(v) for v in rf.values()), default=3.0)
                min_rf_m = min(min_rf_m, cur_min_rf)
                facing_target_closely = t_vis and t_off is not None and abs(t_off) < 0.28
                if cur_min_rf < 0.12 and not facing_target_closely:
                    near_collision_count += 1

                if not moving:
                    max_post_stop_speed = max(max_post_stop_speed, fwd_spd)

                if prev_moving and not moving:
                    post_stop_window = 3
                    rf_at_stop_onset = rf_c
                    xy_at_stop_onset = end_xy
                    yaw_at_stop_onset = end_yaw_deg
                elif (
                    not moving
                    and post_stop_window > 0
                    and rf_at_stop_onset is not None
                    and xy_at_stop_onset is not None
                    and end_xy is not None
                ):
                    drift = math.hypot(end_xy[0] - xy_at_stop_onset[0], end_xy[1] - xy_at_stop_onset[1])
                    yaw_drift = (
                        abs(((end_yaw_deg - yaw_at_stop_onset + 180.0) % 360.0) - 180.0)
                        if (end_yaw_deg is not None and yaw_at_stop_onset is not None)
                        else 0.0
                    )
                    if drift < 0.35 and yaw_drift < 3.0:
                        drop = min(rf_at_stop_onset - rf_c, drift + 0.02)
                        if drop > braking_rf_drop_m:
                            braking_rf_drop_m = drop
                    post_stop_window -= 1

                prev_moving = moving
                prev_rf_center = rf_c
                prev_yaw_deg = end_yaw_deg
            except (urllib.error.URLError, http.client.HTTPException, TimeoutError, ConnectionError, OSError, ValueError) as exc:
                error_count += 1
                last_error = str(exc)

            elapsed_iter = time.perf_counter() - t0
            sleep_s = period - elapsed_iter
            if sleep_s > 0:
                time.sleep(min(sleep_s, 0.10))
    except KeyboardInterrupt:
        pass

    t_wall_end = time.perf_counter()
    wall_time_s = max(1e-4, t_wall_end - t_wall_start)
    if sample_count > 0 and prev_poll_wall is not None:
        tail_dt = max(0.0, t_wall_end - prev_poll_wall)
        if error_count > 0:
            tail_dt = min(period, tail_dt)
        if last_is_active:
            active_drive_time_s += tail_dt
        else:
            agent_think_time_s += tail_dt
    sim_time_s = max(0.0, (last_sim_t - first_sim_t)) if (first_sim_t is not None and last_sim_t is not None) else 0.0

    lat_sorted = sorted(latencies_ms)
    if lat_sorted:
        p95_idx = min(len(lat_sorted) - 1, int(0.95 * len(lat_sorted)))
        http_lat = {
            "avg": round(sum(lat_sorted) / len(lat_sorted), 2),
            "p95": round(lat_sorted[p95_idx], 2),
            "max": round(lat_sorted[-1], 2),
        }
    else:
        http_lat = {"avg": 0.0, "p95": 0.0, "max": 0.0}

    goal_reached = bool(
        final_target_visible
        and final_target_offset_x is not None
        and abs(final_target_offset_x) < 0.25
        and final_rf_center is not None
        and 0.15 <= final_rf_center <= 0.35
    )

    center_to_target_m = (
        round(math.hypot(target_xy[0] - end_xy[0], target_xy[1] - end_xy[1]), 4)
        if (target_xy is not None and end_xy is not None)
        else None
    )

    contact_count = max(0, latest_contact_count - (base_contact_count or 0))
    arena_spec = ARENAS.get(arena_id or "")
    reference_path_m = (
        round(arena_spec.reference_path_m, 4)
        if (
            arena_spec is not None
            and arena_spec.reference_path_m > 0.0
            and target_xy is not None
            and math.hypot(
                target_xy[0] - arena_spec.default_target_xy[0],
                target_xy[1] - arena_spec.default_target_xy[1],
            )
            <= 0.05
        )
        else None
    )
    path_efficiency = (
        round(min(1.0, reference_path_m / path_length_m), 3)
        if (reference_path_m is not None and goal_reached and path_length_m > 0.0)
        else None
    )
    summary = {
        "started_at": started_iso,
        "arena": arena_id,
        "wall_time_s": round(wall_time_s, 2),
        "sim_time_s": round(sim_time_s, 2),
        "active_drive_time_s": round(active_drive_time_s, 2),
        "agent_think_time_s": round(agent_think_time_s, 2),
        "realtime_ratio": round(sim_time_s / wall_time_s, 3) if sim_time_s > 0 else 0.0,
        "sample_count": sample_count,
        "error_count": error_count,
        "last_error": last_error,
        "http_latency_ms": http_lat,
        "duty_cycle_pct": round((100.0 * active_samples) / sample_count, 1) if sample_count > 0 else 0.0,
        "command_burst_count": command_burst_count,
        "unique_cells_visited": len(cell_last_path),
        "backtrack_revisit_count": backtrack_revisit_count,
        "redundant_revisit_count": redundant_revisit_count,
        "start_xy": start_xy,
        "end_xy": end_xy,
        "end_yaw_deg": end_yaw_deg,
        "target_xy": target_xy,
        "center_to_target_m": center_to_target_m,
        "path_length_m": round(path_length_m, 4),
        "reference_path_m": reference_path_m,
        "path_efficiency": path_efficiency,
        "final_rf_center_m": final_rf_center,
        "final_target_visible": final_target_visible,
        "final_target_offset_x": final_target_offset_x,
        "goal_reached": goal_reached,
        "min_rangefinder_m": round(min_rf_m, 4) if sample_count > 0 else None,
        "near_collision_count": near_collision_count,
        "contact_count": contact_count,
        "anomalies": {
            "braking_rf_drop_m": round(braking_rf_drop_m, 4),
            "max_post_stop_speed_m_s": round(max_post_stop_speed, 4),
        },
    }

    tmp_file = resolved_out.with_name(f"{resolved_out.name}.tmp")
    tmp_file.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    tmp_file.replace(resolved_out)
    resolved_stop.unlink(missing_ok=True)
    return summary


def stop_telemetry(
    out_path: Path | str = DEFAULT_TELEMETRY_PATH,
    stop_flag_path: Path | str = DEFAULT_STOP_FLAG_PATH,
    wait_timeout_s: float = 3.0,
) -> dict:
    """Gửi tín hiệu dừng cho tiến trình Hộp đen chạy nền và trả về báo cáo JSON vừa xuất."""
    _require_finite(wait_timeout_s=wait_timeout_s)
    if wait_timeout_s <= 0.0:
        raise ValueError("Tham số 'wait_timeout_s' phải lớn hơn 0.")

    resolved_out = Path(out_path).resolve()
    resolved_stop = Path(stop_flag_path).resolve()
    resolved_stop.parent.mkdir(parents=True, exist_ok=True)

    prev_mtime_ns = resolved_out.stat().st_mtime_ns if resolved_out.is_file() else None
    resolved_stop.write_text("stop\n", encoding="utf-8")

    deadline = time.monotonic() + wait_timeout_s
    while time.monotonic() < deadline:
        if resolved_out.is_file() and not resolved_stop.exists():
            cur_mtime = resolved_out.stat().st_mtime_ns
            if prev_mtime_ns is None or cur_mtime >= prev_mtime_ns:
                return json.loads(resolved_out.read_text(encoding="utf-8"))
        time.sleep(0.03)

    resolved_stop.unlink(missing_ok=True)
    raise RuntimeError(
        f"Hộp đen không phản hồi sau {wait_timeout_s:.1f}s (tiến trình 'telemetry record' chưa chạy hoặc đã kết thúc)."
    )
