import threading
import time
import pytest
from sim_server import SimulationApp
import robot_ctl
from robot_sim.skills import approach_target, closed_loop_move, closed_loop_turn, explore_maze, scan_for_target


def _wrap_deg(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


def test_closed_loop_turn_small_and_large_angles_and_skills_validate_inputs():
    app = SimulationApp()
    host, port = app.start_http_server(host="127.0.0.1", port=0)
    url = f"http://{host}:{port}"
    call_fn = lambda path, method="GET", payload=None: robot_ctl._call_bridge(url, path, method, payload)
    stop_evt = threading.Event()
    holder: dict = {}
    err_holder: list[BaseException] = []

    def worker():
        try:
            call_fn("/reset", "POST", {"target_xy": [-1.4, 1.15]})
            # Small-angle precision checks (+6 deg and -12 deg)
            s1 = closed_loop_turn(call_fn, 6.0, max_speed_deg_s=60.0, include_vision=False)
            yaw_after_6 = call_fn("/status", "GET")["debug"]["robot_yaw_deg"]
            s2 = closed_loop_turn(call_fn, -12.0, max_speed_deg_s=60.0, include_vision=False)
            yaw_after_minus6 = call_fn("/status", "GET")["debug"]["robot_yaw_deg"]

            # Large-angle +180 boundary crossing (+150 deg then +66 deg from -6 deg => +210 deg = -150 deg)
            res1 = closed_loop_turn(call_fn, 150.0, max_speed_deg_s=95.0, include_vision=False)
            res2 = closed_loop_turn(call_fn, 66.0, max_speed_deg_s=95.0, include_vision=True)
            raw_status = call_fn("/status", "GET")

            # Fast-settle 90 deg high-rate turn used inside explore_maze
            t_fast_start = float(raw_status["sim_time"])
            res_fast = closed_loop_turn(
                call_fn,
                -90.0,
                max_speed_deg_s=240.0,
                tol_deg=1.4,
                include_vision=False,
                release_stop=False,
                fast_settle=True,
            )

            holder["s1"] = s1
            holder["s2"] = s2
            holder["yaw_after_6"] = yaw_after_6
            holder["yaw_after_minus6"] = yaw_after_minus6
            holder["res1"] = res1
            holder["res2"] = res2
            holder["true_yaw"] = raw_status["debug"]["robot_yaw_deg"]
            holder["res_fast"] = res_fast
            holder["fast_sim_dt"] = float(res_fast["after_status"]["sim_time"]) - t_fast_start

            for fn, kwargs in (
                (closed_loop_turn, {"degrees": 0.0}),
                (closed_loop_move, {"meters": 0.0}),
                (closed_loop_move, {"meters": 0.5, "speed_m_s": -0.1}),
                (scan_for_target, {"max_degrees": 0.0}),
                (scan_for_target, {"direction": "up"}),
                (approach_target, {"stop_distance": 0.05}),
                (approach_target, {"stop_distance": 2.0}),
                (explore_maze, {"rule": "up"}),
                (explore_maze, {"max_steps": 0}),
                (explore_maze, {"speed_m_s": -0.1}),
            ):
                with pytest.raises(ValueError):
                    fn(call_fn, **kwargs)
        except BaseException as exc:
            err_holder.append(exc)
        finally:
            stop_evt.set()

    t = threading.Thread(target=worker, daemon=True)
    t.start()
    try:
        while not stop_evt.is_set():
            app.tick_headless(step_batch=12)
            time.sleep(0.004)
        t.join(timeout=20.0)
    finally:
        app.shutdown()

    if err_holder:
        raise err_holder[0]

    assert abs(holder["s1"]["measured_turn_deg"] - 6.0) < 1.0
    assert abs(holder["yaw_after_6"] - 6.0) < 1.1
    assert abs(holder["s2"]["measured_turn_deg"] - (-12.0)) < 1.0
    assert abs(holder["yaw_after_minus6"] - (-6.0)) < 1.2

    assert "debug" not in holder["res1"]["after_status"]
    assert "debug" not in holder["res2"]["after_status"]
    assert "vision" in holder["res2"]["after_status"]
    assert abs(_wrap_deg(holder["true_yaw"] - (-150.0))) < 2.0
    assert abs(holder["res_fast"]["measured_turn_deg"] - (-90.0)) < 2.5
    assert holder["fast_sim_dt"] < 2.2


def test_closed_loop_move_holds_heading_under_1_deg_and_safety_stops():
    app = SimulationApp()
    host, port = app.start_http_server(host="127.0.0.1", port=0)
    url = f"http://{host}:{port}"
    call_fn = lambda path, method="GET", payload=None: robot_ctl._call_bridge(url, path, method, payload)
    stop_evt = threading.Event()
    holder: dict = {}
    err_holder: list[BaseException] = []

    def worker():
        try:
            # 1. Turn -35 deg into open space and drive 1.0m at 0.35 m/s; check ground-truth yaw drift < 1.0 deg
            call_fn("/reset", "POST", {"target_xy": [-1.4, 1.15]})
            closed_loop_turn(call_fn, -135.0, max_speed_deg_s=90.0)
            yaw_before = call_fn("/status", "GET")["debug"]["robot_yaw_deg"]
            mv = closed_loop_move(call_fn, meters=1.0, speed_m_s=0.35, include_vision=True)
            st_after = call_fn("/status", "GET")
            yaw_after = st_after["debug"]["robot_yaw_deg"]

            # 2. Also verify fallback velocity integration (when live server lacks 'odom_dist_m') stays accurate & straight
            call_fn("/reset", "POST", {"target_xy": [-1.4, 1.15]})

            def legacy_call_fn(path: str, method: str = "GET", payload: dict | None = None) -> dict:
                res = call_fn(path, method, payload)
                if method == "GET" and isinstance(res, dict) and "sensors" in res:
                    sensors = {k: v for k, v in res["sensors"].items() if k != "odom_dist_m"}
                    res = {**res, "sensors": sensors}
                return res

            mv_legacy = closed_loop_move(legacy_call_fn, meters=0.80, speed_m_s=0.35)
            legacy_true_x = call_fn("/status", "GET")["debug"]["robot_xy"][0]

            # 3. Place target 0.45m directly ahead and request 0.90m forward move -> must safety-stop at rf_c < 0.22m
            call_fn("/reset", "POST", {"target_xy": [0.55, 0.0]})
            mv_stop = closed_loop_move(call_fn, meters=0.90, speed_m_s=0.30)

            holder["mv"] = mv
            holder["yaw_drift"] = abs(_wrap_deg(yaw_after - yaw_before))
            holder["mv_legacy"] = mv_legacy
            holder["legacy_true_x"] = legacy_true_x
            holder["mv_stop"] = mv_stop
        except BaseException as exc:
            err_holder.append(exc)
        finally:
            stop_evt.set()

    t = threading.Thread(target=worker, daemon=True)
    t.start()
    try:
        while not stop_evt.is_set():
            app.tick_headless(step_batch=12)
            time.sleep(0.004)
        t.join(timeout=20.0)
    finally:
        app.shutdown()

    if err_holder:
        raise err_holder[0]

    assert "debug" not in holder["mv"]["after_status"]
    assert "vision" in holder["mv"]["after_status"]
    assert 0.94 <= holder["mv"]["measured_meters"] <= 1.06
    assert holder["yaw_drift"] < 1.0

    assert 0.68 <= holder["legacy_true_x"] <= 0.92
    assert "debug" not in holder["mv_legacy"]["after_status"]

    assert holder["mv_stop"]["stopped_early"] is True
    assert holder["mv_stop"]["stop_reason"] == "obstacle_ahead"
    assert "debug" not in holder["mv_stop"]["after_status"]


def test_scan_tight_centering_and_long_range_corridor_approach():
    app = SimulationApp()
    host, port = app.start_http_server(host="127.0.0.1", port=0)
    url = f"http://{host}:{port}"
    call_fn = lambda path, method="GET", payload=None: robot_ctl._call_bridge(url, path, method, payload)
    stop_evt = threading.Event()
    holder: dict = {}
    err_holder: list[BaseException] = []

    def worker():
        try:
            # 1. Target initially visible near left edge of FOV: scan must center tightly (<= 0.10) & report degrees > 15
            call_fn("/reset", "POST", {"target_xy": [1.2, 0.55]})
            scan_in_fov = scan_for_target(call_fn, max_degrees=180.0, direction="left", angular_speed_deg_s=75.0)

            # 2. Occluded Round-1 challenge [-1.40, 1.15]: move 0.85m -> scan -> single long-range corridor approach
            #    (also inject a 3-step mid-corridor occlusion on steps 6..8 to verify transient target-loss recovery!)
            call_fn("/reset", "POST", {"target_xy": [-1.40, 1.15]})
            closed_loop_move(call_fn, meters=0.85, speed_m_s=0.35)
            scan_occ = scan_for_target(call_fn, max_degrees=360.0, direction="left", angular_speed_deg_s=85.0)

            vis_polls = {"n": 0}

            def occluding_call_fn(path: str, method: str = "GET", payload: dict | None = None) -> dict:
                res = call_fn(path, method, payload)
                if method == "GET" and "vision=1" in path:
                    vis_polls["n"] += 1
                    if 6 <= vis_polls["n"] <= 8:
                        res = {
                            **res,
                            "vision": {**res["vision"], "target_visible": False, "target": None},
                        }
                return res

            app_occ = approach_target(occluding_call_fn, stop_distance=0.25)

            holder["scan_in_fov"] = scan_in_fov
            holder["scan_occ"] = scan_occ
            holder["app_occ"] = app_occ
        except BaseException as exc:
            err_holder.append(exc)
        finally:
            stop_evt.set()

    t = threading.Thread(target=worker, daemon=True)
    t.start()
    try:
        while not stop_evt.is_set():
            app.tick_headless(step_batch=12)
            time.sleep(0.004)
        t.join(timeout=30.0)
    finally:
        app.shutdown()

    if err_holder:
        raise err_holder[0]

    s_fov = holder["scan_in_fov"]
    assert s_fov["target_found"] is True
    assert s_fov["scanned_degrees"] > 15.0
    assert abs(s_fov["after_status"]["vision"]["target"]["offset_x"]) <= 0.10
    assert "debug" not in s_fov["after_status"]

    s_occ = holder["scan_occ"]
    assert s_occ["target_found"] is True
    assert abs(s_occ["after_status"]["vision"]["target"]["offset_x"]) <= 0.10
    assert "debug" not in s_occ["after_status"]

    a_occ = holder["app_occ"]
    assert a_occ["reached"] is True, f"Corridor approach failed: {a_occ}"
    assert "debug" not in a_occ["after_status"]
    vis_end = a_occ["after_status"]["vision"]
    rf_c = a_occ["after_status"]["sensors"]["rangefinders_m"]["center"]
    assert vis_end["target_visible"] is True
    assert abs(vis_end["target"]["offset_x"]) < 0.25
    assert 0.15 <= rf_c <= 0.35


def test_scan_occlusion_recovery_and_approach_smooth_decel_and_safety_stop(tmp_path):
    from robot_sim.telemetry import record_telemetry, stop_telemetry

    app = SimulationApp()
    host, port = app.start_http_server(host="127.0.0.1", port=0)
    url = f"http://{host}:{port}"
    raw_call = lambda path, method="GET", payload=None: robot_ctl._call_bridge(url, path, method, payload)
    stop_evt = threading.Event()
    holder: dict = {}
    err_holder: list[BaseException] = []

    tel_out = tmp_path / "telemetry_r3.json"
    tel_stop = tmp_path / ".telemetry_r3_stop"

    def worker():
        try:
            # 1. Verify try...finally safety stop in approach_target when an exception occurs mid-run
            raw_call("/reset", "POST", {"target_xy": [1.1, 0.0]})
            calls_log: list[tuple[str, str, dict | None]] = []
            vis_step = {"n": 0}

            def failing_call_fn(path: str, method: str = "GET", payload: dict | None = None) -> dict:
                calls_log.append((path, method, payload))
                if method == "GET" and "vision=1" in path:
                    vis_step["n"] += 1
                    if vis_step["n"] == 3:
                        raise RuntimeError("Simulated mid-approach sensor fault")
                return raw_call(path, method, payload)

            with pytest.raises(RuntimeError, match="Simulated mid-approach sensor fault"):
                approach_target(failing_call_fn, stop_distance=0.25)

            holder["stop_called_on_error"] = calls_log[-1][:2] == ("/stop", "POST")
            holder["moving_after_error"] = raw_call("/status", "GET")["moving"]

            # 2. Confirm stationary opt-out (--no-recover-occlusion / recover_occlusion=False) stays False when occluded
            raw_call("/reset", "POST", {"target_xy": [-1.40, 1.15]})
            wheel_cmds: list[dict] = []

            def blind_call_fn(path: str, method: str = "GET", payload: dict | None = None) -> dict:
                if path == "/wheels" and method == "POST" and isinstance(payload, dict):
                    wheel_cmds.append(dict(payload))
                res = raw_call(path, method, payload)
                if isinstance(res, dict) and "debug" in res:
                    return {k: v for k, v in res.items() if k != "debug"}
                return res

            scan_no_rec = scan_for_target(
                blind_call_fn, max_degrees=360.0, direction="left", angular_speed_deg_s=85.0, recover_occlusion=False
            )

            # 3. Occluded arena [-1.40, 1.15] from (0, 0) solved in 2 calls under 10Hz telemetry:
            #    scan_for_target(..., recover_occlusion=True) -> approach_target(...)
            #    Strip 'debug' BEFORE scan/approach receive any status to prove 0 ground-truth leakage!
            raw_call("/reset", "POST", {"target_xy": [-1.40, 1.15]})
            tel_thread = threading.Thread(
                target=record_telemetry,
                kwargs={"base_url": url, "out_path": tel_out, "stop_flag_path": tel_stop, "hz": 10.0},
                daemon=True,
            )
            tel_thread.start()
            time.sleep(0.08)

            scan_rec = scan_for_target(
                blind_call_fn, max_degrees=360.0, direction="left", angular_speed_deg_s=85.0, recover_occlusion=True
            )
            time.sleep(0.32)
            wheel_cmds.clear()
            app_res = approach_target(blind_call_fn, stop_distance=0.25)

            tel_summary = stop_telemetry(out_path=tel_out, stop_flag_path=tel_stop, wait_timeout_s=3.0)
            tel_thread.join(timeout=2.0)

            holder["scan_no_rec"] = scan_no_rec
            holder["scan_rec"] = scan_rec
            holder["app_res"] = app_res
            holder["approach_wheel_cmds"] = wheel_cmds
            holder["tel_summary"] = tel_summary
        except BaseException as exc:
            err_holder.append(exc)
        finally:
            stop_evt.set()

    t = threading.Thread(target=worker, daemon=True)
    t.start()
    try:
        while not stop_evt.is_set():
            app.tick_headless(step_batch=12)
            time.sleep(0.004)
        t.join(timeout=35.0)
    finally:
        app.shutdown()

    if err_holder:
        raise err_holder[0]

    assert holder["stop_called_on_error"] is True
    assert holder["moving_after_error"] is False

    assert holder["scan_no_rec"]["target_found"] is False
    assert holder["scan_no_rec"]["occlusion_recovered"] is False

    s_rec = holder["scan_rec"]
    assert s_rec["target_found"] is True, f"Occlusion recovery failed: {s_rec}"
    assert s_rec["occlusion_recovered"] is True
    assert abs(s_rec["after_status"]["vision"]["target"]["offset_x"]) <= 0.10
    assert "debug" not in s_rec["after_status"]

    a_res = holder["app_res"]
    assert a_res["reached"] is True, f"Approach after occlusion recovery failed: {a_res}"
    assert "debug" not in a_res["after_status"]

    # Check watchdog duration >= 0.55 on cruising steps and smooth deceleration before stop
    nonzero_cmds = [c for c in holder["approach_wheel_cmds"] if (abs(c["left"]) + abs(c["right"])) > 0.01]
    assert nonzero_cmds, "Expected non-zero wheel commands during approach"
    assert all(c["duration"] >= 0.55 for c in nonzero_cmds)
    fwd_speeds = [0.5 * (c["left"] + c["right"]) * 0.05 for c in nonzero_cmds]
    assert max(fwd_speeds) >= 0.30
    assert fwd_speeds[-1] <= 0.22

    tel = holder["tel_summary"]
    assert tel["goal_reached"] is True, f"Telemetry goal_reached False: {tel}"
    assert tel["anomalies"]["braking_rf_drop_m"] < 0.15, f"Braking drop too high: {tel}"
    assert tel["anomalies"]["max_post_stop_speed_m_s"] < 0.10, f"Post-stop speed too high: {tel}"


def test_maze_a_occlusion_recovery_and_corridor_move_maintain_wall_clearance():
    from robot_sim.arenas import resolve_arena_model

    app = SimulationApp(model_path=resolve_arena_model("maze_a"))
    host, port = app.start_http_server(host="127.0.0.1", port=0)
    url = f"http://{host}:{port}"
    raw_call = lambda path, method="GET", payload=None: robot_ctl._call_bridge(url, path, method, payload)
    stop_evt = threading.Event()
    holder: dict = {"min_rf": 99.0, "near_count": 0}
    err_holder: list[BaseException] = []

    def tracked_call_fn(path: str, method: str = "GET", payload: dict | None = None) -> dict:
        res = raw_call(path, method, payload)
        if path.startswith("/status") and isinstance(res, dict) and "sensors" in res:
            rf = res["sensors"]["rangefinders_m"]
            cur_min = min(float(v) for v in rf.values())
            holder["min_rf"] = min(holder["min_rf"], cur_min)
            if cur_min < 0.12:
                holder["near_count"] += 1
            if "debug" in res:
                res = {k: v for k, v in res.items() if k != "debug"}
        return res

    def worker():
        try:
            holder["scan"] = scan_for_target(
                tracked_call_fn, max_degrees=360.0, direction="left", recover_occlusion=True
            )
            holder["move"] = closed_loop_move(tracked_call_fn, meters=1.5, speed_m_s=0.35, include_vision=True)
        except BaseException as exc:
            err_holder.append(exc)
        finally:
            stop_evt.set()

    t = threading.Thread(target=worker, daemon=True)
    t.start()
    try:
        while not stop_evt.is_set():
            app.tick_headless(step_batch=12)
            time.sleep(0.004)
        t.join(timeout=30.0)
    finally:
        app.shutdown()

    if err_holder:
        raise err_holder[0]

    assert holder["scan"]["occlusion_recovered"] is True
    assert holder["near_count"] == 0, f"Unexpected near-collision events: {holder['near_count']}"
    assert holder["min_rf"] >= 0.135, f"Wall/corner clearance too tight: min_rf={holder['min_rf']:.4f}m"


def _run_explore_and_approach(
    arena_name: str,
    rule: str,
    *,
    target_xy: tuple[float, float] | None = None,
    drop_odom_xy: bool = False,
    tick_batch: int = 12,
    tick_sleep: float = 0.004,
    **explore_kwargs,
) -> tuple[dict, dict, dict]:
    from robot_sim.arenas import resolve_arena_model

    app = SimulationApp(model_path=resolve_arena_model(arena_name))
    if target_xy is not None:
        app.sim.reset(target_xy=target_xy)
    host, port = app.start_http_server(host="127.0.0.1", port=0)
    url = f"http://{host}:{port}"
    raw_call = lambda path, method="GET", payload=None: robot_ctl._call_bridge(url, path, method, payload)
    stop_evt = threading.Event()
    holder: dict = {"min_rf": 99.0, "near_count": 0}
    err_holder: list[BaseException] = []

    def tracked_call_fn(path: str, method: str = "GET", payload: dict | None = None) -> dict:
        res = raw_call(path, method, payload)
        if path.startswith("/status") and isinstance(res, dict) and "sensors" in res:
            rf = res["sensors"]["rangefinders_m"]
            vis = res.get("vision", {})
            t_vis = bool(vis.get("target_visible", False))
            t_obj = vis.get("target")
            t_off = float(t_obj["offset_x"]) if (t_vis and isinstance(t_obj, dict)) else None
            facing_target = t_vis and t_off is not None and abs(t_off) < 0.28
            cur_min = min(float(v) for v in rf.values())
            if not facing_target:
                holder["min_rf"] = min(holder["min_rf"], cur_min)
                if cur_min < 0.12:
                    holder["near_count"] += 1
            if drop_odom_xy:
                sensors = {
                    k: v for k, v in res["sensors"].items() if k not in ("odom_x_m", "odom_y_m")
                }
                res = {**res, "sensors": sensors}
            if "debug" in res:
                res = {k: v for k, v in res.items() if k != "debug"}
        return res

    def worker():
        try:
            exp_res = explore_maze(tracked_call_fn, rule=rule, **explore_kwargs)
            holder["exp_res"] = exp_res
            if exp_res["target_found"]:
                holder["app_res"] = approach_target(tracked_call_fn, stop_distance=0.25, max_steps=120)
        except BaseException as exc:
            err_holder.append(exc)
        finally:
            stop_evt.set()

    t = threading.Thread(target=worker, daemon=True)
    t.start()
    try:
        while not stop_evt.is_set():
            app.tick_headless(step_batch=tick_batch)
            time.sleep(tick_sleep)
        t.join(timeout=60.0)
    finally:
        app.shutdown()

    if err_holder:
        raise err_holder[0]

    return holder["exp_res"], holder.get("app_res", {}), holder


@pytest.mark.parametrize("rule", ["right", "left"])
def test_explore_maze_solves_maze_a_with_zero_near_collisions(rule):
    exp_res, app_res, stats = _run_explore_and_approach("maze_a", rule, max_steps=400, speed_m_s=0.36)
    assert exp_res["target_found"] is True, f"explore_maze(rule={rule}) failed to find target: {exp_res}"
    assert "debug" not in exp_res["after_status"]
    assert app_res["reached"] is True, f"approach_target after explore failed: {app_res}"
    assert "debug" not in app_res["after_status"]
    assert stats["near_count"] == 0, f"Near-collision events={stats['near_count']} (min_rf={stats['min_rf']:.4f}m)"
    assert stats["min_rf"] >= 0.12, f"Wall clearance too tight: min_rf={stats['min_rf']:.4f}m"


@pytest.mark.parametrize("rule", ["right", "left"])
def test_explore_maze_solves_maze_b_with_zero_near_collisions(rule):
    exp_res, app_res, stats = _run_explore_and_approach("maze_b", rule)
    assert exp_res["target_found"] is True, f"explore_maze(maze_b, rule={rule}) failed: {exp_res}"
    max_steps_bound = 560 if rule == "right" else 830
    assert exp_res["steps"] <= max_steps_bound, f"Too many explore steps on maze_b ({rule}): {exp_res['steps']}"
    assert "debug" not in exp_res["after_status"]
    assert app_res["reached"] is True, f"approach_target on maze_b ({rule}) failed: {app_res}"
    assert "debug" not in app_res["after_status"]
    end_st = app_res["after_status"]
    max_sim_time_s = 145.0 if rule == "right" else 255.0
    assert float(end_st["sim_time"]) <= max_sim_time_s, f"maze_b ({rule}) sim_time too slow: {end_st['sim_time']}s"
    assert end_st["vision"]["target_visible"] is True
    assert abs(float(end_st["vision"]["target"]["offset_x"])) < 0.20
    assert 0.15 <= float(end_st["sensors"]["rangefinders_m"]["center"]) <= 0.35
    assert stats["near_count"] == 0, f"Near-collision events={stats['near_count']} (min_rf={stats['min_rf']:.4f}m)"
    assert stats["min_rf"] >= 0.12, f"Wall clearance too tight: min_rf={stats['min_rf']:.4f}m"


@pytest.mark.parametrize("rule", ["right", "left"])
def test_explore_maze_solves_maze_c_with_zero_near_collisions(rule):
    exp_res, app_res, stats = _run_explore_and_approach(
        "maze_c", rule, tick_batch=20, tick_sleep=0.002
    )
    assert exp_res["target_found"] is True, f"explore_maze(maze_c, rule={rule}) failed: {exp_res}"
    if rule == "left":
        assert exp_res["loop_escaped"] is True
    assert "debug" not in exp_res["after_status"]
    assert app_res["reached"] is True, f"approach_target on maze_c ({rule}) failed: {app_res}"
    assert "debug" not in app_res["after_status"]
    end_st = app_res["after_status"]
    assert end_st["vision"]["target_visible"] is True
    assert abs(float(end_st["vision"]["target"]["offset_x"])) < 0.20
    assert 0.15 <= float(end_st["sensors"]["rangefinders_m"]["center"]) <= 0.35
    assert stats["near_count"] == 0, f"Near-collision events={stats['near_count']} (min_rf={stats['min_rf']:.4f}m)"
    assert stats["min_rf"] >= 0.14, f"Wall clearance too tight: min_rf={stats['min_rf']:.4f}m"


@pytest.mark.parametrize("drop_odom_xy", [False, True])
def test_explore_maze_escapes_maze_c_island_to_outer_dead_end_and_supports_legacy_odom(drop_odom_xy):
    exp_res, app_res, stats = _run_explore_and_approach(
        "maze_c",
        "right",
        target_xy=(-1.6, 1.6),
        drop_odom_xy=drop_odom_xy,
        tick_batch=20,
        tick_sleep=0.002,
    )
    assert exp_res["target_found"] is True, f"Failed to escape island (drop_odom_xy={drop_odom_xy}): {exp_res}"
    assert exp_res["loop_escaped"] is True
    assert "debug" not in exp_res["after_status"]
    assert app_res["reached"] is True, f"approach_target failed on outer dead-end: {app_res}"
    assert "debug" not in app_res["after_status"]
    end_st = app_res["after_status"]
    assert end_st["vision"]["target_visible"] is True
    assert abs(float(end_st["vision"]["target"]["offset_x"])) < 0.25
    assert 0.15 <= float(end_st["sensors"]["rangefinders_m"]["center"]) <= 0.35
    assert stats["near_count"] == 0, f"Near-collision events={stats['near_count']} (min_rf={stats['min_rf']:.4f}m)"
    assert stats["min_rf"] >= 0.12, f"Wall clearance too tight: min_rf={stats['min_rf']:.4f}m"


def test_scan_360_no_recover_stops_without_large_overshoot():
    from robot_sim.arenas import resolve_arena_model

    app = SimulationApp(model_path=resolve_arena_model("maze_c"))
    host, port = app.start_http_server(host="127.0.0.1", port=0)
    url = f"http://{host}:{port}"
    call_fn = lambda path, method="GET", payload=None: robot_ctl._call_bridge(url, path, method, payload)
    stop_evt = threading.Event()
    holder: dict = {}
    err_holder: list[BaseException] = []

    def worker():
        try:
            holder["scan"] = scan_for_target(
                call_fn, max_degrees=360.0, direction="left", recover_occlusion=False
            )
        except BaseException as exc:
            err_holder.append(exc)
        finally:
            stop_evt.set()

    t = threading.Thread(target=worker, daemon=True)
    t.start()
    try:
        while not stop_evt.is_set():
            app.tick_headless(step_batch=12)
            time.sleep(0.004)
        t.join(timeout=15.0)
    finally:
        app.shutdown()

    if err_holder:
        raise err_holder[0]

    assert holder["scan"]["target_found"] is False
    assert 356.0 <= holder["scan"]["scanned_degrees"] <= 363.0


@pytest.mark.parametrize("phase_wait", [0.0, 2.5, 5.0])
def test_approach_target_crosses_dynamic_a_three_lanes_with_zero_contacts(phase_wait):
    from robot_sim.arenas import resolve_arena_model

    app = SimulationApp(model_path=resolve_arena_model("dynamic_a"))
    host, port = app.start_http_server(host="127.0.0.1", port=0)
    url = f"http://{host}:{port}"
    raw_call = lambda path, method="GET", payload=None: robot_ctl._call_bridge(url, path, method, payload)
    stop_evt = threading.Event()
    holder: dict = {"min_rf": 99.0, "near_count": 0}
    err_holder: list[BaseException] = []

    def tracked_call_fn(path: str, method: str = "GET", payload: dict | None = None) -> dict:
        res = raw_call(path, method, payload)
        if path.startswith("/status") and isinstance(res, dict) and "sensors" in res:
            rf = res["sensors"]["rangefinders_m"]
            vis = res.get("vision", {})
            t_vis = bool(vis.get("target_visible", False))
            t_obj = vis.get("target")
            facing_target = t_vis and isinstance(t_obj, dict) and abs(float(t_obj["offset_x"])) < 0.28
            cur_min = min(float(v) for v in rf.values())
            if not facing_target:
                holder["min_rf"] = min(holder["min_rf"], cur_min)
                if cur_min < 0.12:
                    holder["near_count"] += 1
            if "debug" in res:
                res = {k: v for k, v in res.items() if k != "debug"}
        return res

    def worker():
        try:
            st0 = tracked_call_fn("/status?vision=1", "GET")
            if phase_wait > 0.0:
                t0 = float(st0["sim_time"])
                while float(tracked_call_fn("/status", "GET")["sim_time"]) - t0 < phase_wait:
                    tracked_call_fn("/wheels", "POST", {"left": 0.0, "right": 0.0, "duration": 0.35})
                    time.sleep(0.01)
                st0 = tracked_call_fn("/status?vision=1", "GET")
            if not st0["vision"]["target_visible"]:
                scan_for_target(tracked_call_fn, max_degrees=360.0, direction="left")
            holder["app_res"] = approach_target(tracked_call_fn, stop_distance=0.25, max_steps=160)
            holder["final_debug"] = raw_call("/status", "GET")["debug"]
        except BaseException as exc:
            err_holder.append(exc)
        finally:
            stop_evt.set()

    t = threading.Thread(target=worker, daemon=True)
    t.start()
    try:
        while not stop_evt.is_set():
            app.tick_headless(step_batch=10)
            time.sleep(0.004)
        t.join(timeout=90.0)
    finally:
        app.shutdown()

    if err_holder:
        raise err_holder[0]

    app_res = holder["app_res"]
    final_debug = holder["final_debug"]
    assert app_res["reached"] is True, f"dynamic_a (wait={phase_wait}) failed: {app_res}"
    assert "debug" not in app_res["after_status"]
    assert final_debug["contact_count"] == 0
    assert holder["near_count"] == 0
    assert holder["min_rf"] >= 0.14


@pytest.mark.parametrize("rule", ["right", "left"])
def test_explore_and_approach_solve_dynamic_b_piston_gates_with_zero_contacts(rule):
    from robot_sim.arenas import resolve_arena_model

    app = SimulationApp(model_path=resolve_arena_model("dynamic_b"))
    host, port = app.start_http_server(host="127.0.0.1", port=0)
    url = f"http://{host}:{port}"
    raw_call = lambda path, method="GET", payload=None: robot_ctl._call_bridge(url, path, method, payload)
    stop_evt = threading.Event()
    holder: dict = {"min_rf": 99.0, "near_count": 0}
    err_holder: list[BaseException] = []

    def tracked_call_fn(path: str, method: str = "GET", payload: dict | None = None) -> dict:
        res = raw_call(path, method, payload)
        if path.startswith("/status") and isinstance(res, dict) and "sensors" in res:
            rf = res["sensors"]["rangefinders_m"]
            vis = res.get("vision", {})
            t_vis = bool(vis.get("target_visible", False))
            t_obj = vis.get("target")
            facing_target = t_vis and isinstance(t_obj, dict) and abs(float(t_obj["offset_x"])) < 0.28
            cur_min = min(float(v) for v in rf.values())
            if not facing_target:
                holder["min_rf"] = min(holder["min_rf"], cur_min)
                if cur_min < 0.12:
                    holder["near_count"] += 1
            if "debug" in res:
                res = {k: v for k, v in res.items() if k != "debug"}
        return res

    def worker():
        try:
            exp_res = explore_maze(tracked_call_fn, rule=rule, max_steps=600)
            holder["exp_res"] = exp_res
            if exp_res["target_found"]:
                holder["app_res"] = approach_target(tracked_call_fn, stop_distance=0.25, max_steps=160)
            holder["final_debug"] = raw_call("/status", "GET")["debug"]
        except BaseException as exc:
            err_holder.append(exc)
        finally:
            stop_evt.set()

    t = threading.Thread(target=worker, daemon=True)
    t.start()
    try:
        while not stop_evt.is_set():
            app.tick_headless(step_batch=10)
            time.sleep(0.004)
        t.join(timeout=120.0)
    finally:
        app.shutdown()

    if err_holder:
        raise err_holder[0]

    exp_res = holder["exp_res"]
    app_res = holder["app_res"]
    final_debug = holder["final_debug"]
    assert exp_res["target_found"] is True, f"dynamic_b explore failed: {exp_res}"
    assert exp_res["gates_yielded"] >= 1
    assert "debug" not in exp_res["after_status"]
    assert app_res["reached"] is True, f"dynamic_b approach failed: {app_res}"
    assert "debug" not in app_res["after_status"]
    assert final_debug["contact_count"] == 0
    assert holder["near_count"] == 0
    assert holder["min_rf"] >= 0.14
