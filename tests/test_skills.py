import threading
import time
import pytest
from sim_server import SimulationApp
import robot_ctl
from robot_sim.skills import approach_target, closed_loop_turn, scan_for_target


def test_closed_loop_turn_crosses_180_boundary_and_skills_validate_inputs():
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
            res1 = closed_loop_turn(call_fn, 150.0, max_speed_deg_s=95.0, include_vision=False)
            res2 = closed_loop_turn(call_fn, 60.0, max_speed_deg_s=95.0, include_vision=True)
            raw_status = call_fn("/status", "GET")
            holder["res1"] = res1
            holder["res2"] = res2
            holder["true_yaw"] = raw_status["debug"]["robot_yaw_deg"]

            for fn, kwargs in (
                (closed_loop_turn, {"degrees": 0.0}),
                (scan_for_target, {"max_degrees": 0.0}),
                (scan_for_target, {"direction": "up"}),
                (approach_target, {"stop_distance": 0.05}),
                (approach_target, {"stop_distance": 2.0}),
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
        t.join(timeout=15.0)
    finally:
        app.shutdown()

    if err_holder:
        raise err_holder[0]

    assert "debug" not in holder["res1"]["after_status"]
    assert "debug" not in holder["res2"]["after_status"]
    assert "vision" in holder["res2"]["after_status"]
    # +150 deg + +60 deg = +210 deg => -150 deg wrapped
    assert -154.5 <= holder["true_yaw"] <= -145.5


def test_scan_and_approach_reach_visible_target_without_debug():
    app = SimulationApp()
    host, port = app.start_http_server(host="127.0.0.1", port=0)
    url = f"http://{host}:{port}"
    call_fn = lambda path, method="GET", payload=None: robot_ctl._call_bridge(url, path, method, payload)
    stop_evt = threading.Event()
    holder: dict = {}
    err_holder: list[BaseException] = []

    def worker():
        try:
            # (-1.2, -1.2) is in the open -X/-Y quadrant with zero obstacles in path
            call_fn("/reset", "POST", {"target_xy": [-1.2, -1.2]})
            scan_res = scan_for_target(call_fn, max_degrees=360.0, direction="right", angular_speed_deg_s=85.0)
            app_res = approach_target(call_fn, stop_distance=0.25, max_steps=65, step_dt=0.16)
            holder["scan"] = scan_res
            holder["approach"] = app_res
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

    assert holder["scan"]["target_found"] is True
    assert "debug" not in holder["scan"]["after_status"]
    assert holder["scan"]["after_status"]["vision"]["target_visible"] is True

    assert holder["approach"]["reached"] is True
    assert "debug" not in holder["approach"]["after_status"]
    rf_c = holder["approach"]["after_status"]["sensors"]["rangefinders_m"]["center"]
    assert 0.15 <= rf_c <= 0.35
