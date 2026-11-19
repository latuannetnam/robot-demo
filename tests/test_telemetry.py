from pathlib import Path
import threading
import time
from sim_server import SimulationApp
import robot_ctl
from robot_sim.telemetry import record_telemetry, stop_telemetry


def test_telemetry_record_clears_stale_stop_flag_and_summarizes_run(tmp_path: Path):
    app = SimulationApp()
    host, port = app.start_http_server(host="127.0.0.1", port=0)
    url = f"http://{host}:{port}"
    out_json = tmp_path / "telemetry_latest.json"
    stop_file = tmp_path / ".telemetry_stop"
    stop_file.write_text("stale", encoding="utf-8")

    stop_sim = threading.Event()
    holder: dict = {}
    err_holder: list[BaseException] = []

    def rec_worker():
        try:
            holder["rec"] = record_telemetry(
                url,
                out_path=out_json,
                stop_flag_path=stop_file,
                hz=20.0,
                max_duration_s=10.0,
            )
        except BaseException as exc:
            err_holder.append(exc)

    def driver_worker():
        try:
            # Give recorder thread 100ms to clear stale stop_file and begin sampling
            time.sleep(0.10)
            robot_ctl._call_bridge(url, "/reset", "POST", {"target_xy": [0.65, 0.0]})
            robot_ctl._call_bridge(url, "/move", "POST", {"meters": 0.25, "speed": 0.25})
            robot_ctl._wait_for_completion(url, duration_s=1.0)
            time.sleep(0.10)
            holder["summary"] = stop_telemetry(out_path=out_json, stop_flag_path=stop_file, wait_timeout_s=3.0)
        except BaseException as exc:
            err_holder.append(exc)
        finally:
            stop_sim.set()

    t_rec = threading.Thread(target=rec_worker, daemon=True)
    t_drv = threading.Thread(target=driver_worker, daemon=True)
    t_rec.start()
    t_drv.start()
    try:
        while not stop_sim.is_set():
            app.tick_headless(step_batch=10)
            time.sleep(0.005)
        t_drv.join(timeout=5.0)
        t_rec.join(timeout=5.0)
    finally:
        app.shutdown()

    if err_holder:
        raise err_holder[0]

    summary = holder["summary"]
    assert out_json.is_file()
    assert out_json.stat().st_size < 4096
    assert summary["arena"] == "2wheel"
    assert summary["sample_count"] >= 3
    assert summary["error_count"] == 0
    assert summary["contact_count"] == 0
    assert summary["path_length_m"] > 0.15
    assert summary["duty_cycle_pct"] > 0.0
    assert "anomalies" in summary
    assert "braking_rf_drop_m" in summary["anomalies"]
    assert "max_post_stop_speed_m_s" in summary["anomalies"]
    assert "goal_reached" in summary


def test_telemetry_recorder_survives_offline_server_and_stop_handles_no_active_recorder(tmp_path: Path, capsys):
    out_json = tmp_path / "offline_telemetry.json"
    stop_file = tmp_path / ".offline_stop"
    res = record_telemetry(
        "http://127.0.0.1:59998",
        out_path=out_json,
        stop_flag_path=stop_file,
        hz=20.0,
        max_duration_s=0.25,
    )
    assert out_json.is_file()
    assert res["arena"] is None
    assert res["error_count"] >= 1

    code = robot_ctl.main(
        [
            "telemetry",
            "stop",
            "--out",
            str(tmp_path / "missing.json"),
            "--stop-file",
            str(tmp_path / ".no_recorder"),
            "--wait-timeout",
            "0.25",
        ]
    )
    err = capsys.readouterr().err
    assert code == 1
    assert "Lỗi" in err or "Hộp đen" in err


def test_telemetry_tracks_arena_and_all_dynamic_rangefinders(tmp_path: Path, monkeypatch):
    out_json = tmp_path / "dyn_rf.json"
    stop_file = tmp_path / ".dyn_stop"
    calls = 0

    def fake_fetch(_base_url: str, timeout: float = 1.5) -> dict:
        nonlocal calls
        calls += 1
        if calls >= 2:
            stop_file.write_text("stop\n", encoding="utf-8")
        return {
            "sim_time": 0.5 * calls,
            "moving": True,
            "sensors": {
                "rangefinders_m": {
                    "left": 2.1,
                    "center": 1.8,
                    "right": 2.4,
                    "side_left": 0.25 if calls == 1 else 0.08,
                },
                "forward_speed_m_s": 0.2,
                "yaw_rate_rad_s": 0.0,
            },
            "vision": {"target_visible": False, "target": None, "obstacles": []},
            "debug": {
                "arena": "maze_a",
                "model": "arena_maze_a.xml",
                "robot_xy": [-1.5, -1.5],
                "robot_yaw_deg": 0.0,
                "target_xy": [1.5, 1.5],
                "contact_count": 0 if calls == 1 else 3,
            },
        }

    monkeypatch.setattr("robot_sim.telemetry._fetch_raw_status", fake_fetch)
    summary = record_telemetry("http://127.0.0.1:8765", out_path=out_json, stop_flag_path=stop_file, hz=50.0)
    assert summary["arena"] == "maze_a"
    assert summary["min_rangefinder_m"] == 0.08
    assert summary["near_collision_count"] == 1
    assert summary["contact_count"] == 3


def test_fetch_raw_status_recovers_from_stale_keepalive_connection():
    import json
    import socket
    from robot_sim.telemetry import _fetch_raw_status, _get_conn_pool

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.bind(("127.0.0.1", 0))
    srv.listen(4)
    port = srv.getsockname()[1]
    url = f"http://127.0.0.1:{port}"
    accepted = {"n": 0}

    def server_worker():
        # First connection returns valid HTTP/1.1 JSON, then closes socket before 2nd request arrives
        c1, _ = srv.accept()
        accepted["n"] += 1
        c1.recv(1024)
        body = json.dumps({"ok": True, "data": {"sim_time": 0.1}}).encode("utf-8")
        c1.sendall(
            b"HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n"
            + f"Content-Length: {len(body)}\r\n\r\n".encode("ascii")
            + body
        )
        time.sleep(0.03)
        c1.close()

        # Second connection (transparent retry) serves fresh JSON cleanly
        c2, _ = srv.accept()
        accepted["n"] += 1
        c2.recv(1024)
        body2 = json.dumps({"ok": True, "data": {"sim_time": 0.2}}).encode("utf-8")
        c2.sendall(
            b"HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\n"
            + f"Content-Length: {len(body2)}\r\n\r\n".encode("ascii")
            + body2
        )
        c2.close()
        srv.close()

    t = threading.Thread(target=server_worker, daemon=True)
    t.start()
    try:
        _get_conn_pool().clear()
        d1 = _fetch_raw_status(url)
        time.sleep(0.06)
        d2 = _fetch_raw_status(url)
        t.join(timeout=2.0)
    finally:
        _get_conn_pool().clear()

    assert d1["sim_time"] == 0.1
    assert d2["sim_time"] == 0.2
    assert accepted["n"] == 2


def test_telemetry_cognitive_metrics_directed_revisits_and_custom_target(tmp_path: Path, monkeypatch):
    import pytest
    from robot_sim.telemetry import record_telemetry

    out_json = tmp_path / "cog_telemetry.json"
    stop_file = tmp_path / ".cog_stop"
    # Sequence:
    # 1) Hub (-1.5, 0.0) W -> N dead-end #1 (>=0.8m) -> return (-1.5, 0.0) facing S (0 redundant)
    # 2) N->S dead-end #2 (>=0.8m) -> return (-1.5, 0.0) facing N and pause/think (0 redundant)
    # 3) Backtrack / re-enter (-1.5, 0.0) facing W a 2nd time -> redundant_revisit_count == 1
    # 4) Travel continuously to dynamic_a target (1.66, 0.0)
    continuous_trace = [
        # (moving, fwd_spd, xy, yaw_deg, target_vis, off_x, rf_c, target_xy)
        (False, 0.0, [-1.50, 0.00], 180.0, False, None, 1.5, [1.66, 0.0]),  # think 1
        (True,  0.3, [-1.50, 0.00], 180.0, False, None, 1.5, [1.66, 0.0]),  # burst 1: cell (-3,0) dir W (#1)
        (True,  0.4, [-1.50, 0.45],  90.0, False, None, 1.2, [1.66, 0.0]),
        (True,  0.4, [-1.50, 0.90],  90.0, False, None, 0.8, [1.66, 0.0]),
        (True,  0.4, [-1.50, 0.45], -90.0, False, None, 1.2, [1.66, 0.0]),
        (False, 0.0, [-1.50, 0.00], -90.0, False, None, 1.5, [1.66, 0.0]),  # backtrack 1 -> (-3,0) dir S (#1), think 2
        (True,  0.4, [-1.50, -0.45], -90.0, False, None, 1.2, [1.66, 0.0]), # burst 2
        (True,  0.4, [-1.50, -0.90], -90.0, False, None, 0.8, [1.66, 0.0]),
        (True,  0.4, [-1.50, -0.45],  90.0, False, None, 1.2, [1.66, 0.0]),
        (False, 0.0, [-1.50, 0.00],  90.0, False, None, 1.5, [1.66, 0.0]),  # backtrack 2 -> (-3,0) dir N (#1), think 3
        (True,  0.4, [-1.15, 0.00],   0.0, False, None, 1.5, [1.66, 0.0]),  # burst 3
        (True,  0.3, [-1.50, 0.00], 180.0, False, None, 1.5, [1.66, 0.0]),  # revisit (-3,0) dir W (#2, >=0.8m from t=0) -> redundant = 1
        (True,  0.4, [-1.15, 0.00],   0.0, False, None, 1.5, [1.66, 0.0]),
        (True,  0.4, [-0.80, 0.00],   0.0, False, None, 1.5, [1.66, 0.0]),
        (True,  0.4, [-0.45, 0.00],   0.0, False, None, 1.5, [1.66, 0.0]),
        (True,  0.4, [-0.10, 0.00],   0.0, False, None, 1.5, [1.66, 0.0]),
        (True,  0.4, [0.25,  0.00],   0.0, False, None, 1.2, [1.66, 0.0]),
        (True,  0.4, [0.60,  0.00],   0.0, False, None, 0.9, [1.66, 0.0]),
        (True,  0.4, [0.95,  0.00],   0.0, False, None, 0.5, [1.66, 0.0]),
        (False, 0.0, [1.22,  0.00],   0.0, True,  0.02, 0.25, [1.66, 0.0]),
    ]
    idx = 0

    def fake_fetch(_url: str, timeout: float = 4.0) -> dict:
        nonlocal idx
        m, spd, xy, yaw, tv, ox, rfc, txy = continuous_trace[idx]
        idx += 1
        if idx >= len(continuous_trace):
            stop_file.write_text("stop\n", encoding="utf-8")
        return {
            "sim_time": 0.2 * idx,
            "moving": m,
            "sensors": {
                "rangefinders_m": {"left": 1.0, "center": rfc, "right": 1.0},
                "forward_speed_m_s": spd,
                "yaw_rate_rad_s": 0.0,
            },
            "vision": {
                "target_visible": tv,
                "target": {"offset_x": ox, "area_pct": 20.0} if tv else None,
                "obstacles": [],
            },
            "debug": {
                "arena": "dynamic_a",
                "robot_xy": xy,
                "robot_yaw_deg": yaw,
                "target_xy": txy,
                "contact_count": 0,
            },
        }

    monkeypatch.setattr("robot_sim.telemetry._fetch_raw_status", fake_fetch)
    s1 = record_telemetry("http://127.0.0.1:8765", out_path=out_json, stop_flag_path=stop_file, hz=50.0)
    assert s1["goal_reached"] is True
    assert s1["command_burst_count"] == 3
    assert s1["backtrack_revisit_count"] >= 2
    assert s1["redundant_revisit_count"] == 1
    assert s1["unique_cells_visited"] >= 5
    assert s1["reference_path_m"] == 2.88
    assert s1["path_efficiency"] == pytest.approx(round(min(1.0, 2.88 / s1["path_length_m"]), 3), abs=1e-3)
    assert s1["active_drive_time_s"] > 0.0
    assert s1["agent_think_time_s"] > 0.0
    assert abs((s1["active_drive_time_s"] + s1["agent_think_time_s"]) - s1["wall_time_s"]) <= 0.15

    # Test B: custom target_xy (> 0.05m from default_target_xy) -> reference_path_m and path_efficiency are None
    # + verify reset_jumped clears old cell visit history
    out_json2 = tmp_path / "custom_target.json"
    stop_file2 = tmp_path / ".custom_stop"
    custom_trace = [
        (True, 0.2, [0.0, 0.0], 0.0, False, None, 1.0, [0.5, 0.9]),
        (True, 0.2, [0.0, 0.9], 0.0, False, None, 1.0, [0.5, 0.9]),
        (False, 0.0, [0.1, 0.9], 0.0, True, 0.01, 0.25, [0.5, 0.9]),
    ]
    c_idx = 0

    def fake_fetch_custom(_url: str, timeout: float = 4.0) -> dict:
        nonlocal c_idx
        m, spd, xy, yaw, tv, ox, rfc, txy = custom_trace[c_idx]
        c_idx += 1
        if c_idx >= len(custom_trace):
            stop_file2.write_text("stop\n", encoding="utf-8")
        return {
            "sim_time": 0.2 * c_idx,
            "moving": m,
            "sensors": {"rangefinders_m": {"center": rfc}, "forward_speed_m_s": spd, "yaw_rate_rad_s": 0.0},
            "vision": {"target_visible": tv, "target": {"offset_x": ox, "area_pct": 20.0} if tv else None, "obstacles": []},
            "debug": {"arena": "dynamic_a", "robot_xy": xy, "robot_yaw_deg": yaw, "target_xy": txy, "contact_count": 0},
        }

    monkeypatch.setattr("robot_sim.telemetry._fetch_raw_status", fake_fetch_custom)
    s2 = record_telemetry("http://127.0.0.1:8765", out_path=out_json2, stop_flag_path=stop_file2, hz=50.0)
    assert s2["reference_path_m"] is None
    assert s2["path_efficiency"] is None
    assert s2["redundant_revisit_count"] == 0


def test_telemetry_optimal_dfs_in_place_spins_yield_zero_redundant_revisits(tmp_path: Path, monkeypatch):
    from robot_sim.telemetry import record_telemetry

    out_json = tmp_path / "hub_dfs.json"
    stop_file = tmp_path / ".hub_stop"
    # Simulate 10Hz optimal DFS at (-1.5, 0.0) hub including gradual in-place yaw rotations
    hub_trace = [
        (True,  0.3, [-1.50,  0.00], 180.0),  # enter hub facing W
        (True,  0.0, [-1.50,  0.00], 135.0),  # spin in place inside hub toward N
        (True,  0.0, [-1.50,  0.00],  90.0),  # settled N inside hub before leaving!
        (True,  0.4, [-1.50,  0.40],  90.0),
        (True,  0.4, [-1.50,  0.80],  90.0),
        (True,  0.0, [-1.50,  0.80], -90.0),  # dead-end U-turn
        (True,  0.4, [-1.50,  0.40], -90.0),
        (True,  0.3, [-1.50,  0.00], -90.0),  # backtrack entry #1: facing S
        (True,  0.4, [-1.50, -0.40], -90.0),
        (True,  0.4, [-1.50, -0.80], -90.0),
        (True,  0.0, [-1.50, -0.80],  90.0),  # dead-end U-turn
        (True,  0.4, [-1.50, -0.40],  90.0),
        (True,  0.3, [-1.50,  0.00],  90.0),  # backtrack entry #2: facing N (must be 0 redundant!)
        (True,  0.0, [-1.50,  0.00],  45.0),  # spin in place toward E
        (True,  0.0, [-1.50,  0.00],   0.0),
        (True,  0.4, [-1.15,  0.00],   0.0),  # exit E
    ]
    idx = 0

    def fake_fetch(_url: str, timeout: float = 4.0) -> dict:
        nonlocal idx
        m, spd, xy, yaw = hub_trace[idx]
        idx += 1
        if idx >= len(hub_trace):
            stop_file.write_text("stop\n", encoding="utf-8")
        return {
            "sim_time": 0.1 * idx,
            "moving": m,
            "sensors": {"rangefinders_m": {"center": 1.0}, "forward_speed_m_s": spd, "yaw_rate_rad_s": 0.0},
            "vision": {"target_visible": False, "target": None, "obstacles": []},
            "debug": {"arena": "maze_c", "robot_xy": xy, "robot_yaw_deg": yaw, "target_xy": [1.6, 1.6], "contact_count": 0},
        }

    monkeypatch.setattr("robot_sim.telemetry._fetch_raw_status", fake_fetch)
    s = record_telemetry("http://127.0.0.1:8765", out_path=out_json, stop_flag_path=stop_file, hz=100.0)
    assert s["backtrack_revisit_count"] >= 2
    assert s["redundant_revisit_count"] == 0


