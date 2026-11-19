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


