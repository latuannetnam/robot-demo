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
            time.sleep(0.55)
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
    assert summary["sample_count"] >= 3
    assert summary["error_count"] == 0
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
