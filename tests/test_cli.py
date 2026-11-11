import json
from pathlib import Path
import socket
import threading
import time
from sim_server import SimulationApp
import robot_ctl


def test_cli_offline_prints_friendly_vietnamese_hint(capsys):
    code = robot_ctl.main(["--url", "http://127.0.0.1:59999", "status"])
    captured = capsys.readouterr()
    assert code == 1
    assert "sim_server.py" in (captured.out + captured.err)


def test_cli_handles_bad_url_scheme_and_remote_disconnect(capsys):
    # 1. Scheme-less URL should return exit code 1 cleanly without traceback
    code_bad_url = robot_ctl.main(["--url", "127.0.0.1:8765", "status"])
    err_bad_url = capsys.readouterr().err
    assert code_bad_url == 1
    assert "http://" in err_bad_url

    # 2. Server closing socket mid-request (RemoteDisconnected) should return exit code 1 cleanly
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    port = srv.getsockname()[1]

    def drop_client():
        conn, _ = srv.accept()
        conn.recv(256)
        conn.close()
        srv.close()

    t = threading.Thread(target=drop_client, daemon=True)
    t.start()
    code_drop = robot_ctl.main(["--url", f"http://127.0.0.1:{port}", "status"])
    err_drop = capsys.readouterr().err
    t.join(timeout=2.0)
    assert code_drop == 1
    assert "sim_server.py" in err_drop


def test_cli_wait_polls_until_stopped_and_reset_stop_work(capsys):
    app = SimulationApp()
    host, port = app.start_http_server(host="127.0.0.1", port=0)
    url = f"http://{host}:{port}"
    stop_evt = threading.Event()
    results: dict = {}

    def run_wait_workflow():
        try:
            assert robot_ctl.main(["--url", url, "reset", "--target-x", "1.0", "--target-y", "-0.8"]) == 0
            capsys.readouterr()

            # 0.10m at 0.25m/s = 0.40s sim_time; under throttled stepping below, takes >0.8s wall-clock
            assert robot_ctl.main(["--url", url, "move", "0.10", "--speed", "0.25", "--wait"]) == 0
            results["move_wait"] = json.loads(capsys.readouterr().out)

            assert robot_ctl.main(["--url", url, "stop"]) == 0
            results["stop"] = json.loads(capsys.readouterr().out)
        finally:
            stop_evt.set()

    t = threading.Thread(target=run_wait_workflow, daemon=True)
    t.start()
    try:
        while not stop_evt.is_set():
            # Intentionally slower than real-time (4ms sim per 12ms wall = ~3x slower) to prove --wait polls
            app.tick_headless(step_batch=2)
            time.sleep(0.012)
        t.join(timeout=10.0)
    finally:
        app.shutdown()

    assert results["move_wait"]["after_status"]["moving"] is False
    assert results["stop"]["stopped"] is True


def test_cli_status_hides_debug_by_default_and_commands_work(capsys, tmp_path: Path):
    app = SimulationApp()
    host, port = app.start_http_server(host="127.0.0.1", port=0)
    url = f"http://{host}:{port}"
    stop_evt = threading.Event()

    def run_cli_calls():
        try:
            assert robot_ctl.main(["--url", url, "status"]) == 0
            out_default = json.loads(capsys.readouterr().out)
            assert "sensors" in out_default and "debug" not in out_default

            assert robot_ctl.main(["--url", url, "status", "--debug"]) == 0
            out_debug = json.loads(capsys.readouterr().out)
            assert "debug" in out_debug

            cam_file = tmp_path / "cam.png"
            assert robot_ctl.main(["--url", url, "camera", "--name", "front_cam", "--out", str(cam_file)]) == 0
            capsys.readouterr()
            assert cam_file.is_file()

            assert robot_ctl.main(["--url", url, "turn", "-45"]) == 0
            capsys.readouterr()

            assert robot_ctl.main(["--url", url, "move", "-0.2"]) == 0
            capsys.readouterr()

            assert robot_ctl.main(["--url", url, "wheels", "-4.0", "4.0", "--duration", "0.2"]) == 0
            capsys.readouterr()

            assert robot_ctl.main(["--url", url, "demo", "--steps", "3"]) == 0
            capsys.readouterr()
        finally:
            stop_evt.set()

    t = threading.Thread(target=run_cli_calls, daemon=True)
    t.start()
    try:
        while not stop_evt.is_set():
            app.tick_headless(step_batch=10)
            time.sleep(0.005)
        t.join(timeout=10.0)
    finally:
        app.shutdown()
