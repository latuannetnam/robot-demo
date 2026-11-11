import json
from pathlib import Path
import threading
import time
from sim_server import SimulationApp
import robot_ctl


def test_cli_offline_prints_friendly_vietnamese_hint(capsys):
    code = robot_ctl.main(["--url", "http://127.0.0.1:59999", "status"])
    captured = capsys.readouterr()
    assert code == 1
    assert "sim_server.py" in (captured.out + captured.err)


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
