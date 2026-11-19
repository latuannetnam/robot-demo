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

    err_holder: list[BaseException] = []

    def run_cli_calls():
        try:
            assert robot_ctl.main(["--url", url, "status"]) == 0
            out_default = json.loads(capsys.readouterr().out)
            assert "sensors" in out_default and "debug" not in out_default

            assert robot_ctl.main(["--url", url, "status", "--debug"]) == 0
            out_debug = json.loads(capsys.readouterr().out)
            assert "debug" in out_debug

            assert robot_ctl.main(["--url", url, "status", "--vision"]) == 0
            out_vis_status = json.loads(capsys.readouterr().out)
            assert "vision" in out_vis_status and "debug" not in out_vis_status

            assert robot_ctl.main(["--url", url, "vision"]) == 0
            out_vis = json.loads(capsys.readouterr().out)
            assert "target_visible" in out_vis and "obstacles" in out_vis

            cam_file = tmp_path / "cam.jpg"
            assert robot_ctl.main(["--url", url, "camera", "--name", "front_cam", "--out", str(cam_file)]) == 0
            cam_info = json.loads(capsys.readouterr().out)
            assert cam_file.is_file() and cam_info["format"] == "JPEG"

            assert robot_ctl.main(["--url", url, "turn", "-45", "--wait", "--vision"]) == 0
            turn_info = json.loads(capsys.readouterr().out)
            assert "vision" in turn_info["after_status"]

            assert robot_ctl.main(["--url", url, "move", "-0.2"]) == 0
            capsys.readouterr()

            assert robot_ctl.main(["--url", url, "wheels", "-4.0", "4.0", "--duration", "0.2"]) == 0
            capsys.readouterr()

            assert robot_ctl.main(["--url", url, "demo", "--steps", "3"]) == 0
            capsys.readouterr()
        except BaseException as exc:
            err_holder.append(exc)
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

    if err_holder:
        raise err_holder[0]


def test_cli_scan_approach_and_reload_never_leak_debug_field(capsys):
    app = SimulationApp()
    host, port = app.start_http_server(host="127.0.0.1", port=0)
    url = f"http://{host}:{port}"
    stop_evt = threading.Event()
    outputs: dict[str, str] = {}
    err_holder: list[BaseException] = []

    def run_cli_calls():
        try:
            assert robot_ctl.main(["--url", url, "reset", "--target-x", "0.75", "--target-y", "0.0"]) == 0
            capsys.readouterr()

            assert robot_ctl.main(["--url", url, "reload", "--vision"]) == 0
            outputs["reload"] = capsys.readouterr().out

            assert robot_ctl.main(["--url", url, "scan", "--max-degrees", "90", "--no-recover-occlusion"]) == 0
            outputs["scan"] = capsys.readouterr().out

            parser = robot_ctl.build_parser()
            assert parser.parse_args(["scan"]).recover_occlusion is True
            assert parser.parse_args(["scan", "--no-recover-occlusion"]).recover_occlusion is False

            assert robot_ctl.main(["--url", url, "approach", "--stop-distance", "0.28"]) == 0
            outputs["approach"] = capsys.readouterr().out
        except BaseException as exc:
            err_holder.append(exc)
        finally:
            stop_evt.set()

    t = threading.Thread(target=run_cli_calls, daemon=True)
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

    for name, raw_out in outputs.items():
        parsed = json.loads(raw_out)
        assert "debug" not in raw_out, f"'{name}' leaked debug field: {raw_out}"
        assert isinstance(parsed, dict)


def test_cli_arenas_and_status_hide_active_arena_without_debug_and_reload_switches_arena(capsys):
    import pytest

    app = SimulationApp()
    host, port = app.start_http_server(host="127.0.0.1", port=0)
    url = f"http://{host}:{port}"
    stop_evt = threading.Event()
    results: dict = {}
    err_holder: list[BaseException] = []

    def run_cli_calls():
        try:
            assert robot_ctl.main(["--url", url, "arenas"]) == 0
            results["arenas_blind"] = json.loads(capsys.readouterr().out)

            # Custom target on 2wheel first, then switch arena via CLI reload --arena maze_a
            assert robot_ctl.main(["--url", url, "reset", "--target-x", "-1.1", "--target-y", "0.9"]) == 0
            capsys.readouterr()

            assert robot_ctl.main(["--url", url, "reload", "--arena", "maze_a"]) == 0
            results["reload_maze_a"] = json.loads(capsys.readouterr().out)

            assert robot_ctl.main(["--url", url, "status", "--vision"]) == 0
            results["status_vis"] = json.loads(capsys.readouterr().out)

            assert robot_ctl.main(["--url", url, "arenas", "--debug"]) == 0
            results["arenas_debug"] = json.loads(capsys.readouterr().out)

            assert robot_ctl.main(["--url", url, "status", "--debug"]) == 0
            results["status_debug"] = json.loads(capsys.readouterr().out)
        except BaseException as exc:
            err_holder.append(exc)
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

    if err_holder:
        raise err_holder[0]

    out_blind = results["arenas_blind"]
    assert len(out_blind["arenas"]) == 6
    assert "active_arena" not in out_blind
    assert "active_model" not in out_blind
    for entry in out_blind["arenas"]:
        assert "model_path" not in entry
        assert "spawn_xy" not in entry
        assert "default_target_xy" not in entry

    assert "debug" not in results["reload_maze_a"]
    assert "debug" not in results["status_vis"]
    assert "arena" not in json.dumps(results["status_vis"])

    out_debug = results["arenas_debug"]
    assert out_debug["active_arena"] == "maze_a"
    assert out_debug["active_model"] == "arena_maze_a.xml"
    assert all("spawn_xy" in e and "default_target_xy" in e for e in out_debug["arenas"])

    # Switching arenas without --no-preserve-target resets to maze_a default target [1.5, 1.5]
    assert results["status_debug"]["debug"]["target_xy"] == [1.5, 1.5]

    parser = robot_ctl.build_parser()
    with pytest.raises(SystemExit):
        parser.parse_args(["reload", "--arena", "maze_a", "--model", "models/arena_2wheel.xml"])


def test_cli_explore_subcommand_runs_and_strips_debug(capsys):
    app = SimulationApp()
    host, port = app.start_http_server(host="127.0.0.1", port=0)
    url = f"http://{host}:{port}"
    stop_evt = threading.Event()
    holder: dict = {}
    err_holder: list[BaseException] = []

    def worker():
        try:
            assert robot_ctl.main(["--url", url, "explore", "--rule", "right", "--max-steps", "5"]) == 0
            holder["out"] = json.loads(capsys.readouterr().out)
        except BaseException as exc:
            err_holder.append(exc)
        finally:
            stop_evt.set()

    t = threading.Thread(target=worker, daemon=True)
    t.start()
    try:
        while not stop_evt.is_set():
            app.tick_headless(step_batch=10)
            time.sleep(0.005)
        t.join(timeout=15.0)
    finally:
        app.shutdown()

    if err_holder:
        raise err_holder[0]

    out = holder["out"]
    assert out["rule"] == "right"
    assert "target_found" in out
    assert "debug" not in out["after_status"]


