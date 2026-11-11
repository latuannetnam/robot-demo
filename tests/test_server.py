import json
from pathlib import Path
import threading
import time
import urllib.error
import urllib.request
from sim_server import SimulationApp


def _request(url: str, method: str = "GET", payload: dict | None = None) -> tuple[int, dict]:
    data = None if payload is None else json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(
        url,
        data=data,
        method=method,
        headers={"Content-Type": "application/json"},
    )
    try:
        with urllib.request.urlopen(req, timeout=5.0) as resp:
            return resp.status, json.loads(resp.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read().decode("utf-8"))


def test_http_bridge_drives_captures_and_survives_bad_requests(tmp_path: Path):
    app = SimulationApp()
    host, port = app.start_http_server(host="127.0.0.1", port=0)  # ephemeral port
    base = f"http://{host}:{port}"
    stop_evt = threading.Event()

    # Drive HTTP requests from a worker thread while THIS main thread runs physics + OpenGL!
    results: dict = {}

    def client_worker():
        try:
            results["reset"] = _request(f"{base}/reset", "POST", {"target_xy": [-1.2, 1.2]})
            results["move"] = _request(f"{base}/move", "POST", {"meters": 0.25, "speed": 0.25})
            time.sleep(0.35)
            results["bad_cam"] = _request(f"{base}/camera", "POST", {"camera": "bad_cam"})
            out_png = str(tmp_path / "front.png")
            results["good_cam"] = _request(f"{base}/camera", "POST", {"camera": "front_cam", "path": out_png})
            results["status"] = _request(f"{base}/status", "GET")
        finally:
            stop_evt.set()

    t = threading.Thread(target=client_worker, daemon=True)
    t.start()
    try:
        while not stop_evt.is_set():
            app.tick_headless(step_batch=10)
            time.sleep(0.005)
        t.join(timeout=5.0)
    finally:
        app.shutdown()

    assert results["reset"][0] == 200
    assert results["move"][0] == 200
    assert results["bad_cam"][0] == 400 and results["bad_cam"][1]["ok"] is False
    assert results["good_cam"][0] == 200 and Path(results["good_cam"][1]["data"]["path"]).is_file()
    assert results["status"][0] == 200
    assert results["status"][1]["data"]["debug"]["robot_xy"][0] > 0.03


def test_reset_and_camera_reject_malformed_types_with_400(tmp_path: Path):
    app = SimulationApp()
    host, port = app.start_http_server(host="127.0.0.1", port=0)
    base = f"http://{host}:{port}"
    stop_evt = threading.Event()
    results: dict = {}

    def client_worker():
        try:
            results["bool_xy"] = _request(f"{base}/reset", "POST", {"target_xy": [True, False]})
            results["str_xy"] = _request(f"{base}/reset", "POST", {"target_xy": "10"})
            results["null_cam"] = _request(f"{base}/camera", "POST", {"camera": None})
            results["dir_path"] = _request(f"{base}/camera", "POST", {"camera": "front_cam", "path": str(tmp_path)})
            results["empty_path"] = _request(f"{base}/camera", "POST", {"camera": "front_cam", "path": "   "})
        finally:
            stop_evt.set()

    t = threading.Thread(target=client_worker, daemon=True)
    t.start()
    try:
        while not stop_evt.is_set():
            app.tick_headless(step_batch=5)
            time.sleep(0.005)
        t.join(timeout=5.0)
    finally:
        app.shutdown()

    for key in ("bool_xy", "str_xy", "null_cam", "dir_path", "empty_path"):
        assert results[key][0] == 400, f"{key} returned {results[key]}"
        assert results[key][1]["ok"] is False
