# MuJoCo 2-Wheel Robot & Antigravity Control Bridge Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build an interactive MuJoCo 2-wheeled differential-drive robot arena with onboard rangefinders and front RGB camera, controlled live via a local HTTP/CLI bridge by either a human user or Antigravity.

**Architecture:** A single MJCF scene (`models/arena_2wheel.xml`) models the 2-wheel + caster robot, walled `4m x 4m` arena, colored obstacles, and movable target. `RobotSimulation` (`robot_sim/core.py`) encapsulates physics stepping, watchdog safety, differential-drive kinematics, sensor normalization, and offscreen PNG camera rendering. `sim_server.py` runs `mujoco.mj_step`, `viewer.sync()`, and `mujoco.Renderer` exclusively on the Windows main thread while a background `http.server` thread queues JSON commands from `robot_ctl.py`.

**Tech Stack:** Python 3.12, `uv`, `mujoco>=3.1`, `numpy>=1.26`, `pillow>=10.0`, `pytest>=8.0`, Python standard library (`http.server`, `urllib.request`, `queue`, `threading`, `argparse`).

**Spec:** `docs/superpowers/specs/2026-11-11-mujoco-2wheel-robot-design.md`

## Global Constraints

- OS & Shell: Windows, PowerShell (`pwsh`), Python 3.12 and `uv` package manager.
- Runtime dependencies strictly limited to `mujoco`, `numpy`, and `pillow` (`pytest` for tests only).
- Windows OpenGL thread safety: `mujoco.mj_step`, `viewer.sync()`, and `mujoco.Renderer.render()` MUST execute only on the main thread; background HTTP threads communicate strictly via `queue.Queue`.
- Anti-goal (no teleportation / god-mode): Never drive the robot by writing `qpos`; `status` separates onboard sensors (`sensors`) from ground-truth coordinates (`debug`), and autonomous/Agent driving uses only onboard sensors + camera.
- Watchdog safety: Every wheel command expires after a bounded duration (capped at `10.0s`) and automatically zeroes actuator controls.
- Arena size `4m x 4m` (`[-2.0, 2.0]` m), chassis `24cm x 20cm`, wheel radius `0.05m`, rangefinder cutoff `3.0m`, camera resolution `640x480`, default HTTP endpoint `127.0.0.1:8765`.
- XML comments and `README.md` written in clear beginner-friendly Vietnamese.

## Review Focus

- MuJoCo `<rangefinder>` returns `-1.0` when nothing is hit within `cutoff="3.0"`; `get_status()` must normalize `< 0` readings to `3.0` and expose boolean `hit` flags so open space never triggers false obstacle alerts.
- Non-finite (`NaN`/`inf`) wheel speeds, non-positive durations, or runaway durations must be rejected (`ValueError` / HTTP `400`) or clamped (`[-15.0, 15.0]` rad/s, max `10.0s`), and expired commands must zero wheel actuators automatically.
- Requesting an unknown camera name on `/camera` or capturing before `captures/` exists must auto-create directories and return HTTP `400` JSON on invalid camera names without crashing the main simulation thread.
- Resetting `target_xy` outside the `4m x 4m` walled interior (beyond `[-1.7, 1.7]`) must raise `ValueError` (HTTP `400`) rather than placing the target outside the walls.
- Running `robot_ctl.py` when `sim_server.py` is not running must print a friendly Vietnamese startup hint and exit with code `1` instead of a Python traceback.

---

### Task 1: Project Scaffolding & 2-Wheel Robot Arena MJCF Model

**Files:**
- Create: `.gitignore`
- Create: `pyproject.toml`
- Create: `models/arena_2wheel.xml`
- Test: `tests/test_model.py`

**Interfaces:**
- Consumes: none
- Produces: `models/arena_2wheel.xml` with:
  - Body `robot` (`freejoint` `robot_free`), child wheel bodies `left_wheel` (hinge `left_wheel_joint`, pos `0.03 0.115 0.05`, radius `0.05`) and `right_wheel` (hinge `right_wheel_joint`, pos `0.03 -0.115 0.05`, radius `0.05`), rear low-friction caster sphere geom (`pos="-0.09 0 0.025"`, radius `0.025`), chassis box geom (`size="0.12 0.10 0.03"`, `pos="0 0 0.06"`).
  - Mocap body `target` (`mocap="true"`, bright green cylinder/sphere geom with `contype="1" conaffinity="1"`), boundary walls at `x,y = ±2.0`, colored obstacle boxes (`obs_red`, `obs_blue`, `obs_yellow`).
  - Cameras: `front_cam` on `robot` (`pos="0.12 0 0.09"`, `xyaxes="0 -1 0 0 0 1"`, `fovy="65"`) and `overview_cam` in `worldbody`.
  - Actuators (in order): velocity servos `left_motor` and `right_motor` (`kv="1.0"`, `ctrlrange="-15 15"`, `forcerange="-2.5 2.5"`).
  - Sensors: `rf_left` (+30 deg yaw), `rf_center` (0 deg yaw, +X forward), `rf_right` (-30 deg yaw) all with `cutoff="3.0"`, plus `left_wheel_vel`, `right_wheel_vel`, `chassis_vel`, `chassis_gyro`.

- [ ] **Step 1: Initialize git, `.gitignore`, and `pyproject.toml`, then sync dependencies**

Create `.gitignore` ignoring `.venv/`, `__pycache__/`, `.pytest_cache/`, `captures/`, `.superpowers/`. Create `pyproject.toml` (`name = "robot-demo"`, `requires-python = ">=3.12"`, dependencies `["mujoco>=3.1", "numpy>=1.26", "pillow>=10.0"]`, optional/dev `pytest>=8.0`).
Run: `git init; uv sync --dev`
Expected: `.venv` created with `mujoco`, `numpy`, `pillow`, `pytest` installed.

- [ ] **Step 2: Write the failing MJCF physics & sensor tests in `tests/test_model.py`**

```python
from pathlib import Path
import mujoco
import numpy as np

MODEL_PATH = Path(__file__).resolve().parents[1] / "models" / "arena_2wheel.xml"


def test_model_loads_and_named_elements_exist():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "robot") >= 0
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "target") >= 0
    for cam in ("front_cam", "overview_cam"):
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, cam) >= 0
    for act in ("left_motor", "right_motor"):
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, act) >= 0
    for sensor in ("rf_left", "rf_center", "rf_right", "left_wheel_vel", "right_wheel_vel", "chassis_vel", "chassis_gyro"):
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, sensor) >= 0


def test_robot_drives_forward_and_turns_cleanly():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    # Move target out of forward path so robot has clear runway
    data.mocap_pos[0] = [1.5, 1.2, 0.12]
    data.ctrl[:] = [6.0, 6.0]
    for _ in range(500):
        mujoco.mj_step(model, data)
    assert data.qpos[0] > 0.20  # moved forward along +X
    assert abs(data.qpos[1]) < 0.05  # stayed straight
    assert abs(data.qpos[2]) < 0.02  # did not tip or bounce

    # Differential spin in place (right wheel forward, left wheel backward => +Z CCW yaw)
    mujoco.mj_resetData(model, data)
    data.ctrl[:] = [-5.0, 5.0]
    for _ in range(300):
        mujoco.mj_step(model, data)
    quat = data.qpos[3:7]
    yaw = np.arctan2(2.0 * (quat[0] * quat[3] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2] ** 2 + quat[3] ** 2))
    assert yaw > 0.3


def test_rangefinder_points_forward_and_front_cam_renders():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    # Place mocap target directly 0.8m in front of robot nose (nose is at x=0.12)
    data.mocap_pos[0] = [0.92, 0.0, 0.10]
    mujoco.mj_forward(model, data)
    rf_center_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "rf_center")
    rf_center_adr = model.sensor_adr[rf_center_id]
    assert 0.55 < data.sensordata[rf_center_adr] < 0.90

    renderer = mujoco.Renderer(model, height=480, width=640)
    try:
        renderer.update_scene(data, camera="front_cam")
        img = renderer.render()
        assert img.shape == (480, 640, 3)
        # Green target in center of frame should produce strong green pixels
        center_patch = img[180:300, 260:380]
        assert int(center_patch[:, :, 1].max()) > 150
    finally:
        renderer.close()
```

- [ ] **Step 3: Run pytest to verify `test_model.py` fails**

Run: `uv run pytest tests/test_model.py -v`
Expected: FAIL (`FileNotFoundError` for `arena_2wheel.xml`).

- [ ] **Step 4: Create `models/arena_2wheel.xml` with Vietnamese comments**

Implement the MJCF scene matching the exact element names, dimensions, sensors, actuators, obstacles (placed off the immediate origin runway `[0..1.0, 0]` so forward test has clear room, e.g., at `(0.9, 0.7)`, `(0.8, -0.7)`, `(-0.9, 0.6)`), and default `target` mocap body at `(1.4, 0.0, 0.12)` (radius `0.10`, height `0.12`, bright green `rgba="0.1 0.9 0.2 1"`). Note: in MuJoCo `<rangefinder>` shoots along `+Z` of its `<site>`, so orient `site_rf_center` with `+Z` pointing along robot `+X` (`zaxis="1 0 0"`), `site_rf_left` at `zaxis="0.866 0.5 0"`, and `site_rf_right` at `zaxis="0.866 -0.5 0"`, placed at `x=0.125, z=0.075` outside the chassis geom so rays do not self-intersect the robot body. Set `<visual><global offwidth="640" offheight="480"/></visual>`.

- [ ] **Step 5: Run pytest to verify `test_model.py` passes**

Run: `uv run pytest tests/test_model.py -v`
Expected: 3 passed.

- [ ] **Step 6: Commit Task 1**

```powershell
git add .gitignore pyproject.toml uv.lock models/arena_2wheel.xml tests/test_model.py
git commit -m "feat: add MuJoCo 2-wheel robot and arena MJCF model"
```

---

### Task 2: Simulation Core, Sensor Normalization, Watchdog & Offscreen Camera

**Files:**
- Create: `robot_sim/__init__.py`
- Create: `robot_sim/core.py`
- Test: `tests/test_core.py`

**Interfaces:**
- Consumes: `models/arena_2wheel.xml` from Task 1.
- Produces in `robot_sim/core.py`:
  - Constants: `WHEEL_RADIUS = 0.05`, `TRACK_WIDTH = 0.23`, `MAX_WHEEL_RAD_S = 15.0`, `MAX_COMMAND_DURATION = 10.0`, `RANGEFINDER_MAX = 3.0`, `ARENA_SAFE_LIMIT = 1.7`.
  - `compute_move_command(meters: float, speed_m_s: float = 0.25) -> tuple[float, float, float]` returning `(left_rad_s, right_rad_s, duration_s)`.
  - `compute_turn_command(degrees: float, angular_deg_s: float = 60.0) -> tuple[float, float, float]` returning `(left_rad_s, right_rad_s, duration_s)` (+degrees = counter-clockwise left turn).
  - `class RobotSimulation`:
    - `__init__(self, model_path: Path | str | None = None)`
    - `close(self) -> None`
    - `set_wheels(self, left_rad_s: float, right_rad_s: float, duration: float = 1.0) -> dict`
    - `move(self, meters: float, speed_m_s: float = 0.25) -> dict`
    - `turn(self, degrees: float, angular_deg_s: float = 60.0) -> dict`
    - `stop(self) -> dict`
    - `reset(self, target_xy: tuple[float, float] | None = None) -> dict`
    - `step_once(self) -> None` and `step_seconds(self, seconds: float) -> None`
    - `get_status(self) -> dict` returning `{"sim_time": float, "moving": bool, "command_remaining_s": float, "sensors": {"rangefinders_m": {"left": float, "center": float, "right": float}, "rangefinder_hits": {"left": bool, "center": bool, "right": bool}, "wheel_vel_rad_s": {"left": float, "right": float}, "forward_speed_m_s": float, "yaw_rate_rad_s": float}, "debug": {"robot_xy": [float, float], "robot_yaw_deg": float, "target_xy": [float, float]}}`
    - `capture_camera(self, camera_name: str = "front_cam", out_path: Path | str | None = None) -> dict` returning `{"camera": str, "path": str, "width": 640, "height": 480}`

- [ ] **Step 1: Write failing unit tests in `tests/test_core.py` (including Review Focus edge cases)**

```python
from pathlib import Path
import math
import pytest
from robot_sim.core import RobotSimulation, RANGEFINDER_MAX, MAX_WHEEL_RAD_S


def test_open_space_rangefinder_normalizes_minus_one():
    sim = RobotSimulation()
    try:
        sim.reset(target_xy=(-1.5, 1.5))
        # Place robot near (-1.6, -1.6) facing +45 deg diagonal (~5.1m > 3.0m cutoff) so raw rangefinder is -1.0
        sim.data.qpos[0:2] = [-1.6, -1.6]
        yaw = math.pi / 4.0
        sim.data.qpos[3:7] = [math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)]
        status = sim.get_status()
        assert status["sensors"]["rangefinder_hits"]["center"] is False
        assert status["sensors"]["rangefinders_m"]["center"] == RANGEFINDER_MAX
    finally:
        sim.close()


def test_watchdog_stops_wheels_after_duration_and_clamps_speed():
    sim = RobotSimulation()
    try:
        res = sim.set_wheels(50.0, -50.0, duration=0.4)
        assert res["left_rad_s"] == MAX_WHEEL_RAD_S
        assert res["right_rad_s"] == -MAX_WHEEL_RAD_S
        sim.step_seconds(0.2)
        assert sim.get_status()["moving"] is True
        sim.step_seconds(0.3)
        st = sim.get_status()
        assert st["moving"] is False
        assert abs(sim.data.ctrl[0]) == 0.0 and abs(sim.data.ctrl[1]) == 0.0
    finally:
        sim.close()


@pytest.mark.parametrize("bad_args", [
    (math.nan, 1.0, 1.0),
    (1.0, math.inf, 1.0),
    (2.0, 2.0, 0.0),
    (2.0, 2.0, -1.0),
])
def test_set_wheels_rejects_invalid_inputs(bad_args):
    sim = RobotSimulation()
    try:
        with pytest.raises(ValueError):
            sim.set_wheels(*bad_args)
    finally:
        sim.close()


def test_move_and_turn_kinematics_and_reset_bounds():
    sim = RobotSimulation()
    try:
        sim.reset(target_xy=(-1.2, 1.2))
        sim.move(0.30, speed_m_s=0.30)
        sim.step_seconds(1.2)
        x_after = sim.get_status()["debug"]["robot_xy"][0]
        assert 0.18 < x_after < 0.42

        sim.reset(target_xy=(-1.2, 1.2))
        sim.turn(90.0, angular_deg_s=90.0)
        sim.step_seconds(1.2)
        yaw_deg = sim.get_status()["debug"]["robot_yaw_deg"]
        assert 55.0 < yaw_deg < 125.0

        with pytest.raises(ValueError):
            sim.reset(target_xy=(5.0, 0.0))
    finally:
        sim.close()


def test_capture_camera_creates_png_and_rejects_unknown_camera(tmp_path: Path):
    sim = RobotSimulation()
    try:
        out_file = tmp_path / "nested" / "front.png"
        info = sim.capture_camera("front_cam", out_path=out_file)
        assert out_file.is_file()
        assert out_file.stat().st_size > 1000
        assert info["width"] == 640 and info["height"] == 480

        with pytest.raises(ValueError):
            sim.capture_camera("nonexistent_cam", out_path=out_file)
    finally:
        sim.close()
```

- [ ] **Step 2: Run pytest to verify `test_core.py` fails**

Run: `uv run pytest tests/test_core.py -v`
Expected: FAIL (`ModuleNotFoundError: No module named 'robot_sim'`).

- [ ] **Step 3: Implement `robot_sim/__init__.py` and `robot_sim/core.py`**

Implement `RobotSimulation`, `compute_move_command`, and `compute_turn_command`:
- Lazily initialize `mujoco.Renderer(self.model, height=480, width=640)` on the thread calling `capture_camera` (so if `RobotSimulation` is constructed before entering the loop or on the main thread, OpenGL binds cleanly on main thread) and close it in `close()`.
- In `step_once()`, check `if self._command_deadline is not None and self.data.time >= self._command_deadline: self.stop()` before `mujoco.mj_step(self.model, self.data)`.
- In `get_status()`, call `mujoco.mj_forward(self.model, self.data)` first so sensor values always reflect the latest state, then normalize `<rangefinder>` values `< 0.0` to `RANGEFINDER_MAX` (`3.0`) with `hit = raw >= 0.0`.
- Cap `duration` in `set_wheels` at `MAX_COMMAND_DURATION = 10.0` (raise `ValueError` if `duration <= 0` or any input is not finite).
- Validate `target_xy` in `reset()` to lie within `[-ARENA_SAFE_LIMIT, ARENA_SAFE_LIMIT]` (`±1.7`).

- [ ] **Step 4: Run pytest to verify `test_core.py` passes**

Run: `uv run pytest tests/test_core.py -v`
Expected: All tests in `tests/test_core.py` and `tests/test_model.py` PASS.

- [ ] **Step 5: Commit Task 2**

```powershell
git add robot_sim/__init__.py robot_sim/core.py tests/test_core.py
git commit -m "feat: add simulation core, sensor normalization, watchdog, and camera capture"
```

---

### Task 3: Thread-Safe Local HTTP Bridge & Live 3D Viewer Server

**Files:**
- Create: `sim_server.py`
- Test: `tests/test_server.py`

**Interfaces:**
- Consumes: `RobotSimulation` from `robot_sim/core.py`.
- Produces in `sim_server.py`:
  - `class SimulationApp`: wraps `RobotSimulation`, a thread-safe `queue.Queue` of `(callable, response_queue)` jobs executed exclusively on the main loop thread via `process_pending_commands()`, plus `start_http_server(host="127.0.0.1", port=8765)` and `shutdown()`.
  - HTTP JSON endpoints on `http://127.0.0.1:<port>`:
    - `GET /status` -> `200 {"ok": true, "data": <get_status()>}`
    - `POST /wheels` body `{"left": float, "right": float, "duration": float}` -> `200 {"ok": true, "data": ...}`
    - `POST /move` body `{"meters": float, "speed": float}` -> `200 {"ok": true, "data": ...}`
    - `POST /turn` body `{"degrees": float, "angular_speed": float}` -> `200 {"ok": true, "data": ...}`
    - `POST /stop` body `{}` -> `200 {"ok": true, "data": ...}`
    - `POST /reset` body `{"target_xy": [x, y] | null}` -> `200 {"ok": true, "data": ...}`
    - `POST /camera` body `{"camera": "front_cam" | "overview_cam", "path": str | null}` -> `200 {"ok": true, "data": ...}`
    - Validation errors (`ValueError`, malformed JSON, unknown route) return `400`/`404` `{"ok": false, "error": "<message>"}` without terminating the simulation loop.
  - CLI entrypoint `python sim_server.py [--host 127.0.0.1] [--port 8765] [--headless] [--duration <seconds>]` (when `--headless` is omitted, launches `mujoco.viewer.launch_passive(sim.model, sim.data)` so the user sees the live 3D window).

- [ ] **Step 1: Write failing integration tests in `tests/test_server.py`**

```python
import json
from pathlib import Path
import threading
import time
import urllib.error
import urllib.request
from sim_server import SimulationApp


def _request(url: str, method: str = "GET", payload: dict | None = None) -> tuple[int, dict]:
    data = None if payload is None else json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(url, data=data, method=method, headers={"Content-Type": "application/json"})
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
```

- [ ] **Step 2: Run pytest to verify `test_server.py` fails**

Run: `uv run pytest tests/test_server.py -v`
Expected: FAIL (`ModuleNotFoundError: No module named 'sim_server'`).

- [ ] **Step 3: Implement `sim_server.py`**

Implement `SimulationApp` (`ThreadingHTTPServer` on a daemon thread pushing callables into `self._cmd_queue`, executed by `process_pending_commands()` inside `tick_headless()` and inside the `mujoco.viewer.launch_passive` real-time loop). Keep `log_message` quiet by default so terminal output stays clean. Print startup banner in Vietnamese showing the HTTP address and quick commands when launched via `main()`.

- [ ] **Step 4: Run pytest to verify `test_server.py` passes**

Run: `uv run pytest tests/test_server.py -v`
Expected: PASS.

- [ ] **Step 5: Commit Task 3**

```powershell
git add sim_server.py tests/test_server.py
git commit -m "feat: add thread-safe local HTTP simulation server and 3D viewer loop"
```

---

### Task 4: CLI Controller (`robot_ctl.py`), Autonomous Demo & Vietnamese Guide (`README.md`)

**Files:**
- Create: `robot_ctl.py`
- Create: `README.md`
- Test: `tests/test_cli.py`

**Interfaces:**
- Consumes: HTTP JSON endpoints from `sim_server.py`.
- Produces:
  - `robot_ctl.py` CLI (`main(argv: list[str] | None = None) -> int`) supporting `--url http://127.0.0.1:8765` and subcommands:
    - `status [--debug]` (prints onboard `sensors` JSON by default so Agent does not accidentally read ground-truth `debug`; passes `debug` only when `--debug` flag is explicitly set!)
    - `camera [--name front_cam|overview_cam] [--out <path>]`
    - `wheels <left_rad_s> <right_rad_s> [--duration 1.0]`
    - `move <meters> [--speed 0.25] [--wait]` (`--wait` sleeps for command duration + `0.1s` and returns updated onboard sensor status)
    - `turn <degrees> [--speed 60.0] [--wait]`
    - `stop`
    - `reset [--target-x <x> --target-y <y>]`
    - `demo [--steps 25]` (runs a pure onboard-sensor reactive exploration / obstacle-avoidance loop using `rf_left`, `rf_center`, `rf_right` so the user can watch classic closed-loop Python control in 3D).
  - `README.md` in Vietnamese covering: quick start (`uv run python sim_server.py` + `uv run python robot_ctl.py ...`), how Antigravity controls the robot via camera + sensors, and a Sim-to-Real mapping table (`<body/>`/`<geom/>` -> chassis, `<joint type="hinge"/>` -> wheel axle, `<velocity/>` -> DC gearmotor + encoder + motor driver, `<rangefinder/>` -> HC-SR04 / VL53L0X ToF, `<camera/>` -> CSI/USB camera, `sim_server.py` HTTP -> Wi-Fi/UART firmware protocol).

- [ ] **Step 1: Write failing CLI tests in `tests/test_cli.py`**

```python
import json
from pathlib import Path
import threading
import time
from sim_server import SimulationApp
import robot_ctl


def test_cli_offline_prints_friendly_vietnamese_hint(capsys):
    # Port 9 is discard / closed on localhost
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
```

- [ ] **Step 2: Run pytest to verify `test_cli.py` fails**

Run: `uv run pytest tests/test_cli.py -v`
Expected: FAIL (`ModuleNotFoundError: No module named 'robot_ctl'`).

- [ ] **Step 3: Implement `robot_ctl.py` and `README.md`**

Implement `robot_ctl.py` using `argparse` (note: pass `allow_abbrev=False` and handle negative positional numbers like `move -0.2` or `turn -45` cleanly — e.g., via `--meters`/`--degrees` optional flags OR pre-normalizing/configuring argparse so negative numbers work naturally; support both `turn -45` and `turn --degrees -45`!). Catch `urllib.error.URLError` to print a clear Vietnamese guide pointing to `uv run python sim_server.py` and return `1`. Strip `"debug"` from `status` output unless `--debug` is passed so Antigravity operates strictly on realistic onboard sensors by default. Write `README.md` in clear Vietnamese.

- [ ] **Step 4: Run full test suite to verify everything passes**

Run: `uv run pytest -v`
Expected: All tests across `test_model.py`, `test_core.py`, `test_server.py`, and `test_cli.py` PASS.

- [ ] **Step 5: Commit Task 4**

```powershell
git add robot_ctl.py README.md tests/test_cli.py docs/superpowers/
git commit -m "feat: add robot_ctl CLI, reactive demo, and Vietnamese documentation"
```
