# Multi-Agent Robot Control, Telemetry & Auto-R&D Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a 3-role Multi-Agent R&D control architecture (`Main Agent`, `robot-driver`, `robot-engineer`) backed by 20 Hz Sim-to-Real Python reflexes, hot-reloadable robot mechanics with anti-cheating guards, and a zero-LLM-token background telemetry blackbox.

**Architecture:** Stabilize the 2-wheel robot mechanically (`front_caster`) and lock arena parameters via `robot_sim/integrity.py`, expose thread-safe `POST /reload` on `sim_server.py`, implement Sim-to-Real closed-loop client skills (`wait_until_settled`, `closed_loop_turn`, `scan_for_target`, `approach_target`) in `robot_sim/skills.py` + `robot_ctl.py`, add a signal-file-controlled 10 Hz blackbox in `robot_sim/telemetry.py`, and package the 3-role orchestration workflow as a progressive-disclosure Workspace Skill under `.agents/skills/robot-multi-agent-rd/`.

**Tech Stack:** Python 3.12 (`.venv\Scripts\python.exe`), `mujoco`, `numpy`, `pillow`, `pytest`, Python stdlib (`xml.etree.ElementTree`, `http.server`, `urllib.request`, `argparse`, `json`, `threading`, `queue`).

**Spec:** `docs/superpowers/specs/2026-11-11-multi-agent-robot-control-design.md`

## Global Constraints

- **OS & Shell:** Windows + PowerShell (`pwsh`), Python 3.12 inside `.venv` (`mujoco`, `numpy`, `pillow`, `pytest` only — install zero new external packages).
- **Fast CLI Invocation:** Use `.venv\Scripts\python.exe` directly for automated/agent CLI and test commands (~0.3s startup vs ~2.5s for `uv run python`), while preserving `uv run python` compatibility for manual user commands.
- **Desktop Isolation (`exebox`):** `sim_server.py` runs in the user's own PowerShell window on `http://127.0.0.1:8765`; never attempt to kill/spawn user desktop windows or bypass sandbox boundaries.
- **OpenGL & MuJoCo Thread Safety:** Every `MjModel`, `MjData`, and `Renderer` read/write (including `reload`) must execute on `SimulationApp`'s main thread via `_cmd_queue`.
- **Sim-to-Real & Anti-Cheating Discipline:** `robot-driver` and client skills (`scan`, `approach`, `closed_loop_turn`, `wait_until_settled`) must never read or output `debug` ground-truth coordinates; `models/arena_2wheel.xml` arena/obstacles/target geometry must remain immutable.

## Review Focus

1. **Corrupt or arena-tampered XML passed to `reload` / `POST /reload`:** Malformed XML or any edit to floor, walls, obstacles, or target must raise `ValueError` (HTTP `400` on `/reload`) and leave the existing `RobotSimulation` running intact without crashing `sim_server.py`.
2. **±180° yaw wrap-around and invalid parameters in `turn`, `scan`, and `approach`:** Gyro integration must accumulate `yaw_rate_rad_s * dt_sim` continuously across the `±180°` boundary without atan2 jumps, and invalid inputs (`degrees == 0`, `max_degrees <= 0`, `stop_distance < 0.12` or `> 1.5`) must fail cleanly with Vietnamese error messages.
3. **Offline/closing server during `telemetry record` and `telemetry stop` with no active recorder:** Connection failures during recording must increment `error_count` and still flush valid JSON on stop; `telemetry stop` with no fresh output must exit with code `1` and a clear Vietnamese message instead of hanging.
4. **Stale `.telemetry_stop` signal file from a prior R&D round:** `telemetry record` must delete any pre-existing `stop_flag_path` before entering its sampling loop so Round 2+ does not terminate at 0 samples.
5. **Ground-truth `debug` leakage in new CLI commands:** `scan`, `approach`, `reload`, and settled `--wait` outputs in `robot_ctl.py` must strip the `"debug"` key from every returned status payload.

---

### Task 1: Arena Integrity Guard, Front Caster Stabilization & `RobotSimulation.reload`

**Files:**
- Create: `robot_sim/integrity.py`
- Modify: `models/arena_2wheel.xml:61-103`
- Modify: `robot_sim/core.py:66-179`
- Modify: `robot_sim/__init__.py`
- Create: `tests/test_integrity.py`
- Modify: `tests/test_core.py`

**Interfaces:**
- Consumes: `DEFAULT_MODEL_PATH`, `RobotSimulation` in `robot_sim/core.py`
- Produces:
  - `robot_sim.integrity.verify_arena_integrity(model_path: Path | str | None = None) -> dict` — validates XML & `MjModel`, returns `{"valid": True, "model_path": str}` or raises `ValueError` (Vietnamese message).
  - `RobotSimulation.reload(self, model_path: Path | str | None = None, preserve_target: bool = True) -> dict` — validates via `verify_arena_integrity`, swaps `self.model`/`self.data`, resets cached `self._renderer`, preserves `mocap_pos[0][:2]` when `preserve_target=True`, returns `self.get_status()`.

- [ ] **Step 1: Write failing tests in `tests/test_integrity.py` and `tests/test_core.py`**

Create `tests/test_integrity.py` testing:
```python
from pathlib import Path
import pytest
from robot_sim.core import DEFAULT_MODEL_PATH, RobotSimulation
from robot_sim.integrity import verify_arena_integrity


def test_default_arena_passes_integrity():
    res = verify_arena_integrity(DEFAULT_MODEL_PATH)
    assert res["valid"] is True


@pytest.mark.parametrize(
    "tamper_src,tamper_dst",
    [
        ('pos="-0.85 0.75 0.12"', 'pos="-1.50 0.75 0.12"'),  # moved obs_yellow
        ('size="0.12 0.12"', 'size="0.02 0.02"'),            # shrunk obs_yellow
        ('size="0.10 0.12"', 'size="0.40 0.12"'),            # enlarged green target
        ('pos="0 2.025 0.125"', 'pos="0 3.025 0.125"'),      # moved wall_north
        ('pos="0.12 0 0.09"', 'pos="0.12 0 1.50"'),          # illegal periscope front_cam height
    ],
)
def test_verify_arena_integrity_rejects_arena_or_periscope_tampering(tmp_path: Path, tamper_src: str, tamper_dst: str):
    xml_text = DEFAULT_MODEL_PATH.read_text(encoding="utf-8")
    assert tamper_src in xml_text
    bad_xml = tmp_path / "tampered.xml"
    bad_xml.write_text(xml_text.replace(tamper_src, tamper_dst, 1), encoding="utf-8")
    with pytest.raises(ValueError):
        verify_arena_integrity(bad_xml)


def test_reload_rejects_tampered_or_corrupt_xml_without_breaking_sim(tmp_path: Path):
    sim = RobotSimulation()
    try:
        sim.reset(target_xy=(-1.4, 1.15))
        corrupt = tmp_path / "corrupt.xml"
        corrupt.write_text("<mujoco><broken></mujoco>", encoding="utf-8")
        with pytest.raises(ValueError):
            sim.reload(model_path=corrupt)

        # Simulation remains intact with previous target_xy preserved
        st = sim.get_status()
        assert st["debug"]["target_xy"] == [-1.4, 1.15]

        # Valid reload preserves current target_xy by default
        reloaded = sim.reload(preserve_target=True)
        assert reloaded["debug"]["target_xy"] == [-1.4, 1.15]
    finally:
        sim.close()
```

And append `test_braking_from_cruise_does_not_strike_floor_with_rangefinder` to `tests/test_core.py`:
```python
def test_braking_from_cruise_does_not_strike_floor_with_rangefinder():
    sim = RobotSimulation()
    try:
        sim.reset(target_xy=(-1.4, 1.15))
        cmd = sim.move(0.75, speed_m_s=0.35)
        sim.step_seconds(cmd["duration_s"])
        # Sample across the 150ms post-stop braking window where nose-dive previously hit 0.78m
        min_center = 3.0
        for _ in range(15):
            sim.step_seconds(0.01)
            min_center = min(min_center, sim.get_status()["sensors"]["rangefinders_m"]["center"])
        assert min_center > 1.05
    finally:
        sim.close()
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `.venv\Scripts\python.exe -m pytest tests/test_integrity.py tests/test_core.py -v`
Expected: FAIL (`ModuleNotFoundError: No module named 'robot_sim.integrity'` and nose-dive `min_center` ~ `0.80m < 1.05m`).

- [ ] **Step 3: Implement `robot_sim/integrity.py`, front caster in `models/arena_2wheel.xml`, and `RobotSimulation.reload` in `robot_sim/core.py`**

1. In `models/arena_2wheel.xml` inside `<body name="robot">` right after `caster`, add a front support sphere touching `z=0` so braking torque cannot pitch the nose downward:
   `<geom name="front_caster" type="sphere" pos="0.095 0 0.025" size="0.025" mass="0.05" friction="0.01 0.001 0.0001" rgba="0.75 0.75 0.78 1"/>`
2. Create `robot_sim/integrity.py` implementing `verify_arena_integrity(model_path: Path | str | None = None) -> dict`:
   - Parse XML via `xml.etree.ElementTree.parse` (catching `ET.ParseError` / `OSError` -> raise `ValueError` in Vietnamese) and load `mujoco.MjModel.from_xml_path` (catching exceptions -> `ValueError`).
   - Pin exact world elements: `floor` (`size="2.0 2.0 0.05"`), `wall_north` (`pos="0 2.025 0.125"`, `size="2.05 0.025 0.125"`), `wall_south` (`pos="0 -2.025 0.125"`, `size="2.05 0.025 0.125"`), `wall_east` (`pos="2.025 0 0.125"`, `size="0.025 2.00 0.125"`), `wall_west` (`pos="-2.025 0 0.125"`, `size="0.025 2.00 0.125"`), `obs_red` (`pos="0.70 0.95 0.12"`, box `size="0.12 0.12 0.12"`), `obs_blue` (`pos="0.85 -0.75 0.12"`, box `size="0.12 0.12 0.12"`), `obs_yellow` (`pos="-0.85 0.75 0.12"`, cylinder `size="0.12 0.12"`), and `target` (`mocap="true"`, `target_geom` cylinder `size="0.10 0.12"`, `contype="1"`, `conaffinity="1"`). Compare numeric vectors within `1e-4` tolerance.
   - Verify `<body name="robot">` has `<freejoint>`, required sensors (`rf_left`, `rf_center`, `rf_right`, `left_wheel_vel`, `right_wheel_vel`, `chassis_vel`, `chassis_gyro`), actuators (`left_motor`, `right_motor`), `front_cam`, and every `geom`/`site`/`camera`/`body` inside `robot` has local `|x| <= 0.25`, `|y| <= 0.20`, `0.0 <= z <= 0.30`.
3. Add `reload(self, model_path: Path | str | None = None, preserve_target: bool = True) -> dict` to `RobotSimulation` in `robot_sim/core.py` (calling `verify_arena_integrity`, loading new `MjModel`/`MjData` before mutating `self`, closing `self._renderer` if open, refreshing `self._sensor_adr`, restoring previous `mocap_pos[0]` when `preserve_target=True`, and returning `self.get_status()`), and export `verify_arena_integrity` in `robot_sim/__init__.py`.

- [ ] **Step 4: Run tests to verify they pass**

Run: `.venv\Scripts\python.exe -m pytest tests/test_integrity.py tests/test_core.py tests/test_model.py -v`
Expected: PASS (all tests green).

- [ ] **Step 5: Commit**

```powershell
git add models/arena_2wheel.xml robot_sim/integrity.py robot_sim/core.py robot_sim/__init__.py tests/test_integrity.py tests/test_core.py
git commit -m "feat(core): add arena integrity guard, front caster stabilization, and model reload"
```

---

### Task 2: Server `POST /reload` & Sim-to-Real Closed-Loop Client Skills (`scan`, `approach`, Gyro `turn`, Settled `--wait`)

**Files:**
- Create: `robot_sim/skills.py`
- Modify: `sim_server.py:20-235`
- Modify: `robot_ctl.py:55-270`
- Create: `tests/test_skills.py`
- Modify: `tests/test_server.py`
- Modify: `tests/test_cli.py`

**Interfaces:**
- Consumes: `RobotSimulation.reload` from Task 1; `_call_bridge`, `_strip_debug` in `robot_ctl.py`.
- Produces:
  - `POST /reload` on `sim_server.py` accepting `{"preserve_target": bool = True, "target_xy": [x, y] | None = None}` -> `200 {"ok": True, "data": status_dict}` or `400 {"ok": False, "error": str}`.
  - `BridgeCall = Callable[[str, str, dict | None], dict]` (`(path, method, payload) -> data_dict`).
  - `robot_sim.skills.wait_until_settled(call_fn: BridgeCall, duration_s: float, include_vision: bool = False, speed_tol: float = 0.015, yaw_tol: float = 0.03, settle_samples: int = 2) -> dict`
  - `robot_sim.skills.closed_loop_turn(call_fn: BridgeCall, degrees: float, max_speed_deg_s: float = 90.0, tol_deg: float = 1.8, timeout_s: float = 8.0, include_vision: bool = False) -> dict`
  - `robot_sim.skills.scan_for_target(call_fn: BridgeCall, max_degrees: float = 360.0, direction: str = "left", angular_speed_deg_s: float = 75.0) -> dict`
  - `robot_sim.skills.approach_target(call_fn: BridgeCall, stop_distance: float = 0.25, max_steps: int = 65, step_dt: float = 0.18) -> dict`
  - CLI subcommands in `robot_ctl.py`: `scan`, `approach`, `reload`, plus upgraded `turn --wait` (uses `closed_loop_turn` by default unless `--open-loop` is passed) and `wait_until_settled` on `move --wait` / `wheels --wait`.

- [ ] **Step 1: Write failing tests in `tests/test_server.py`, `tests/test_skills.py`, and `tests/test_cli.py`**

1. Append `test_server_reload_preserves_target_and_rejects_invalid_model_with_400` to `tests/test_server.py` verifying `POST /reload` keeps `target_xy = [-1.3, 1.1]` when `preserve_target=True`, accepts optional `"target_xy": [1.1, -0.9]`, and returns `400` on `"preserve_target": "not_bool"` or invalid `target_xy`.
2. Create `tests/test_skills.py` running `SimulationApp` headless on an ephemeral port in a background thread (stepping physics cleanly) to test:
   - `test_closed_loop_turn_crosses_180_boundary_and_skills_validate_inputs`:
     - Reset target to `(-1.4, 1.15)`; call `closed_loop_turn(call_fn, 150.0)` then `closed_loop_turn(call_fn, 60.0)` (crossing `+180°` to `-150°`!). Assert true ground-truth `debug.robot_yaw_deg` from `app.sim.get_status()` is within `[-154.0, -146.0]` (error `< 4°` across two turns vs ~`21°` open-loop loss!) and `"debug" not in res["after_status"]`.
     - Verify `ValueError` on `closed_loop_turn(..., 0.0)`, `scan_for_target(..., max_degrees=0)`, `scan_for_target(..., direction="up")`, `approach_target(..., stop_distance=0.05)`, and `approach_target(..., stop_distance=2.0)`.
   - `test_scan_and_approach_reach_visible_target_without_debug`:
     - Reset target to `(-1.2, -1.2)` (behind/right of robot with clear corridor).
     - Call `scan_res = scan_for_target(call_fn, max_degrees=360.0, direction="right")` -> assert `scan_res["target_found"] is True` and `scan_res["after_status"]["vision"]["target_visible"] is True`.
     - Call `app_res = approach_target(call_fn, stop_distance=0.25)` -> assert `app_res["reached"] is True` and `0.15 <= app_res["after_status"]["sensors"]["rangefinders_m"]["center"] <= 0.35`.
3. Append `test_cli_scan_approach_and_reload_never_leak_debug_field` to `tests/test_cli.py` invoking `robot_ctl.main` for `reload`, `scan --max-degrees 90`, and `approach --stop-distance 0.28`, asserting exit code `0` and `"debug"` nowhere in the serialized JSON output.

- [ ] **Step 2: Run tests to verify they fail**

Run: `.venv\Scripts\python.exe -m pytest tests/test_server.py tests/test_skills.py tests/test_cli.py -v`
Expected: FAIL (`ModuleNotFoundError: No module named 'robot_sim.skills'`, `POST /reload` returns `404`).

- [ ] **Step 3: Implement `/reload` in `sim_server.py`, `robot_sim/skills.py`, and CLI subcommands in `robot_ctl.py`**

1. In `sim_server.py`:
   - Add `self._viewer_reload_pending = False` to `SimulationApp.__init__`.
   - Handle `POST /reload`: validate `preserve_target` is strictly `bool` (default `True`), call `sim.reload(preserve_target=preserve_target)` and if `target_xy is not None`, call `sim.reset(target_xy=target_xy)`; set `self._viewer_reload_pending = True`.
   - In `SimulationApp.run_loop` (GUI branch): when `self._viewer_reload_pending` is `True` outside `viewer.lock()`, clear the flag and if `viewer._get_sim() is not None`, call `viewer._get_sim().load(self.sim.model, self.sim.data, str(self.sim.model_path))` so an open 3D window seamlessly binds to the reloaded model/data.
2. Create `robot_sim/skills.py`:
   - Strip `"debug"` from every returned status dict.
   - `wait_until_settled`: poll `/status` (and `/status?vision=1` once settled if `include_vision=True`) until `not st["moving"]` and `abs(st["sensors"]["forward_speed_m_s"]) <= speed_tol` and `abs(st["sensors"]["yaw_rate_rad_s"]) <= yaw_tol` for `settle_samples` consecutive checks (cap extra settle wait at `0.6s`).
   - `closed_loop_turn`: validate `degrees != 0` and `max_speed_deg_s > 0`; integrate `accum_deg += math.degrees(yaw_rate_rad_s * dt_sim)` across short `0.08s` `/wheels` commands (using proportional angular speed `clamp(2.8 * remaining_deg, 12.0, max_speed_deg_s)` converted via `TRACK_WIDTH` / `WHEEL_RADIUS`), call `/stop` when `abs(remaining_deg) <= tol_deg` or `timeout_s` expires, then `wait_until_settled`; return `{"requested_degrees": round(degrees, 2), "measured_turn_deg": round(accum_deg, 2), "after_status": settled}`.
   - `scan_for_target`: validate `0 < max_degrees <= 720` and `direction in ("left", "right")`; spin in place with `/wheels` (`duration=0.22s`), integrating `abs(yaw_rate_rad_s * dt_sim)` while polling `/status?vision=1` every `0.05s`. Stop immediately when `vision["target_visible"]` is `True`; if `abs(offset_x) > 0.15`, apply a small centering `closed_loop_turn(-offset_x * 28.0)` so the target finishes centered; return `{"target_found": bool, "scanned_degrees": round(scanned_deg, 2), "after_status": settled}`.
   - `approach_target`: validate `0.12 <= stop_distance <= 1.5`, `max_steps > 0`, `step_dt > 0`; loop up to `max_steps` reading `/status?vision=1`:
     - Stop & return `{"reached": True, "reason": "reached_target", "steps": step, "after_status": settled}` when `target_visible and abs(offset_x) < 0.24 and rf["center"] <= stop_distance + 0.03`.
     - If `target_visible` is `False` for 4 consecutive steps, stop & return `{"reached": False, "reason": "target_lost", ...}`.
     - If a non-target obstacle blocks close ahead (`(not target_ahead and rf["center"] < 0.36)` or `rf["left"] < 0.24` or `rf["right"] < 0.24`), if `rf["center"] < 0.22` stop & return `{"reached": False, "reason": "obstacle_blocked", ...}`, else blend repulsive turn away from the closer side beam.
     - Otherwise drive smoothly toward target slowing down as `rf["center"]` approaches `stop_distance`.
3. Update `robot_ctl.py`:
   - Delegate `_wait_for_completion` to `wait_until_settled`.
   - On `turn`: add `--open-loop` flag; when `--wait` is set and not `--open-loop`, run `closed_loop_turn`.
   - Add subcommands:
     - `scan` (`--max-degrees` default `360.0`, `--direction` choices `["left", "right"]` default `"left"`, `--speed` default `75.0`)
     - `approach` (`--stop-distance` default `0.25`, `--max-steps` default `65`, `--step-dt` default `0.18`)
     - `reload` (`--no-preserve-target` flag, optional `--target-x` / `--target-y`, `--vision` flag; strips `debug`).
   - Catch `ValueError` from skills alongside `(BridgeOfflineError, BridgeRequestError)` -> print `[Lỗi điều khiển robot] ...` to `stderr` and return `1`.

- [ ] **Step 4: Run tests to verify they pass**

Run: `.venv\Scripts\python.exe -m pytest tests/test_server.py tests/test_skills.py tests/test_cli.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```powershell
git add sim_server.py robot_sim/skills.py robot_ctl.py tests/test_server.py tests/test_skills.py tests/test_cli.py
git commit -m "feat(skills): add closed-loop turn, scan, approach, settled wait, and server reload"
```

---

### Task 3: Zero-Token Background Telemetry Blackbox (`robot_sim/telemetry.py` & `robot_ctl.py telemetry`)

**Files:**
- Create: `robot_sim/telemetry.py`
- Modify: `robot_ctl.py`
- Create: `tests/test_telemetry.py`

**Interfaces:**
- Consumes: `GET /status?vision=1` on `sim_server.py` (which returns raw server payload including `debug` ground-truth).
- Produces:
  - `DEFAULT_TELEMETRY_PATH = PROJECT_ROOT / "captures" / "telemetry_latest.json"`
  - `DEFAULT_STOP_FLAG_PATH = PROJECT_ROOT / "captures" / ".telemetry_stop"`
  - `robot_sim.telemetry.record_telemetry(base_url: str, out_path: Path = DEFAULT_TELEMETRY_PATH, stop_flag_path: Path = DEFAULT_STOP_FLAG_PATH, hz: float = 10.0, max_duration_s: float = 300.0) -> dict`
  - `robot_sim.telemetry.stop_telemetry(out_path: Path = DEFAULT_TELEMETRY_PATH, stop_flag_path: Path = DEFAULT_STOP_FLAG_PATH, wait_timeout_s: float = 3.0) -> dict`
  - CLI subcommand `robot_ctl.py telemetry {record,stop}` (`--out`, `--stop-file`, `--hz`, `--max-duration`).

- [ ] **Step 1: Write failing tests in `tests/test_telemetry.py`**

Create `tests/test_telemetry.py` covering:
1. `test_telemetry_record_clears_stale_stop_flag_and_summarizes_run(tmp_path: Path)`:
   - Start headless `SimulationApp` on ephemeral port.
   - Pre-create a stale `stop_file = tmp_path / ".telemetry_stop"` (`stop_file.write_text("stale")`) to prove startup clears it!
   - Launch `record_telemetry(url, out_path=out_json, stop_flag_path=stop_file, hz=20.0, max_duration_s=10.0)` in a worker thread while driving the robot `0.25m` toward target `(0.65, 0.0)` (`target_geom` radius `0.10m`), then call `summary = stop_telemetry(out_path=out_json, stop_flag_path=stop_file)`.
   - Assert `summary["sample_count"] >= 3`, `summary["error_count"] == 0`, `summary["path_length_m"] > 0.15`, `summary["duty_cycle_pct"] > 0.0`, `"anomalies"` in summary (`"braking_rf_drop_m"`, `"max_post_stop_speed_m_s"`), `"goal_reached"` in summary, and serialized `out_json.stat().st_size < 4096`.
2. `test_telemetry_recorder_survives_offline_server_and_stop_handles_no_active_recorder(tmp_path: Path, capsys)`:
   - Run `record_telemetry("http://127.0.0.1:59998", out_path=out_json, stop_flag_path=stop_file, hz=20.0, max_duration_s=0.25)` against an offline port -> assert it returns without raising, `error_count >= 1`, and writes `out_json`.
   - Call `robot_ctl.main(["telemetry", "stop", "--out", str(tmp_path / "missing.json"), "--stop-file", str(tmp_path / ".no_recorder")])` -> assert exit code `1` and Vietnamese error on `stderr`.

- [ ] **Step 2: Run tests to verify they fail**

Run: `.venv\Scripts\python.exe -m pytest tests/test_telemetry.py -v`
Expected: FAIL (`ModuleNotFoundError: No module named 'robot_sim.telemetry'`).

- [ ] **Step 3: Implement `robot_sim/telemetry.py` and `telemetry` subcommands in `robot_ctl.py`**

1. In `robot_sim/telemetry.py`:
   - Validate `hz > 0` and `max_duration_s > 0`.
   - At the start of `record_telemetry`, unlink `stop_flag_path` if it exists (`missing_ok=True`) and record `started_epoch = time.time()`.
   - Loop until `stop_flag_path.exists()` or `wall_elapsed >= max_duration_s`: request `GET /status?vision=1` (`timeout=1.5s`), catching network/HTTP errors into `error_count` + `last_error`.
   - Compute compact summary dict (< 2 KB JSON): `started_at`, `wall_time_s`, `sim_time_s`, `realtime_ratio`, `sample_count`, `error_count`, `last_error`, `http_latency_ms` (`avg`, `p95`, `max`), `duty_cycle_pct`, `start_xy`, `end_xy`, `target_xy`, `path_length_m`, `final_rf_center_m`, `final_target_visible`, `final_target_offset_x`, `goal_reached` (`target_visible and abs(offset_x) < 0.25 and 0.15 <= rf_center <= 0.35`), `min_rangefinder_m`, `near_collision_count` (`min_rf < 0.12` while not facing target), and `anomalies` (`braking_rf_drop_m`, `max_post_stop_speed_m_s`).
   - Write `out_path` atomically via `out_path.with_suffix(".tmp")` -> `replace(out_path)`, then unlink `stop_flag_path` (`missing_ok=True`).
   - In `stop_telemetry`: record pre-existing `mtime` of `out_path` (if any), write `"stop"` to `stop_flag_path`, poll up to `wait_timeout_s` for `out_path` to appear with a newer `mtime` (and `not stop_flag_path.exists()`), then return parsed JSON; if timeout expires with no fresh file, unlink `stop_flag_path` and raise `RuntimeError` with a clear Vietnamese message.
2. In `robot_ctl.py`, add `telemetry` subparser with `record` and `stop` actions, catching `RuntimeError` / `ValueError` cleanly into exit code `1`.

- [ ] **Step 4: Run tests to verify they pass**

Run: `.venv\Scripts\python.exe -m pytest tests/test_telemetry.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```powershell
git add robot_sim/telemetry.py robot_ctl.py tests/test_telemetry.py
git commit -m "feat(telemetry): add background 10Hz blackbox recorder and CLI stop/summary"
```

---

### Task 4: Antigravity Multi-Agent R&D Workspace Skill, `AGENTS.md` & `README.md`

**Files:**
- Create: `.agents/skills/robot-multi-agent-rd/SKILL.md`
- Create: `.agents/skills/robot-multi-agent-rd/references/driver-prompt.md`
- Create: `.agents/skills/robot-multi-agent-rd/references/engineer-prompt.md`
- Create: `AGENTS.md`
- Modify: `README.md`
- Create: `tests/test_agent_skill.py`

**Interfaces:**
- Consumes: CLI commands (`status --vision`, `scan`, `approach`, `move`, `turn`, `stop`, `reset`, `reload`, `telemetry record`, `telemetry stop`) and `verify_arena_integrity` from Tasks 1–3.
- Produces: Discoverable Antigravity workspace skill `robot-multi-agent-rd` + root `AGENTS.md` rules + user documentation in `README.md`.

- [ ] **Step 1: Write failing test in `tests/test_agent_skill.py`**

Create `tests/test_agent_skill.py` checking:
```python
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_workspace_skill_and_agents_rules_follow_antigravity_conventions():
    skill_md = ROOT / ".agents" / "skills" / "robot-multi-agent-rd" / "SKILL.md"
    driver_ref = ROOT / ".agents" / "skills" / "robot-multi-agent-rd" / "references" / "driver-prompt.md"
    engineer_ref = ROOT / ".agents" / "skills" / "robot-multi-agent-rd" / "references" / "engineer-prompt.md"
    agents_md = ROOT / "AGENTS.md"

    assert skill_md.is_file()
    text = skill_md.read_text(encoding="utf-8")
    assert text.startswith("---\n")
    frontmatter = text.split("---\n", 2)[1]
    assert "name: robot-multi-agent-rd" in frontmatter
    assert "description:" in frontmatter
    assert "telemetry record" in text and "telemetry stop" in text
    assert "3" in text  # max 3 R&D rounds

    assert driver_ref.is_file() and "--debug" in driver_ref.read_text(encoding="utf-8")
    assert engineer_ref.is_file() and "pytest" in engineer_ref.read_text(encoding="utf-8")

    assert agents_md.is_file()
    agents_text = agents_md.read_text(encoding="utf-8")
    assert not agents_text.startswith("---\n")  # standalone AGENTS.md must NOT use YAML frontmatter
    assert len(agents_text.encode("utf-8")) < 4000
    assert ".venv\\Scripts\\python.exe" in agents_text
```

- [ ] **Step 2: Run test to verify it fails**

Run: `.venv\Scripts\python.exe -m pytest tests/test_agent_skill.py -v`
Expected: FAIL (`assert skill_md.is_file()` fails).

- [ ] **Step 3: Create `.agents/skills/robot-multi-agent-rd/`, `AGENTS.md`, and update `README.md`**

1. Create `.agents/skills/robot-multi-agent-rd/SKILL.md` with YAML frontmatter (`name: robot-multi-agent-rd`, third-person `description` triggering whenever the user asks to run multi-agent robot navigation, benchmark, monitor, or auto-improve the robot) and concise orchestration steps for **Main Agent**:
   - Define `robot-driver` (`EnableWriteTools: true` for CLI commands, prompt from `references/driver-prompt.md`) and `robot-engineer` (`EnableWriteTools: true`, prompt from `references/engineer-prompt.md`) via `define_subagent`.
   - Up to **3 R&D rounds**:
     1. Set challenge (`reset --target-x <X> --target-y <Y> --vision`).
     2. Start background blackbox: `.venv\Scripts\python.exe robot_ctl.py telemetry record` (`WaitMsBeforeAsync: 500`).
     3. Invoke `robot-driver` (recommend `Model: "flash"` when user wants maximum speed, or default `"inherit"`).
     4. On Driver completion, run `.venv\Scripts\python.exe robot_ctl.py telemetry stop` to read ONLY the ~1 KB scorecard JSON (never dump raw subagent transcripts into Main Agent context).
     5. Evaluate pass criteria (`goal_reached == true`, `near_collision_count == 0`, `wall_time_s <= 90`, clean sensor behavior). Stop early if passed; otherwise invoke `robot-engineer` with the 1 KB scorecard + Driver summary, verify `.venv\Scripts\python.exe -m pytest` passes, revert (`git checkout -- models robot_sim robot_ctl.py`) if a subsequent round regresses, and report a compact round-by-round comparison table to the user.
2. Create `references/driver-prompt.md` (Sim-to-Real rules: use `.venv\Scripts\python.exe robot_ctl.py` with `status --vision`, `scan`, `approach`, `move`, `turn`, `stop`; strictly forbid `--debug`, `reset`, `reload`, `demo`, `telemetry`, and reading source/telemetry files; report in `< 250` words).
3. Create `references/engineer-prompt.md` (Forensics & R&D rules: inspect `captures/telemetry_latest.json`, Driver report, `models/arena_2wheel.xml`, `robot_sim/`, `robot_ctl.py`; allowed to tune `<body name="robot">` mechanics/sensors/actuators and Python control code; forbidden from altering arena/obstacles/target; must run `.venv\Scripts\python.exe -m pytest -v` until 100% green and run `.venv\Scripts\python.exe robot_ctl.py reload` if XML or server code changed; report in `< 300` words).
4. Create concise root `AGENTS.md` (plain Markdown, no frontmatter, < 30 lines) documenting `.venv\Scripts\python.exe` preference, Sim-to-Real `--debug` boundary, arena immutability, and pointer to `robot-multi-agent-rd`.
5. Update `README.md` adding a concise Vietnamese section on `scan`, `approach`, `reload`, `telemetry`, and how to trigger the 3-role Multi-Agent R&D team in Antigravity (including a 1-line note reminding the user to restart `sim_server.py` once to enable `/reload`).

- [ ] **Step 4: Run full test suite to verify everything passes**

Run: `.venv\Scripts\python.exe -m pytest -v`
Expected: PASS (all existing + new tests green).

- [ ] **Step 5: Commit**

```powershell
git add .agents/skills/robot-multi-agent-rd AGENTS.md README.md tests/test_agent_skill.py
git commit -m "docs(agents): add multi-agent R&D workspace skill, AGENTS.md rules, and README guide"
```
