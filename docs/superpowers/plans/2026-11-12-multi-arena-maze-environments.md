# Multi-Arena XML Architecture & 3-Tier Maze Environments Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Refactor the MuJoCo 2-wheel robot simulation into a shared `models/robot_2wheel.xml` hardware include plus four locked arena XML files (`2wheel`, `maze_a`, `maze_b`, `maze_c`) with live 3D arena hot-swapping, per-arena spawn poses, dynamic rangefinder discovery on reload, and multi-arena anti-cheating guards.

**Architecture:** Extract `<body name="robot">`, `<actuator>`, and `<sensor>` into `models/robot_2wheel.xml` (`<mujocoinclude>`) included by `models/arena_2wheel.xml` and three progressive maze XMLs (`arena_maze_a.xml`, `arena_maze_b.xml`, `arena_maze_c.xml`). Centralize canonical arena specifications in `robot_sim/arenas.py` consumed by `robot_sim/integrity.py`, `robot_sim/core.py`, `sim_server.py`, and `robot_ctl.py` so arenas can be selected at startup or hot-reloaded on the live 3D viewer while keeping `robot-driver` blind to arena identity.

**Tech Stack:** Python 3.11+, MuJoCo (`mujoco`), NumPy, Pillow, `pytest`, standard library `xml.etree.ElementTree`, `http.server`, and `argparse`.

**Spec:** `docs/superpowers/specs/2026-11-12-multi-arena-maze-environments-design.md`

## Global Constraints

- Always execute Python commands and `pytest` on Windows directly via `.venv\Scripts\python.exe` (never `uv run python` in automated runs).
- Do **not** pre-implement maze-solving skills (wall-following, SLAM, side rangefinders) in `models/robot_2wheel.xml` or `robot_sim/skills.py`; preserve the current 3-ray robot and skills so the Multi-Agent R&D team solves `maze_a` autonomously.
- Strict Sim-to-Real isolation: `arena` / `active_arena` / `model` identifiers appear **only** inside `"debug"` (or when `--debug` / `?debug=1` is explicitly passed) and are never exposed to `robot-driver` or hardcoded in `robot_sim/skills.py`.
- All maze interior and perimeter walls must be `0.28m` tall (`pos z = 0.14`, `size z = 0.14`), opaque (`alpha = 1.0`), collision-enabled, and neutral gray (`rgba="0.48 0.52 0.57 1"` or `rgba="0.58 0.60 0.64 1"`) so they never trigger false color masks in `analyze_vision`.
- `<include file="robot_2wheel.xml"/>` must be confined strictly to the same directory as the loading arena XML (reject absolute paths and `..` traversal), while `verify_arena_integrity` remains compatible with standalone single-file XMLs used in unit tests.
- Maintain 100% backward compatibility with all existing CLI flags, HTTP endpoints, and all 38 existing tests on the default `models/arena_2wheel.xml` map.

## Review Focus

1. **Path traversal via `<include file="..."/>` or `POST /reload` (`{"model": "../pyproject.toml"}`):** Must raise `ValueError` / HTTP `400` without reading outside `models/` or altering the active simulation (covered in Task 1 & Task 3).
2. **Cross-arena `POST /reload` with `target_xy` overlapping a wall in the destination maze (e.g. `{"arena": "maze_a", "target_xy": [1.5, 1.0]}`):** Must fail atomically with HTTP `400` before mutating `sim.model`, `sim.data`, `sim_time`, or `_viewer_reload_pending` (covered in Task 2 & Task 3).
3. **Switching between arenas via `reload` without explicit `preserve_target` (`maze_a` → `2wheel`):** Must reset `target_xy` to the destination arena's default target (`[1.4, 0.0]`) and `robot_xy` to its `robot_spawn`, whereas same-arena `reload` preserves custom `target_xy` (covered in Task 2).
4. **Hot-reloading `robot_2wheel.xml` after adding a new `<rangefinder name="rf_side_left" cutoff="1.5"/>`:** Must dynamically populate `sensors.rangefinders_m["side_left"]` and normalize no-hit values against that sensor's own `1.5m` cutoff rather than hardcoded `3.0m` (covered in Task 2).
5. **Arena identity leakage to blind `robot-driver`:** `robot_ctl.py status --vision` and `robot_ctl.py arenas` (without `--debug`) must omit `arena`, `model`, and `active_arena` (covered in Task 3).

---

### Task 1: Shared `robot_2wheel.xml`, Arena Registry, 3 Maze XMLs & Multi-Arena Integrity Guard

**Files:**
- Create: `models/robot_2wheel.xml`
- Modify: `models/arena_2wheel.xml`
- Create: `models/arena_maze_a.xml`
- Create: `models/arena_maze_b.xml`
- Create: `models/arena_maze_c.xml`
- Create: `robot_sim/arenas.py`
- Modify: `robot_sim/integrity.py`
- Create: `tests/test_arenas.py`
- Modify: `tests/test_integrity.py`

**Interfaces:**
- Consumes: `mujoco.MjModel.from_xml_path`, `xml.etree.ElementTree`
- Produces:
  - `robot_sim/arenas.py`:
    - `MODELS_DIR: Path` (`PROJECT_ROOT / "models"`)
    - `ROBOT_INCLUDE_FILENAME = "robot_2wheel.xml"`
    - `@dataclass(frozen=True) class ArenaSpec`: `arena_id: str`, `aliases: tuple[str, ...]`, `filename: str`, `description: str`, `floor_size: tuple[float, float, float]`, `walls: dict[str, tuple[list[float], list[float]]]`, `obstacles: dict[str, tuple[list[float], str, str, list[float], list[float]]]`, `maze_walls: dict[str, tuple[list[float], list[float], list[float]]]`, `spawn_xy: tuple[float, float]`, `spawn_yaw_deg: float`, `default_target_xy: tuple[float, float]`
    - `ARENAS: dict[str, ArenaSpec]` (keys: `"2wheel"`, `"maze_a"`, `"maze_b"`, `"maze_c"`)
    - `resolve_arena_model(arena_or_path: str | Path | None = None, *, restrict_to_models_dir: bool = False) -> Path`
    - `identify_arena_id(model_path: Path | str) -> str` (returns matching `arena_id` or `"custom"`)
    - `list_arenas(include_debug: bool = False) -> list[dict]` (returns `id`, `aliases`, `filename`, `description`; adds `model_path`, `spawn_xy`, `default_target_xy` when `include_debug=True`)
  - `robot_sim/integrity.py`:
    - `verify_arena_integrity(model_path: Path | str | None = None) -> dict` returning `{"valid": True, "model_path": str(resolved), "arena": arena_id}`

- [ ] **Step 1: Write failing tests in `tests/test_arenas.py` and `tests/test_integrity.py`**

Create `tests/test_arenas.py`:
```python
from pathlib import Path
import pytest
from robot_sim.arenas import ARENAS, identify_arena_id, list_arenas, resolve_arena_model
from robot_sim.integrity import verify_arena_integrity


@pytest.mark.parametrize(
    "key,expected_id,expected_file",
    [
        (None, "2wheel", "arena_2wheel.xml"),
        ("2wheel", "2wheel", "arena_2wheel.xml"),
        ("default", "2wheel", "arena_2wheel.xml"),
        ("basic", "2wheel", "arena_2wheel.xml"),
        ("maze", "maze_a", "arena_maze_a.xml"),
        ("maze_a", "maze_a", "arena_maze_a.xml"),
        ("maze_b", "maze_b", "arena_maze_b.xml"),
        ("maze_c", "maze_c", "arena_maze_c.xml"),
    ],
)
def test_resolve_arena_aliases_and_verify_integrity(key, expected_id, expected_file):
    path = resolve_arena_model(key)
    assert path.name == expected_file
    assert identify_arena_id(path) == expected_id
    res = verify_arena_integrity(path)
    assert res["valid"] is True
    assert res["arena"] == expected_id


def test_resolve_arena_rejects_unknown_name_and_outside_models_when_restricted(tmp_path: Path):
    with pytest.raises(ValueError):
        resolve_arena_model("nonexistent_arena")

    outside = tmp_path / "arena_2wheel.xml"
    outside.write_text("<mujoco/>", encoding="utf-8")
    with pytest.raises(ValueError):
        resolve_arena_model(str(outside), restrict_to_models_dir=True)


def test_list_arenas_contains_all_four_maps():
    catalog = list_arenas()
    assert [item["id"] for item in catalog] == ["2wheel", "maze_a", "maze_b", "maze_c"]
    assert "maze" in catalog[1]["aliases"]
```

Update `tests/test_integrity.py` so `tmp_path` fixtures copy `models/robot_2wheel.xml` alongside `arena_*.xml` when testing arena/robot tampering, and add tests for:
1. `test_verify_arena_integrity_rejects_maze_wall_tampering` (shrinking or moving `maze_w02` or changing `robot_spawn` in `arena_maze_a.xml` raises `ValueError`),
2. `test_verify_arena_integrity_rejects_include_path_traversal` (`<include file="../outside.xml"/>` or `<include file="C:/outside.xml"/>` raises `ValueError`),
3. `test_verify_arena_integrity_checks_robot_extents_relative_to_off_origin_spawn` (`arena_maze_a.xml` with `robot` at `(-1.5, -1.5)` passes when valid, and raises `ValueError` when `chassis` size in `tmp_path / "robot_2wheel.xml"` is inflated to `"1.50 1.50 0.03"`).

- [ ] **Step 2: Run `pytest` to verify the new tests fail**

Run: `.venv\Scripts\python.exe -m pytest tests/test_arenas.py tests/test_integrity.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'robot_sim.arenas'`.

- [ ] **Step 3: Create `models/robot_2wheel.xml`, update `models/arena_2wheel.xml`, and create `models/arena_maze_a.xml`, `models/arena_maze_b.xml`, `models/arena_maze_c.xml`**

- Extract `<body name="robot" pos="0 0 0">...</body>` (wrapped in `<worldbody>`), `<actuator>...</actuator>`, and `<sensor>...</sensor>` verbatim from `models/arena_2wheel.xml` into `models/robot_2wheel.xml` under root `<mujocoinclude>`.
- In `models/arena_2wheel.xml`, add `<site name="robot_spawn" pos="0 0 0" euler="0 0 0" size="0.01" rgba="0 0 0 0"/>` inside `<worldbody>` and replace the extracted robot/actuator/sensor blocks with `<include file="robot_2wheel.xml"/>` before `</mujoco>`.
- Author `models/arena_maze_a.xml` (7 `maze_w01..maze_w07` geoms, `robot_spawn` at `-1.5 -1.5 0` `euler="0 0 0"`, `target` at `1.5 1.5 0.12`), `models/arena_maze_b.xml` (12 `maze_w01..maze_w12` geoms, `robot_spawn` at `0 0 0` `euler="0 0 0"`, `target` at `1.6 1.6 0.12`), and `models/arena_maze_c.xml` (11 `maze_w01..maze_w11` geoms with `maze_w03`, `maze_w07`, `maze_w09` using `rgba="0.58 0.60 0.64 1"`, `robot_spawn` at `0 0 0` `euler="0 0 -90"`, `target` at `1.6 1.6 0.12`), matching every coordinate and `overview_cam` (`pos="0 -3.3 4.2" xyaxes="1 0 0 0 0.78 0.62" fovy="52"`) in Spec §4.2.

- [ ] **Step 4: Implement `robot_sim/arenas.py` and multi-arena `robot_sim/integrity.py`**

- In `robot_sim/arenas.py`, define `ArenaSpec` and `ARENAS` with exact wall/obstacle/maze_wall/spawn/target tables from Spec §4.1–4.2. Match arena specs in `identify_arena_id` by filename (`arena_2wheel.xml` → `"2wheel"`, `arena_maze_a.xml` → `"maze_a"`, `arena_maze_b.xml` → `"maze_b"`, `arena_maze_c.xml` → `"maze_c"`, fallback `"2wheel"` when standalone XML in `tmp_path` has `obs_red` or match via XML `<mujoco model="...">` attribute: `"arena_2wheel"`, `"arena_maze_a"`, `"arena_maze_b"`, `"arena_maze_c"`!).
  *(Using `<mujoco model="arena_...">` to select the `ArenaSpec` in `verify_arena_integrity` ensures even renamed `tmp_path / "tampered.xml"` copies are validated against the exact `ArenaSpec` of the map they came from!)*
- In `robot_sim/integrity.py`:
  - Inspect root children: allow `<include file="robot_2wheel.xml"/>` only when `file` is a plain filename (`Path(inc_file).name == inc_file` and `not Path(inc_file).is_absolute()` and `".."` not in `inc_file`) existing at `resolved.parent / inc_file`; parse that included XML to locate `<body name="robot">` (while still supporting inline `<body name="robot">` if no `<include>` is used).
  - Whitelist `<worldbody>` children against the matched `ArenaSpec`: `light`, `camera`, optional `site[@name='robot_spawn']` (required on maze arenas; validated against `spec.spawn_xy` and `spec.spawn_yaw_deg`), `floor`, `spec.walls`, `spec.maze_walls`, `spec.obstacles`, `target`, and (if inline) `robot`.
  - In `_check_robot_compiled_geom_extents`, subtract `robot_origin = mj_data.xpos[robot_body_id]` from `mj_data.geom_xpos[gid]` before checking `|dx| + hx <= 0.25`, `|dy| + hy <= 0.20`, `dz - hz >= -0.002`, `dz + hz <= 0.30`.

- [ ] **Step 5: Run `pytest` to verify Task 1 passes with zero regressions**

Run: `.venv\Scripts\python.exe -m pytest tests/test_arenas.py tests/test_integrity.py tests/test_model.py -v`
Expected: PASS (all new arena & integrity tests and existing model tests green).

- [ ] **Step 6: Commit Task 1**

```powershell
git add models/ robot_sim/arenas.py robot_sim/integrity.py tests/test_arenas.py tests/test_integrity.py
git commit -m "feat(arenas): extract shared robot_2wheel.xml, add 3 maze XMLs, and lock multi-arena integrity"
```

---

### Task 2: Per-Arena Spawn Pose, Dynamic Rangefinder Discovery, Maze Geometry Verification & Smart `reload`

**Files:**
- Modify: `robot_sim/core.py`
- Modify: `robot_sim/telemetry.py`
- Modify: `tests/test_arenas.py`
- Modify: `tests/test_core.py`
- Modify: `tests/test_telemetry.py`

**Interfaces:**
- Consumes: `ArenaSpec`, `ARENAS`, `identify_arena_id`, `resolve_arena_model` from `robot_sim/arenas.py`; `verify_arena_integrity` from `robot_sim/integrity.py`.
- Produces:
  - `RobotSimulation.__init__(self, model_path: Path | str | None = None) -> None` (accepts arena alias or path via `resolve_arena_model`, applies `robot_spawn` pose)
  - `RobotSimulation.reload(self, model_path: Path | str | None = None, preserve_target: bool | None = None, target_xy: tuple[float, float] | list[float] | None = None) -> dict` (atomic: validates integrity + `target_xy` against `new_model` before mutating `self`; defaults `preserve_target` to `(resolved == self.model_path)` when `None`; applies `robot_spawn`)
  - `RobotSimulation.validate_target_xy(self, target_xy: tuple[float, float] | list[float], *, model: mujoco.MjModel | None = None) -> tuple[float, float]` (checks `[-1.7, 1.7]`, rejects `r = 0.10m` cylinder overlap with `wall_*` / `maze_w*` boxes, and rejects center-point containment inside `obs_*` boxes/cylinders)
  - `RobotSimulation.get_status(self, include_vision: bool = False) -> dict` (`sensors.rangefinders_m` & `sensors.rangefinder_hits` dynamically include every `<rangefinder>` sensor in `self.model` keyed by stripping leading `"rf_"`; `debug` includes `"arena": str` and `"model": str`)
  - `record_telemetry(...) -> dict` (includes `"arena": str | None` in summary JSON and computes `min_rangefinder_m` / `near_collision_count` across all `rf.values()`)

- [ ] **Step 1: Write failing tests in `tests/test_arenas.py`, `tests/test_core.py`, and `tests/test_telemetry.py`**

Add to `tests/test_arenas.py`:
```python
import math
from collections import deque
import numpy as np
import pytest
from robot_sim.core import RobotSimulation


@pytest.mark.parametrize(
    "arena_id,exp_spawn,exp_yaw,exp_target,pre_goal_xy,pre_goal_yaw",
    [
        ("maze_a", [-1.5, -1.5], 0.0, [1.5, 1.5], (0.5, 1.5), 0.0),
        ("maze_b", [0.0, 0.0], 0.0, [1.6, 1.6], (0.8, 1.6), 0.0),
        ("maze_c", [0.0, 0.0], -90.0, [1.6, 1.6], (0.8, 1.6), 0.0),
    ],
)
def test_maze_spawn_occludes_target_360_has_clear_corridor_and_reveals_at_final_turn(
    arena_id, exp_spawn, exp_yaw, exp_target, pre_goal_xy, pre_goal_yaw
):
    sim = RobotSimulation(model_path=arena_id)
    try:
        st = sim.get_status(include_vision=True)
        assert st["debug"]["arena"] == arena_id
        assert st["debug"]["robot_xy"] == pytest.approx(exp_spawn, abs=1e-3)
        assert st["debug"]["robot_yaw_deg"] == pytest.approx(exp_yaw, abs=0.5)
        assert st["debug"]["target_xy"] == pytest.approx(exp_target, abs=1e-3)

        # 1. Target must be 100% occluded across full 360° spin at spawn, with zero false obstacle colors
        for deg in range(0, 360, 30):
            rad = math.radians(deg)
            sim.data.qpos[0:2] = exp_spawn
            sim.data.qpos[3:7] = [math.cos(rad / 2.0), 0.0, 0.0, math.sin(rad / 2.0)]
            vis = sim.analyze_vision("front_cam")
            assert vis["target_visible"] is False, f"{arena_id} leaked target at yaw={deg}°"
            assert vis["obstacles"] == []

        # 2. Continuous 2D clearance check (robot clearance radius 0.35m => >= 0.70m corridor width)
        step = 0.10
        xs = np.round(np.arange(-1.8, 1.801, step), 3)
        ys = np.round(np.arange(-1.8, 1.801, step), 3)
        wall_boxes = []
        for gid in range(sim.model.ngeom):
            gname = sim.model.geom(gid).name
            if gname.startswith(("wall_", "maze_w")):
                cx, cy, _ = sim.model.geom_pos[gid]
                sx, sy, _ = sim.model.geom_size[gid]
                wall_boxes.append((float(cx), float(cy), float(sx), float(sy)))

        def is_clear(px: float, py: float, radius: float = 0.35) -> bool:
            for cx, cy, sx, sy in wall_boxes:
                dx = max(abs(px - cx) - sx, 0.0)
                dy = max(abs(py - cy) - sy, 0.0)
                if math.hypot(dx, dy) < radius - 1e-6:
                    return False
            return True

        start_idx = (int(round((exp_spawn[0] - xs[0]) / step)), int(round((exp_spawn[1] - ys[0]) / step)))
        goal_idx = (int(round((exp_target[0] - xs[0]) / step)), int(round((exp_target[1] - ys[0]) / step)))
        assert is_clear(xs[start_idx[0]], ys[start_idx[1]])
        assert is_clear(xs[goal_idx[0]], ys[goal_idx[1]])

        visited = {start_idx}
        q = deque([start_idx])
        while q:
            ix, iy = q.popleft()
            if (ix, iy) == goal_idx:
                break
            for nix, niy in ((ix + 1, iy), (ix - 1, iy), (ix, iy + 1), (ix, iy - 1)):
                if 0 <= nix < len(xs) and 0 <= niy < len(ys) and (nix, niy) not in visited:
                    if is_clear(float(xs[nix]), float(ys[niy])):
                        visited.add((nix, niy))
                        q.append((nix, niy))
        assert goal_idx in visited, f"{arena_id} lacks a >=0.70m wide continuous path from spawn to target"

        # 3. At the final turn cell facing East toward target, target becomes clearly visible
        rad_final = math.radians(pre_goal_yaw)
        sim.data.qpos[0:2] = list(pre_goal_xy)
        sim.data.qpos[3:7] = [math.cos(rad_final / 2.0), 0.0, 0.0, math.sin(rad_final / 2.0)]
        vis_final = sim.analyze_vision("front_cam")
        assert vis_final["target_visible"] is True
    finally:
        sim.close()
```

Add to `tests/test_core.py`:
1. `test_reload_switches_arena_spawn_and_resets_target_unless_same_arena`:
   - Start on `2wheel`, `sim.reset(target_xy=(-1.2, 0.9))` → `sim.reload()` keeps `[-1.2, 0.9]`.
   - `sim.reload(model_path="maze_a")` switches to `maze_a` with `robot_xy == [-1.5, -1.5]` and `target_xy == [1.5, 1.5]`.
   - `sim.reload(model_path="2wheel")` switches back to `2wheel` with `robot_xy == [0.0, 0.0]` and `target_xy == [1.4, 0.0]`.
2. `test_validate_target_xy_rejects_maze_wall_and_obstacle_center_overlap_while_preserving_near_obstacle_cli_coord`:
   - On `2wheel`: `sim.validate_target_xy((1.0, -0.8))` succeeds (`(1.0, -0.8)` compatibility); `sim.validate_target_xy((0.85, -0.75))` (inside `obs_blue`) raises `ValueError`.
   - On `maze_a`: `sim.validate_target_xy((1.5, 1.0))` (on `maze_w02`) raises `ValueError`; `sim.reload(model_path="maze_a", target_xy=(1.5, 1.0))` raises `ValueError` while leaving previous arena state untouched.
3. `test_reload_discovers_extra_rangefinder_and_respects_per_sensor_cutoff(tmp_path)`:
   - Copy `models/arena_2wheel.xml` and `models/robot_2wheel.xml` to `tmp_path`; inject `<site name="site_rf_side_left" pos="0 0.09 0.075" zaxis="0 1 0" size="0.008"/>` inside `robot` and `<rangefinder name="rf_side_left" site="site_rf_side_left" cutoff="1.5"/>` inside `<sensor>` of `tmp_path / "robot_2wheel.xml"`.
   - `sim.reload(model_path=tmp_path / "arena_2wheel.xml")` succeeds, `st["sensors"]["rangefinders_m"]["side_left"] == 1.5` and `st["sensors"]["rangefinder_hits"]["side_left"] is False` (since `wall_north` is `2.0m` away `> 1.5m` cutoff!).

Update `tests/test_telemetry.py` to assert `"arena"` is recorded in `telemetry` summary JSON.

- [ ] **Step 2: Run `pytest` to verify the new Task 2 tests fail**

Run: `.venv\Scripts\python.exe -m pytest tests/test_arenas.py tests/test_core.py tests/test_telemetry.py -v`
Expected: FAIL (`RobotSimulation` does not yet resolve arena names, apply `robot_spawn`, discover extra rangefinders, or check maze walls in `validate_target_xy`).

- [ ] **Step 3: Implement spawn pose, dynamic rangefinder discovery, wall-aware `validate_target_xy`, and atomic `reload` in `robot_sim/core.py` & `robot_sim/telemetry.py`**

- In `robot_sim/core.py`:
  - Helper `_discover_sensors(model: mujoco.MjModel) -> tuple[dict[str, int], dict[str, tuple[int, float]]]`:
    - Locate fixed sensors (`left_wheel_vel`, `right_wheel_vel`, `chassis_vel`, `chassis_gyro`).
    - Iterate `sid in range(model.nsensor)`: whenever `int(model.sensor_type[sid]) == int(mujoco.mjtSensor.mjSENS_RANGEFINDER)`, extract `sname`, map `key = sname[3:] if sname.startswith("rf_") else sname`, `cutoff = float(model.sensor_cutoff[sid]) if float(model.sensor_cutoff[sid]) > 0 else RANGEFINDER_MAX`, and store `(int(model.sensor_adr[sid]), cutoff)`.
  - Helper `_read_spawn_pose(model: mujoco.MjModel) -> tuple[float, float, float]`:
    - Look up `site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "robot_spawn")`; if `< 0`, return `(0.0, 0.0, 0.0)`. Otherwise read `pos = model.site_pos[site_id]` and convert `quat = model.site_quat[site_id]` (`w, x, y, z`) to `yaw_rad = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))`.
  - Helper `_apply_spawn_pose(model: mujoco.MjModel, data: mujoco.MjData) -> None`:
    - Set `sx, sy, syaw = _read_spawn_pose(model)`, `data.qpos[0] = sx`, `data.qpos[1] = sy`, `data.qpos[3:7] = [cos(syaw / 2.0), 0.0, 0.0, sin(syaw / 2.0)]`.
  - In `validate_target_xy(self, target_xy, *, model: mujoco.MjModel | None = None) -> tuple[float, float]`:
    - Keep `[-1.7, 1.7]` check; against `target_model = model if model is not None else self.model`, reject `r = 0.10m` circle-vs-AABB overlap on any geom named `wall_*` or `maze_w*`, and reject center-point `(tx, ty)` containment inside any body `obs_*`'s child geom (`box` or `cylinder`).
  - In `reload(self, model_path=None, preserve_target=None, target_xy=None)`:
    - Resolve `resolved = resolve_arena_model(model_path) if model_path is not None else self.model_path`.
    - Validate `preserve_target` is `bool | None`, run `verify_arena_integrity(resolved)`, compile `new_model = mujoco.MjModel.from_xml_path(str(resolved))`, and if `target_xy is not None`, call `tx, ty = self.validate_target_xy(target_xy, model=new_model)` **before** closing renderer or mutating `self.*`.
    - Determine `keep_prev = (resolved == self.model_path) if preserve_target is None else preserve_target`.
    - Apply `_apply_spawn_pose(new_model, new_data)`, set `new_data.mocap_pos[0]` from `target_xy` (if given), else `prev_target_pos` (if `keep_prev`), else `new_default_target_pos`, and call `mujoco.mj_forward(new_model, new_data)`.
- In `robot_sim/telemetry.py`:
  - Track `arena_id: str | None = None` from `dbg.get("arena")`, compute `cur_min_rf = min((float(v) for v in rf.values()), default=3.0)`, and include `"arena": arena_id` in `summary`.

- [ ] **Step 4: Run `pytest` to verify Task 2 passes**

Run: `.venv\Scripts\python.exe -m pytest -v`
Expected: PASS (all Task 1, Task 2, and existing tests pass).

- [ ] **Step 5: Commit Task 2**

```powershell
git add robot_sim/core.py robot_sim/telemetry.py tests/test_arenas.py tests/test_core.py tests/test_telemetry.py
git commit -m "feat(core,telemetry): add arena spawn pose, dynamic rangefinder reload, and maze validation"
```

---

### Task 3: HTTP Server, CLI Arena Commands & Multi-Agent R&D Documentation

**Files:**
- Modify: `sim_server.py`
- Modify: `robot_ctl.py`
- Modify: `AGENTS.md`
- Modify: `.agents/skills/robot-multi-agent-rd/SKILL.md`
- Modify: `.agents/skills/robot-multi-agent-rd/references/driver-prompt.md`
- Modify: `.agents/skills/robot-multi-agent-rd/references/engineer-prompt.md`
- Modify: `README.md`
- Modify: `tests/test_server.py`
- Modify: `tests/test_cli.py`
- Modify: `tests/test_agent_skill.py`

**Interfaces:**
- Consumes: `list_arenas`, `identify_arena_id`, `resolve_arena_model` from `robot_sim/arenas.py`; `RobotSimulation.reload` from `robot_sim/core.py`.
- Produces:
  - `sim_server.py`:
    - CLI flags `--arena` (default `None`) and `--model` (default `None`, mutually exclusive)
    - `GET /arenas` (optional `?debug=1`: returns `{"arenas": list_arenas(include_debug=debug)}` plus `"active_arena"` & `"active_model"` only when `debug` is truthy)
    - `POST /reload` accepting optional `"arena"`, `"model"` (mutually exclusive; resolved with `restrict_to_models_dir=True`), `"preserve_target"` (`bool | None`, default `None`), and `"target_xy"`
  - `robot_ctl.py`:
    - `robot_ctl.py arenas [--debug]`
    - `robot_ctl.py reload [--arena <name> | --model <path>] [--no-preserve-target] [--target-x X --target-y Y] [--vision]`

- [ ] **Step 1: Write failing tests in `tests/test_server.py`, `tests/test_cli.py`, and `tests/test_agent_skill.py`**

Add to `tests/test_server.py`:
1. `test_server_arenas_endpoint_and_hot_reload_between_arenas`:
   - `GET /arenas` omits `"active_arena"`; `GET /arenas?debug=1` returns `"active_arena": "2wheel"`.
   - `POST /reload {"arena": "maze"}` returns `200`, sets `_viewer_reload_pending = True`, `robot_xy == [-1.5, -1.5]`, `target_xy == [1.5, 1.5]`, and `GET /arenas?debug=1` reports `"active_arena": "maze_a"`.
2. `test_server_reload_rejects_path_traversal_and_cross_arena_wall_target_atomically`:
   - From `2wheel` after moving forward (`robot_xy[0] > 0.04`), `POST /reload {"model": "../pyproject.toml"}`, `POST /reload {"arena": "unknown"}`, `POST /reload {"arena": "maze_a", "model": "models/arena_2wheel.xml"}`, and `POST /reload {"arena": "maze_a", "target_xy": [1.5, 1.0]}` all return `400` while `active_arena` remains `"2wheel"`, `_viewer_reload_pending` stays `False`, and `robot_xy` / `sim_time` are preserved untouched.

Add to `tests/test_cli.py`:
1. `test_cli_arenas_and_status_hide_active_arena_without_debug_and_reload_switches_arena`:
   - `robot_ctl.py arenas` returns 4 arenas with `"active_arena" not in out`.
   - `robot_ctl.py reload --arena maze_a` returns `0` with `"debug" not in out`.
   - `robot_ctl.py status --vision` has `"debug" not in out` and `"arena" not in json.dumps(out)`.
   - `robot_ctl.py arenas --debug` reports `"active_arena": "maze_a"`.

Update `tests/test_agent_skill.py`:
- Assert `AGENTS.md`, `SKILL.md`, and `engineer-prompt.md` reference `models/robot_2wheel.xml` and `--arena` while keeping `AGENTS.md` `< 4000` bytes without YAML frontmatter.

- [ ] **Step 2: Run `pytest` to verify the new Task 3 tests fail**

Run: `.venv\Scripts\python.exe -m pytest tests/test_server.py tests/test_cli.py tests/test_agent_skill.py -v`
Expected: FAIL (`GET /arenas` returns `404` and `robot_ctl.py` lacks `arenas` / `reload --arena`).

- [ ] **Step 3: Implement `sim_server.py`, `robot_ctl.py`, and workspace skill/docs updates**

- In `sim_server.py`:
  - Add `GET /arenas`: parse `debug` query param (`"1"`, `"true"`, `"yes"`); return `{"arenas": list_arenas(include_debug=debug)}` and, only when `debug` is truthy, add `"active_arena": identify_arena_id(sim.model_path)` and `"active_model": sim.model_path.name`.
  - In `POST /reload`: reject if both `"arena"` and `"model"` are non-null (`ValueError`); validate strings if provided; resolve `new_model_path = resolve_arena_model(arena or model, restrict_to_models_dir=True)` when either is provided, else `None`; pass `preserve_target = body.get("preserve_target")` (default `None`) and `target_xy = body.get("target_xy")` into `sim.reload(model_path=new_model_path, preserve_target=preserve_target, target_xy=target_xy)`; set `app._viewer_reload_pending = True` only after `sim.reload(...)` succeeds.
  - In `main()`: add mutually exclusive `--arena` and `--model`, resolve via `resolve_arena_model(args.arena or args.model)`, pass to `SimulationApp(model_path=...)`.
- In `robot_ctl.py`:
  - Add `arenas` subparser with `--debug`.
  - On `reload` subparser, add mutually exclusive group `--arena` / `--model`; send `preserve_target = False if args.no_preserve_target else None` (so omitting `--no-preserve-target` preserves target on same-arena reload and resets to default target when switching arenas!), plus `"arena": args.arena`, `"model": args.model`, `"target_xy": target_xy`.
- Update `AGENTS.md`, `.agents/skills/robot-multi-agent-rd/SKILL.md`, `.agents/skills/robot-multi-agent-rd/references/driver-prompt.md`, `.agents/skills/robot-multi-agent-rd/references/engineer-prompt.md`, and `README.md`:
  - Document `models/robot_2wheel.xml` as the shared editable robot hardware file for `robot-engineer` (within Sim-to-Real chassis bounds) and all `models/arena_*.xml` (`2wheel`, `maze_a`, `maze_b`, `maze_c`) as strictly immutable environment files.
  - Document `robot_ctl.py reload --arena maze_a` (and `arenas`), prohibit hardcoding map/junction coordinates in `robot_sim/skills.py`, and recommend `--max-duration 360` (and relaxing `wall_time_s` threshold to `<= 180s` for multi-corridor mazes) in `SKILL.md`.

- [ ] **Step 4: Run the full test suite to verify 100% pass**

Run: `.venv\Scripts\python.exe -m pytest -v`
Expected: PASS (all existing + new tests pass with zero warnings/errors).

- [ ] **Step 5: Commit Task 3**

```powershell
git add sim_server.py robot_ctl.py AGENTS.md README.md .agents/skills/robot-multi-agent-rd/ tests/test_server.py tests/test_cli.py tests/test_agent_skill.py
git commit -m "feat(server,cli,docs): add live multi-arena switching, arenas endpoint, and maze R&D docs"
```
