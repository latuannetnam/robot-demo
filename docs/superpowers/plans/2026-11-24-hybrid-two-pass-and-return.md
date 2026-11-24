# Hybrid-Brain Two-Pass Speed-Run & Return-to-Home Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Enable Mode C (`hybrid-brain`) to record `kind: "goal"` nodes upon docking, execute multi-hop System 2 routes (`go --route`) and exact node arrivals (`--stop-at-node`), preserve learned maps across resets (`reset --keep-topo` & Studio UI), and grade both Return-to-Home (Direction A) and Two-Pass Micromouse Speed-Run (Direction B) runs cleanly.

**Architecture:** `TopologicalOdometer` preserves corridor approach headings across `target_ready` -> `dock` to anchor a bidirectional `"goal"` node and supports zeroing visit counters across resets (`reset_for_repass`). `go_to_decision` executes System 2's explicit cardinal sequence (`route`) at 20 Hz with event-driven early abort, intermediate waypoint anti-overshoot braking, and onboard-odometry stopping (`arrived_node`), while `telemetry.py` tracks outbound-to-return phase transitions without false redundant-revisit penalties.

**Tech Stack:** Python 3.12+/3.13, MuJoCo (`>=4.0.0`), MuJoCo Studio (`studio/`), `pytest`. Always invoke CLI/tests with `.\.venv\Scripts\python.exe`.

**Spec:** `docs/superpowers/specs/2026-11-24-hybrid-two-pass-and-return-design.md`

## Global Constraints

- **CLI & Test Runner on Windows:** Always run `.\.venv\Scripts\python.exe` (never `uv run python`).
- **Strict Sim-to-Real Fairness:** Zero BFS/Dijkstra/A* pathfinding inside `robot_sim/topo.py` or `robot_sim/skills.py` — System 2 supplies `--route` explicitly; never read `debug.robot_xy`, `debug.target_xy`, or `/arenas` inside `skills.py`, `topo.py`, or `robot-driver-hybrid`.
- **Hardware & Arena Immutability:** Never touch `models/robot_2wheel.xml` or any `models/arena_*.xml`.
- **100% Backward Compatibility:** Default `robot_ctl.py reset` (without `--keep-topo`), `read_topo_studio_summary` (3-tuple), existing `go` / `dock` / `telemetry` keys, Mode A, Mode B, and all existing tests must remain unchanged.

## Review Focus

1. **Oblique visual-servoing dock yaw & open-space cancelled `pending_leg`:** When `dock` finishes at an oblique angle (e.g. `+28°`) after `target_ready` at `0°`, `record_dock` must use the `target_ready` corridor `approach_yaw_deg` (`p_in = "W"`); on `open_space` where `turn`/`move` cleared `pending_leg`, `record_dock` must still connect `Node #0 <-> Node #G` rather than creating an isolated goal node (tested in Task 1).
2. **`reset --keep-topo` with relocated target (`--target-x/--target-y`) & `_check_auto_reset` safety:** Preserving a map when `sim_time` drops to `0.0` must not trigger `_check_auto_reset` wipe on the next command, and passing `target_changed=True` must downgrade stale `"goal"` nodes back to `"junction"`/`"dead_end"` (tested in Task 1).
3. **Intermediate T-junction passthrough, blocked next hop, & green-pillar pull-back during `--stop-at-node 0`:** `go --route` must brake at `next_node_xy` when turning (`C_{idx+1} != C_idx`), stop with `event == "route_blocked"` if the next cardinal is walled off, and ignore `target_ready` when returning to a non-goal node (`stop_at_node=0`) right in front of the green target (tested in Task 2).
4. **False `redundant_revisit_count` on Return-to-Home & premature `returned_to_start` while still moving:** `telemetry.py` must clear directed-cell visit history once the robot departs a docked goal (`return_phase_started`) and require `not moving` within `<= 0.35 m` of `start_xy` before setting `returned_to_start = True` (tested in Task 3).
5. **Studio UI `Reset (Keep Topo)` vs `Reset Robot Spawn` & `ArenaSelectEvent`:** `ResetRobotEvent(keep_topo=True)` preserves nodes + `topo_goal_node` with zeroed visits, whereas `ResetRobotEvent(keep_topo=False)` and `ArenaSelectEvent` wipe `.topo_state.json` and reset `topo_goal_node = -1` (tested in Task 4).

---

### Task 1: Goal Node Anchoring (`record_dock`) & Repass Reset (`reset_for_repass`)

**Files:**
- Modify: `robot_sim/topo.py`
- Modify: `robot_sim/skills.py:3099-3265` (`dock_target` adds `"moved_m"`)
- Test: `tests/test_hybrid_brain.py`

**Interfaces:**
- Produces in `robot_sim/topo.py`:
  - `read_topo_studio_details(base_url: str = "http://127.0.0.1:8765", state_path: Path | str = DEFAULT_TOPO_STATE_PATH) -> tuple[int, int, int, int]` returning `(node_count, cur_id, open_exits, goal_id)` (`goal_id = -1` if none); keeps `read_topo_studio_summary` returning `(node_count, cur_id, open_exits)`.
  - `TopologicalOdometer.finish_go(...)`: when `event == "target_ready"`, initializes `pending_leg` from `current_node` if `None`, records `pending_leg["approach_yaw_deg"] = round(yaw_deg, 3)`, and preserves `pending_leg` unless `matched` is already a `"goal"` node.
  - `TopologicalOdometer.record_dock(self, moved_m: float, status: dict, *, reached: bool) -> dict`
  - `TopologicalOdometer.reset_for_repass(self, status: dict | None = None, *, target_changed: bool = False) -> dict`
- Produces in `robot_sim/skills.py`:
  - `dock_target(...) -> dict` includes `"moved_m": round(max(0.0, end_odom - start_odom), 4)` on all return paths (`0.0` on immediate refusal).

- [ ] **Step 1: Write failing unit tests in `tests/test_hybrid_brain.py`**

Add tests covering:
- `test_topo_record_dock_uses_approach_yaw_and_connects_open_space_without_pending_leg`:
  1. Corridor `Node #0` -> `finish_go("target_ready", 1.2, status_yaw_0, in_corridor=True)` -> `record_dock(0.45, status_yaw_28_at_1_65, reached=True)` creates `Node #1` with `kind == "goal"`, `current_node == 1`, `nodes[0]["exits"]["E"]["to_node"] == 1`, `nodes[1]["exits"]["W"]["to_node"] == 0`, `dist_m == pytest.approx(1.65, abs=0.01)`, and no `"open"` exit into the target (`rf_center = 0.27`).
  2. Open-space `Node #0` (`kind == "open_space"`) -> `cancel_pending_leg()` -> `record_dock(0.35, status_at_2_8_0_1, reached=True)` creates `Node #1` (`kind == "goal"`) connected bidirectionally to `Node #0`.
  3. `record_dock(..., reached=False)` leaves `pending_leg` intact and creates no `"goal"` node.
- `test_topo_reset_for_repass_preserves_map_zeros_visits_and_handles_target_change`:
  1. Build graph `Node #0 <-> Node #1 (junction) <-> Node #2 (goal)` with `out_visits >= 1` at `sim_time = 45.0, odom_dist = 8.5`.
  2. Call `reset_for_repass(_mk_status(sim_time=0.0, x=0.0, y=0.0, odom_dist=0.0, yaw_deg=0.0))` -> `current_node == 0`, all `out_visits == 0` and `in_visits == 0`, `nodes[2]["kind"] == "goal"`, and a subsequent `observe_status` at `sim_time = 0.1` does NOT wipe `nodes`.
  3. Calling `reset_for_repass(..., target_changed=True)` downgrades `nodes[2]["kind"]` from `"goal"` to `"dead_end"` (1 exit) and `read_topo_studio_details` reports `goal_id == -1` (vs `2` before downgrade).

- [ ] **Step 2: Run tests to verify failure**

Run: `.\.venv\Scripts\python.exe -m pytest tests/test_hybrid_brain.py -k "record_dock or reset_for_repass" -v`
Expected: FAIL (`AttributeError` / `ImportError`).

- [ ] **Step 3: Implement `record_dock`, `reset_for_repass`, `read_topo_studio_details`, and `dock_target["moved_m"]`**

Follow Spec §5.1 & §5.5 in `robot_sim/topo.py` and track `odom0 = float(st["sensors"].get("odom_dist_m", 0.0))` in `robot_sim/skills.py:dock_target` so every returned dict includes `"moved_m"`.

- [ ] **Step 4: Run tests to verify pass**

Run: `.\.venv\Scripts\python.exe -m pytest tests/test_hybrid_brain.py -v`
Expected: PASS (all existing + 2 new tests).

- [ ] **Step 5: Commit**

```powershell
git add robot_sim/topo.py robot_sim/skills.py tests/test_hybrid_brain.py
git commit -m "feat(topo): add goal node anchoring on dock and map-preserving repass reset"
```

---

### Task 2: Multi-Hop `go --route`, `--stop-at-node`, & CLI Integration

**Files:**
- Modify: `robot_sim/skills.py:2403-2800` (`advance_segment`), `robot_sim/skills.py:3290-3469` (`go_to_decision`)
- Modify: `robot_ctl.py` (`go --route`, `go --stop-at-node`, `dock` -> `record_dock`, `reset --keep-topo`)
- Test: `tests/test_hybrid_brain.py`, `tests/test_cli.py`

**Interfaces:**
- Consumes from Task 1: `TopologicalOdometer.record_dock`, `TopologicalOdometer.reset_for_repass`, `dock_target["moved_m"]`.
- Produces in `robot_sim/skills.py`:
  - `advance_segment(..., *, _out_meta: dict | None = None, _target_odom_xy: tuple[float, float] | None = None, _ignore_target_ready: bool = False) -> dict`
  - `go_to_decision(..., *, route: list[str] | tuple[str, ...] | str | None = None, stop_at_node: int | None = None) -> dict` returning `"route_progress": {"requested": list[str], "completed": int, "remaining": list[str], "stop_at_node": int | None}` whenever `route is not None or stop_at_node is not None`, plus events `"arrived_node"` and `"route_blocked"`.
- Produces in `robot_ctl.py`:
  - `robot_ctl.py go [--route E,N,W] [--stop-at-node INT]`
  - `robot_ctl.py reset [--keep-topo]`
  - `robot_ctl.py dock` updates `.topo_state.json` via `record_dock` and attaches `output["topo"]` when an active topo session exists.

- [ ] **Step 1: Write failing unit & CLI tests in `tests/test_hybrid_brain.py` and `tests/test_cli.py`**

Add tests covering:
- `test_go_to_decision_route_validation_anti_overshoot_blocked_hop_and_stop_at_node`:
  1. Rejects simultaneous `cardinal="E"` + `route="E,N"`, empty/invalid `route="E,X"`, `stop_at_node=-1`, `stop_at_node=True`, or out-of-range `stop_at_node=99`.
  2. Multi-hop `route="E,N"` across a T-junction whose side wall is short still brakes at `Node #1.xy` (`next_node_xy` anti-overshoot) and completes both hops in one call (`route_progress == {"requested": ["E", "N"], "completed": 2, "remaining": [], "stop_at_node": None}`).
  3. If hop 2 (`"N"`) is physically blocked (`rf_center = 0.30`), stops immediately with `event == "route_blocked"` and `route_progress["remaining"] == ["N"]`.
  4. Starting directly in front of a visible green target (`target_visible=True`, `rf_center=0.27`) at `Node #1 (goal)` and calling `go_to_decision(..., route=["W"], stop_at_node=0)` ignores `target_ready`, drives back to `(0.0, 0.0)`, and stops with `event == "arrived_node"` within `<= 0.22 m` of `Node #0`.
  5. Open-space `Node #0 (open_space)` at `(0, 0)` with robot at `(1.2, 0.4)`: calling `go_to_decision(..., stop_at_node=0)` auto-aims toward `(0, 0)` and returns `event == "arrived_node"`.
- `test_cli_go_route_dock_goal_and_reset_keep_topo` in `tests/test_cli.py`:
  Verifies `robot_ctl.py dock` attaches `topo` with `kind == "goal"`, `robot_ctl.py reset --keep-topo` preserves `nodes` with `visits == 0`, default `robot_ctl.py reset` deletes `.topo_state.json`, and `robot_ctl.py go --route E,N --stop-at-node 0` parses and forwards arguments cleanly.

- [ ] **Step 2: Run tests to verify failure**

Run: `.\.venv\Scripts\python.exe -m pytest tests/test_hybrid_brain.py tests/test_cli.py -k "route or keep_topo" -v`
Expected: FAIL.

- [ ] **Step 3: Implement `route` & `stop_at_node` in `robot_sim/skills.py` and wire CLI flags in `robot_ctl.py`**

Implement Spec §5.2:
- In `advance_segment`: skip `_is_target_ready_for_dock` checks when `_ignore_target_ready=True`; when `_target_odom_xy=(tx, ty)` is set, compute `d_goal = math.hypot(odom_x - tx, odom_y - ty)` on each loop tick (`moved >= 0.12` or initial `d_goal <= 0.22`) and return `_finish_result("arrived_node", st, settle=True)` when `d_goal <= 0.18` or (`d_goal <= 0.32` and `d_goal > prev_d_goal + 0.015`).
- In `go_to_decision`: support `route` (`1..16` cardinals; total default budget `6.0 * len(steps)` when `max_dist == 6.0` and `len(steps) > 1`, capped at `<= 6.0` per hop), intermediate `next_node_xy` braking when `C_{idx+1} != C_idx` on an `"explored"` port, blocked-hop detection (`"route_blocked"`), open-space auto-heading to `stop_at_node`, and longitudinal fine-trim (`<= 0.22 m`) if stopping `0.20..0.42 m` short of `stop_at_node`.
- In `robot_ctl.py`: add `--route` and `--stop-at-node` to `go`, `--keep-topo` to `reset`, and `record_dock` hook to `dock`.

- [ ] **Step 4: Run tests to verify pass**

Run: `.\.venv\Scripts\python.exe -m pytest tests/test_hybrid_brain.py tests/test_cli.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```powershell
git add robot_sim/skills.py robot_ctl.py tests/test_hybrid_brain.py tests/test_cli.py
git commit -m "feat(skills,cli): add go --route, --stop-at-node, dock topo anchoring, and reset --keep-topo"
```

---

### Task 3: Two-Phase Outbound / Return Blackbox Metrics in `telemetry.py`

**Files:**
- Modify: `robot_sim/telemetry.py`
- Test: `tests/test_telemetry.py`

**Interfaces:**
- Produces in `robot_sim/telemetry.py` (`record_telemetry` summary dict):
  - Preserves all existing keys unchanged (`goal_reached`, `path_efficiency`, `redundant_revisit_count`, ...).
  - Adds: `goal_docked: bool`, `start_to_end_m: float | None`, `outbound_path_length_m: float | None`, `return_path_length_m: float | None`, `returned_to_start: bool`, `return_efficiency: float | None`.

- [ ] **Step 1: Write failing telemetry tests in `tests/test_telemetry.py`**

Add `test_telemetry_round_trip_return_to_start_and_moving_guard`:
1. Simulate an outbound trace on `maze_b` (`start_xy = [0.0, 0.0]`, `reference_path_m = 6.5`) that explores a dead-end (`backtrack`), docks cleanly (`moving=False`, `target_visible=True`, `offset_x=0.02`, `rf_center=0.27`), then drives back through the same hub in the same direction as the earlier dead-end backtrack and stops at `[0.08, -0.05]` (`moving=False`, `target_visible=False`):
   - Assert `goal_reached is False` (ended at start, not target), `goal_docked is True`, `returned_to_start is True`, `start_to_end_m <= 0.15`, `outbound_path_length_m > 6.5`, `return_path_length_m > 0.4`, `return_efficiency > 0.80`, and `redundant_revisit_count == 0` (no false penalty on return phase).
2. Verify that if the robot passes within `0.15 m` of `start_xy` while still `moving=True` at the final sample, `returned_to_start is False`.

- [ ] **Step 2: Run test to verify failure**

Run: `.\.venv\Scripts\python.exe -m pytest tests/test_telemetry.py -k "round_trip" -v`
Expected: FAIL (`KeyError: 'goal_docked'`).

- [ ] **Step 3: Implement outbound/return phase tracking in `robot_sim/telemetry.py`**

Follow Spec §5.3 (`docked_now`, `goal_docked`, `outbound_path_length_m`, clearing `cell_last_path` & `dir_cell_last_path` when `return_phase_started` triggers after `path_length_m - outbound_path_length_m >= 0.15`, resetting phase state on `reset_jumped`, and computing `returned_to_start` + `return_efficiency`).

- [ ] **Step 4: Run tests to verify pass**

Run: `.\.venv\Scripts\python.exe -m pytest tests/test_telemetry.py -v`
Expected: PASS (all existing + new telemetry tests).

- [ ] **Step 5: Commit**

```powershell
git add robot_sim/telemetry.py tests/test_telemetry.py
git commit -m "feat(telemetry): track round-trip goal_docked, return_efficiency, and phase-clean revisits"
```

---

### Task 4: MuJoCo Studio `Goal: #<id>` HUD & `Reset (Keep Topo)` Button

**Files:**
- Modify: `studio/messages.py`
- Modify: `studio/sim_plugin.py`
- Modify: `studio/viewer_plugin.py`
- Test: `tests/test_hybrid_brain.py`, `tests/test_agent_skill.py`

**Interfaces:**
- Consumes from Task 1: `read_topo_studio_details`, `TopologicalOdometer.reset_for_repass`, `TopologicalOdometer.reset_session`.
- Produces:
  - `studio/messages.py`: `ResetRobotEvent(keep_topo: bool = False)`, `RobotTelemetry.topo_goal_node: int = -1`.
  - `studio/sim_plugin.py`: `_poll_topo_summary() -> tuple[int, int, int, int]` using `read_topo_studio_details`; `on_reset_robot` + `on_arena_select` + `on_post_model` honoring `keep_topo` vs full session reset.
  - `studio/viewer_plugin.py`: `Reset (Keep Topo)` button beside `Reset Robot Spawn`, and ` | Goal: #{t.topo_goal_node}` suffix on `Topo Graph:` line when `t.topo_goal_node >= 0`.

- [ ] **Step 1: Write failing Studio tests in `tests/test_hybrid_brain.py` and `tests/test_agent_skill.py`**

Update/add tests verifying:
- `ResetRobotEvent()` defaults to `keep_topo is False` and `ResetRobotEvent(keep_topo=True)` sets `keep_topo is True`; `RobotTelemetry().topo_goal_node == -1`.
- `SimPlugin` with a populated `.topo_state.json` containing `Node #2 (kind="goal", out_visits=1)`:
  - `_poll_topo_summary()` returns `(3, ..., ..., 2)`.
  - Handling `ResetRobotEvent(keep_topo=True)` preserves all 3 nodes + `goal_id == 2` and resets `visits == 0`.
  - Handling `ResetRobotEvent(keep_topo=False)` deletes `.topo_state.json` and resets summary to `(0, -1, 0, -1)`.

- [ ] **Step 2: Run tests to verify failure**

Run: `.\.venv\Scripts\python.exe -m pytest tests/test_hybrid_brain.py tests/test_agent_skill.py -k "studio" -v`
Expected: FAIL.

- [ ] **Step 3: Implement Studio messages, sim_plugin reset handling, and viewer_plugin UI**

Follow Spec §5.5 across `studio/messages.py`, `studio/sim_plugin.py`, and `studio/viewer_plugin.py`. Ensure balanced `imgui.Begin`/`imgui.End` and `##` widget ID safety.

- [ ] **Step 4: Run tests to verify pass**

Run: `.\.venv\Scripts\python.exe -m pytest tests/test_hybrid_brain.py tests/test_agent_skill.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```powershell
git add studio/messages.py studio/sim_plugin.py studio/viewer_plugin.py tests/test_hybrid_brain.py tests/test_agent_skill.py
git commit -m "feat(studio): add Reset (Keep Topo) UI button and Goal node HUD indicator"
```

---

### Task 5: Mode C Prompts/Docs & Headless E2E Verification (Direction A + B)

**Files:**
- Modify: `.agents/skills/robot-multi-agent-rd/references/driver-hybrid-prompt.md`
- Modify: `.agents/skills/robot-multi-agent-rd/SKILL.md`
- Modify: `README.md`
- Modify: `docs/mujoco-studio-runbook.md`
- Test: `tests/test_hybrid_brain.py`, `tests/test_agent_skill.py`

**Interfaces:**
- Consumes Tasks 1–4 (`go --route`, `--stop-at-node`, `reset --keep-topo`, `dock` `"topo"`, `telemetry` round-trip fields).
- Produces updated Mode C operational docs/prompts supporting Standard One-Way, Direction A (Return-to-Home), and Direction B (Two-Pass Micromouse with fresh Pass-2 subagent), plus headless MuJoCo integration tests proving both workflows on `maze_b` and `2wheel`.

- [ ] **Step 1: Write failing E2E headless & prompt-contract tests**

1. In `tests/test_agent_skill.py`: assert `driver-hybrid-prompt.md` and `SKILL.md` document `--route`, `--stop-at-node`, `reset --keep-topo`, `arrived_node`, `route_blocked`, Direction A (`returned_to_start`), and Direction B (fresh `robot-driver-hybrid` subagent on Pass 2).
2. In `tests/test_hybrid_brain.py`: add `test_hybrid_brain_direction_a_return_to_home_and_direction_b_speedrun_on_maze_b`:
   - **Pass 1 + Return-to-Home (Direction A) on `maze_b`:** Blind topological policy explores `maze_b`, docks at green target (`record_dock` creates `"goal"` node), computes shortest `dist_m` path over `"explored"` edges back to `Node #0`, calls `go_to_decision(..., route=home_route, stop_at_node=0)`, and asserts `event == "arrived_node"`, `contact_count == 0`, and world distance to spawn `<= 0.35 m`.
   - **Pass 2 Speed-run (Direction B) on `maze_b`:** Calls `sim.reset()` + `topo.reset_for_repass(sim.get_status())`, computes shortest `dist_m` path from `Node #0` to `Node #G`, executes **1** `go_to_decision(..., route=goal_route)` + `dock_target`, and asserts `dock["reached"] is True`, `contact_count == 0`, and `pass2_odom_dist <= 7.8` (`path_efficiency >= 0.83`, vs `> 20 m` in Pass 1).

- [ ] **Step 2: Run new tests to verify failure**

Run: `.\.venv\Scripts\python.exe -m pytest tests/test_agent_skill.py tests/test_hybrid_brain.py -k "direction_a or hybrid_brain_mode_c" -v`
Expected: FAIL on prompt/doc assertions (and/or any edge gap in E2E).

- [ ] **Step 3: Update Mode C prompts, skill workflow, `README.md`, and `docs/mujoco-studio-runbook.md`**

Update `.agents/skills/robot-multi-agent-rd/references/driver-hybrid-prompt.md`, `.agents/skills/robot-multi-agent-rd/SKILL.md`, `README.md`, and `docs/mujoco-studio-runbook.md` per Spec §5.4–5.5.

- [ ] **Step 4: Run full test suite to verify zero regressions**

Run: `.\.venv\Scripts\python.exe -m pytest -v`
Expected: 100% PASS across the entire repository, with `git status` showing zero changes to `models/`.

- [ ] **Step 5: Commit**

```powershell
git add .agents/skills/robot-multi-agent-rd/SKILL.md .agents/skills/robot-multi-agent-rd/references/driver-hybrid-prompt.md README.md docs/mujoco-studio-runbook.md docs/superpowers/specs/2026-11-24-hybrid-two-pass-and-return-design.md docs/superpowers/plans/2026-11-24-hybrid-two-pass-and-return.md tests/test_agent_skill.py tests/test_hybrid_brain.py
git commit -m "feat(mode-c): add Direction A return-to-home and Direction B two-pass speed-run workflows"
```
