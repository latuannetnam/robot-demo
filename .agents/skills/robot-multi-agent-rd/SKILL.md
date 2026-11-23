---
name: robot-multi-agent-rd
description: >-
  Orchestrates the 3-role Multi-Agent Robot Control, Telemetry Monitoring & Auto-R&D workflow
  (Main Agent coordinator + blind Sim-to-Real robot-driver subagent + diagnostics/tuning robot-engineer subagent).
  Use whenever the user asks to run multi-agent robot navigation, benchmark, monitor, or automatically diagnose and improve the MuJoCo robot.
---

# Multi-Agent Robot Control & Auto-R&D (`robot-multi-agent-rd`)

Keep **Main Agent** context ultra-lean (~1–2 KB scorecard per round) while delegating verbose driving loops to a blind driver subagent and physics/code forensics to `robot-engineer`.

## Execution Modes

This skill supports three operating modes:
- **Mode A — `auto-skill` (Default):** Unless the user explicitly requests `agent-brain` (Mode B) or `hybrid-brain` (Mode C), run **Mode A** using closed-loop autonomous navigation skills (`explore`, `approach`, `scan`) inside `robot-driver`.
- **Mode B — `agent-brain` (Cognitive Experiment):** Activated when the user asks for `agent-brain` / Mode B / Agent-as-the-brain navigation. Uses `robot-driver-brain` with stateless 20 Hz reflex primitives (`advance`, `cross-gate`, `dock`, `turn`, `move`, `scan --no-recover-occlusion`) while the AI subagent handles 100% of topological graph memory and path planning. Wall-clock `wall_time_s` is **not** a pass/fail gate in Mode B.
- **Mode C — `hybrid-brain` (System 1 / System 2 Hybrid):** Activated when the user asks for `hybrid-brain` / Mode C / System 1-2 navigation. Uses `robot-driver-hybrid` with System 1 combined motion & objective topological odometry (`topo show`, `go`, `cross-gate`, `dock`) while System 2 (the AI subagent) owns 100% of branch selection, dead-end backtracking, loop escape, static detours, and gate sequencing.

## 1. Register Specialized Subagents Once Per Session

Before Round 1, read the reference templates and register the needed subagent types via `define_subagent`:
- **`robot-driver`** (`EnableWriteTools: true`, `EnableSubagentTools: false`, Mode A default): System prompt from [driver-prompt.md](./references/driver-prompt.md).
- **`robot-driver-brain`** (`EnableWriteTools: true`, `EnableSubagentTools: false`, Mode B): System prompt from [driver-brain-prompt.md](./references/driver-brain-prompt.md).
- **`robot-driver-hybrid`** (`EnableWriteTools: true`, `EnableSubagentTools: false`, Mode C): System prompt from [driver-hybrid-prompt.md](./references/driver-hybrid-prompt.md).
- **`robot-engineer`** (`EnableWriteTools: true`, `EnableSubagentTools: false`): System prompt from [engineer-prompt.md](./references/engineer-prompt.md).

## 2. R&D Loop Protocol (Maximum 3 Rounds)

For Round `i = 1..3`:

1. **Set Arena Challenge (Main Agent):**
   Switch to the requested arena (`2wheel`, `maze_a` / alias `maze`, `maze_b`, `maze_c`, `dynamic_a` / alias `dynamic`, or `dynamic_b`) or reset target coordinates dynamically without restarting the 3D viewer:
   ```powershell
   # Switch arena (automatically uses the arena's default spawn & target unless --target-x/--target-y are passed)
   .venv\Scripts\python.exe robot_ctl.py reload --arena maze_a --vision
   # Or set a custom valid target on the current arena:
   .venv\Scripts\python.exe robot_ctl.py reset --target-x <X> --target-y <Y> --vision
   ```
   *(Main Agent may inspect available maps and active arena via `.venv\Scripts\python.exe robot_ctl.py arenas --debug`. If `status` fails with `BridgeOfflineError` or `/arenas` is missing on an older server process, ask the user to restart `uv run python sim_server.py` once in their PowerShell window.)*

2. **Launch Zero-Token Background Blackbox (`WaitMsBeforeAsync: 500`):**
   Use `--max-duration 360` in Mode A (`auto-skill`), `--max-duration 900` in Mode B (`agent-brain`), or `--max-duration 480` in Mode C (`hybrid-brain`):
   ```powershell
   # Mode A (default auto-skill):
   .venv\Scripts\python.exe robot_ctl.py telemetry record --hz 10 --max-duration 360
   # Mode B (agent-brain):
   .venv\Scripts\python.exe robot_ctl.py telemetry record --hz 10 --max-duration 900
   # Mode C (hybrid-brain):
   .venv\Scripts\python.exe robot_ctl.py telemetry record --hz 10 --max-duration 480
   ```

3. **Dispatch Driver Subagent (`invoke_subagent`):**
   - Pass `TypeName: "robot-driver"` (Mode A), `TypeName: "robot-driver-brain"` (Mode B), or `TypeName: "robot-driver-hybrid"` (Mode C), `Model: "inherit"` (or `"flash"` when requested).
   - Brief: state ONLY the mission goal (`vision.target_visible == true`, `|offset_x| < 0.25`, `0.15 <= rf_center <= 0.35`, zero collisions) — NEVER reveal `<X>, <Y>` coordinates, spawn pose, or which arena (`2wheel` / `maze_a` / `maze_b` / `maze_c` / `dynamic_a` / `dynamic_b`) is loaded!
   - End turn and wait for reactive wakeup (do **not** poll).

4. **Collect Compact 1 KB Scorecard:**
   Once the driver subagent replies, stop the blackbox and read the summary JSON:
   ```powershell
   .venv\Scripts\python.exe robot_ctl.py telemetry stop
   ```
   Do **not** read the driver's full `transcript.jsonl` into Main Agent context unless debugging a tool crash.

5. **Grade & Decide:**
   - **Mode A (`auto-skill`) Pass & Optimal Criteria:**
     - `goal_reached == true`, `contact_count == 0`, `near_collision_count == 0`, `error_count == 0`, `anomalies.braking_rf_drop_m < 0.15`
     - `wall_time_s <= 90.0` (`2wheel`), `<= 120.0` (`dynamic_a`), or `<= 180.0` (`maze_a`, `maze_b`, `maze_c`, `dynamic_b`)
   - **Mode B (`agent-brain`) Pass & Cognitive Quality Criteria:**
     - Mandatory: `goal_reached == true`, `contact_count == 0`, `near_collision_count == 0`, `error_count == 0`
     - Spatial Memory & Cognitive Quality (no `wall_time_s` pass/fail cutoff): `redundant_revisit_count == 0`, high `path_efficiency` (`reference_path_m / path_length_m`), low `backtrack_revisit_count`, plus observability breakdown (`active_drive_time_s`, `agent_think_time_s`, `command_burst_count`). Trigger `robot-engineer` (max 3 rounds) on any mandatory failure or `redundant_revisit_count > 0`.
   - **Mode C (`hybrid-brain`) Pass & Hybrid Quality Criteria:**
     - Mandatory: `goal_reached == true`, `contact_count == 0`, `near_collision_count == 0`, `error_count == 0`
     - Cognitive & Efficiency Quality: `redundant_revisit_count == 0`, report full `path_efficiency`, `wall_time_s`, `active_drive_time_s`, `agent_think_time_s`, `command_burst_count` for 3-way comparison (`Mode A` vs `Mode B` vs `Mode C`). Trigger `robot-engineer` (max 3 rounds) on any mandatory failure or `redundant_revisit_count > 0`.
   - If passing criteria hold (or `i == 3`), stop the loop and present the round-by-round scorecard table to the user.
   - Otherwise, record current `git rev-parse HEAD` (or diff checkpoint) and dispatch **`robot-engineer`** (`invoke_subagent` with `TypeName: "robot-engineer"`), passing the active mode (`Mode A`, `Mode B`, or `Mode C`), `telemetry stop` JSON scorecard, the driver's summary, and specific bottleneck to fix.
   - **Regression Guard:** After Round `i + 1` runs, if `pytest` broke or Round `i + 1` regressed, revert unverified edits via `git checkout -- models/robot_2wheel.xml robot_sim robot_ctl.py` (and the active mode's prompt file `driver-brain-prompt.md` or `driver-hybrid-prompt.md`) and run `.venv\Scripts\python.exe robot_ctl.py reload`.
