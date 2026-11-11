---
name: robot-multi-agent-rd
description: >-
  Orchestrates the 3-role Multi-Agent Robot Control, Telemetry Monitoring & Auto-R&D workflow
  (Main Agent coordinator + blind Sim-to-Real robot-driver subagent + diagnostics/tuning robot-engineer subagent).
  Use whenever the user asks to run multi-agent robot navigation, benchmark, monitor, or automatically diagnose and improve the MuJoCo robot.
---

# Multi-Agent Robot Control & Auto-R&D (`robot-multi-agent-rd`)

Keep **Main Agent** context ultra-lean (~1–2 KB scorecard per round) while delegating verbose driving loops to `robot-driver` and physics/code forensics to `robot-engineer`.

## 1. Register Specialized Subagents Once Per Session

Before Round 1, read the two reference templates and register both subagent types via `define_subagent`:
- **`robot-driver`** (`EnableWriteTools: true`, `EnableSubagentTools: false`): System prompt from [driver-prompt.md](./references/driver-prompt.md).
- **`robot-engineer`** (`EnableWriteTools: true`, `EnableSubagentTools: false`): System prompt from [engineer-prompt.md](./references/engineer-prompt.md).

## 2. R&D Loop Protocol (Maximum 3 Rounds)

For Round `i = 1..3`:

1. **Set Arena Challenge (Main Agent):**
   ```powershell
   .venv\Scripts\python.exe robot_ctl.py reset --target-x <X> --target-y <Y> --vision
   ```
   *(If `status` fails with `BridgeOfflineError` or `gyro_yaw_deg` / `/reload` is missing on an older server process, ask the user to restart `uv run python sim_server.py` once in their PowerShell window.)*

2. **Launch Zero-Token Background Blackbox (`WaitMsBeforeAsync: 500`):**
   ```powershell
   .venv\Scripts\python.exe robot_ctl.py telemetry record --hz 10 --max-duration 240
   ```

3. **Dispatch `robot-driver` Subagent (`invoke_subagent`):**
   - Pass `TypeName: "robot-driver"`, `Model: "inherit"` (or `"flash"` when user explicitly requests fast/light model).
   - Brief: state the mission goal (`vision.target_visible == true`, `|offset_x| < 0.25`, `0.15 <= rf_center <= 0.35`, zero collisions) without revealing `<X>, <Y>` coordinates!
   - End turn and wait for reactive wakeup (do **not** poll).

4. **Collect Compact 1 KB Scorecard:**
   Once `robot-driver` replies, stop the blackbox and read the summary JSON:
   ```powershell
   .venv\Scripts\python.exe robot_ctl.py telemetry stop
   ```
   Do **not** read `robot-driver`'s full `transcript.jsonl` into Main Agent context unless debugging a tool crash.

5. **Grade & Decide:**
   - **Pass & Optimal Criteria:**
     - `goal_reached == true`
     - `near_collision_count == 0` and `error_count == 0`
     - `wall_time_s <= 90.0`
     - `anomalies.braking_rf_drop_m < 0.15`
   - If **Pass & Optimal** (or `i == 3`), stop the loop and present the round-by-round scorecard table to the user.
   - Otherwise, record current `git rev-parse HEAD` (or diff checkpoint) and dispatch **`robot-engineer`** (`invoke_subagent` with `TypeName: "robot-engineer"`), passing:
     - The `telemetry stop` JSON scorecard,
     - `robot-driver`'s short summary,
     - Specific bottleneck to fix (e.g., occlusion recovery, obstacle clearance, approach speed, sensor tilt, actuator tuning).
   - **Regression Guard:** After Round `i + 1` runs, if `pytest` broke or Round `i + 1` regressed severely (e.g., failed `goal_reached` or introduced collisions compared to Round `i`), revert unverified edits via `git checkout -- models/arena_2wheel.xml robot_sim robot_ctl.py` and run `.venv\Scripts\python.exe robot_ctl.py reload`.
