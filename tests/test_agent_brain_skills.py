import math
import threading
import time

import pytest

from robot_ctl import _call_bridge
from robot_sim.arenas import resolve_arena_model
from robot_sim.core import MAX_WHEEL_RAD_S, WHEEL_RADIUS
from sim_server import SimulationApp
from robot_sim.skills import (
    advance_segment,
    closed_loop_turn,
    compute_open_branches,
    cross_dynamic_gate,
    dock_target,
)


class _MockBotBridge:
    """Stateless-skill test harness simulating kinematic odom/yaw/sensors without debug leaks."""

    def __init__(self, states: list[dict], *, advance_odom_on_drive: bool = True) -> None:
        self.states = list(states)
        self.idx = 0
        self.calls: list[tuple[str, str, dict | None]] = []
        self.odom = 0.0
        self.sim_time = 0.0
        self.advance_odom_on_drive = advance_odom_on_drive

    def _current(self) -> dict:
        raw = self.states[min(self.idx, len(self.states) - 1)]
        st = {
            "sim_time": float(raw.get("sim_time", self.sim_time)),
            "moving": bool(raw.get("moving", False)),
            "sensors": {
                "gyro_yaw_deg": float(raw.get("gyro_yaw_deg", 0.0)),
                "odom_dist_m": float(raw.get("odom_dist_m", self.odom)),
                "forward_speed_m_s": float(raw.get("forward_speed_m_s", 0.0)),
                "yaw_rate_rad_s": float(raw.get("yaw_rate_rad_s", 0.0)),
                "rangefinders_m": dict(raw.get("rangefinders_m", {"left": 1.0, "center": 2.0, "right": 1.0, "left90": 0.45, "right90": 0.45})),
            },
            "vision": dict(raw.get("vision", {"target_visible": False, "target": None, "obstacles": []})),
            # Deliberately include debug internally so tests verify skills strip it cleanly!
            "debug": {"arena": "mock", "robot_xy": [0.0, 0.0], "target_xy": [1.5, 0.0], "contact_count": 0},
        }
        return st

    def __call__(self, path: str, method: str = "GET", payload: dict | None = None) -> dict:
        self.calls.append((path, method, payload))
        if path.startswith("/status"):
            st = self._current()
            if self.idx < len(self.states) - 1:
                self.idx += 1
            else:
                self.sim_time += 0.12
            return st
        if path == "/wheels" and isinstance(payload, dict):
            lw = float(payload.get("left", 0.0))
            rw = float(payload.get("right", 0.0))
            dur = float(payload.get("duration", 0.1))
            if self.advance_odom_on_drive and (lw + rw) > 0.2 and abs(lw - rw) < 4.0:
                self.odom += 0.5 * (lw + rw) * WHEEL_RADIUS * min(dur, 0.12)
            self.sim_time += min(dur, 0.15)
            return {"ok": True, "moving": (abs(lw) > 0.01 or abs(rw) > 0.01)}
        if path == "/stop":
            return {"ok": True, "moving": False}
        return {"ok": True}


def test_agent_brain_skills_validate_inputs_and_compute_open_branches():
    dummy = _MockBotBridge([{}])

    for bad in (-1.0, 0.0, float("nan"), float("inf")):
        with pytest.raises(ValueError):
            advance_segment(dummy, max_dist=bad)

    for bad_spd in (-0.2, 0.0, 0.01, MAX_WHEEL_RAD_S * WHEEL_RADIUS + 0.05, float("nan")):
        with pytest.raises(ValueError):
            advance_segment(dummy, speed_m_s=bad_spd)

    for bad_dt in (0.01, 1.5, float("inf")):
        with pytest.raises(ValueError):
            advance_segment(dummy, step_dt=bad_dt)

    for bad_t in (0.0, -2.0, float("nan"), float("inf")):
        with pytest.raises(ValueError):
            cross_dynamic_gate(dummy, timeout_s=bad_t)

    for bad_stop in (0.05, 1.6, float("nan")):
        with pytest.raises(ValueError):
            dock_target(dummy, stop_distance=bad_stop)
    for bad_steps in (0, -3, float("inf"), float("nan"), True):
        with pytest.raises(ValueError):
            dock_target(dummy, max_steps=bad_steps)

    assert compute_open_branches({"center": 0.9, "left": 0.5, "right": 0.5, "left90": 0.8, "right90": 0.3}) == [
        "left",
        "straight",
    ]
    assert compute_open_branches({"center": 0.35, "left": 0.4, "right": 0.4, "left90": 0.8, "right90": 0.8}) == [
        "left",
        "right",
    ]


def test_advance_segment_events_stateless_unit():
    # 1. T-junction: left90 opens after moved >= 0.22 m
    t_states = [
        {"sim_time": 0.0, "odom_dist_m": 0.00, "rangefinders_m": {"left": 0.5, "center": 2.0, "right": 0.5, "left90": 0.40, "right90": 0.40}},
        {"sim_time": 0.1, "odom_dist_m": 0.15, "rangefinders_m": {"left": 0.5, "center": 1.8, "right": 0.5, "left90": 0.40, "right90": 0.40}},
        {"sim_time": 0.2, "odom_dist_m": 0.30, "rangefinders_m": {"left": 0.6, "center": 1.6, "right": 0.5, "left90": 0.95, "right90": 0.40}},
        {"sim_time": 0.3, "odom_dist_m": 0.48, "rangefinders_m": {"left": 0.6, "center": 1.4, "right": 0.5, "left90": 0.95, "right90": 0.40}},
        {"sim_time": 0.4, "odom_dist_m": 0.58, "moving": False, "rangefinders_m": {"left": 0.6, "center": 1.3, "right": 0.5, "left90": 0.95, "right90": 0.40}},
    ]
    res_junc = advance_segment(_MockBotBridge(t_states, advance_odom_on_drive=False), max_dist=1.8, step_dt=0.05)
    assert res_junc["event"] == "junction"
    assert "left" in res_junc["open_branches"]
    assert "debug" not in res_junc["after_status"]

    # 2. Dead end: front wall with both sides closed -> event == "dead_end", no 180° auto-turn
    dead_states = [
        {"sim_time": 0.0, "odom_dist_m": 0.0, "rangefinders_m": {"left": 0.5, "center": 0.7, "right": 0.5, "left90": 0.40, "right90": 0.40}},
        {"sim_time": 0.1, "odom_dist_m": 0.3, "rangefinders_m": {"left": 0.35, "center": 0.35, "right": 0.35, "left90": 0.38, "right90": 0.38}},
        {"sim_time": 0.2, "odom_dist_m": 0.32, "moving": False, "rangefinders_m": {"left": 0.35, "center": 0.34, "right": 0.35, "left90": 0.38, "right90": 0.38}},
    ]
    dead_bridge = _MockBotBridge(dead_states, advance_odom_on_drive=False)
    res_dead = advance_segment(dead_bridge, max_dist=1.8, step_dt=0.05)
    assert res_dead["event"] == "dead_end"
    assert res_dead["open_branches"] == []
    assert "debug" not in res_dead["after_status"]
    assert not any(
        p == "/wheels" and pl is not None and abs(float(pl.get("left", 0.0)) + float(pl.get("right", 0.0))) < 0.5 and abs(float(pl.get("left", 0.0))) > 2.5
        for p, _m, pl in dead_bridge.calls
    )

    # 3. L-corner: front wall blocked, left open -> junction with ["left"]
    l_states = [
        {"sim_time": 0.0, "odom_dist_m": 0.0, "rangefinders_m": {"left": 0.5, "center": 0.35, "right": 0.4, "left90": 0.90, "right90": 0.35}},
        {"sim_time": 0.1, "odom_dist_m": 0.0, "moving": False, "rangefinders_m": {"left": 0.5, "center": 0.35, "right": 0.4, "left90": 0.90, "right90": 0.35}},
    ]
    res_l = advance_segment(_MockBotBridge(l_states, advance_odom_on_drive=False), max_dist=1.8, step_dt=0.05)
    assert res_l["event"] == "junction"
    assert res_l["open_branches"] == ["left"]
    assert "debug" not in res_l["after_status"]

    # 4. Dynamic gate ahead: obstacle shifts offset_x >= 0.018 within 0.42 s
    dyn_states = [
        {"sim_time": 0.0, "rangefinders_m": {"left": 0.8, "center": 0.65, "right": 0.8, "left90": 0.85, "right90": 0.85},
         "vision": {"target_visible": False, "target": None, "obstacles": [{"color": "red", "offset_x": 0.05, "area_pct": 12.0}]}},
        {"sim_time": 0.15, "rangefinders_m": {"left": 0.8, "center": 0.65, "right": 0.8, "left90": 0.85, "right90": 0.85},
         "vision": {"target_visible": False, "target": None, "obstacles": [{"color": "red", "offset_x": 0.12, "area_pct": 11.8}]}},
        {"sim_time": 0.30, "moving": False, "rangefinders_m": {"left": 0.8, "center": 0.65, "right": 0.8, "left90": 0.85, "right90": 0.85},
         "vision": {"target_visible": False, "target": None, "obstacles": [{"color": "red", "offset_x": 0.15, "area_pct": 11.5}]}},
    ]
    res_dyn = advance_segment(_MockBotBridge(dyn_states, advance_odom_on_drive=False), max_dist=1.8, step_dt=0.05)
    assert res_dyn["event"] == "dynamic_gate_ahead"
    assert res_dyn["obstacle"]["motion"] == "dynamic"
    assert "debug" not in res_dyn["after_status"]

    # 5. Static obstacle ahead: stationary >= 0.42 s
    stat_states = [
        {"sim_time": 0.0, "rangefinders_m": {"left": 0.8, "center": 0.58, "right": 0.8, "left90": 0.90, "right90": 0.42},
         "vision": {"target_visible": False, "target": None, "obstacles": [{"color": "blue", "offset_x": 0.0, "area_pct": 14.0}]}},
        {"sim_time": 0.18, "rangefinders_m": {"left": 0.8, "center": 0.58, "right": 0.8, "left90": 0.90, "right90": 0.42},
         "vision": {"target_visible": False, "target": None, "obstacles": [{"color": "blue", "offset_x": 0.0, "area_pct": 14.0}]}},
        {"sim_time": 0.45, "moving": False, "rangefinders_m": {"left": 0.8, "center": 0.58, "right": 0.8, "left90": 0.90, "right90": 0.42},
         "vision": {"target_visible": False, "target": None, "obstacles": [{"color": "blue", "offset_x": 0.0, "area_pct": 14.0}]}},
    ]
    res_stat = advance_segment(_MockBotBridge(stat_states, advance_odom_on_drive=False), max_dist=1.8, step_dt=0.05)
    assert res_stat["event"] == "static_obstacle_ahead"
    assert res_stat["obstacle"]["motion"] == "static"
    assert res_stat["flank_clearance_m"]["left"] > res_stat["flank_clearance_m"]["right"]
    assert "debug" not in res_stat["after_status"]

    # 6. Target ready
    tgt_states = [
        {"sim_time": 0.0, "moving": False, "rangefinders_m": {"left": 1.0, "center": 0.75, "right": 1.0, "left90": 0.45, "right90": 0.45},
         "vision": {"target_visible": True, "target": {"offset_x": 0.04, "area_pct": 3.5}, "obstacles": []}},
    ]
    res_tgt = advance_segment(_MockBotBridge(tgt_states, advance_odom_on_drive=False), max_dist=1.8, step_dt=0.05)
    assert res_tgt["event"] == "target_ready"
    assert "debug" not in res_tgt["after_status"]

    # 7. Max dist clean finish
    max_states = [
        {"sim_time": 0.0, "odom_dist_m": 0.0, "rangefinders_m": {"left": 0.9, "center": 2.5, "right": 0.9, "left90": 0.45, "right90": 0.45}},
        {"sim_time": 0.1, "odom_dist_m": 0.45, "rangefinders_m": {"left": 0.9, "center": 2.1, "right": 0.9, "left90": 0.45, "right90": 0.45}},
        {"sim_time": 0.2, "odom_dist_m": 0.85, "moving": False, "rangefinders_m": {"left": 0.9, "center": 1.7, "right": 0.9, "left90": 0.45, "right90": 0.45}},
    ]
    res_max = advance_segment(_MockBotBridge(max_states, advance_odom_on_drive=False), max_dist=0.80, step_dt=0.05)
    assert res_max["event"] == "max_dist"
    assert "debug" not in res_max["after_status"]


def test_cross_dynamic_gate_and_dock_target_refusal_unit():
    # Static obstacle -> cross_dynamic_gate refuses without driving forward
    stat_gate_states = [
        {"sim_time": 0.0, "rangefinders_m": {"left": 0.8, "center": 0.55, "right": 0.8, "left90": 0.9, "right90": 0.9},
         "vision": {"target_visible": False, "target": None, "obstacles": [{"color": "blue", "offset_x": 0.0, "area_pct": 14.0}]}},
        {"sim_time": 0.20, "rangefinders_m": {"left": 0.8, "center": 0.55, "right": 0.8, "left90": 0.9, "right90": 0.9},
         "vision": {"target_visible": False, "target": None, "obstacles": [{"color": "blue", "offset_x": 0.0, "area_pct": 14.0}]}},
        {"sim_time": 0.46, "moving": False, "rangefinders_m": {"left": 0.8, "center": 0.55, "right": 0.8, "left90": 0.9, "right90": 0.9},
         "vision": {"target_visible": False, "target": None, "obstacles": [{"color": "blue", "offset_x": 0.0, "area_pct": 14.0}]}},
    ]
    gate_bridge = _MockBotBridge(stat_gate_states, advance_odom_on_drive=False)
    res_gate = cross_dynamic_gate(gate_bridge, timeout_s=5.0)
    assert res_gate["crossed"] is False
    assert res_gate["reason"] == "static_obstacle"
    assert "debug" not in res_gate["after_status"]
    assert not any(
        p == "/wheels" and pl is not None and (float(pl.get("left", 0.0)) > 0.01 or float(pl.get("right", 0.0)) > 0.01)
        for p, _m, pl in gate_bridge.calls
    )

    # dock_target refusals
    d_not_vis = dock_target(
        _MockBotBridge([{"vision": {"target_visible": False, "target": None, "obstacles": []}}]),
    )
    assert d_not_vis["reached"] is False and d_not_vis["reason"] == "not_visible"
    assert "debug" not in d_not_vis["after_status"]

    d_too_far = dock_target(
        _MockBotBridge([
            {
                "rangefinders_m": {"left": 2.2, "center": 2.2, "right": 2.2, "left90": 1.0, "right90": 1.0},
                "vision": {"target_visible": True, "target": {"offset_x": 0.0, "area_pct": 0.25}, "obstacles": []},
            }
        ]),
    )
    assert d_too_far["reached"] is False and d_too_far["reason"] == "too_far"
    assert "debug" not in d_too_far["after_status"]

    # Spec 5.2.C threshold: rf_center > 1.25 and area_pct < 0.90 -> too_far
    d_too_far_spec = dock_target(
        _MockBotBridge([
            {
                "rangefinders_m": {"left": 1.5, "center": 1.50, "right": 1.5, "left90": 1.0, "right90": 1.0},
                "vision": {"target_visible": True, "target": {"offset_x": 0.0, "area_pct": 0.60}, "obstacles": []},
            }
        ]),
    )
    assert d_too_far_spec["reached"] is False and d_too_far_spec["reason"] == "too_far"
    assert "debug" not in d_too_far_spec["after_status"]

    # Frozen sim_time wait in cross_dynamic_gate breaks fast instead of hanging 18s
    frozen_gate = _MockBotBridge(
        [
            {"sim_time": 1.0, "rangefinders_m": {"left": 0.9, "center": 0.55, "right": 0.9, "left90": 0.4, "right90": 0.4},
             "vision": {"target_visible": False, "target": None, "obstacles": []}},
        ],
        advance_odom_on_drive=False,
    )
    t_start = time.monotonic()
    res_frozen = cross_dynamic_gate(frozen_gate, timeout_s=5.0)
    assert (time.monotonic() - t_start) < 2.0
    assert res_frozen["crossed"] is False and res_frozen["reason"] == "timeout"

    # Stalled sprint odom_dist_m=0 across loop does not falsely report crossed=True, cleared
    stalled_sprint_states = [
        {"sim_time": float(i) * 0.1, "odom_dist_m": 0.0,
         "rangefinders_m": {"left": 0.9, "center": 1.2, "right": 0.9, "left90": 0.4, "right90": 0.4},
         "vision": {"target_visible": False, "target": None, "obstacles": []}}
        for i in range(160)
    ]
    res_stalled = cross_dynamic_gate(_MockBotBridge(stalled_sprint_states, advance_odom_on_drive=False), timeout_s=3.0)
    assert res_stalled["crossed"] is False and res_stalled["reason"] == "timeout"

    d_blocked = dock_target(
        _MockBotBridge([
            {
                "rangefinders_m": {"left": 0.8, "center": 0.34, "right": 0.8, "left90": 0.9, "right90": 0.9},
                "vision": {
                    "target_visible": True,
                    "target": {"offset_x": 0.0, "area_pct": 2.0},
                    "obstacles": [{"color": "red", "offset_x": 0.02, "area_pct": 12.0}],
                },
            }
        ]),
    )
    assert d_blocked["reached"] is False and d_blocked["reason"] == "obstacle_blocking"
    assert "debug" not in d_blocked["after_status"]


def _run_headless_episode(arena_name: str, worker_fn):
    app = SimulationApp(model_path=resolve_arena_model(arena_name))
    host, port = app.start_http_server(host="127.0.0.1", port=0)
    url = f"http://{host}:{port}"
    call_fn = lambda path, method="GET", payload=None: _call_bridge(url, path, method, payload)
    stop_evt = threading.Event()
    result: dict = {}
    err_holder: list[BaseException] = []

    def worker() -> None:
        try:
            worker_fn(call_fn, result)
            result["raw_debug"] = call_fn("/status?vision=1", "GET")["debug"]
        except BaseException as exc:
            err_holder.append(exc)
        finally:
            stop_evt.set()

    t = threading.Thread(target=worker, daemon=True)
    t.start()
    try:
        while not stop_evt.is_set():
            app.tick_headless(step_batch=12)
            time.sleep(0.004)
        t.join(timeout=20.0)
    finally:
        app.shutdown()

    if err_holder:
        raise err_holder[0]
    return result


def test_agent_brain_skills_solve_maze_a_on_explicit_agent_turns():
    def _solve_maze_a(call_fn, out: dict) -> None:
        # Blind cognitive controller using ONLY advance_segment, closed_loop_turn, and dock_target
        gx, gy = 0.0, 0.0
        visited_exits: set[tuple[tuple[int, int], int]] = set()
        dock_res = None
        for _ in range(18):
            res = advance_segment(call_fn, max_dist=1.8, speed_m_s=0.52, step_dt=0.10)
            assert "debug" not in res["after_status"]
            yaw_deg = float(res["after_status"]["sensors"]["gyro_yaw_deg"])
            yaw_rad = math.radians(yaw_deg)
            gx += float(res["moved_m"]) * math.cos(yaw_rad)
            gy += float(res["moved_m"]) * math.sin(yaw_rad)

            if res["event"] == "target_ready":
                dock_res = dock_target(call_fn, stop_distance=0.25)
                assert "debug" not in dock_res["after_status"]
                if dock_res["reached"]:
                    break
            elif res["event"] in ("junction", "dead_end"):
                branches = res["open_branches"]
                if not branches:
                    closed_loop_turn(call_fn, degrees=180.0, max_speed_deg_s=220.0, tol_deg=1.5, fast_settle=True)
                    continue
                cell = (round(gx / 0.6), round(gy / 0.6))
                dir_offsets = {"left": 90.0, "straight": 0.0, "right": -90.0}
                ordered = sorted(
                    branches,
                    key=lambda b: (
                        (cell, int(((yaw_deg + dir_offsets[b] + 45.0) % 360.0) // 90.0)) in visited_exits,
                        {"left": 0, "straight": 1, "right": 2}[b],
                    ),
                )
                pick = ordered[0]
                exit_dir = int(((yaw_deg + dir_offsets[pick] + 45.0) % 360.0) // 90.0)
                visited_exits.add((cell, exit_dir))
                if dir_offsets[pick] != 0.0:
                    closed_loop_turn(
                        call_fn,
                        degrees=dir_offsets[pick],
                        max_speed_deg_s=220.0,
                        tol_deg=1.5,
                        fast_settle=True,
                    )
        out["dock_res"] = dock_res

    result = _run_headless_episode("maze_a", _solve_maze_a)
    assert result["dock_res"] is not None and result["dock_res"]["reached"] is True
    assert result["raw_debug"]["contact_count"] == 0


def test_agent_brain_skills_cross_dynamic_a_lane_by_lane_and_dynamic_b_s_curve():
    # Part 1: dynamic_a lane-by-lane crossing (>= 2 separate cross_dynamic_gate calls)
    def _solve_dyn_a(call_fn, out: dict) -> None:
        cross_calls = 0
        dock_a = None
        for _ in range(10):
            res = advance_segment(call_fn, max_dist=1.8, speed_m_s=0.55, step_dt=0.10)
            assert "debug" not in res["after_status"]
            if res["event"] == "dynamic_gate_ahead":
                cross_calls += 1
                g_res = cross_dynamic_gate(call_fn, timeout_s=12.0)
                assert g_res["crossed"] is True
                assert "debug" not in g_res["after_status"]
            elif res["event"] == "target_ready":
                dock_a = dock_target(call_fn, stop_distance=0.25)
                assert "debug" not in dock_a["after_status"]
                if dock_a["reached"]:
                    break
        out["cross_calls"] = cross_calls
        out["dock"] = dock_a

    res_a = _run_headless_episode("dynamic_a", _solve_dyn_a)
    assert res_a["cross_calls"] >= 2
    assert res_a["dock"] is not None and res_a["dock"]["reached"] is True
    assert res_a["raw_debug"]["contact_count"] == 0

    # Part 2: dynamic_b S-curve + 3 moving bottleneck gates
    def _solve_dyn_b(call_fn, out: dict) -> None:
        dock_b = None
        for _ in range(16):
            res = advance_segment(call_fn, max_dist=1.8, speed_m_s=0.52, step_dt=0.10)
            assert "debug" not in res["after_status"]
            if res["event"] == "dynamic_gate_ahead":
                g_res = cross_dynamic_gate(call_fn, timeout_s=12.0)
                assert g_res["crossed"] is True
                assert "debug" not in g_res["after_status"]
            elif res["event"] == "target_ready":
                dock_b = dock_target(call_fn, stop_distance=0.25)
                assert "debug" not in dock_b["after_status"]
                if dock_b["reached"]:
                    break
            elif res["event"] in ("junction", "dead_end"):
                branches = res["open_branches"]
                if "left" in branches:
                    closed_loop_turn(call_fn, degrees=90.0, max_speed_deg_s=220.0, tol_deg=1.5, fast_settle=True)
                elif "right" in branches:
                    closed_loop_turn(call_fn, degrees=-90.0, max_speed_deg_s=220.0, tol_deg=1.5, fast_settle=True)
                elif "straight" not in branches:
                    closed_loop_turn(call_fn, degrees=180.0, max_speed_deg_s=220.0, tol_deg=1.5, fast_settle=True)
        out["dock"] = dock_b

    res_b = _run_headless_episode("dynamic_b", _solve_dyn_b)
    assert res_b["dock"] is not None and res_b["dock"]["reached"] is True
    assert res_b["raw_debug"]["contact_count"] == 0


def test_advance_segment_2wheel_classifies_static_obstacle_without_dynamic_false_positive():
    def _probe_2wheel_static(call_fn, out: dict) -> None:
        # Turn toward static obs_red at (0.70, 0.95) (~+53.6 deg) and cruise into it
        closed_loop_turn(call_fn, degrees=53.6, max_speed_deg_s=180.0, tol_deg=1.0, fast_settle=True)
        res = advance_segment(call_fn, max_dist=1.4, speed_m_s=0.50, step_dt=0.10)
        assert "debug" not in res["after_status"]
        out["res"] = res

    result = _run_headless_episode("2wheel", _probe_2wheel_static)
    assert result["res"]["event"] == "static_obstacle_ahead"
    assert result["res"]["obstacle"]["motion"] == "static"
    assert result["raw_debug"]["contact_count"] == 0


def test_cross_dynamic_gate_parks_outside_piston_sweep_during_llm_think_pause():
    def _park_after_gate1(call_fn, out: dict) -> None:
        res = advance_segment(call_fn, max_dist=1.8, speed_m_s=0.52, step_dt=0.10)
        assert res["event"] == "dynamic_gate_ahead"
        g_res = cross_dynamic_gate(call_fn, timeout_s=12.0)
        assert g_res["crossed"] is True
        t_park0 = float(g_res["after_status"]["sim_time"])
        while float(call_fn("/status", "GET")["sim_time"]) - t_park0 < 6.5:
            call_fn("/wheels", "POST", {"left": 0.0, "right": 0.0, "duration": 0.35})
            time.sleep(0.02)
        out["gate_res"] = g_res

    result = _run_headless_episode("dynamic_b", _park_after_gate1)
    assert result["raw_debug"]["contact_count"] == 0
