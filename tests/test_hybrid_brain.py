from __future__ import annotations

from pathlib import Path
import pytest

from robot_sim.topo import (
    NODE_MATCH_RADIUS_M,
    TopologicalOdometer,
    branches_to_cardinals,
    cardinal_to_deg,
    is_cardinal_aligned,
    opposite_cardinal,
    read_topo_studio_summary,
    yaw_to_cardinal,
)


def _mk_status(
    *,
    sim_time: float = 0.0,
    x: float = 0.0,
    y: float = 0.0,
    odom_dist: float = 0.0,
    yaw_deg: float = 0.0,
    rf: dict[str, float] | None = None,
) -> dict:
    default_rf = {
        "left": 0.45,
        "center": 0.32,
        "right": 0.45,
        "left90": 1.40,
        "right90": 0.32,
        "left120": 0.55,
        "right120": 0.42,
    }
    return {
        "sim_time": sim_time,
        "moving": False,
        "sensors": {
            "odom_x_m": x,
            "odom_y_m": y,
            "odom_dist_m": odom_dist,
            "gyro_yaw_deg": yaw_deg,
            "forward_speed_m_s": 0.0,
            "yaw_rate_rad_s": 0.0,
            "rangefinders_m": rf or default_rf,
        },
        "vision": {"target_visible": False, "target": None, "obstacles": []},
        "debug": {"arena": "maze_b", "robot_xy": [x, y]},
    }


def test_topo_cardinal_helpers_and_validation():
    assert NODE_MATCH_RADIUS_M == pytest.approx(0.42)
    assert yaw_to_cardinal(0.0) == "E"
    assert yaw_to_cardinal(89.4) == "N"
    assert yaw_to_cardinal(-179.0) == "W"
    assert yaw_to_cardinal(-90.0) == "S"
    assert cardinal_to_deg("n") == 90.0
    assert opposite_cardinal("E") == "W"
    assert is_cardinal_aligned(14.5) is True
    assert is_cardinal_aligned(53.6) is False
    assert branches_to_cardinals(["left", "straight"], 90.0) == ["W", "N"]
    with pytest.raises(ValueError):
        cardinal_to_deg("NE")
    with pytest.raises(ValueError):
        opposite_cardinal(123)  # type: ignore[arg-type]


def test_topo_rear_blindspot_oblique_guard_and_pending_leg_cancel(tmp_path: Path):
    state_file = tmp_path / ".topo_state.json"
    topo = TopologicalOdometer("http://127.0.0.1:8765", state_path=state_file)

    # t=0 ở hành lang: phía trước (E) và phải (S) bị chặn, trái (N) mở, sau lưng (W) là uninspected
    snap0 = topo.observe_status(_mk_status(sim_time=0.0, x=0.0, y=0.0, yaw_deg=0.0))
    assert snap0["current_node"] == 0
    exits0 = snap0["nodes"][0]["exits"]
    assert exits0["N"]["state"] == "open"
    assert exits0["W"]["state"] == "uninspected"
    assert "E" not in exits0 and "S" not in exits0
    assert "debug" not in str(snap0)
    assert "suggested_exit" not in snap0 and "parent" not in snap0["nodes"][0]

    # Review Focus #1: Xoay chéo +53.6° (lệch > 15° so với trục 90°) không được làm mất "W": "uninspected"
    snap_oblique = topo.observe_status(
        _mk_status(
            sim_time=0.4,
            x=0.0,
            y=0.0,
            yaw_deg=53.6,
            rf={"left": 0.4, "center": 0.30, "right": 0.4, "left90": 0.30, "right90": 0.30},
        )
    )
    assert snap_oblique["nodes"][0]["exits"]["W"]["state"] == "uninspected"

    # Xoay thẳng lên hướng Bắc (+90°): left90 nay nhìn thẳng sang hướng Tây (W = 1.50m -> nâng thành "open")
    snap_north = topo.observe_status(
        _mk_status(
            sim_time=0.8,
            x=0.0,
            y=0.0,
            yaw_deg=90.0,
            rf={"left": 0.8, "center": 1.6, "right": 0.8, "left90": 1.50, "right90": 0.32},
        )
    )
    assert snap_north["nodes"][0]["exits"]["W"]["state"] == "open"

    # Bắt đầu rời Node #0 rồi dừng lửng và gọi cancel_pending_leg -> xóa sạch pending_leg
    topo.begin_leg(_mk_status(sim_time=0.8, x=0.0, y=0.0, yaw_deg=90.0))
    mid = topo.record_progress(0.65, _mk_status(sim_time=2.0, x=0.0, y=0.65, odom_dist=0.65, yaw_deg=90.0))
    assert mid["current_node"] is None and mid["pending_from_node"] == 0
    canceled = topo.cancel_pending_leg(_mk_status(sim_time=2.5, x=0.35, y=0.95, odom_dist=1.10, yaw_deg=25.0))
    assert canceled["pending_from_node"] is None


def test_topo_l_corridor_ports_pending_gate_backtrack_vs_loop_closed_and_autoreset(tmp_path: Path):
    state_file = tmp_path / ".topo_state.json"
    url_a = "http://127.0.0.1:8765"
    url_b = "http://127.0.0.1:9999"
    topo = TopologicalOdometer(url_a, state_path=state_file)

    # Khởi tạo Node #0 với E và N cùng mở
    rf_en = {"left": 0.9, "center": 1.8, "right": 0.4, "left90": 1.5, "right90": 0.32}
    topo.observe_status(_mk_status(sim_time=0.1, x=0.0, y=0.0, odom_dist=0.0, yaw_deg=0.0, rf=rf_en))

    # Rời Node #0 theo cổng E (0°), dừng trước cổng động, vượt cổng, rồi ôm góc chữ L lên hướng Bắc (+90°) vào Node #1 (1.0, 1.2)
    topo.begin_leg(_mk_status(sim_time=0.1, x=0.0, y=0.0, odom_dist=0.0, yaw_deg=0.0, rf=rf_en))
    topo.finish_go(
        "dynamic_gate_ahead",
        0.50,
        _mk_status(sim_time=1.0, x=0.50, y=0.0, odom_dist=0.50, yaw_deg=0.0),
        in_corridor=True,
    )
    topo.record_progress(0.50, _mk_status(sim_time=2.5, x=1.00, y=0.0, odom_dist=1.00, yaw_deg=0.0))
    rf_j1 = {"left": 0.8, "center": 0.32, "right": 0.4, "left90": 1.5, "right90": 1.5}  # W và E mở khi nhìn hướng Bắc
    snap1 = topo.finish_go(
        "junction",
        1.20,
        _mk_status(sim_time=5.0, x=1.00, y=1.20, odom_dist=2.20, yaw_deg=90.0, rf=rf_j1),
        in_corridor=True,
    )
    assert snap1["current_node"] == 1
    assert snap1["loop_closed"] is False
    # Ở Node #0 cổng xuất phát là E; ở Node #1 hướng tới là N nên cổng nhìn ngược lại là S!
    assert snap1["nodes"][0]["exits"]["E"] == {
        "state": "explored",
        "to_node": 1,
        "out_visits": 1,
        "in_visits": 0,
        "dist_m": 2.2,
    }
    assert snap1["nodes"][1]["exits"]["S"] == {
        "state": "explored",
        "to_node": 0,
        "out_visits": 0,
        "in_visits": 1,
        "dist_m": 2.2,
    }

    # Review Focus #2 (a): Đi lùi từ Node #1 qua cổng S về lại Node #0 (khớp <= 0.42m, giữ nguyên neo [0.0, 0.0]) -> loop_closed == False!
    topo.begin_leg(_mk_status(sim_time=5.5, x=1.00, y=1.20, odom_dist=2.20, yaw_deg=-90.0))
    snap_back = topo.finish_go(
        "junction",
        2.18,
        _mk_status(
            sim_time=9.0,
            x=0.18,
            y=0.05,
            odom_dist=4.38,
            yaw_deg=180.0,
            rf={"left": 0.4, "center": 0.32, "right": 0.9, "left90": 0.32, "right90": 1.5},
        ),
        in_corridor=True,
    )
    assert snap_back["current_node"] == 0
    assert snap_back["nodes"][0]["xy"] == [0.0, 0.0]
    assert snap_back["loop_closed"] is False
    assert snap_back["nodes"][1]["exits"]["S"]["out_visits"] == 1
    assert snap_back["nodes"][0]["exits"]["E"]["in_visits"] == 1

    # Review Focus #2 (b): Từ Node #0 đi cổng N (chưa explored) vòng sang cổng W của Node #1 -> loop_closed == True!
    topo.begin_leg(_mk_status(sim_time=9.5, x=0.0, y=0.0, odom_dist=4.38, yaw_deg=90.0))
    snap_loop = topo.finish_go(
        "junction",
        2.20,
        _mk_status(
            sim_time=13.0,
            x=0.95,
            y=1.22,
            odom_dist=6.58,
            yaw_deg=0.0,
            rf={"left": 0.4, "center": 1.4, "right": 0.8, "left90": 0.32, "right90": 1.5},
        ),
        in_corridor=True,
    )
    assert snap_loop["current_node"] == 1
    assert snap_loop["loop_closed"] is True
    assert snap_loop["nodes"][0]["exits"]["N"]["to_node"] == 1
    assert snap_loop["nodes"][1]["exits"]["W"]["to_node"] == 0

    # Mở phiên độc lập trên url_b rồi kiểm tra auto-reset trên url_a khi sim_time/odom quay về 0
    topo_b = TopologicalOdometer(url_b, state_path=state_file)
    topo_b.observe_status(_mk_status(sim_time=1.0, x=0.0, y=0.0))
    assert read_topo_studio_summary(url_a, state_path=state_file)[0] == 2

    snap_reset = topo.observe_status(_mk_status(sim_time=0.05, x=0.0, y=0.0, odom_dist=0.0, yaw_deg=0.0, rf=rf_en))
    assert len(snap_reset["nodes"]) == 1 and snap_reset["current_node"] == 0
    assert TopologicalOdometer.has_active_session(url_b, state_path=state_file) is True


def test_read_topo_studio_summary_handles_missing_and_corrupt_json_gracefully(tmp_path: Path):
    missing = tmp_path / "missing.json"
    assert read_topo_studio_summary("http://127.0.0.1:8765", state_path=missing) == (0, -1, 0)

    corrupt = tmp_path / "corrupt.json"
    corrupt.write_text("{truncated_json: [1, 2", encoding="utf-8")
    assert read_topo_studio_summary("http://127.0.0.1:8765", state_path=corrupt) == (0, -1, 0)

    topo = TopologicalOdometer("http://127.0.0.1:8765", state_path=corrupt)
    topo.observe_status(_mk_status(sim_time=0.2, x=0.0, y=0.0, yaw_deg=0.0))
    node_cnt, cur_id, open_cnt = read_topo_studio_summary("http://127.0.0.1:8765", state_path=corrupt)
    assert node_cnt == 1 and cur_id == 0 and open_cnt == 1


from collections import deque
from robot_sim.skills import cross_dynamic_gate, dock_target, go_to_decision
from tests.test_agent_brain_skills import _run_headless_episode


class _HybridMockBridge:
    """Mock bridge có đầy đủ odom_x_m / odom_y_m / odom_dist_m / gyro_yaw_deg cho kiểm thử đơn vị go_to_decision."""

    def __init__(self, states: list[dict]) -> None:
        self.states = list(states)
        self.idx = 0

    def __call__(self, path: str, method: str = "GET", payload: dict | None = None) -> dict:
        if path.startswith("/status"):
            raw = self.states[min(self.idx, len(self.states) - 1)]
            if self.idx < len(self.states) - 1:
                self.idx += 1
            return {
                "sim_time": float(raw.get("sim_time", 0.0)),
                "moving": bool(raw.get("moving", False)),
                "sensors": {
                    "odom_x_m": float(raw.get("odom_x_m", 0.0)),
                    "odom_y_m": float(raw.get("odom_y_m", 0.0)),
                    "odom_dist_m": float(raw.get("odom_dist_m", 0.0)),
                    "gyro_yaw_deg": float(raw.get("gyro_yaw_deg", 0.0)),
                    "forward_speed_m_s": 0.0,
                    "yaw_rate_rad_s": 0.0,
                    "rangefinders_m": dict(
                        raw.get(
                            "rangefinders_m",
                            {"left": 1.0, "center": 2.0, "right": 1.0, "left90": 0.45, "right90": 0.45},
                        )
                    ),
                },
                "vision": dict(raw.get("vision", {"target_visible": False, "target": None, "obstacles": []})),
                "debug": {"arena": "mock", "robot_xy": [0.0, 0.0]},
            }
        return {"ok": True, "moving": False}


def test_go_to_decision_validates_args_and_handles_open_junction_long_corridor_and_zero_move_guard():
    dummy = _HybridMockBridge([{}])
    with pytest.raises(ValueError):
        go_to_decision(dummy, cardinal="E", heading_deg=90.0)
    with pytest.raises(ValueError):
        go_to_decision(dummy, max_dist=0.0)
    with pytest.raises(ValueError):
        go_to_decision(dummy, max_chain=0)

    # Review Focus #3: Xuất phát giữa ngã 4 mở (left90=1.2, right90=1.2), đi 2.3m trong hành lang hẹp tới góc L (2.30, 0.0) ["left"],
    # tự động xoay +90° sang đoạn 2; ở đoạn 2 moved_m=0.05 (< 0.20m) gặp ["left"] -> dừng ngay không xoay lần 2!
    states = [
        # observe_status đầu lệnh tại (0, 0)
        {
            "sim_time": 0.0,
            "odom_x_m": 0.0,
            "odom_y_m": 0.0,
            "odom_dist_m": 0.0,
            "gyro_yaw_deg": 0.0,
            "rangefinders_m": {"left": 1.1, "center": 2.8, "right": 1.1, "left90": 1.2, "right90": 1.2, "left120": 0.9, "right120": 0.9},
        },
        # seg 1 bắt đầu tại ngã 4 mở -> vào hành lang (left90=0.40, right90=0.40) suốt 2.30m -> góc chữ L mở trái tại (2.30, 0.0)
        {
            "sim_time": 0.1,
            "odom_x_m": 0.0,
            "odom_y_m": 0.0,
            "odom_dist_m": 0.0,
            "gyro_yaw_deg": 0.0,
            "rangefinders_m": {"left": 1.1, "center": 2.8, "right": 1.1, "left90": 1.2, "right90": 1.2, "left120": 0.9, "right120": 0.9},
        },
        {
            "sim_time": 0.5,
            "odom_x_m": 0.9,
            "odom_y_m": 0.0,
            "odom_dist_m": 0.9,
            "gyro_yaw_deg": 0.0,
            "rangefinders_m": {"left": 0.8, "center": 1.9, "right": 0.8, "left90": 0.40, "right90": 0.40, "left120": 0.45, "right120": 0.45},
        },
        {
            "sim_time": 1.0,
            "odom_x_m": 1.95,
            "odom_y_m": 0.0,
            "odom_dist_m": 1.95,
            "gyro_yaw_deg": 0.0,
            "rangefinders_m": {"left": 0.8, "center": 0.85, "right": 0.8, "left90": 0.40, "right90": 0.40, "left120": 0.45, "right120": 0.45},
        },
        {
            "sim_time": 1.3,
            "odom_x_m": 2.30,
            "odom_y_m": 0.0,
            "odom_dist_m": 2.30,
            "gyro_yaw_deg": 0.0,
            "moving": False,
            "rangefinders_m": {"left": 0.5, "center": 0.34, "right": 0.4, "left90": 1.40, "right90": 0.36, "left120": 0.85, "right120": 0.42},
        },
        {
            "sim_time": 1.4,
            "odom_x_m": 2.30,
            "odom_y_m": 0.0,
            "odom_dist_m": 2.30,
            "gyro_yaw_deg": 0.0,
            "moving": False,
            "rangefinders_m": {"left": 0.5, "center": 0.34, "right": 0.4, "left90": 1.40, "right90": 0.36, "left120": 0.85, "right120": 0.42},
        },
        # pha closed_loop_turn +90° sang hướng Bắc tại (2.30, 0.0)
        {
            "sim_time": 1.6,
            "odom_x_m": 2.30,
            "odom_y_m": 0.0,
            "odom_dist_m": 2.30,
            "gyro_yaw_deg": 90.0,
            "moving": False,
            "rangefinders_m": {"left": 0.5, "center": 1.40, "right": 0.5, "left90": 0.36, "right90": 0.36, "left120": 0.42, "right120": 0.42},
        },
        {
            "sim_time": 1.7,
            "odom_x_m": 2.30,
            "odom_y_m": 0.0,
            "odom_dist_m": 2.30,
            "gyro_yaw_deg": 90.0,
            "moving": False,
            "rangefinders_m": {"left": 0.5, "center": 1.40, "right": 0.5, "left90": 0.36, "right90": 0.36, "left120": 0.42, "right120": 0.42},
        },
        # seg 2 chỉ nhích 0.05m rồi bị chặn trước với ["left"] -> moved_m < 0.20m nên phải dừng auto-chain ngay!
        {
            "sim_time": 1.8,
            "odom_x_m": 2.30,
            "odom_y_m": 0.05,
            "odom_dist_m": 2.35,
            "gyro_yaw_deg": 90.0,
            "moving": False,
            "rangefinders_m": {"left": 0.5, "center": 0.34, "right": 0.4, "left90": 1.30, "right90": 0.36, "left120": 0.85, "right120": 0.42},
        },
        {
            "sim_time": 1.9,
            "odom_x_m": 2.30,
            "odom_y_m": 0.05,
            "odom_dist_m": 2.35,
            "gyro_yaw_deg": 90.0,
            "moving": False,
            "rangefinders_m": {"left": 0.5, "center": 0.34, "right": 0.4, "left90": 1.30, "right90": 0.36, "left120": 0.85, "right120": 0.42},
        },
    ]
    topo = TopologicalOdometer(auto_persist=False)
    res = go_to_decision(_HybridMockBridge(states), max_dist=6.0, step_dt=0.05, topo=topo)
    assert res["segments_chained"] == 2
    assert res["auto_l_turns"] == ["left"]
    assert res["moved_m"] >= 2.25
    assert res["open_cardinals"] == ["W"]
    assert "debug" not in res["after_status"]


def _choose_next_cardinal_from_topo(topo_dict: dict) -> str:
    """Bộ lập lịch System 2 mẫu trong kiểm thử chỉ đọc thuần JSON topo (ưu tiên open -> BFS trên cạnh out_visits == 0 -> uninspected)."""
    nodes = {n["id"]: n for n in topo_dict["nodes"]}
    cur = topo_dict["current_node"]
    assert cur is not None
    # 1. Cổng "open" ngay tại nút hiện tại
    for port, info in nodes[cur]["exits"].items():
        if info["state"] == "open":
            return port
    # 2. BFS tìm nút gần nhất còn cổng "open", sau đó mới xét "uninspected"; ưu tiên cạnh explored có out_visits == 0
    for target_state in ("open", "uninspected"):
        if target_state == "uninspected" and "uninspected" in {i["state"] for i in nodes[cur]["exits"].values()}:
            return next(p for p, i in nodes[cur]["exits"].items() if i["state"] == "uninspected")
        for allow_repeat in (False, True):
            q: deque[tuple[int, str | None]] = deque([(cur, None)])
            seen = {cur}
            while q:
                u, first_port = q.popleft()
                if u != cur and any(i["state"] == target_state for i in nodes[u]["exits"].values()):
                    assert first_port is not None
                    return first_port
                for port, info in nodes[u]["exits"].items():
                    if info["state"] == "explored" and info["to_node"] is not None:
                        if not allow_repeat and info["out_visits"] >= 1:
                            continue
                        nxt = int(info["to_node"])
                        if nxt not in seen:
                            seen.add(nxt)
                            q.append((nxt, port if first_port is None else first_port))
    raise AssertionError(f"Không tìm thấy nhánh đi tiếp trên đồ thị: {topo_dict}")


@pytest.mark.parametrize("arena_name,max_decisions", [("maze_a", 10), ("maze_b", 12), ("maze_c", 18), ("dynamic_b", 12)])
def test_hybrid_brain_solves_mazes_and_dynamic_s_curve_with_zero_contacts(arena_name: str, max_decisions: int):
    def _driver(call_fn, out: dict) -> None:
        topo = TopologicalOdometer(auto_persist=False)
        snap = topo.observe_status(call_fn("/status?vision=1", "GET"))
        decisions = 0
        dock_res = None
        next_card: str | None = None
        if snap["current_node"] is not None and snap["nodes"][0]["kind"] != "open_space":
            next_card = _choose_next_cardinal_from_topo(snap)

        for _ in range(max_decisions):
            decisions += 1
            res = go_to_decision(call_fn, cardinal=next_card, max_dist=6.0, speed_m_s=0.55, topo=topo)
            assert "debug" not in res["after_status"]
            next_card = None
            if res["event"] == "target_ready":
                dock_res = dock_target(call_fn, stop_distance=0.25)
                if dock_res["reached"]:
                    break
            elif res["event"] == "dynamic_gate_ahead":
                g_res = cross_dynamic_gate(call_fn, timeout_s=12.0)
                assert g_res["crossed"] is True
                topo.record_progress(g_res["moved_m"], g_res["after_status"])
                yaw_g = float(g_res["after_status"]["sensors"]["gyro_yaw_deg"])
                cards_g = branches_to_cardinals(g_res["open_branches"], yaw_g)
                if "straight" not in g_res["open_branches"] and cards_g:
                    next_card = cards_g[0]
            elif res["topo"]["current_node"] is not None:
                next_card = _choose_next_cardinal_from_topo(res["topo"])

        out["decisions"] = decisions
        out["dock"] = dock_res

    result = _run_headless_episode(arena_name, _driver)
    assert result["dock"] is not None and result["dock"]["reached"] is True
    assert result["raw_debug"]["contact_count"] == 0


def test_topo_reverse_motion_does_not_wipe_graph(tmp_path: Path):
    """Critical review fix: signed odom_dist_m decreasing during reverse backup while sim_time advances must NOT reset graph."""
    state_file = tmp_path / ".topo_state.json"
    topo = TopologicalOdometer("http://127.0.0.1:8765", state_path=state_file)
    rf_open = {"left": 0.9, "center": 1.8, "right": 0.9, "left90": 0.35, "right90": 0.35}
    topo.observe_status(_mk_status(sim_time=0.1, x=0.0, y=0.0, odom_dist=0.0, yaw_deg=0.0, rf=rf_open))
    topo.begin_leg(_mk_status(sim_time=0.1, x=0.0, y=0.0, odom_dist=0.0, yaw_deg=0.0, rf=rf_open))
    snap1 = topo.finish_go(
        "junction",
        1.20,
        _mk_status(
            sim_time=3.0,
            x=1.20,
            y=0.0,
            odom_dist=1.20,
            yaw_deg=0.0,
            rf={"left": 0.6, "center": 0.34, "right": 0.6, "left90": 1.4, "right90": 1.4},
        ),
        in_corridor=True,
    )
    assert len(snap1["nodes"]) == 2

    # Robot reverses -0.25m (signed odom_dist drops 1.20 -> 0.95 while sim_time advances 3.0 -> 4.2)
    snap_rev = topo.observe_status(
        _mk_status(sim_time=4.2, x=0.95, y=0.0, odom_dist=0.95, yaw_deg=0.0, rf=rf_open)
    )
    assert len(snap_rev["nodes"]) == 2
    assert snap_rev["nodes"][0]["exits"]["E"]["to_node"] == 1
    assert snap_rev["nodes"][1]["exits"]["W"]["to_node"] == 0


def test_topo_short_self_return_does_not_overwrite_parent_edge(tmp_path: Path):
    """Important review fix: u == dest_id with 0.25 <= d < 0.80 must NOT create self-loop or overwrite parent exit."""
    state_file = tmp_path / ".topo_state.json"
    topo = TopologicalOdometer("http://127.0.0.1:8765", state_path=state_file)
    rf_corr = {"left": 0.8, "center": 1.8, "right": 0.8, "left90": 0.35, "right90": 0.35}
    topo.observe_status(_mk_status(sim_time=0.1, x=0.0, y=0.0, odom_dist=0.0, yaw_deg=0.0, rf=rf_corr))
    topo.begin_leg(_mk_status(sim_time=0.1, x=0.0, y=0.0, odom_dist=0.0, yaw_deg=0.0, rf=rf_corr))
    rf_j1 = {"left": 0.8, "center": 0.9, "right": 0.8, "left90": 1.5, "right90": 0.35}
    topo.finish_go(
        "junction",
        1.20,
        _mk_status(sim_time=2.5, x=1.20, y=0.0, odom_dist=1.20, yaw_deg=0.0, rf=rf_j1),
        in_corridor=True,
    )

    # Leave Node #1 heading East, stop 0.32m later still within 0.42m of Node #1 -> must NOT overwrite Node #1["W"] -> 0!
    topo.begin_leg(_mk_status(sim_time=2.6, x=1.20, y=0.0, odom_dist=1.20, yaw_deg=0.0, rf=rf_j1))
    snap = topo.finish_go(
        "junction",
        0.32,
        _mk_status(
            sim_time=3.4,
            x=1.52,
            y=0.0,
            odom_dist=1.52,
            yaw_deg=0.0,
            rf={"left": 0.4, "center": 0.34, "right": 0.4, "left90": 1.5, "right90": 0.35},
        ),
        in_corridor=True,
    )
    assert snap["current_node"] == 1
    assert snap["nodes"][1]["exits"]["W"]["to_node"] == 0
    assert snap["nodes"][1]["exits"].get("E", {}).get("to_node") != 1

    # Minor #1: phantom "open" front exit is deleted at 0.50 <= rf_center < 0.62 (matching compute_open_branches)
    topo.nodes[1]["exits"]["E"] = {"state": "open", "to_node": None, "out_visits": 0, "in_visits": 0, "dist_m": None}
    snap_prune = topo.observe_status(
        _mk_status(
            sim_time=3.8,
            x=1.20,
            y=0.0,
            odom_dist=1.52,
            yaw_deg=0.0,
            rf={"left": 0.45, "center": 0.56, "right": 0.45, "left90": 1.5, "right90": 0.35},
        )
    )
    assert "E" not in snap_prune["nodes"][1]["exits"]


def test_go_to_decision_small_positive_max_dist_does_not_assert():
    """Re-graded Important fix: valid small positive max_dist <= 0.02 must return cleanly instead of AssertionError."""
    states = [
        {"sim_time": 0.0, "odom_x_m": 0.0, "odom_y_m": 0.0, "odom_dist_m": 0.0, "gyro_yaw_deg": 0.0},
        {"sim_time": 0.1, "odom_x_m": 0.01, "odom_y_m": 0.0, "odom_dist_m": 0.01, "gyro_yaw_deg": 0.0, "moving": False},
    ]
    res = go_to_decision(_HybridMockBridge(states), max_dist=0.01, step_dt=0.05)
    assert res["event"] in {"max_dist", "junction", "dead_end", "target_ready"}
    assert "debug" not in res["after_status"]


def test_studio_sim_plugin_poll_topo_summary_throttles_and_survives_corrupt_file(tmp_path: Path, monkeypatch):
    """Important review fix + Task 4: runtime unit test for SimPlugin._poll_topo_summary() 4-tuple, goal_id, keep_topo reset, and corrupt JSON."""
    import sys
    from robot_sim.topo import read_topo_studio_details

    studio_dir = str((Path(__file__).resolve().parents[1] / "studio").resolve())
    if studio_dir not in sys.path:
        sys.path.insert(0, studio_dir)
    import messages as studio_msgs
    import sim_plugin as sp_mod

    assert studio_msgs.ResetRobotEvent().keep_topo is False
    assert studio_msgs.ResetRobotEvent(keep_topo=True).keep_topo is True
    assert studio_msgs.RobotTelemetry().topo_goal_node == -1

    state_file = tmp_path / ".topo_state.json"
    monkeypatch.setattr(sp_mod, "DEFAULT_TOPO_STATE_PATH", state_file)
    monkeypatch.setattr(
        sp_mod,
        "read_topo_studio_details",
        lambda base_url="http://127.0.0.1:8765": read_topo_studio_details(base_url, state_path=state_file),
    )

    plugin_inst = sp_mod.SimPlugin()
    plugin_inst._topo_poll_wall = -10.0

    # 1. Missing file -> (0, -1, 0, -1)
    assert plugin_inst._poll_topo_summary() == (0, -1, 0, -1)

    # 2. Write valid state file within 0.5s window -> throttled, still returns cached (0, -1, 0, -1)
    topo = TopologicalOdometer("http://127.0.0.1:8765", state_path=state_file)
    rf_cor = {"left": 0.9, "center": 1.8, "right": 0.9, "left90": 0.35, "right90": 0.35}
    topo.observe_status(_mk_status(sim_time=0.2, x=0.0, y=0.0, yaw_deg=0.0, rf=rf_cor))
    assert plugin_inst._poll_topo_summary() == (0, -1, 0, -1)

    # 3. After >= 0.5s elapsed -> picks up (1, 0, 1, -1)
    plugin_inst._topo_poll_wall -= 0.6
    assert plugin_inst._poll_topo_summary() == (1, 0, 1, -1)

    # 3a. Anchor a goal node -> reports goal_id == 1
    topo.begin_leg(_mk_status(sim_time=0.3, x=0.0, y=0.0, odom_dist=0.0, yaw_deg=0.0, rf=rf_cor))
    topo.finish_go("target_ready", 1.0, _mk_status(sim_time=2.0, x=1.0, y=0.0, odom_dist=1.0, yaw_deg=0.0, rf=rf_cor), in_corridor=True)
    topo.record_dock(0.3, _mk_status(sim_time=2.5, x=1.3, y=0.0, odom_dist=1.3, yaw_deg=0.0, rf={"left": 0.7, "center": 0.27, "right": 0.7, "left90": 0.35, "right90": 0.35}), reached=True)
    plugin_inst._topo_poll_wall -= 0.6
    plugin_inst._topo_mtime_ns = -1
    assert plugin_inst._poll_topo_summary() == (2, 1, 0, 1)

    # 3b. keep_topo=True reset preserves nodes + goal_id == 1 and zeroes visits
    plugin_inst._sync_topo_on_ui_reset(keep_topo=True, status=_mk_status(sim_time=0.0, x=0.0, y=0.0, odom_dist=0.0, yaw_deg=0.0, rf=rf_cor))
    assert plugin_inst._topo_summary == (2, 0, 0, 1)
    reloaded = TopologicalOdometer("http://127.0.0.1:8765", state_path=state_file)
    assert reloaded.nodes[0]["exits"]["E"]["out_visits"] == 0
    assert reloaded.nodes[1]["kind"] == "goal"

    # 3c. keep_topo=False reset deletes session and resets summary to (0, -1, 0, -1)
    plugin_inst._sync_topo_on_ui_reset(keep_topo=False, status=None)
    assert not TopologicalOdometer.has_active_session("http://127.0.0.1:8765", state_path=state_file)
    plugin_inst._topo_poll_wall -= 0.6
    assert plugin_inst._poll_topo_summary() == (0, -1, 0, -1)

    # 4. Corrupt JSON after >= 0.5s -> no exception, safe fallback (0, -1, 0, -1)
    state_file.write_text("{broken", encoding="utf-8")
    plugin_inst._topo_poll_wall -= 0.6
    plugin_inst._topo_mtime_ns = -1
    assert plugin_inst._poll_topo_summary() == (0, -1, 0, -1)


def test_topo_passthrough_of_known_intermediate_node_does_not_orphan_it(tmp_path: Path):
    """Regression test: rolling straight past an already-known intermediate Node #2 on backtrack must not sever Node #2."""
    state_file = tmp_path / ".topo_state.json"
    topo = TopologicalOdometer("http://127.0.0.1:8765", state_path=state_file)
    rf_w = {"left": 0.4, "center": 1.6, "right": 0.4, "left90": 0.35, "right90": 0.35}
    topo.observe_status(_mk_status(sim_time=0.0, x=0.0, y=0.0, odom_dist=0.0, yaw_deg=180.0, rf=rf_w))

    # Node #0 --W--> Node #1 (-0.8, 0.0) with S open
    topo.begin_leg(_mk_status(sim_time=0.1, x=0.0, y=0.0, odom_dist=0.0, yaw_deg=180.0, rf=rf_w))
    rf_j1 = {"left": 0.8, "center": 1.6, "right": 0.4, "left90": 1.5, "right90": 0.35}
    topo.finish_go(
        "junction",
        0.80,
        _mk_status(sim_time=1.5, x=-0.80, y=0.0, odom_dist=0.80, yaw_deg=180.0, rf=rf_j1),
        in_corridor=True,
    )

    # Node #1 --W--> Node #2 (-0.9, -1.5) dead_end
    topo.begin_leg(_mk_status(sim_time=1.6, x=-0.80, y=0.0, odom_dist=0.80, yaw_deg=180.0, rf=rf_j1))
    topo.finish_go(
        "dead_end",
        2.80,
        _mk_status(
            sim_time=6.0,
            x=-0.90,
            y=-1.50,
            odom_dist=3.60,
            yaw_deg=0.0,
            rf={"left": 0.35, "center": 0.32, "right": 0.35, "left90": 0.35, "right90": 0.35},
        ),
        in_corridor=True,
    )

    # Backtrack from Node #2 heading W, accidentally rolling past Node #1 all the way back to Node #0 heading E
    topo.begin_leg(_mk_status(sim_time=6.1, x=-0.90, y=-1.50, odom_dist=3.60, yaw_deg=180.0))
    snap = topo.finish_go(
        "junction",
        3.55,
        _mk_status(
            sim_time=11.0,
            x=-0.10,
            y=-0.02,
            odom_dist=7.15,
            yaw_deg=0.0,
            rf={"left": 0.4, "center": 0.32, "right": 0.4, "left90": 1.4, "right90": 0.35},
        ),
        in_corridor=True,
    )
    assert snap["current_node"] == 0
    assert snap["loop_closed"] is False
    # Intermediate Node #1 MUST remain connected to both Node #0 and Node #2 (never orphaned!)
    assert snap["nodes"][0]["exits"]["W"]["to_node"] == 1
    assert snap["nodes"][2]["exits"]["W"]["to_node"] == 1
    assert snap["nodes"][1]["exits"]["S"]["state"] == "open"
    assert snap["nodes"][1]["exits"]["E"]["out_visits"] == 1
    assert snap["nodes"][0]["exits"]["W"]["in_visits"] == 1


def test_cross_dynamic_gate_passes_when_piston_retracted_at_edge():
    """When a piston gate is already retracted into its side pocket (|offset_x| >= 0.85) with clear center/sides, cross-gate must cross instead of timing out."""
    sim_t = 0.0
    odom_d = 0.0

    def fake_call(path: str, method: str, body: dict | None = None):
        nonlocal sim_t, odom_d
        if path == "/wheels":
            sim_t += 0.25
            if body and float(body.get("left", 0.0)) > 1.0:
                odom_d += 0.14
            return {"ok": True}
        if path == "/stop":
            return {"stopped": True, "sim_time": sim_t}
        if path.startswith("/status"):
            return {
                "sim_time": round(sim_t, 3),
                "moving": False,
                "sensors": {
                    "rangefinders_m": {
                        "left": 0.49,
                        "center": 1.31 if odom_d < 0.7 else 0.48,
                        "right": 1.04,
                        "left90": 0.53,
                        "right90": 0.39,
                        "left120": 1.04,
                        "right120": 0.47,
                    },
                    "gyro_yaw_deg": 88.4,
                    "odom_dist_m": round(odom_d, 4),
                    "odom_x_m": 1.72,
                    "odom_y_m": round(odom_d, 4),
                    "forward_speed_m_s": 0.0,
                },
                "vision": {
                    "target_visible": False,
                    "target": None,
                    "obstacles": [
                        {
                            "color": "blue",
                            "offset_x": -0.92,
                            "side": "left",
                            "area_pct": 2.45,
                            "v_span": 0.14,
                        }
                    ]
                    if odom_d < 0.3
                    else [],
                },
            }
        raise AssertionError(f"Unexpected call: {path}")

    res = cross_dynamic_gate(fake_call, timeout_s=4.0)
    assert res["crossed"] is True
    assert res["moved_m"] >= 0.65


def test_topo_record_dock_uses_approach_yaw_and_connects_open_space_without_pending_leg(tmp_path: Path):
    """Task 1: dock at oblique angle uses target_ready corridor approach_yaw_deg; open_space connects Node #0 <-> #G even if pending_leg was cancelled."""
    from robot_sim.topo import read_topo_studio_details

    # 1. Corridor approach at yaw=0° -> oblique dock at yaw=+28°
    state_file = tmp_path / ".topo_corridor.json"
    topo = TopologicalOdometer("http://127.0.0.1:8765", state_path=state_file)
    rf_start = {"left": 0.9, "center": 1.8, "right": 0.9, "left90": 0.35, "right90": 0.35}
    topo.observe_status(_mk_status(sim_time=0.1, x=0.0, y=0.0, odom_dist=0.0, yaw_deg=0.0, rf=rf_start))
    topo.begin_leg(_mk_status(sim_time=0.1, x=0.0, y=0.0, odom_dist=0.0, yaw_deg=0.0, rf=rf_start))
    snap_ready = topo.finish_go(
        "target_ready",
        1.20,
        _mk_status(
            sim_time=2.5,
            x=1.20,
            y=0.0,
            odom_dist=1.20,
            yaw_deg=0.0,
            rf={"left": 0.9, "center": 0.72, "right": 0.9, "left90": 0.40, "right90": 0.40},
        ),
        in_corridor=True,
    )
    assert snap_ready["pending_from_node"] == 0

    # Unsuccessful dock must preserve pending_leg and create no goal node
    snap_fail = topo.record_dock(
        0.10,
        _mk_status(
            sim_time=2.8,
            x=1.30,
            y=0.02,
            odom_dist=1.30,
            yaw_deg=12.0,
            rf={"left": 0.8, "center": 0.60, "right": 0.8, "left90": 0.40, "right90": 0.40},
        ),
        reached=False,
    )
    assert len(snap_fail["nodes"]) == 1
    assert snap_fail["pending_from_node"] == 0

    # Successful dock at oblique +28° (still anchors goal using approach_yaw=0° -> p_in='W')
    snap_goal = topo.record_dock(
        0.45,
        _mk_status(
            sim_time=3.5,
            x=1.65,
            y=0.18,
            odom_dist=1.65,
            yaw_deg=28.0,
            rf={"left": 0.7, "center": 0.27, "right": 0.7, "left90": 0.45, "right90": 0.45},
        ),
        reached=True,
    )
    assert snap_goal["current_node"] == 1
    assert snap_goal["pending_from_node"] is None
    assert len(snap_goal["nodes"]) == 2
    assert snap_goal["nodes"][1]["kind"] == "goal"
    assert snap_goal["nodes"][0]["exits"]["E"]["to_node"] == 1
    assert snap_goal["nodes"][1]["exits"]["W"]["to_node"] == 0
    assert snap_goal["nodes"][1]["exits"]["W"]["dist_m"] == pytest.approx(1.65, abs=0.01)
    assert "E" not in snap_goal["nodes"][1]["exits"]
    assert read_topo_studio_details("http://127.0.0.1:8765", state_path=state_file) == (2, 1, 0, 1)

    # 2. Open-space start where static-obstacle detour cancelled pending_leg before dock
    state_open = tmp_path / ".topo_open.json"
    topo_open = TopologicalOdometer("http://127.0.0.1:8765", state_path=state_open)
    rf_open = {"left": 2.0, "center": 2.5, "right": 2.0, "left90": 1.8, "right90": 1.8}
    topo_open.observe_status(_mk_status(sim_time=0.1, x=0.0, y=0.0, odom_dist=0.0, yaw_deg=0.0, rf=rf_open))
    assert topo_open.is_open_space_start() is True
    topo_open.cancel_pending_leg(_mk_status(sim_time=2.0, x=1.5, y=0.4, odom_dist=1.8, yaw_deg=-25.0, rf=rf_open))
    snap_open_goal = topo_open.record_dock(
        0.35,
        _mk_status(
            sim_time=3.0,
            x=2.80,
            y=0.10,
            odom_dist=3.10,
            yaw_deg=-5.0,
            rf={"left": 1.2, "center": 0.26, "right": 1.2, "left90": 1.6, "right90": 1.6},
        ),
        reached=True,
    )
    assert snap_open_goal["current_node"] == 1
    assert snap_open_goal["nodes"][1]["kind"] == "goal"
    assert snap_open_goal["nodes"][0]["exits"]["E"]["to_node"] == 1
    assert snap_open_goal["nodes"][1]["exits"]["W"]["to_node"] == 0


def test_topo_reset_for_repass_preserves_map_zeros_visits_and_handles_target_change(tmp_path: Path):
    """Task 1: reset_for_repass keeps nodes/edges, zeroes visit counts, prevents auto-reset wipe at sim_time=0, and downgrades goal on target_changed."""
    from robot_sim.topo import read_topo_studio_details

    state_file = tmp_path / ".topo_repass.json"
    topo = TopologicalOdometer("http://127.0.0.1:8765", state_path=state_file)
    rf_cor = {"left": 0.9, "center": 1.8, "right": 0.9, "left90": 0.35, "right90": 0.35}
    topo.observe_status(_mk_status(sim_time=0.1, x=0.0, y=0.0, odom_dist=0.0, yaw_deg=0.0, rf=rf_cor))
    topo.begin_leg(_mk_status(sim_time=0.1, x=0.0, y=0.0, odom_dist=0.0, yaw_deg=0.0, rf=rf_cor))
    topo.finish_go(
        "junction",
        1.20,
        _mk_status(
            sim_time=3.0,
            x=1.20,
            y=0.0,
            odom_dist=1.20,
            yaw_deg=0.0,
            rf={"left": 0.8, "center": 0.32, "right": 0.8, "left90": 1.5, "right90": 0.35},
        ),
        in_corridor=True,
    )
    topo.begin_leg(_mk_status(sim_time=3.1, x=1.20, y=0.0, odom_dist=1.20, yaw_deg=90.0))
    topo.finish_go(
        "target_ready",
        1.00,
        _mk_status(sim_time=5.0, x=1.20, y=1.00, odom_dist=2.20, yaw_deg=90.0, rf=rf_cor),
        in_corridor=True,
    )
    topo.record_dock(
        0.40,
        _mk_status(
            sim_time=6.0,
            x=1.20,
            y=1.40,
            odom_dist=2.60,
            yaw_deg=90.0,
            rf={"left": 0.7, "center": 0.26, "right": 0.7, "left90": 0.35, "right90": 0.35},
        ),
        reached=True,
    )
    assert read_topo_studio_details("http://127.0.0.1:8765", state_path=state_file)[3] == 2

    # Repass reset with same target
    snap_repass = topo.reset_for_repass(
        _mk_status(sim_time=0.0, x=0.0, y=0.0, odom_dist=0.0, yaw_deg=0.0, rf=rf_cor),
        target_changed=False,
    )
    assert snap_repass["current_node"] == 0
    assert len(snap_repass["nodes"]) == 3
    assert snap_repass["nodes"][2]["kind"] == "goal"
    for node in snap_repass["nodes"]:
        for info in node["exits"].values():
            assert info["out_visits"] == 0
            assert info["in_visits"] == 0

    # Reload from disk and observe at sim_time=0.1 -> MUST NOT auto-reset wipe!
    topo2 = TopologicalOdometer("http://127.0.0.1:8765", state_path=state_file)
    snap_after = topo2.observe_status(_mk_status(sim_time=0.1, x=0.0, y=0.0, odom_dist=0.0, yaw_deg=0.0, rf=rf_cor))
    assert len(snap_after["nodes"]) == 3
    assert snap_after["nodes"][2]["kind"] == "goal"

    # Repass reset with target_changed=True downgrades stale goal node
    snap_reloc = topo2.reset_for_repass(
        _mk_status(sim_time=0.0, x=0.0, y=0.0, odom_dist=0.0, yaw_deg=0.0, rf=rf_cor),
        target_changed=True,
    )
    assert snap_reloc["nodes"][2]["kind"] == "dead_end"
    assert read_topo_studio_details("http://127.0.0.1:8765", state_path=state_file)[3] == -1


def test_dock_target_reports_moved_m():
    """Task 1: dock_target returns moved_m on both refusal and reached paths."""
    from robot_sim.skills import dock_target

    # Refusal path
    st_far = {
        "sim_time": 1.0,
        "moving": False,
        "sensors": {"rangefinders_m": {"left": 1.5, "center": 1.5, "right": 1.5}, "odom_dist_m": 2.0},
        "vision": {"target_visible": False, "target": None, "obstacles": []},
    }
    res_ref = dock_target(lambda p, m, b=None: st_far)
    assert res_ref["reached"] is False
    assert res_ref["moved_m"] == 0.0

    # Reached path
    odom_val = {"d": 2.0}

    def fake_dock_call(path: str, method: str, body: dict | None = None):
        if path == "/wheels":
            if body and float(body.get("left", 0.0)) > 0.1:
                odom_val["d"] += 0.20
            return {"ok": True}
        if path == "/stop":
            return {"stopped": True, "sim_time": 1.5}
        return {
            "sim_time": 1.5,
            "moving": False,
            "sensors": {
                "rangefinders_m": {
                    "left": 0.9,
                    "center": 0.42 if odom_val["d"] < 2.15 else 0.26,
                    "right": 0.9,
                },
                "odom_dist_m": round(odom_val["d"], 4),
                "forward_speed_m_s": 0.0,
                "yaw_rate_rad_s": 0.0,
                "gyro_yaw_deg": 0.0,
            },
            "vision": {
                "target_visible": True,
                "target": {"offset_x": 0.02, "side": "center", "area_pct": 12.0, "v_span": 0.45},
                "obstacles": [],
            },
        }

    res_ok = dock_target(fake_dock_call, stop_distance=0.25)
    assert res_ok["reached"] is True
    assert res_ok["moved_m"] == pytest.approx(0.20, abs=0.01)


def test_go_to_decision_route_validation_anti_overshoot_blocked_hop_and_stop_at_node(tmp_path: Path):
    """Task 2: route validation, intermediate waypoint anti-overshoot braking, route_blocked, ignore target_ready on return, and open-space stop_at_node."""
    import math
    from robot_sim.skills import go_to_decision

    dummy_st = _mk_status(sim_time=0.1, x=0.0, y=0.0, odom_dist=0.0, yaw_deg=0.0)
    topo_v = TopologicalOdometer("http://127.0.0.1:8765", state_path=tmp_path / ".v.json")
    topo_v.observe_status(dummy_st)
    for bad_kwargs in (
        {"cardinal": "E", "route": "E,N"},
        {"heading_deg": 0.0, "route": "E"},
        {"route": ""},
        {"route": "E,X"},
        {"stop_at_node": -1},
        {"stop_at_node": True},
        {"stop_at_node": 99},
    ):
        with pytest.raises(ValueError):
            go_to_decision(lambda p, m, b=None: dummy_st, topo=topo_v, **bad_kwargs)

    # Helper stateful fake robot for pure kinematics unit verification
    def _make_kinematic_world(
        *,
        x0: float,
        y0: float,
        yaw0: float,
        rf_fn,
        vis_fn=None,
        t0: float = 1.0,
        odom0: float = 0.0,
    ):
        state = {"t": t0, "x": x0, "y": y0, "yaw": yaw0, "odom": odom0}

        def call_fn(path: str, method: str, body: dict | None = None):
            if path == "/wheels" and body:
                l_w = float(body.get("left", 0.0))
                r_w = float(body.get("right", 0.0))
                dt = 0.08
                state["t"] += dt
                v_lin = 0.5 * (l_w + r_w) * 0.05
                w_yaw_deg = math.degrees((r_w - l_w) * 0.05 / 0.23)
                state["yaw"] += w_yaw_deg * dt
                rad = math.radians(state["yaw"])
                dx = v_lin * math.cos(rad) * dt
                dy = v_lin * math.sin(rad) * dt
                state["x"] += dx
                state["y"] += dy
                state["odom"] += v_lin * dt
                return {"ok": True}
            if path == "/stop":
                return {"stopped": True, "sim_time": round(state["t"], 3)}
            rf = rf_fn(state)
            vis = vis_fn(state) if vis_fn else {"target_visible": False, "target": None, "obstacles": []}
            return {
                "sim_time": round(state["t"], 3),
                "moving": False,
                "sensors": {
                    "rangefinders_m": rf,
                    "gyro_yaw_deg": round(state["yaw"], 3),
                    "odom_dist_m": round(state["odom"], 4),
                    "odom_x_m": round(state["x"], 4),
                    "odom_y_m": round(state["y"], 4),
                    "forward_speed_m_s": 0.0,
                    "yaw_rate_rad_s": 0.0,
                },
                "vision": vis,
            }

        return state, call_fn

    # 1. Multi-hop route="E,N" where Node #1=(1.0,0.0) has NO side-wall opening cue (anti-overshoot next_node_xy brakes at Node #1!)
    topo_m = TopologicalOdometer("http://127.0.0.1:8765", state_path=tmp_path / ".m.json")
    rf_cor = {"left": 0.9, "center": 2.0, "right": 0.9, "left90": 0.35, "right90": 0.35, "left120": 0.45, "right120": 0.45}
    topo_m.observe_status(_mk_status(sim_time=0.1, x=0.0, y=0.0, odom_dist=0.0, yaw_deg=0.0, rf=rf_cor))
    topo_m.begin_leg(_mk_status(sim_time=0.1, x=0.0, y=0.0, odom_dist=0.0, yaw_deg=0.0, rf=rf_cor))
    topo_m.finish_go(
        "junction",
        1.0,
        _mk_status(
            sim_time=2.0,
            x=1.0,
            y=0.0,
            odom_dist=1.0,
            yaw_deg=0.0,
            rf={"left": 0.9, "center": 1.8, "right": 0.9, "left90": 1.5, "right90": 0.35, "left120": 1.2, "right120": 0.45},
        ),
        in_corridor=True,
    )
    topo_m.begin_leg(_mk_status(sim_time=2.1, x=1.0, y=0.0, odom_dist=1.0, yaw_deg=90.0, rf=rf_cor))
    topo_m.finish_go(
        "junction",
        1.0,
        _mk_status(sim_time=4.0, x=1.0, y=1.0, odom_dist=2.0, yaw_deg=90.0, rf=rf_cor),
        in_corridor=True,
    )
    topo_m.reset_for_repass(_mk_status(sim_time=0.0, x=0.0, y=0.0, odom_dist=0.0, yaw_deg=0.0, rf=rf_cor))

    def _rf_multi(s):
        # Corridor east with narrow walls even past (1,0) so ONLY next_node_xy stops hop 0 at (1,0)!
        if abs(s["yaw"] % 360.0) < 45.0 or abs(s["yaw"] % 360.0) > 315.0:
            return {"left": 0.9, "center": max(0.32, 2.5 - s["x"]), "right": 0.9, "left90": 0.35, "right90": 0.35, "left120": 0.45, "right120": 0.45}
        return {"left": 0.9, "center": max(0.30, 1.32 - s["y"]), "right": 0.9, "left90": 0.35, "right90": 0.35, "left120": 0.45, "right120": 0.45}

    st_m, call_m = _make_kinematic_world(x0=0.0, y0=0.0, yaw0=0.0, rf_fn=_rf_multi)
    res_m = go_to_decision(call_m, route="E,N", stop_at_node=2, topo=topo_m)
    assert res_m["event"] == "arrived_node"
    assert res_m["route_progress"] == {
        "requested": ["E", "N"],
        "completed": 2,
        "remaining": [],
        "stop_at_node": 2,
    }
    assert math.hypot(st_m["x"] - 1.0, st_m["y"] - 1.0) <= 0.25

    # 2. Blocked hop 2 ("N" is walled off at Node #1) -> stops with event == "route_blocked"
    topo_m.reset_for_repass(_mk_status(sim_time=0.0, x=0.0, y=0.0, odom_dist=0.0, yaw_deg=0.0, rf=rf_cor))

    def _rf_blocked(s):
        if abs(s["yaw"] % 360.0) < 45.0 or abs(s["yaw"] % 360.0) > 315.0:
            return {"left": 0.9, "center": max(0.32, 2.5 - s["x"]), "right": 0.9, "left90": 0.35, "right90": 0.35, "left120": 0.45, "right120": 0.45}
        return {"left": 0.35, "center": 0.30, "right": 0.35, "left90": 0.35, "right90": 0.35, "left120": 0.45, "right120": 0.45}

    _, call_blk = _make_kinematic_world(x0=0.0, y0=0.0, yaw0=0.0, rf_fn=_rf_blocked)
    res_blk = go_to_decision(call_blk, route="E,N", topo=topo_m)
    assert res_blk["event"] == "route_blocked"
    assert res_blk["route_progress"]["completed"] == 1
    assert res_blk["route_progress"]["remaining"] == ["N"]

    # 3. Return from docked goal (target visible right in front) with route=["W"], stop_at_node=0 -> ignores target_ready
    topo_ret = TopologicalOdometer("http://127.0.0.1:8765", state_path=tmp_path / ".ret.json")
    topo_ret.observe_status(_mk_status(sim_time=0.1, x=0.0, y=0.0, odom_dist=0.0, yaw_deg=0.0, rf=rf_cor))
    topo_ret.begin_leg(_mk_status(sim_time=0.1, x=0.0, y=0.0, odom_dist=0.0, yaw_deg=0.0, rf=rf_cor))
    topo_ret.finish_go("target_ready", 1.0, _mk_status(sim_time=2.0, x=1.0, y=0.0, odom_dist=1.0, yaw_deg=0.0, rf=rf_cor), in_corridor=True)
    topo_ret.record_dock(0.3, _mk_status(sim_time=2.5, x=1.3, y=0.0, odom_dist=1.3, yaw_deg=0.0, rf=rf_cor), reached=True)

    st_ret, call_ret = _make_kinematic_world(
        x0=1.3,
        y0=0.0,
        yaw0=0.0,
        t0=2.6,
        odom0=1.3,
        rf_fn=lambda s: {"left": 0.9, "center": 0.27 if abs(s["yaw"]) < 45.0 else max(0.32, s["x"] + 0.35), "right": 0.9, "left90": 0.35, "right90": 0.35, "left120": 0.45, "right120": 0.45},
        vis_fn=lambda s: {
            "target_visible": abs(s["yaw"]) < 45.0,
            "target": {"offset_x": 0.0, "side": "center", "area_pct": 18.0, "v_span": 0.6} if abs(s["yaw"]) < 45.0 else None,
            "obstacles": [],
        },
    )
    res_ret = go_to_decision(call_ret, route=["W"], stop_at_node=0, topo=topo_ret)
    assert res_ret["event"] == "arrived_node"
    assert res_ret["topo"]["current_node"] == 0
    assert math.hypot(st_ret["x"], st_ret["y"]) <= 0.22

    # 4. Open-space stop_at_node=0 auto-heading from (1.2, 0.4) back to (0, 0)
    topo_os = TopologicalOdometer("http://127.0.0.1:8765", state_path=tmp_path / ".os.json")
    rf_os = {"left": 2.5, "center": 2.5, "right": 2.5, "left90": 2.0, "right90": 2.0, "left120": 2.0, "right120": 2.0}
    topo_os.observe_status(_mk_status(sim_time=0.1, x=0.0, y=0.0, odom_dist=0.0, yaw_deg=0.0, rf=rf_os))
    st_os, call_os = _make_kinematic_world(x0=1.2, y0=0.4, yaw0=0.0, t0=2.0, odom0=1.3, rf_fn=lambda s: rf_os)
    res_os = go_to_decision(call_os, stop_at_node=0, topo=topo_os)
    assert res_os["event"] == "arrived_node"
    assert math.hypot(st_os["x"], st_os["y"]) <= 0.22


def _shortest_explored_route(topo_dict: dict, src_id: int, dst_id: int) -> list[str]:
    """System 2 test helper: Dijkstra shortest cardinal port sequence over explored edges using dist_m."""
    import heapq

    nodes = {int(n["id"]): n for n in topo_dict["nodes"]}
    pq: list[tuple[float, int, list[str]]] = [(0.0, src_id, [])]
    best: dict[int, float] = {src_id: 0.0}
    while pq:
        cost, u, path = heapq.heappop(pq)
        if u == dst_id:
            return path
        if cost > best.get(u, float("inf")) + 1e-9:
            continue
        for port, info in nodes[u]["exits"].items():
            if info.get("state") == "explored" and info.get("to_node") is not None:
                v = int(info["to_node"])
                w = float(info.get("dist_m") or 1.0)
                nc = cost + max(0.05, w)
                if nc < best.get(v, float("inf")) - 1e-9:
                    best[v] = nc
                    heapq.heappush(pq, (nc, v, path + [port]))
    raise AssertionError(f"No explored path from {src_id} to {dst_id} in {topo_dict}")


def test_hybrid_brain_direction_a_return_to_home_and_direction_b_speedrun_on_maze_b():
    """Task 5 E2E: headless MuJoCo maze_b proves Direction A (Explore -> Dock -> Return-to-Home Node #0) and Direction B (reset_for_repass -> 1-command Speed-run to Goal)."""
    import math

    def _driver(call_fn, out: dict) -> None:
        topo = TopologicalOdometer(auto_persist=False)
        snap = topo.observe_status(call_fn("/status?vision=1", "GET"))
        next_card: str | None = _choose_next_cardinal_from_topo(snap)
        dock_res = None

        # Pass 1: Blind exploration to green target
        for _ in range(14):
            res = go_to_decision(call_fn, cardinal=next_card, max_dist=6.0, speed_m_s=0.55, topo=topo)
            next_card = None
            if res["event"] == "target_ready":
                dock_res = dock_target(call_fn, stop_distance=0.25)
                assert dock_res["reached"] is True
                snap = topo.record_dock(dock_res["moved_m"], dock_res["after_status"], reached=True)
                break
            elif res["topo"]["current_node"] is not None:
                next_card = _choose_next_cardinal_from_topo(res["topo"])

        assert dock_res is not None and dock_res["reached"] is True
        goal_id = snap["current_node"]
        assert goal_id is not None and snap["nodes"][goal_id]["kind"] == "goal"
        out["pass1_odom"] = float(dock_res["after_status"]["sensors"]["odom_dist_m"])

        # Direction A: Return-to-Home to Node #0 via single go --route ... --stop-at-node 0
        home_route = _shortest_explored_route(snap, goal_id, 0)
        assert len(home_route) >= 2
        ret_res = go_to_decision(call_fn, route=home_route, stop_at_node=0, speed_m_s=0.55, topo=topo)
        out["return_res"] = ret_res
        ret_st = call_fn("/status", "GET")
        out["return_world_err_m"] = math.hypot(
            float(ret_st["sensors"]["odom_x_m"]),
            float(ret_st["sensors"]["odom_y_m"]),
        )
        out["return_dbg_after"] = call_fn("/status?vision=1", "GET")

        # Direction B: Reset robot spawn while keeping learned Topo map -> Pass 2 Speed-run!
        reset_st = call_fn("/reset", "POST", {})
        snap_p2 = topo.reset_for_repass(reset_st, target_changed=False)
        assert snap_p2["current_node"] == 0
        goal_nodes = [int(n["id"]) for n in snap_p2["nodes"] if n["kind"] == "goal"]
        assert len(goal_nodes) == 1
        speed_route = _shortest_explored_route(snap_p2, 0, goal_nodes[0])
        p2_go = go_to_decision(call_fn, route=speed_route, speed_m_s=0.55, topo=topo)
        assert p2_go["event"] == "target_ready"
        p2_dock = dock_target(call_fn, stop_distance=0.25)
        out["pass2_go"] = p2_go
        out["pass2_dock"] = p2_dock
        out["pass2_odom"] = float(p2_dock["after_status"]["sensors"]["odom_dist_m"])

    result = _run_headless_episode("maze_b", _driver)
    assert result["return_res"]["event"] == "arrived_node"
    assert result["return_res"]["topo"]["current_node"] == 0
    assert result["return_world_err_m"] <= 0.35
    assert result["pass2_dock"]["reached"] is True
    assert result["pass2_odom"] <= 9.0
    assert result["pass2_odom"] < result["pass1_odom"] * 0.55
    assert result["raw_debug"]["contact_count"] == 0


def test_minor_polish_open_space_repass_dock_obstacle_hop_and_tight_budget(tmp_path: Path):
    """Minor #1-#3 regression tests: no double-count on Pass-2 open_space dock, obstacle-aware route pre-check, and max_dist on tight inter-hop budget."""
    import math
    from robot_sim.skills import go_to_decision

    # Minor #1: Pass-2 open_space finish_go("target_ready") already matched goal_id -> record_dock must NOT double-count visits!
    topo = TopologicalOdometer("http://127.0.0.1:8765", state_path=tmp_path / ".m1.json")
    rf_open = {"left": 2.0, "center": 2.5, "right": 2.0, "left90": 1.8, "right90": 1.8}
    topo.observe_status(_mk_status(sim_time=0.1, x=0.0, y=0.0, odom_dist=0.0, yaw_deg=0.0, rf=rf_open))
    topo.begin_leg(_mk_status(sim_time=0.1, x=0.0, y=0.0, odom_dist=0.0, yaw_deg=0.0, rf=rf_open))
    topo.finish_go("target_ready", 1.5, _mk_status(sim_time=2.0, x=1.5, y=0.0, odom_dist=1.5, yaw_deg=0.0, rf=rf_open), in_corridor=False)
    topo.record_dock(0.3, _mk_status(sim_time=2.5, x=1.8, y=0.0, odom_dist=1.8, yaw_deg=0.0, rf={"left": 1.2, "center": 0.26, "right": 1.2, "left90": 1.8, "right90": 1.8}), reached=True)

    # Pass 2 repass reset -> drive straight to existing goal_id=1
    topo.reset_for_repass(_mk_status(sim_time=0.0, x=0.0, y=0.0, odom_dist=0.0, yaw_deg=0.0, rf=rf_open))
    topo.begin_leg(_mk_status(sim_time=0.1, x=0.0, y=0.0, odom_dist=0.0, yaw_deg=0.0, rf=rf_open))
    topo.finish_go("target_ready", 1.75, _mk_status(sim_time=2.0, x=1.75, y=0.0, odom_dist=1.75, yaw_deg=0.0, rf=rf_open), in_corridor=False)
    snap2 = topo.record_dock(0.05, _mk_status(sim_time=2.2, x=1.80, y=0.0, odom_dist=1.80, yaw_deg=0.0, rf={"left": 1.2, "center": 0.26, "right": 1.2, "left90": 1.8, "right90": 1.8}), reached=True)
    assert snap2["nodes"][0]["exits"]["E"]["out_visits"] == 1
    assert snap2["nodes"][1]["exits"]["W"]["in_visits"] == 1

    # Minor #2 & #3: Kinematic two-hop route ("E,N")
    rf_cor = {"left": 0.9, "center": 2.0, "right": 0.9, "left90": 0.35, "right90": 0.35, "left120": 0.45, "right120": 0.45}
    topo2 = TopologicalOdometer("http://127.0.0.1:8765", state_path=tmp_path / ".m23.json")
    topo2.observe_status(_mk_status(sim_time=0.1, x=0.0, y=0.0, odom_dist=0.0, yaw_deg=0.0, rf=rf_cor))
    topo2.begin_leg(_mk_status(sim_time=0.1, x=0.0, y=0.0, odom_dist=0.0, yaw_deg=0.0, rf=rf_cor))
    topo2.finish_go("junction", 1.0, _mk_status(sim_time=2.0, x=1.0, y=0.0, odom_dist=1.0, yaw_deg=0.0, rf={"left": 0.9, "center": 0.32, "right": 0.9, "left90": 1.5, "right90": 0.35, "left120": 1.2, "right120": 0.45}), in_corridor=True)
    topo2.reset_for_repass(_mk_status(sim_time=0.0, x=0.0, y=0.0, odom_dist=0.0, yaw_deg=0.0, rf=rf_cor))

    def _run_sim(max_dist_val: float, obs_on_north: bool):
        st = {"t": 0.5, "x": 0.0, "y": 0.0, "yaw": 0.0, "odom": 0.0}

        def call_fn(path: str, method: str, body: dict | None = None):
            if path == "/wheels" and body:
                lw, rw = float(body.get("left", 0.0)), float(body.get("right", 0.0))
                dt = 0.08
                st["t"] += dt
                v = 0.5 * (lw + rw) * 0.05
                st["yaw"] += math.degrees((rw - lw) * 0.05 / 0.23) * dt
                r = math.radians(st["yaw"])
                st["x"] += v * math.cos(r) * dt
                st["y"] += v * math.sin(r) * dt
                st["odom"] += v * dt
                return {"ok": True}
            if path == "/stop":
                return {"stopped": True, "sim_time": round(st["t"], 3)}
            facing_e = abs(st["yaw"] % 360.0) < 45.0 or abs(st["yaw"] % 360.0) > 315.0
            if facing_e:
                rf = {"left": 0.9, "center": max(0.32, 1.32 - st["x"]), "right": 0.9, "left90": 1.5 if st["x"] >= 0.9 else 0.35, "right90": 0.35, "left120": 1.2 if st["x"] >= 0.9 else 0.45, "right120": 0.45}
                vis = {"target_visible": False, "target": None, "obstacles": []}
            else:
                rf = {"left": 0.35, "center": 0.30, "right": 0.35, "left90": 0.35, "right90": 0.35, "left120": 0.45, "right120": 0.45}
                vis = {
                    "target_visible": False,
                    "target": None,
                    "obstacles": [{"color": "red", "offset_x": 0.0, "side": "center", "area_pct": 14.0, "v_span": 0.45}] if obs_on_north else [],
                }
            return {
                "sim_time": round(st["t"], 3),
                "moving": False,
                "sensors": {
                    "rangefinders_m": rf,
                    "gyro_yaw_deg": round(st["yaw"], 3),
                    "odom_dist_m": round(st["odom"], 4),
                    "odom_x_m": round(st["x"], 4),
                    "odom_y_m": round(st["y"], 4),
                    "forward_speed_m_s": 0.0,
                    "yaw_rate_rad_s": 0.0,
                },
                "vision": vis,
            }

        return go_to_decision(call_fn, route="E,N", max_dist=max_dist_val, topo=topo2)

    # Minor #2: Colored obstacle right after turning North -> classified as static_obstacle_ahead (NOT route_blocked!)
    res_obs = _run_sim(max_dist_val=6.0, obs_on_north=True)
    assert res_obs["event"] == "static_obstacle_ahead"

    # Minor #3: Explicit tight max_dist=0.96 exhausted right after hop 1 -> reports event == "max_dist" with remaining=["N"]
    topo2.reset_for_repass(_mk_status(sim_time=0.0, x=0.0, y=0.0, odom_dist=0.0, yaw_deg=0.0, rf=rf_cor))
    res_budget = _run_sim(max_dist_val=0.96, obs_on_north=False)
    assert res_budget["event"] == "max_dist"
    assert res_budget["route_progress"]["remaining"] == ["N"]


def test_corridor_side_branch_static_ignored_and_dynamic_backup_proportional():
    """Corridor side-branch obstacle doesn't halt before junction, and tight moving gate triggers proportional backup (0.14-0.18m)."""
    from robot_sim.skills import advance_segment

    # 1) Static obstacle in open left side-branch of a corridor -> ignored so robot reaches junction/corner;
    #    centered or open-space obstacle still triggers static_obstacle_ahead.
    def _run_static_check(*, rf_init: dict, offset_x: float):
        st_state = {"t": 0.5, "x": 0.0, "odom": 0.0, "stopped": False}

        def call_fn(path: str, method: str, body: dict | None = None):
            if path == "/wheels" and body:
                lw, rw = float(body.get("left", 0.0)), float(body.get("right", 0.0))
                dt = 0.08
                st_state["t"] += dt
                v = 0.5 * (lw + rw) * 0.05
                st_state["x"] += v * dt
                st_state["odom"] += abs(v) * dt
                return {"ok": True}
            if path == "/stop":
                st_state["stopped"] = True
                return {"stopped": True, "sim_time": round(st_state["t"], 3)}
            # Advance a bit then hit front wall at x >= 0.45 so advance_segment terminates cleanly if ignored
            rf = dict(rf_init)
            if st_state["x"] >= 0.45:
                rf["center"] = 0.25
                obs = []
            else:
                obs = [{"color": "blue", "offset_x": offset_x, "side": "left" if offset_x < 0 else "center", "area_pct": 4.67, "v_span": 0.246}]
            return {
                "sim_time": round(st_state["t"], 3),
                "moving": False,
                "sensors": {
                    "rangefinders_m": rf,
                    "gyro_yaw_deg": 0.0,
                    "odom_dist_m": round(st_state["odom"], 4),
                    "odom_x_m": round(st_state["x"], 4),
                    "odom_y_m": 0.0,
                    "forward_speed_m_s": 0.0,
                    "yaw_rate_rad_s": 0.0,
                },
                "vision": {"target_visible": False, "target": None, "obstacles": obs},
            }

        return advance_segment(call_fn, max_dist=1.2)

    rf_side_open = {"left": 0.83, "center": 0.67, "right": 0.72, "left90": 1.32, "right90": 0.34, "left120": 0.85, "right120": 0.38}
    res_side = _run_static_check(rf_init=rf_side_open, offset_x=-0.32)
    assert res_side["event"] != "static_obstacle_ahead"

    res_center = _run_static_check(rf_init=rf_side_open, offset_x=0.0)
    assert res_center["event"] == "static_obstacle_ahead"

    rf_open_space = {"left": 0.83, "center": 0.67, "right": 0.72, "left90": 1.50, "right90": 1.50, "left120": 1.50, "right120": 1.50}
    res_open = _run_static_check(rf_init=rf_open_space, offset_x=-0.32)
    assert res_open["event"] == "static_obstacle_ahead"

    # 2) Close moving gate in narrow corridor (est_dist_m ~ 0.19 < 0.20, min_f = 0.17) -> backs up 0.14m < |back_m| <= 0.18m
    dyn_state = {"t": 0.5, "x": 0.0, "odom": 0.0, "v": 0.0, "polls": 0, "backup_m": 0.0}

    def dyn_call(path: str, method: str, body: dict | None = None):
        if path == "/wheels" and body:
            lw, rw = float(body.get("left", 0.0)), float(body.get("right", 0.0))
            dt = 0.02
            dyn_state["t"] += dt
            v = 0.5 * (lw + rw) * 0.05
            dyn_state["v"] = v
            dx = v * dt
            dyn_state["x"] += dx
            dyn_state["odom"] += dx
            if dx < 0:
                dyn_state["backup_m"] += abs(dx)
            return {"ok": True}
        if path == "/stop":
            dyn_state["v"] = 0.0
            return {"stopped": True, "sim_time": round(dyn_state["t"], 3)}
        dyn_state["polls"] += 1
        # Simulate moving obstacle shifting offset_x across polls so _probe_front_obstacle_motion classifies "dynamic"
        ox = 0.0 if dyn_state["polls"] <= 1 else 0.08
        # During backup, x decreases from 0.0 toward -0.18 so min_f rises linearly from 0.17
        cur_min_f = min(0.50, max(0.17, 0.17 - dyn_state["x"]))
        return {
            "sim_time": round(dyn_state["t"], 3),
            "moving": abs(dyn_state["v"]) > 1e-3,
            "sensors": {
                "rangefinders_m": {"left": cur_min_f, "center": cur_min_f, "right": 0.25, "left90": 0.40, "right90": 0.40, "left120": 0.45, "right120": 0.45},
                "gyro_yaw_deg": 0.0,
                "odom_dist_m": round(dyn_state["odom"], 4),
                "odom_x_m": round(dyn_state["x"], 4),
                "odom_y_m": 0.0,
                "forward_speed_m_s": round(dyn_state["v"], 4),
                "yaw_rate_rad_s": 0.0,
            },
            "vision": {
                "target_visible": False,
                "target": None,
                "obstacles": [{"color": "blue", "offset_x": ox, "side": "center", "area_pct": 42.0, "v_span": 0.90}],
            },
        }

    res_dyn = advance_segment(dyn_call, max_dist=1.2)
    assert res_dyn["event"] == "dynamic_gate_ahead"
    assert 0.14 < dyn_state["backup_m"] <= 0.22

