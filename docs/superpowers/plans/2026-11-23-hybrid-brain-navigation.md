# Hybrid-Brain Navigation (Mode C) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Xây dựng kiến trúc điều hướng lai System 1 / System 2 (`Mode C: hybrid-brain`) gồm bộ định vị đồ thị khách quan `TopologicalOdometer` (`robot_sim/topo.py`), kỹ năng vận động gộp `go_to_decision` (`robot_ctl.py go`, `robot_ctl.py topo`), tích hợp hiển thị trên MuJoCo Studio (`studio/`), và subagent `robot-driver-hybrid` giải trọn vẹn cả 6 sa bàn MuJoCo với `contact_count == 0` và `redundant_revisit_count == 0` mà không ảnh hưởng Mode A, Mode B cùng 106 kiểm thử hiện hành.

**Architecture:** Tầng System 1 (`robot_sim/topo.py` + `go_to_decision` trong `robot_sim/skills.py`) thực thi vòng kín 20 Hz gồm xoay khóa hướng đầu lệnh, chạy giữ tâm hành lang, tự động nối tiếp (`auto-chain`) qua các khúc cua chữ L chỉ có duy nhất 1 lối rẽ ngang không có đích/vật cản, và ghi nhận đồ thị Odometry hai chiều (`out_visits` / `in_visits`, `pending_leg`, `uninspected`, `loop_closed`) lưu nguyên tử theo phiên `--url` tại `captures/.topo_state.json`. Tầng System 2 (`robot-driver-hybrid`) nắm 100% quyền quyết định tại mọi ngã giao `>= 2` nhánh, ngõ cụt, vòng lặp đảo chữ U, lách vật cản tĩnh và vượt từng cổng động. Tầng MuJoCo Studio (`studio/sim_plugin.py`, `viewer_plugin.py`) đọc tóm tắt đồ thị theo `mtime` ở tần số `<= 2 Hz` để hiển thị trực tiếp lên bảng điều khiển mà không gây xung đột khóa tệp trên Windows.

**Tech Stack:** Python 3.13, MuJoCo 3.x (`SimulationApp` & `mujoco.studio`), HTTP keep-alive JSON bridge (`robot_ctl.py`), `pytest`.

**Spec:** [2026-11-23-hybrid-brain-navigation-design.md](file:///d:/latuan/Programming/robot-demo/docs/superpowers/specs/2026-11-23-hybrid-brain-navigation-design.md)

---

## Global Constraints

- **CLI Windows tốc độ cao:** Mọi lệnh Python và kiểm thử phải gọi trực tiếp `.venv\Scripts\python.exe` (tuyệt đối không gọi `uv run python`).
- **Kỷ luật Sim-to-Real & Ranh giới Công bằng:** `robot_sim/topo.py`, `go_to_decision`, `robot_ctl.py` (không `--debug`) và `driver-hybrid-prompt.md` tuyệt đối không đọc/xuất `debug` (`robot_xy`, `target_xy`, `arena`), không gọi `/arenas`, không dùng `overview_cam`, không tự quay đầu `180°` ở ngõ cụt, không tự chọn nhánh khi `>= 2` hướng mở, không lan truyền nhãn `dead_end` ngược lên nút cha và không xuất trường gợi ý nhánh.
- **Tính bất biến của phần cứng & sa bàn:** Tuyệt đối không chỉnh sửa `models/robot_2wheel.xml` hay bất kỳ tệp `models/arena_*.xml` nào (`verify_arena_integrity` kiểm soát toàn bộ 6 sa bàn).
- **Bảo toàn 100% Mode A & Mode B:** Giữ nguyên chữ ký và giá trị trả về của `explore_maze`, `approach_target`, `scan_for_target`, `advance_segment`, `cross_dynamic_gate`, `dock_target` trong `robot_sim/skills.py`; toàn bộ 106 kiểm thử hiện có phải tiếp tục PASS 100%.
- **An toàn khóa tệp trên Windows & MuJoCo Studio:** Ghi `captures/.topo_state.json` nguyên tử qua `.tmp -> replace` kèm retry ngắn `3 x 15ms` khi gặp `PermissionError`; tuyệt đối không đọc tệp đĩa trực tiếp trong `viewer_plugin.on_build_gui` (60 FPS).

---

## Review Focus

5 lớp đầu vào / tình huống lỗi biên rút từ đặc tả được ghim kiểm thử trực tiếp vào các tác vụ bên dưới:
1. **Xoay chéo lệch lưới `90°` (`> 15°`, ví dụ `+53.6°` khi lách vật cản trên `2wheel`) và hủy `pending_leg` (Task 1):** Khi `abs(wrap180(gyro_yaw_deg - nearest_90)) > 15.0°`, `TopologicalOdometer.observe_status` không được gán nhầm tia `left90`/`center`/`right90` vào các cổng `E/N/W/S` hay xóa oan cổng `"uninspected"` của `Node #0`; khi gọi `cancel_pending_leg()`, hành trình treo phải xóa sạch về `None`.
2. **Đi lùi trên cạnh đã `"explored"` so với khép dây cung vòng lặp mới (Task 1):** Đi xuôi `Node #0 --(E)--> Node #1` rồi đi ngược `Node #1 --(W)--> Node #0` phải tăng đúng `out_visits` / `in_visits` ở từng đầu cổng nhưng giữ **`loop_closed == False`**; chỉ khi chạm `Node` đã tồn tại qua cặp cổng chưa từng `"explored"` nối với nhau mới bật **`loop_closed == True`**.
3. **Xuất phát từ giữa tâm ngã 4 mở đi vào hành lang dài `> 2.0 m` dẫn tới góc chữ L & chốt chống xoay lặp tại chỗ (Task 2):** Xuất phát tại tâm giao lộ (`left90 > 0.75`, `right90 > 0.75`) vẫn phải được cấp `seg_max = 3.5 m` để không đứt giữa hành lang dài và vẫn được `auto-chain` góc chữ L cuối đoạn đó; ngược lại, từ đoạn nối thứ 2 trở đi nếu `moved_m < 0.20 m` thì phải ngắt chuỗi ngay để không xoay lặp tại chỗ.
4. **Không tạo `.topo_state.json` khi chạy Mode A / Mode B & cách ly đa phiên `--url` (Task 3):** Các lệnh `status`, `advance`, `turn`, `move`, `cross-gate`, `dock`, `explore`, `approach` khi chưa mở phiên `topo` cho `--url` không được tạo tệp `.topo_state.json`; khi có 2 `--url` cùng lưu trong tệp, `topo reset` hoặc `reset` trên URL A không được ảnh hưởng tới phiên của URL B.
5. **Tệp `.topo_state.json` bị hỏng JSON hoặc thiếu tệp khi MuJoCo Studio đang chạy (Task 1 & Task 4):** Hàm đọc tóm tắt Studio `read_topo_studio_summary` (Task 1) và bộ đệm `<= 2 Hz` trong `studio/sim_plugin.py` (Task 4) phải chịu lỗi an toàn (trả `(0, -1, 0)` hoặc giữ giá trị hợp lệ gần nhất) mà không ném ngoại lệ ra luồng mô phỏng vật lý.

---

## File Structure

- **Create:** `robot_sim/topo.py` — các hàm quy đổi hướng/cardinal dùng chung (`yaw_to_cardinal`, `cardinal_to_deg`, `opposite_cardinal`, `is_cardinal_aligned`, `branches_to_cardinals`), lớp `TopologicalOdometer` lưu đa phiên theo `base_url` trong `captures/.topo_state.json`, và helper đọc an toàn `read_topo_studio_summary`.
- **Modify:** `robot_sim/skills.py` — thêm tham số nội bộ tùy chọn `_out_meta: dict | None = None` vào `advance_segment` (không đổi hành vi/giá trị trả về) để xuất cờ `saw_corridor_wall`, và cài đặt `go_to_decision(...)`.
- **Modify:** `robot_ctl.py` — thêm 2 lệnh CLI `go` và `topo` (`show` / `reset`), bổ sung `open_cardinals` cho đầu ra CLI `cross-gate`, và nối đồng bộ có điều kiện với `TopologicalOdometer` khi phiên của `--url` đang hoạt động.
- **Modify:** `studio/messages.py`, `studio/sim_plugin.py`, `studio/viewer_plugin.py` — thêm 3 trường tóm tắt `topo_*` vào `RobotTelemetry`, đọc theo `mtime` (`<= 2 Hz`) trong `sim_plugin.py`, hiển thị gọn 1 dòng trong `viewer_plugin.py`.
- **Create:** `.agents/skills/robot-multi-agent-rd/references/driver-hybrid-prompt.md` — System Prompt cho subagent `robot-driver-hybrid` (Mode C).
- **Modify:** `.agents/skills/robot-multi-agent-rd/SKILL.md`, `.agents/skills/robot-multi-agent-rd/references/engineer-prompt.md`, `README.md` — đăng ký và tài liệu hóa Mode C (`hybrid-brain`) bên cạnh Mode A (mặc định) và Mode B.
- **Create / Modify Tests:** `tests/test_hybrid_brain.py` (mới), `tests/test_cli.py`, `tests/test_agent_skill.py`.

---

### Task 1: Bộ Định vị Đồ thị Khách quan `TopologicalOdometer` (`robot_sim/topo.py`)

**Files:**
- Create: `robot_sim/topo.py`
- Create: `tests/test_hybrid_brain.py`

**Interfaces:**
- Consumes: `PROJECT_ROOT`, `_require_finite` từ `robot_sim.core`; `compute_open_branches` từ `robot_sim.skills`.
- Produces (trong `robot_sim/topo.py`):
  - Hằng số: `CARDINALS = ("E", "N", "W", "S")`, `NODE_MATCH_RADIUS_M = 0.42`, `CARDINAL_ALIGN_TOL_DEG = 15.0`, `DEFAULT_TOPO_STATE_PATH = PROJECT_ROOT / "captures" / ".topo_state.json"`.
  - Hàm quy đổi:
    - `normalize_base_url(base_url: str) -> str`
    - `yaw_to_cardinal(yaw_deg: float) -> str`
    - `cardinal_to_deg(cardinal: str) -> float`
    - `opposite_cardinal(cardinal: str) -> str`
    - `is_cardinal_aligned(yaw_deg: float, tol_deg: float = CARDINAL_ALIGN_TOL_DEG) -> bool`
    - `branches_to_cardinals(open_branches: list[str], yaw_deg: float) -> list[str]`
    - `read_topo_studio_summary(base_url: str = "http://127.0.0.1:8765", state_path: Path | str = DEFAULT_TOPO_STATE_PATH) -> tuple[int, int, int]`
  - Lớp `TopologicalOdometer`:
    - `__init__(self, base_url: str = "http://127.0.0.1:8765", state_path: Path | str = DEFAULT_TOPO_STATE_PATH, *, auto_persist: bool = True) -> None`
    - `has_active_session(cls, base_url: str, state_path: Path | str = DEFAULT_TOPO_STATE_PATH) -> bool`
    - `reset_session(cls, base_url: str, state_path: Path | str = DEFAULT_TOPO_STATE_PATH) -> None`
    - `nearest_existing_node(self, x_m: float, y_m: float, radius_m: float = NODE_MATCH_RADIUS_M) -> int | None`
    - `is_open_space_start(self) -> bool`
    - `observe_status(self, status: dict) -> dict`
    - `begin_leg(self, status: dict) -> None`
    - `record_progress(self, moved_m: float, status: dict) -> dict`
    - `cancel_pending_leg(self, status: dict | None = None) -> dict`
    - `finish_go(self, event: str, moved_m: float, status: dict, *, in_corridor: bool) -> dict`
    - `export_dict(self) -> dict`

- [ ] **Step 1: Write the failing unit tests for `robot_sim/topo.py` in `tests/test_hybrid_brain.py`**

Tạo mới `tests/test_hybrid_brain.py` kiểm tra toàn bộ quy đổi cardinal, tạo `Node #0` (`start` có `"W": "uninspected"` vs `open_space`), phân giải `"uninspected"` khi xoay thẳng trục `<= 15°` và giữ nguyên khi xoay chéo `53.6°` (Review Focus #1), cạnh 2 đầu cổng qua góc chữ L + `pending_leg` qua cổng động, `loop_closed` phân biệt đi lùi cạnh cũ với dây cung vòng lặp mới (Review Focus #2), tự động reset phiên, cách ly `base_url`, và không chứa trường gợi ý hay `"debug"`:

```python
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
    default_rf = {"left": 0.45, "center": 0.32, "right": 0.45, "left90": 1.40, "right90": 0.32, "left120": 0.55, "right120": 0.42}
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
        _mk_status(sim_time=0.4, x=0.0, y=0.0, yaw_deg=53.6, rf={"left": 0.4, "center": 0.30, "right": 0.4, "left90": 0.30, "right90": 0.30})
    )
    assert snap_oblique["nodes"][0]["exits"]["W"]["state"] == "uninspected"

    # Xoay thẳng lên hướng Bắc (+90°): left90 nay nhìn thẳng sang hướng Tây (W = 1.50m -> nâng thành "open")
    snap_north = topo.observe_status(
        _mk_status(sim_time=0.8, x=0.0, y=0.0, yaw_deg=90.0, rf={"left": 0.8, "center": 1.6, "right": 0.8, "left90": 1.50, "right90": 0.32})
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
    topo.finish_go("dynamic_gate_ahead", 0.50, _mk_status(sim_time=1.0, x=0.50, y=0.0, odom_dist=0.50, yaw_deg=0.0), in_corridor=True)
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
    assert snap1["nodes"][0]["exits"]["E"] == {"state": "explored", "to_node": 1, "out_visits": 1, "in_visits": 0, "dist_m": 2.2}
    assert snap1["nodes"][1]["exits"]["S"] == {"state": "explored", "to_node": 0, "out_visits": 0, "in_visits": 1, "dist_m": 2.2}

    # Review Focus #2 (a): Đi lùi từ Node #1 qua cổng S về lại Node #0 (khớp <= 0.42m, giữ nguyên neo [0.0, 0.0]) -> loop_closed == False!
    topo.begin_leg(_mk_status(sim_time=5.5, x=1.00, y=1.20, odom_dist=2.20, yaw_deg=-90.0))
    snap_back = topo.finish_go(
        "junction",
        2.18,
        _mk_status(sim_time=9.0, x=0.18, y=0.05, odom_dist=4.38, yaw_deg=180.0, rf={"left": 0.4, "center": 0.32, "right": 0.9, "left90": 0.32, "right90": 1.5}),
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
        _mk_status(sim_time=13.0, x=0.95, y=1.22, odom_dist=6.58, yaw_deg=0.0, rf={"left": 0.4, "center": 1.4, "right": 0.8, "left90": 0.32, "right90": 1.5}),
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
```

- [ ] **Step 2: Run test to verify it fails**

Run: `.\.venv\Scripts\python.exe -m pytest tests/test_hybrid_brain.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'robot_sim.topo'`

- [ ] **Step 3: Implement `robot_sim/topo.py`**

Tạo `robot_sim/topo.py` cài đặt đầy đủ các giao diện ở mục **Interfaces** của Task 1 theo đúng Mục `5.1` của Spec:
- `normalize_base_url(base_url)` chuẩn hóa `scheme://host:port/path` (bỏ dấu `/` thừa cuối).
- Lưu tệp JSON cấu trúc `{"sessions": {norm_url: session_dict}}` nguyên tử bằng tệp tạm `.tmp` cùng thư mục rồi `replace`, bọc vòng lặp `3` lần (`time.sleep(0.015)`) khi gặp `PermissionError` trên Windows.
- Tự động reset phiên theo đúng 3 nhánh điều kiện ở Mục `5.1.2` của Spec (`sim_time` giảm `> 1e-3`, `odom_dist_m` giảm `> 1e-3`, hoặc xe trở về gốc `(0, 0)` với `odom_dist_m < 0.03` sau khi `last_odom_dist_m > 0.10` / trở về `(odom=0, yaw=0)` sau khi đã xoay `> 2.0°`).
- Tại `Node #0`, đặt `kind = "open_space"` khi `left90 > 0.75 and right90 > 0.75 and center > 0.85`, ngược lại đặt `kind = "start"` và khởi tạo cổng `opposite_cardinal(heading_cardinal)` với `"state": "uninspected"`.
- Khi `is_cardinal_aligned(gyro_yaw_deg, 15.0)` là `True` tại `Node #k` (`kind != "open_space"`): ánh xạ 3 hướng `left` (`+90°`), `straight` (`0°`), `right` (`-90°`) quanh hướng chính hiện tại: hướng nào nằm trong `branches_to_cardinals(compute_open_branches(rf), gyro_yaw_deg)` thì nâng `"uninspected" -> "open"` (hoặc thêm `"open"` nếu chưa có); hướng nào không mở mà đang `"uninspected"` thì xóa khỏi `exits`.

- [ ] **Step 4: Run test to verify it passes**

Run: `.\.venv\Scripts\python.exe -m pytest tests/test_hybrid_brain.py -v`
Expected: PASS (`4 passed`)

- [ ] **Step 5: Commit (if running outside restricted git sandbox)**

```powershell
git add robot_sim/topo.py tests/test_hybrid_brain.py
git commit -m "feat(topo): add objective Sim-to-Real TopologicalOdometer for Mode C"
```

---

### Task 2: Kỹ năng Vận động Gộp System 1 `go_to_decision` (`robot_sim/skills.py`)

**Files:**
- Modify: `robot_sim/skills.py:2403-2776` (và thêm `go_to_decision` ở cuối tệp)
- Modify: `tests/test_hybrid_brain.py`

**Interfaces:**
- Consumes: `TopologicalOdometer`, `branches_to_cardinals`, `cardinal_to_deg`, `NODE_MATCH_RADIUS_M` từ `robot_sim.topo`; `advance_segment`, `closed_loop_turn`, `compute_open_branches`, `_clean_status`, `_wrap_angle_deg`, `_require_finite` trong `robot_sim.skills`.
- Produces (trong `robot_sim/skills.py`):
  - `advance_segment(..., _out_meta: dict | None = None) -> dict` (giữ nguyên 100% chữ ký mặc định và dict trả về; nếu `_out_meta` là `dict`, ghi `_out_meta["saw_corridor_wall"] = bool(saw_left_wall or saw_right_wall)` trước khi return).
  - `go_to_decision(call_fn: BridgeCall, *, cardinal: str | None = None, heading_deg: float | None = None, max_dist: float = 6.0, speed_m_s: float = 0.55, step_dt: float = 0.10, max_chain: int = 6, auto_chain: bool = True, topo: TopologicalOdometer | None = None) -> dict` trả về:
    `{"event", "moved_m", "segments_chained", "auto_l_turns", "open_branches", "open_cardinals", "obstacle", "flank_clearance_m", "topo", "after_status"}` (nếu `topo is None`, tự tạo `TopologicalOdometer(auto_persist=False)` trong RAM để hàm luôn trả cấu trúc `topo` nhất quán mà không ghi đĩa ngoài ý muốn).

- [ ] **Step 1: Write the failing unit & MuJoCo headless integration tests for `go_to_decision` in `tests/test_hybrid_brain.py`**

Thêm vào cuối `tests/test_hybrid_brain.py` các kiểm thử:
1. Kiểm tra tham số đầu vào (`cardinal` và `heading_deg` loại trừ nhau, `max_dist <= 0`, `max_chain < 1`, `NaN`).
2. Kiểm thử mock cho Review Focus #3: xuất phát từ tâm giao lộ mở (`left90 = 1.2, right90 = 1.2`) đi qua hành lang dài `2.4 m` (`saw_corridor_wall = True` ở giữa đoạn) tới góc chữ L chỉ có `["left"]` -> không bị cắt ở `1.8 m` và tự động `auto-chain` sang đoạn 2; nếu đoạn 2 trả `moved_m = 0.05 < 0.20` ở góc cua kế tiếp -> ngắt chuỗi ngay không xoay tiếp lần 2.
3. Kiểm thử tích hợp vật lý headless MuJoCo (`SimulationApp` thực) giải trọn vẹn **`maze_a`**, **`maze_b`** (nhánh sau lưng `Node #0` + ngõ cụt gấp khúc `4.8 m`), **`maze_c`** (vòng lặp chữ U với `out_visits == 0` khi lùi), và **`dynamic_b`** (hành lang chữ S + 3 cổng piston) bằng bộ điều khiển System 2 tối giản đọc thuần `res["topo"]` + `cross_dynamic_gate` + `dock_target` với `contact_count == 0`!

```python
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
                    "rangefinders_m": dict(raw.get("rangefinders_m", {"left": 1.0, "center": 2.0, "right": 1.0, "left90": 0.45, "right90": 0.45})),
                },
                "vision": dict(raw.get("vision", {"target_visible": False, "target": None, "obstacles": []})),
                "debug": {"arena": "mock", "robot_xy": [0.0, 0.0]},
            }
        return {"ok": True, "moving": False}


def test_go_to_decision_validates_args_and_handles_open_junction_long_corridor_and_zero_move_guard(tmp_path: Path):
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
        {"sim_time": 0.0, "odom_x_m": 0.0, "odom_y_m": 0.0, "odom_dist_m": 0.0, "gyro_yaw_deg": 0.0, "rangefinders_m": {"left": 1.1, "center": 2.8, "right": 1.1, "left90": 1.2, "right90": 1.2, "left120": 0.9, "right120": 0.9}},
        # seg 1 bắt đầu tại ngã 4 mở -> vào hành lang (left90=0.40, right90=0.40) suốt 2.30m -> góc chữ L mở trái tại (2.30, 0.0)
        {"sim_time": 0.1, "odom_x_m": 0.0, "odom_y_m": 0.0, "odom_dist_m": 0.0, "gyro_yaw_deg": 0.0, "rangefinders_m": {"left": 1.1, "center": 2.8, "right": 1.1, "left90": 1.2, "right90": 1.2, "left120": 0.9, "right120": 0.9}},
        {"sim_time": 0.5, "odom_x_m": 0.9, "odom_y_m": 0.0, "odom_dist_m": 0.9, "gyro_yaw_deg": 0.0, "rangefinders_m": {"left": 0.8, "center": 1.9, "right": 0.8, "left90": 0.40, "right90": 0.40, "left120": 0.45, "right120": 0.45}},
        {"sim_time": 1.0, "odom_x_m": 1.95, "odom_y_m": 0.0, "odom_dist_m": 1.95, "gyro_yaw_deg": 0.0, "rangefinders_m": {"left": 0.8, "center": 0.85, "right": 0.8, "left90": 0.40, "right90": 0.40, "left120": 0.45, "right120": 0.45}},
        {"sim_time": 1.3, "odom_x_m": 2.30, "odom_y_m": 0.0, "odom_dist_m": 2.30, "gyro_yaw_deg": 0.0, "moving": False, "rangefinders_m": {"left": 0.5, "center": 0.34, "right": 0.4, "left90": 1.40, "right90": 0.36, "left120": 0.85, "right120": 0.42}},
        {"sim_time": 1.4, "odom_x_m": 2.30, "odom_y_m": 0.0, "odom_dist_m": 2.30, "gyro_yaw_deg": 0.0, "moving": False, "rangefinders_m": {"left": 0.5, "center": 0.34, "right": 0.4, "left90": 1.40, "right90": 0.36, "left120": 0.85, "right120": 0.42}},
        # pha closed_loop_turn +90° sang hướng Bắc tại (2.30, 0.0)
        {"sim_time": 1.6, "odom_x_m": 2.30, "odom_y_m": 0.0, "odom_dist_m": 2.30, "gyro_yaw_deg": 90.0, "moving": False, "rangefinders_m": {"left": 0.5, "center": 1.40, "right": 0.5, "left90": 0.36, "right90": 0.36, "left120": 0.42, "right120": 0.42}},
        {"sim_time": 1.7, "odom_x_m": 2.30, "odom_y_m": 0.0, "odom_dist_m": 2.30, "gyro_yaw_deg": 90.0, "moving": False, "rangefinders_m": {"left": 0.5, "center": 1.40, "right": 0.5, "left90": 0.36, "right90": 0.36, "left120": 0.42, "right120": 0.42}},
        # seg 2 chỉ nhích 0.05m rồi bị chặn trước với ["left"] -> moved_m < 0.20m nên phải dừng auto-chain ngay!
        {"sim_time": 1.8, "odom_x_m": 2.30, "odom_y_m": 0.05, "odom_dist_m": 2.35, "gyro_yaw_deg": 90.0, "moving": False, "rangefinders_m": {"left": 0.5, "center": 0.34, "right": 0.4, "left90": 1.30, "right90": 0.36, "left120": 0.85, "right120": 0.42}},
        {"sim_time": 1.9, "odom_x_m": 2.30, "odom_y_m": 0.05, "odom_dist_m": 2.35, "gyro_yaw_deg": 90.0, "moving": False, "rangefinders_m": {"left": 0.5, "center": 0.34, "right": 0.4, "left90": 1.30, "right90": 0.36, "left120": 0.85, "right120": 0.42}},
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
            q = deque([(cur, None)])
            seen = {cur}
            while q:
                u, first_port = q.popleft()
                if u != cur and any(i["state"] == target_state for i in nodes[u]["exits"].values()):
                    return first_port
                for port, info in nodes[u]["exits"].items():
                    if info["state"] == "explored" and info["to_node"] is not None:
                        if not allow_repeat and info["out_visits"] >= 1:
                            continue
                        nxt = info["to_node"]
                        if nxt not in seen:
                            seen.add(nxt)
                            q.append((nxt, port if first_port is None else first_port))
    raise AssertionError(f"Không tìm thấy nhánh đi tiếp trên đồ thị: {topo_dict}")


@pytest.mark.parametrize("arena_name,max_decisions", [("maze_a", 10), ("maze_b", 12), ("maze_c", 12), ("dynamic_b", 12)])
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
```

- [ ] **Step 2: Run test to verify it fails**

Run: `.\.venv\Scripts\python.exe -m pytest tests/test_hybrid_brain.py -k "go_to_decision or solves_mazes" -v`
Expected: FAIL with `ImportError: cannot import name 'go_to_decision' from 'robot_sim.skills'`

- [ ] **Step 3: Implement `_out_meta` hook in `advance_segment` and `go_to_decision` in `robot_sim/skills.py`**

Cập nhật `robot_sim/skills.py` theo Mục `5.2` của Spec:
1. Thêm tham số keyword tùy chọn `_out_meta: dict | None = None` vào `advance_segment`: bên trong `_finish_result`, nếu `_out_meta is not None`, gán `_out_meta["saw_corridor_wall"] = bool(saw_left_wall or saw_right_wall)` (khởi tạo `saw_left_wall = l90_0 <= 0.55`, `saw_right_wall = r90_0 <= 0.55` trước cả các nhánh thoát sớm để luôn đã được định nghĩa).
2. Cài đặt `go_to_decision`:
   - Kiểm tra hợp lệ `max_dist > 0`, `speed_m_s`, `step_dt`, `max_chain >= 1` (số nguyên, không nhận `bool`), và loại trừ `cardinal is not None and heading_deg is not None`.
   - Khởi tạo `active_topo = topo if topo is not None else TopologicalOdometer(auto_persist=False)`.
   - Bước 1: gọi `/status?vision=1`, gọi `active_topo.observe_status(st)`. Nếu có `cardinal` hoặc `heading_deg`, tính `turn_deg = _wrap_angle_deg(target_deg - cur_yaw)`; nếu `abs(turn_deg) > 3.0`, gọi `closed_loop_turn(call_fn, degrees=turn_deg, max_speed_deg_s=200.0, tol_deg=1.2, include_vision=True, fast_settle=True)` và gọi lại `active_topo.observe_status(st)`. Sau đó gọi `active_topo.begin_leg(st)`.
   - Bước 2: lặp tối đa `max_chain` chặng (hoặc `1` chặng nếu `auto_chain=False`):
     - Chọn `seg_max = min(1.8, remaining_dist)` chỉ khi `(min(l90, r90, l120, r120) > 1.35)` hoặc `(active_topo.is_open_space_start() and l90 > 0.85 and r90 > 0.85)`; ngược lại cấp `seg_max = min(3.5, remaining_dist)`.
     - Gọi `seg_res = advance_segment(call_fn, max_dist=seg_max, speed_m_s=speed_m_s, step_dt=step_dt, _out_meta=seg_meta)`, cộng dồn `total_moved_m += seg_res["moved_m"]`, cập nhật `any_corridor_wall |= seg_meta["saw_corridor_wall"]`.
     - Kiểm tra đủ 5 điều kiện `Auto-Chain` (Mục `5.2.2` của Spec) + chốt `(segments_chained == 1 or seg_res["moved_m"] >= 0.20)` + `remaining_dist > 0.15` + `segments_chained < max_chain`:
       - Nếu thỏa: tính góc khóa `90°` kế tiếp `next_yaw = round((cur_yaw + (90.0 if branch == "left" else -90.0)) / 90.0) * 90.0`, gọi `closed_loop_turn(..., fast_settle=True)`, thêm `branch` vào `auto_l_turns`, và chạy chặng kế tiếp.
       - Nếu không thỏa: dừng vòng lặp, gọi `topo_snap = active_topo.finish_go(seg_res["event"], total_moved_m, seg_res["after_status"], in_corridor=corridor_context)`, tính `open_cardinals = branches_to_cardinals(seg_res["open_branches"], end_yaw)`, và trả JSON kết quả đã lọc sạch `debug`.

- [ ] **Step 4: Run tests to verify Task 2 passes without breaking Mode B skills**

Run: `.\.venv\Scripts\python.exe -m pytest tests/test_hybrid_brain.py tests/test_agent_brain_skills.py -v`
Expected: PASS (`100% passed`)

- [ ] **Step 5: Commit (if running outside restricted git sandbox)**

```powershell
git add robot_sim/skills.py tests/test_hybrid_brain.py
git commit -m "feat(skills): implement System 1 go_to_decision with L-corner auto-chaining"
```

---

### Task 3: Giao diện CLI `go` & `topo` và Đồng bộ Phiên trong `robot_ctl.py`

**Files:**
- Modify: `robot_ctl.py:12-29, 250-280, 360-480`
- Modify: `tests/test_cli.py`

**Interfaces:**
- Consumes: `go_to_decision`, `compute_open_branches` từ `robot_sim.skills`; `TopologicalOdometer`, `branches_to_cardinals`, `DEFAULT_TOPO_STATE_PATH` từ `robot_sim.topo`.
- Produces (CLI `robot_ctl.py`):
  - `robot_ctl.py go [--cardinal {E,N,W,S} | --heading-deg <DEG>] [--max-dist 6.0] [--speed 0.55] [--step-dt 0.10] [--max-chain 6] [--no-chain]`
  - `robot_ctl.py topo {show,reset}`
  - Bổ sung trường `"open_cardinals"` vào JSON đầu ra CLI của `cross-gate` (và đính kèm `"topo"` khi `TopologicalOdometer.has_active_session(args.url)` là `True`).
  - Đồng bộ có điều kiện (chỉ khi `has_active_session(args.url)` là `True`): `reset`/`reload` gọi `reset_session`; `turn --wait`/`scan`/`dock` gọi `observe_status` (hoặc `record_progress` với `dock`); `move`/`wheels` khi không đứng tại nút gọi `cancel_pending_leg`.

- [ ] **Step 1: Write the failing CLI tests in `tests/test_cli.py`**

Thêm vào cuối `tests/test_cli.py` kiểm thử cho `go`, `topo show`, `topo reset`, `cross-gate` trả `open_cardinals`, cách ly đa phiên `--url`, và bảo đảm Mode A/B không tạo `.topo_state.json` (Review Focus #4):

```python
def test_cli_hybrid_go_topo_isolation_and_no_mode_ab_state_file(capsys, tmp_path: Path, monkeypatch):
    monkeypatch.setattr(robot_ctl, "DEFAULT_TOPO_STATE_PATH", tmp_path / ".topo_state.json")
    state_file = tmp_path / ".topo_state.json"

    app = SimulationApp()
    host, port = app.start_http_server(host="127.0.0.1", port=0)
    url = f"http://{host}:{port}"
    stop_evt = threading.Event()
    out: dict = {}
    err_holder: list[BaseException] = []

    def worker():
        try:
            # Review Focus #4: Mode B commands alone must NOT create .topo_state.json
            assert robot_ctl.main(["--url", url, "reset", "--target-x", "1.1", "--target-y", "0.0"]) == 0
            capsys.readouterr()
            assert robot_ctl.main(["--url", url, "advance", "--max-dist", "0.25", "--speed", "0.35"]) == 0
            capsys.readouterr()
            assert not state_file.exists()

            # Mode C: topo show initializes session & strips debug
            assert robot_ctl.main(["--url", url, "topo", "show"]) == 0
            out["topo_show"] = json.loads(capsys.readouterr().out)
            assert state_file.is_file()

            # Add a second URL session to verify isolated reset
            from robot_sim.topo import TopologicalOdometer
            TopologicalOdometer("http://127.0.0.1:59888", state_path=state_file).observe_status(
                out["topo_show"]["after_status"]
            )

            # go moves toward target and strips debug
            assert robot_ctl.main(["--url", url, "go", "--cardinal", "E", "--max-dist", "1.2", "--speed", "0.45"]) == 0
            out["go"] = json.loads(capsys.readouterr().out)

            # Invalid mutually exclusive flags return exit code 2 (argparse)
            try:
                robot_ctl.main(["--url", url, "go", "--cardinal", "E", "--heading-deg", "90"])
                out["mutex_rejected"] = False
            except SystemExit as exc:
                capsys.readouterr()
                out["mutex_rejected"] = exc.code != 0

            # topo reset clears only `url`, keeping 59888 session intact
            assert robot_ctl.main(["--url", url, "topo", "reset"]) == 0
            out["topo_reset"] = json.loads(capsys.readouterr().out)
            out["other_alive"] = TopologicalOdometer.has_active_session("http://127.0.0.1:59888", state_path=state_file)
            out["self_alive"] = TopologicalOdometer.has_active_session(url, state_path=state_file)
        except BaseException as exc:
            err_holder.append(exc)
        finally:
            stop_evt.set()

    t = threading.Thread(target=worker, daemon=True)
    t.start()
    try:
        while not stop_evt.is_set():
            app.tick_headless(step_batch=10)
            time.sleep(0.004)
        t.join(timeout=20.0)
    finally:
        app.shutdown()

    if err_holder:
        raise err_holder[0]

    assert "topo" in out["topo_show"] and "open_cardinals" in out["topo_show"]
    assert "debug" not in json.dumps(out["topo_show"])
    assert out["go"]["event"] in {"target_ready", "max_dist", "static_obstacle_ahead"}
    assert "topo" in out["go"] and "open_cardinals" in out["go"]
    assert "debug" not in json.dumps(out["go"])
    assert out["mutex_rejected"] is True
    assert out["topo_reset"]["reset"] is True
    assert out["other_alive"] is True and out["self_alive"] is False
```

- [ ] **Step 2: Run test to verify it fails**

Run: `.\.venv\Scripts\python.exe -m pytest tests/test_cli.py -k "hybrid_go_topo" -v`
Expected: FAIL (`invalid choice: 'topo'`)

- [ ] **Step 3: Implement `go`, `topo`, and conditional session sync hooks in `robot_ctl.py`**

Cập nhật `robot_ctl.py` theo Mục `5.2.4` của Spec:
- Import `DEFAULT_TOPO_STATE_PATH`, `TopologicalOdometer`, `branches_to_cardinals` từ `robot_sim.topo` và `go_to_decision`, `compute_open_branches` từ `robot_sim.skills` (dùng biến mức module `DEFAULT_TOPO_STATE_PATH` để kiểm thử dễ dàng monkeypatch).
- Trong `build_parser()`:
  - Thêm subparser `go` với `mutually_exclusive_group()` cho `--cardinal` (`choices=["E", "N", "W", "S"]`, cho phép chữ thường qua `type=str.upper`) và `--heading-deg` (`float`), cùng `--max-dist` (`6.0`), `--speed` (`0.55`), `--step-dt` (`0.10`), `--max-chain` (`6`), `--no-chain` (`action="store_true"`).
  - Thêm subparser `topo` với subcommand bắt buộc `topo_action` gồm `show` và `reset`.
- Trong `main()`:
  - Xử lý `go` (khởi tạo `TopologicalOdometer(args.url, state_path=DEFAULT_TOPO_STATE_PATH)` truyền vào `go_to_decision(..., auto_chain=not args.no_chain)`).
  - Xử lý `topo show` và `topo reset`.
  - Ở `cross-gate`: luôn tính `output["open_cardinals"] = branches_to_cardinals(output["open_branches"], yaw_deg)`; nếu `TopologicalOdometer.has_active_session(args.url, state_path=DEFAULT_TOPO_STATE_PATH)` là `True`, cập nhật `record_progress` và gán `output["topo"]`.
  - Ở `reset` / `reload`: nếu `has_active_session`, gọi `reset_session`. Ở `turn` (`--wait`), `scan`, `dock`: nếu `has_active_session`, gọi `observe_status(output["after_status"])`. Ở `move` / `wheels`: nếu `has_active_session`, gọi `cancel_pending_leg(output.get("after_status"))`.

- [ ] **Step 4: Run CLI tests to verify all pass**

Run: `.\.venv\Scripts\python.exe -m pytest tests/test_cli.py -v`
Expected: PASS (`100% passed`)

- [ ] **Step 5: Commit (if running outside restricted git sandbox)**

```powershell
git add robot_ctl.py tests/test_cli.py
git commit -m "feat(cli): add go and topo subcommands with isolated session sync"
```

---

### Task 4: Hiển thị Telemetry Topo trên MuJoCo Studio & Cấu hình Đa Tác tử Mode C

**Files:**
- Modify: `studio/messages.py:31-53`
- Modify: `studio/sim_plugin.py:224-260, 845-881`
- Modify: `studio/viewer_plugin.py:128-146`
- Create: `.agents/skills/robot-multi-agent-rd/references/driver-hybrid-prompt.md`
- Modify: `.agents/skills/robot-multi-agent-rd/SKILL.md`
- Modify: `.agents/skills/robot-multi-agent-rd/references/engineer-prompt.md`
- Modify: `README.md`
- Modify: `tests/test_hybrid_brain.py`, `tests/test_agent_skill.py`

**Interfaces:**
- Consumes: `read_topo_studio_summary`, `DEFAULT_TOPO_STATE_PATH` từ `robot_sim.topo`.
- Produces:
  - `RobotTelemetry` trong `studio/messages.py` có thêm 3 trường có giá trị mặc định ở cuối dataclass:
    `topo_node_count: int = 0`, `topo_current_node: int = -1`, `topo_open_exits: int = 0`.
  - Dòng hiển thị thụ động trong `studio/viewer_plugin.py` khi `t.topo_node_count > 0`:
    `Topo Graph: {t.topo_node_count} nodes | Current: {cur_str} | Open exits: {t.topo_open_exits}`.
  - `.agents/skills/robot-multi-agent-rd/references/driver-hybrid-prompt.md`, cập nhật `SKILL.md` & `engineer-prompt.md` cho **Mode C — `hybrid-brain`**.

- [ ] **Step 1: Write the failing tests for Mode C skill docs & Studio telemetry contract**

Thêm vào cuối `tests/test_agent_skill.py`:
```python
def test_hybrid_brain_mode_c_docs_driver_prompt_and_studio_messages():
    skill_md = ROOT / ".agents" / "skills" / "robot-multi-agent-rd" / "SKILL.md"
    hybrid_ref = ROOT / ".agents" / "skills" / "robot-multi-agent-rd" / "references" / "driver-hybrid-prompt.md"
    engineer_ref = ROOT / ".agents" / "skills" / "robot-multi-agent-rd" / "references" / "engineer-prompt.md"
    studio_msgs = (ROOT / "studio" / "messages.py").read_text(encoding="utf-8")
    studio_sim = (ROOT / "studio" / "sim_plugin.py").read_text(encoding="utf-8")
    studio_viewer = (ROOT / "studio" / "viewer_plugin.py").read_text(encoding="utf-8")

    assert hybrid_ref.is_file(), "driver-hybrid-prompt.md does not exist"
    hybrid_text = hybrid_ref.read_text(encoding="utf-8")
    for kw in (
        ".venv\\Scripts\\python.exe robot_ctl.py",
        "topo show",
        "go",
        "cross-gate",
        "dock",
        "scan --no-recover-occlusion",
        "camera --name front_cam",
        "--debug",
        "arenas",
        "topo reset",
        "explore",
        "approach",
        "overview_cam",
        "out_visits == 0",
        "redundant_revisit_count == 0",
        "loop_closed",
        "static_obstacle_ahead",
    ):
        assert kw in hybrid_text

    skill_text = skill_md.read_text(encoding="utf-8")
    engineer_text = engineer_ref.read_text(encoding="utf-8")
    for kw in ("Mode A", "Mode B", "Mode C", "hybrid-brain", "robot-driver-hybrid", "driver-hybrid-prompt.md", "--max-duration 480"):
        assert kw in skill_text
    for kw in ("Mode C", "hybrid-brain", "driver-hybrid-prompt.md", "topo.py", "go_to_decision"):
        assert kw in engineer_text

    assert "topo_node_count: int = 0" in studio_msgs
    assert "topo_current_node: int = -1" in studio_msgs
    assert "topo_open_exits: int = 0" in studio_msgs
    assert "read_topo_studio_summary" in studio_sim
    assert "Topo Graph:" in studio_viewer
    assert ".topo_state.json" not in studio_viewer  # viewer must never perform 60 FPS disk I/O
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `.\.venv\Scripts\python.exe -m pytest tests/test_agent_skill.py -k "mode_c" -v`
Expected: FAIL (`driver-hybrid-prompt.md does not exist`)

- [ ] **Step 3: Implement MuJoCo Studio telemetry fields & Mode C multi-agent skill prompts/docs**

1. Cập nhật `studio/messages.py`: thêm `topo_node_count: int = 0`, `topo_current_node: int = -1`, `topo_open_exits: int = 0` vào cuối `RobotTelemetry`.
2. Cập nhật `studio/sim_plugin.py`: thêm bộ đệm `_topo_poll_wall: float = 0.0`, `_topo_mtime_ns: int = -1`, `_topo_summary: tuple[int, int, int] = (0, -1, 0)`; cứ mỗi `>= 0.5s` (`<= 2 Hz`) trong `_publish_telemetry` kiểm tra `DEFAULT_TOPO_STATE_PATH.stat().st_mtime_ns` (bọc `try/except OSError`), nếu đổi thì gọi `read_topo_studio_summary()` cập nhật `_topo_summary` (khi tệp bị xóa hoặc khi `api_reset`/`api_reload`/`ArenaSelectEvent`/`ResetRobotEvent` chạy thì đặt lại `(0, -1, 0)`), và truyền vào `RobotTelemetry(...)`.
3. Cập nhật `studio/viewer_plugin.py`: nếu `t.topo_node_count > 0`, hiển thị `Topo Graph: {t.topo_node_count} nodes | Current: #{t.topo_current_node} (hoặc mid-edge nếu < 0) | Open exits: {t.topo_open_exits}` (tuyệt đối không đọc tệp trong `viewer_plugin.py`).
4. Tạo `.agents/skills/robot-multi-agent-rd/references/driver-hybrid-prompt.md` và cập nhật `SKILL.md`, `engineer-prompt.md`, `README.md` theo đúng Mục `5.3` của Spec (bảo toàn mọi từ khóa cũ của Mode A và Mode B).

- [ ] **Step 4: Run the ENTIRE test suite + arena integrity check to verify 100% pass**

Run: `.\.venv\Scripts\python.exe -m pytest -v`
Expected: PASS (`100% passed`, toàn bộ 106 bài kiểm thử cũ + các bài kiểm thử mới của Mode C đều xanh, `test_integrity.py` xác nhận 0 thay đổi trên `models/`).

- [ ] **Step 5: Commit (if running outside restricted git sandbox)**

```powershell
git add studio/ .agents/skills/robot-multi-agent-rd/ README.md tests/test_agent_skill.py tests/test_hybrid_brain.py
git commit -m "feat(mode-c): add robot-driver-hybrid multi-agent workflow and Studio topo HUD"
```
