# Agent-Brain Navigation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Xây dựng bộ công cụ điều khiển vô trạng thái theo sự kiện (`advance`, `cross-gate`, `dock`), các chỉ số đánh giá trí nhớ/tư duy trong hộp đen (`telemetry.py`, `arenas.py`), và chế độ subagent `robot-driver-brain` (Mode B `agent-brain`) để AI Agent làm bộ não tìm đường duy nhất trên cả 6 sa bàn MuJoCo mà vẫn giữ nguyên toàn bộ baseline `auto-skill` và 94 kiểm thử hiện hành.

**Architecture:** Ba kỹ năng phản xạ ngắn hạn ở 20 Hz (`advance_segment`, `cross_dynamic_gate`, `dock_target`) trong `robot_sim/skills.py` và `robot_ctl.py` chỉ giữ làn an toàn và trả quyền về cho Agent ngay khi chạm sự kiện địa hình hoặc vật cản đầu tiên (không tự quay đầu, không lưu dấu chân hay luật bám tường). Tầng `robot_sim/telemetry.py` và `robot_sim/arenas.py` tách `active_drive_time_s` khỏi `agent_think_time_s`, đếm lượt tái thăm ô có hướng `(cell, dir_q)` (`redundant_revisit_count`), và tính `path_efficiency` so với độ dài đường trắc địa ngắn nhất `reference_path_m`. Tầng `.agents/skills/robot-multi-agent-rd/` giữ Mode A (`auto-skill`) làm mặc định và bổ sung Mode B (`agent-brain`) kèm giao thức Sổ tay Đồ thị (Topological Scratchpad) trong `references/driver-brain-prompt.md`.

**Tech Stack:** Python 3.13, MuJoCo 3.x (`SimulationApp`), HTTP keep-alive JSON bridge (`robot_ctl.py`), `pytest`.

**Spec:** [2026-11-19-agent-brain-navigation-design.md](file:///C:/Users/latuan/Programming/robot-demo/docs/superpowers/specs/2026-11-19-agent-brain-navigation-design.md)

---

## Global Constraints

- **CLI Windows tốc độ cao:** Mọi lệnh Python/kiểm thử phải gọi trực tiếp `.venv\Scripts\python.exe` (tuyệt đối không dùng `uv run python`).
- **Kỷ luật Sim-to-Real tuyệt đối:** Các hàm trong `robot_sim/skills.py`, `robot_ctl.py` (chế độ mặc định) và `driver-brain-prompt.md` không bao giờ đọc/tiết lộ `debug`, `robot_xy`, `target_xy`, `active_arena`, `arenas` hay `overview_cam`; không bao giờ gán cứng tọa độ bản đồ, tên sa bàn, quy tắc bám tường `left`/`right`, hay thuật toán Trémaux/U-turn trong các skill mới.
- **Tính bất biến của mô hình XML:** Tuyệt đối không chỉnh sửa `models/robot_2wheel.xml` hay bất kỳ tệp địa hình `models/arena_*.xml` nào.
- **An toàn vật lý thời gian thực `1.0x`:** Mọi kỹ năng vận động mới phải hãm đứng (`_stop_and_settle`) ngoài vùng quét piston/vật cản trước khi trả kết quả về CLI.
- **Bảo toàn baseline:** Toàn bộ 94 kiểm thử hiện tại cùng các lệnh `explore`, `approach`, `scan` cũ phải tiếp tục hoạt động và pass 100% không thoái lui.

---

## Review Focus

5 lớp đầu vào / tình huống lỗi tiềm ẩn từ đặc tả được ghim kiểm thử trực tiếp vào các tác vụ bên dưới:
1. **Đổi mục tiêu tùy chỉnh (`reset --target-x/y`) làm lệch chuẩn trắc địa (Task 1):** Khi `debug.target_xy` sai khác `> 0.05 m` so với `ArenaSpec.default_target_xy` hoặc `arena_id` là `"custom"`/`None`, `record_telemetry` phải xuất `reference_path_m = None` và `path_efficiency = None` thay vì lấy nhầm hằng số của mục tiêu mặc định.
2. **Ngã tư bậc 3 có hai nhánh cụt liên tiếp như nút `(-1.6, 0.0)` của `maze_c` (Task 1):** Đi qua cùng một ô lưới `0.50 m` theo 3 hướng phần tư 90° khác nhau (lần lượt cách nhau `>= 0.80 m`) chỉ được tăng `backtrack_revisit_count == 2` và giữ **`redundant_revisit_count == 0`**; chỉ khi quay lại **cùng cặp có hướng `(cell, dir_q)`** sau khi đi thêm `>= 0.80 m` mới tăng `redundant_revisit_count`.
3. **Tham số không hợp lệ (`NaN`/`inf`/âm) & rò rỉ `debug` (Task 2):** Truyền giá trị không hợp lệ (`max_dist <= 0`, `speed_m_s <= 0`, `timeout_s <= 0`, `stop_distance` ngoài `[0.12, 1.5]`, `NaN`/`inf`) cho `advance_segment`, `cross_dynamic_gate`, `dock_target` phải ném `ValueError` ngay lập tức, và trường `after_status` trả về ở mọi nhánh thoát đều phải loại bỏ `"debug"`.
4. **Phân biệt nhầm vật cản tĩnh khi gọi `cross-gate` hoặc đích bị chắn/quá xa khi gọi `dock` (Task 2):** Gọi `cross_dynamic_gate` trước một khối tĩnh đứng yên `0.42 s` phải trả `{"crossed": False, "reason": "static_obstacle"}` mà không húc vào vật cản; gọi `dock_target` khi có vật cản chắn giữa hoặc đích xa `rf_center > 1.25 m` (`area_pct < 0.9`) phải dừng an toàn và trả `reason in {"obstacle_blocking", "too_far"}`.
5. **Giao diện CLI mới & xử lý lỗi tham số trên `robot_ctl.py` (Task 3):** Các lệnh `advance`, `cross-gate`, `dock` phải xuất JSON sạch (không chứa `"debug"`) với exit code `0` khi hợp lệ, và trả exit code `1` kèm thông báo `[Lỗi điều khiển robot]` ra `stderr` khi truyền tham số sai.

---

## File Structure

- **Modify:** `robot_sim/arenas.py` — thêm `reference_path_m: float = 0.0` vào `ArenaSpec` và gán hằng số trắc địa cho 6 sa bàn.
- **Modify:** `robot_sim/telemetry.py` — bổ sung đo `active_drive_time_s`, `agent_think_time_s`, `command_burst_count`, `unique_cells_visited`, `backtrack_revisit_count`, `redundant_revisit_count`, `reference_path_m`, `path_efficiency` vào `record_telemetry`.
- **Modify:** `robot_sim/skills.py` — tách dùng chung các helper phân loại/né tĩnh gọn nhẹ (giữ nguyên hành vi `explore_maze`/`approach_target`) và cài đặt `compute_open_branches`, `advance_segment`, `cross_dynamic_gate`, `dock_target`.
- **Modify:** `robot_ctl.py` — nối 3 subcommand CLI `advance`, `cross-gate`, `dock`.
- **Create:** `.agents/skills/robot-multi-agent-rd/references/driver-brain-prompt.md` — System Prompt cho subagent `robot-driver-brain` với giao thức Sổ tay Đồ thị (Topological Scratchpad) và bộ lệnh `agent-brain`.
- **Modify:** `.agents/skills/robot-multi-agent-rd/SKILL.md`, `.agents/skills/robot-multi-agent-rd/references/engineer-prompt.md`, `README.md` — bổ sung Mode B (`agent-brain`) bên cạnh Mode A mặc định và tài liệu hóa bộ lệnh + tiêu chí mới.
- **Create / Modify Tests:** `tests/test_arenas.py`, `tests/test_telemetry.py`, `tests/test_agent_brain_skills.py` (tạo mới), `tests/test_cli.py`, `tests/test_agent_skill.py`.

---

### Task 1: Chỉ số Trí nhớ Không gian, Thời gian Suy nghĩ & Mốc Trắc địa `reference_path_m`

**Files:**
- Modify: `robot_sim/arenas.py:45-295`
- Modify: `robot_sim/telemetry.py:74-314`
- Test: `tests/test_arenas.py`, `tests/test_telemetry.py`

**Interfaces:**
- Consumes: `ARENAS` trong `robot_sim/arenas.py`, `debug` (`robot_xy`, `robot_yaw_deg`, `target_xy`, `arena`) trong `robot_sim/telemetry.py`.
- Produces:
  - `ArenaSpec.reference_path_m: float = 0.0` với các giá trị: `2wheel: 0.96`, `maze_a: 4.22`, `maze_b: 6.50`, `maze_c: 3.39`, `dynamic_a: 2.88`, `dynamic_b: 3.16`.
  - Trường JSON bổ sung trong `record_telemetry(...) -> dict`: `active_drive_time_s: float`, `agent_think_time_s: float`, `command_burst_count: int`, `unique_cells_visited: int`, `backtrack_revisit_count: int`, `redundant_revisit_count: int`, `reference_path_m: float | None`, `path_efficiency: float | None`.

- [ ] **Step 1: Write the failing tests for `ArenaSpec.reference_path_m` and new telemetry cognitive metrics**

Thêm vào cuối `tests/test_arenas.py`:
```python
def test_arenas_define_positive_geodesic_reference_paths():
    from robot_sim.arenas import ARENAS

    expected = {
        "2wheel": 0.96,
        "maze_a": 4.22,
        "maze_b": 6.50,
        "maze_c": 3.39,
        "dynamic_a": 2.88,
        "dynamic_b": 3.16,
    }
    assert set(ARENAS) == set(expected)
    for arena_id, ref_m in expected.items():
        assert ARENAS[arena_id].reference_path_m == pytest.approx(ref_m, abs=1e-4)
```

Thêm vào cuối `tests/test_telemetry.py` (bao phủ Review Focus #1, #2, `command_burst_count`, `active_drive_time_s`/`agent_think_time_s`, và reset sạch khi `reset_jumped`):
```python
def test_telemetry_cognitive_metrics_directed_revisits_and_custom_target(tmp_path: Path, monkeypatch):
    out_json = tmp_path / "cog_telemetry.json"
    stop_file = tmp_path / ".cog_stop"
    # Chuỗi giả lập:
    # 1) Ở nút (-1.5, 0.0) đi hướng Tây -> Bắc (ngõ cụt 1 đi thêm >=0.8m) -> quay lại (-1.5, 0.0) hướng Nam (0 redundant)
    # 2) Đi ngõ cụt 2 (>=0.8m) -> quay lại (-1.5, 0.0) hướng Bắc (0 redundant)
    # 3) Tái phạm quay lại đúng (-1.5, 0.0) hướng Tây lần thứ 2 -> redundant_revisit_count == 1
    # 4) Cuối cùng tới đích ở (1.66, 0.0) của dynamic_a với moving xen kẽ đứng yên
    trace = [
        # (moving, fwd_spd, xy, yaw_deg, target_vis, off_x, rf_c, target_xy)
        (False, 0.0, [-1.50, 0.00], 180.0, False, None, 1.5, [1.66, 0.0]),  # nghỉ think 1
        (True,  0.3, [-1.50, 0.00], 180.0, False, None, 1.5, [1.66, 0.0]),  # burst 1: ô (-3,0) dir W (lần 1)
        (True,  0.4, [-1.50, 0.45],  90.0, False, None, 1.2, [1.66, 0.0]),
        (True,  0.4, [-1.50, 0.90],  90.0, False, None, 0.8, [1.66, 0.0]),
        (True,  0.4, [-1.50, 0.45], -90.0, False, None, 1.2, [1.66, 0.0]),
        (False, 0.0, [-1.50, 0.00], -90.0, False, None, 1.5, [1.66, 0.0]),  # backtrack 1 vào (-3,0), dir S (lần 1) -> think 2
        (True,  0.4, [-1.50, -0.45], -90.0, False, None, 1.2, [1.66, 0.0]), # burst 2
        (True,  0.4, [-1.50, -0.90], -90.0, False, None, 0.8, [1.66, 0.0]),
        (True,  0.4, [-1.50, -0.45],  90.0, False, None, 1.2, [1.66, 0.0]),
        (True,  0.3, [-1.50, 0.00],  90.0, False, None, 1.5, [1.66, 0.0]),  # backtrack 2 vào (-3,0), dir N (lần 1) -> vẫn 0 redundant
        (True,  0.4, [-1.05, 0.00],   0.0, False, None, 1.5, [1.66, 0.0]),
        (True,  0.4, [-0.60, 0.00],   0.0, False, None, 1.5, [1.66, 0.0]),
        (True,  0.4, [-1.05, 0.00], 180.0, False, None, 1.5, [1.66, 0.0]),
        (True,  0.3, [-1.50, 0.00], 180.0, False, None, 1.5, [1.66, 0.0]),  # quay lại (-3,0) dir W lần 2 (>=0.8m) -> redundant = 1
        (False, 0.0, [1.22, 0.00],    0.0, True,  0.02, 0.25, [1.66, 0.0]), # jump reset hoặc test normal arrival; dùng bước liên tục ở dưới
    ]
    # Test A: không jump ở mẫu cuối -> thay [-1.50, 0.00] -> nối liên tiếp để không reset_jumped
    continuous_trace = trace[:-1] + [
        (True, 0.4, [-1.15, 0.00], 0.0, False, None, 1.5, [1.66, 0.0]),
        (True, 0.4, [-0.80, 0.00], 0.0, False, None, 1.5, [1.66, 0.0]),
        (True, 0.4, [-0.45, 0.00], 0.0, False, None, 1.5, [1.66, 0.0]),
        (True, 0.4, [-0.10, 0.00], 0.0, False, None, 1.5, [1.66, 0.0]),
        (True, 0.4, [0.25,  0.00], 0.0, False, None, 1.2, [1.66, 0.0]),
        (True, 0.4, [0.60,  0.00], 0.0, False, None, 0.9, [1.66, 0.0]),
        (True, 0.4, [0.95,  0.00], 0.0, False, None, 0.5, [1.66, 0.0]),
        (False, 0.0, [1.22, 0.00], 0.0, True, 0.02, 0.25, [1.66, 0.0]),
    ]
    idx = 0

    def fake_fetch(_url: str, timeout: float = 4.0) -> dict:
        nonlocal idx
        m, spd, xy, yaw, tv, ox, rfc, txy = continuous_trace[idx]
        idx += 1
        if idx >= len(continuous_trace):
            stop_file.write_text("stop\n", encoding="utf-8")
        return {
            "sim_time": 0.2 * idx,
            "moving": m,
            "sensors": {
                "rangefinders_m": {"left": 1.0, "center": rfc, "right": 1.0},
                "forward_speed_m_s": spd,
                "yaw_rate_rad_s": 0.0,
            },
            "vision": {
                "target_visible": tv,
                "target": {"offset_x": ox, "area_pct": 20.0} if tv else None,
                "obstacles": [],
            },
            "debug": {
                "arena": "dynamic_a",
                "robot_xy": xy,
                "robot_yaw_deg": yaw,
                "target_xy": txy,
                "contact_count": 0,
            },
        }

    monkeypatch.setattr("robot_sim.telemetry._fetch_raw_status", fake_fetch)
    s1 = record_telemetry("http://127.0.0.1:8765", out_path=out_json, stop_flag_path=stop_file, hz=50.0)
    assert s1["goal_reached"] is True
    assert s1["command_burst_count"] == 3
    assert s1["backtrack_revisit_count"] >= 2
    assert s1["redundant_revisit_count"] == 1
    assert s1["unique_cells_visited"] >= 5
    assert s1["reference_path_m"] == 2.88
    assert s1["path_efficiency"] == pytest.approx(round(min(1.0, 2.88 / s1["path_length_m"]), 3), abs=1e-3)
    assert s1["active_drive_time_s"] > 0.0
    assert s1["agent_think_time_s"] > 0.0
    assert abs((s1["active_drive_time_s"] + s1["agent_think_time_s"]) - s1["wall_time_s"]) <= 0.15

    # Test B: custom target_xy (> 0.05m từ default_target_xy) -> reference_path_m và path_efficiency phải là None
    # + kiểm chứng reset_jumped xóa lịch sử ô cũ
    out_json2 = tmp_path / "custom_target.json"
    stop_file2 = tmp_path / ".custom_stop"
    custom_trace = [
        (True, 0.2, [0.0, 0.0], 0.0, False, None, 1.0, [0.5, 0.9]),
        (True, 0.2, [0.0, 0.9], 0.0, False, None, 1.0, [0.5, 0.9]),   # step > 0.45 -> reset_jumped
        (False, 0.0, [0.1, 0.9], 0.0, True, 0.01, 0.25, [0.5, 0.9]),
    ]
    c_idx = 0

    def fake_fetch_custom(_url: str, timeout: float = 4.0) -> dict:
        nonlocal c_idx
        m, spd, xy, yaw, tv, ox, rfc, txy = custom_trace[c_idx]
        c_idx += 1
        if c_idx >= len(custom_trace):
            stop_file2.write_text("stop\n", encoding="utf-8")
        return {
            "sim_time": 0.2 * c_idx,
            "moving": m,
            "sensors": {"rangefinders_m": {"center": rfc}, "forward_speed_m_s": spd, "yaw_rate_rad_s": 0.0},
            "vision": {"target_visible": tv, "target": {"offset_x": ox, "area_pct": 20.0} if tv else None, "obstacles": []},
            "debug": {"arena": "dynamic_a", "robot_xy": xy, "robot_yaw_deg": yaw, "target_xy": txy, "contact_count": 0},
        }

    monkeypatch.setattr("robot_sim.telemetry._fetch_raw_status", fake_fetch_custom)
    s2 = record_telemetry("http://127.0.0.1:8765", out_path=out_json2, stop_flag_path=stop_file2, hz=50.0)
    assert s2["reference_path_m"] is None
    assert s2["path_efficiency"] is None
    assert s2["redundant_revisit_count"] == 0
```

- [ ] **Step 2: Run tests to verify failure**

Run: `.venv\Scripts\python.exe -m pytest tests/test_arenas.py tests/test_telemetry.py -v`
Expected: FAIL (`ArenaSpec` has no `reference_path_m` / `KeyError: 'command_burst_count'`).

- [ ] **Step 3: Implement `ArenaSpec.reference_path_m` and cognitive telemetry metrics**

1. Trong `robot_sim/arenas.py`:
   - Thêm trường `reference_path_m: float = 0.0` vào dataclass `ArenaSpec`.
   - Gán giá trị cho 6 sa bàn trong `ARENAS`: `"2wheel": 0.96`, `"maze_a": 4.22`, `"maze_b": 6.50`, `"maze_c": 3.39`, `"dynamic_a": 2.88`, `"dynamic_b": 3.16`.
2. Trong `robot_sim/telemetry.py`:
   - Import `ARENAS` từ `robot_sim.arenas`.
   - Khởi tạo biến theo dõi: `active_drive_time_s = 0.0`, `agent_think_time_s = 0.0`, `prev_poll_wall: float | None = None`, `command_burst_count = 0`, `last_cell: tuple[int, int] | None = None`, `last_dir_cell: tuple[tuple[int, int], int] | None = None`, `cell_last_path: dict[tuple[int, int], float] = {}`, `dir_cell_last_path: dict[tuple[tuple[int, int], int], float] = {}`, `backtrack_revisit_count = 0`, `redundant_revisit_count = 0`.
   - Trong vòng lặp mẫu (mỗi khi fetch thành công):
     - Tính `dt_sample = (t0 - prev_poll_wall)` nếu `prev_poll_wall is not None` else `0.0`; đặt `prev_poll_wall = t0`.
     - Xác định `is_active = bool(moving or fwd_spd > 0.02 or yaw_spd > 0.04)`; cộng `dt_sample` vào `active_drive_time_s` nếu `is_active`, ngược lại cộng vào `agent_think_time_s`.
     - Khi `moving and not prev_moving`: `command_burst_count += 1`.
     - Nếu `reset_jumped` xảy ra: reset `cell_last_path.clear()`, `dir_cell_last_path.clear()`, `last_cell = None`, `last_dir_cell = None`, `backtrack_revisit_count = 0`, `redundant_revisit_count = 0`.
     - Với `curr_xy` hợp lệ: `cell = (round(float(curr_xy[0]) / 0.50), round(float(curr_xy[1]) / 0.50))`. Nếu `cell != last_cell` và `cell in cell_last_path` và `(path_length_m - cell_last_path[cell]) >= 0.80`: tăng `backtrack_revisit_count += 1`. Cập nhật `cell_last_path[cell] = path_length_m` và `last_cell = cell`.
     - Tương tự với `end_yaw_deg` hợp lệ: `dir_q = int(((end_yaw_deg + 45.0) % 360.0) // 90.0)`, `dcell = (cell, dir_q)`. Nếu `dcell != last_dir_cell` và `dcell in dir_cell_last_path` và `(path_length_m - dir_cell_last_path[dcell]) >= 0.80`: tăng `redundant_revisit_count += 1`. Cập nhật `dir_cell_last_path[dcell] = path_length_m` và `last_dir_cell = dcell`.
   - Chuẩn hóa tổng `active_drive_time_s` + `agent_think_time_s` so với `wall_time_s` (từ `t_wall_start` tới kết thúc) sao cho chênh lệch tối đa không quá nửa chu kỳ poll đầu; tính `spec = ARENAS.get(arena_id or "")`, nếu `spec is not None` và `target_xy is not None` và `math.hypot(target_xy[0] - spec.default_target_xy[0], target_xy[1] - spec.default_target_xy[1]) <= 0.05` thì `reference_path_m = spec.reference_path_m`, ngược lại `None`; `path_efficiency = round(min(1.0, reference_path_m / path_length_m), 3)` khi `reference_path_m is not None` và `goal_reached` và `path_length_m > 0.0`, ngược lại `None`.
   - Thêm cả 8 khóa mới vào dict `summary` (giữ size file `< 2 KB`).

- [ ] **Step 4: Run tests to verify pass**

Run: `.venv\Scripts\python.exe -m pytest tests/test_arenas.py tests/test_telemetry.py -v`
Expected: PASS (toàn bộ tests cũ + 2 test mới đều xanh).

- [ ] **Step 5: Commit**

```powershell
git add robot_sim/arenas.py robot_sim/telemetry.py tests/test_arenas.py tests/test_telemetry.py
git commit -m "feat(arenas,telemetry): add geodesic reference paths and cognitive brain metrics"
```

---

### Task 2: Bộ 3 Công cụ Skill Vô trạng thái (`advance_segment`, `cross_dynamic_gate`, `dock_target`)

**Files:**
- Modify: `robot_sim/skills.py`
- Create: `tests/test_agent_brain_skills.py`

**Interfaces:**
- Consumes: `_require_finite`, `_clean_status`, `_stop_and_settle`, `_settle_at_zero_hold`, `closed_loop_turn`, `closed_loop_move`, `_estimate_obs_cam_dist_m`, `_wrap_angle_deg`, `MAX_WHEEL_RAD_S`, `WHEEL_RADIUS`, `TRACK_WIDTH`, `FRONT_CAM_TAN_HALF_FOVX` trong `robot_sim/skills.py`.
- Produces (xuất khẩu ở module top-level trong `robot_sim/skills.py`):
  - `compute_open_branches(rf: dict[str, float]) -> list[str]`
  - `advance_segment(call_fn: BridgeCall, max_dist: float = 1.8, speed_m_s: float = 0.55, step_dt: float = 0.10) -> dict`
  - `cross_dynamic_gate(call_fn: BridgeCall, timeout_s: float = 12.0) -> dict`
  - `dock_target(call_fn: BridgeCall, stop_distance: float = 0.25, max_steps: int = 80, step_dt: float = 0.12) -> dict`

- [ ] **Step 1: Write the failing tests in `tests/test_agent_brain_skills.py`**

Tạo `tests/test_agent_brain_skills.py` kiểm thử cả unit (mock bridge) lẫn tích hợp MuJoCo headless (`SimulationApp`):
1. `test_agent_brain_skills_validate_inputs_and_compute_open_branches`:
   - Kiểm tra Review Focus #3: truyền `NaN`, `inf`, `max_dist <= 0`, `speed_m_s <= 0`, `step_dt` ngoài `[0.04, 1.0]` vào `advance_segment`; `timeout_s <= 0` vào `cross_dynamic_gate`; `stop_distance` ngoài `[0.12, 1.5]` hoặc `max_steps <= 0` vào `dock_target` đều `pytest.raises(ValueError)`.
   - Kiểm tra `compute_open_branches`: `{"center": 0.9, "left": 0.5, "right": 0.5, "left90": 0.8, "right90": 0.3}` trả `["left", "straight"]`; khi `center=0.35`, `left90=0.8`, `right90=0.8` trả `["left", "right"]`.
2. `test_advance_segment_events_stateless_unit`:
   - Mock các kịch bản sự kiện trả `after_status` không có khóa `"debug"`:
     - Ngã ba (`left90` chuyển mở sau `moved >= 0.22 m` -> `event == "junction"`, `"left" in res["open_branches"]`).
     - Ngõ cụt (`center < 0.40`, `left90 < 0.52`, `right90 < 0.52` -> `event == "dead_end"`, `res["open_branches"] == []`, **không tự quay 180°**).
     - Góc L (`center < 0.40`, `left90 > 0.52` -> `event == "junction"`, `res["open_branches"] == ["left"]`).
     - Vật cản động xuất hiện & dịch chuyển trong 0.42 s -> `event == "dynamic_gate_ahead"`, `res["obstacle"]["motion"] == "dynamic"`.
     - Vật cản tĩnh đứng yên 0.42 s -> `event == "static_obstacle_ahead"`, `res["obstacle"]["motion"] == "static"`, có `flank_clearance_m`.
     - Đích ở cự ly gần (`target_visible=True`, `rf_center=0.75`) -> `event == "target_ready"`.
     - Đi hết `max_dist` sạch sẽ -> `event == "max_dist"`.
3. `test_cross_dynamic_gate_and_dock_target_refusal_unit`:
   - Kiểm tra Review Focus #4:
     - Gọi `cross_dynamic_gate` khi vật cản đứng yên suốt 0.42 s trả `{"crossed": False, "reason": "static_obstacle"}` và không gửi lệnh tiến xuyên vật cản.
     - Gọi `dock_target` khi `target_visible=False` trả `{"reached": False, "reason": "not_visible"}`; khi `center=2.2, area_pct=0.25` trả `{"reached": False, "reason": "too_far"}`; khi có obstacle ở gần hơn đích (`closer_center_obs` hoặc `min_front_rf < 0.38`) trả `{"reached": False, "reason": "obstacle_blocking"}`, mọi `after_status` đều không chứa `"debug"`.
4. `test_agent_brain_skills_solve_maze_a_on_explicit_agent_turns`:
   - Chạy `SimulationApp(resolve_arena_model("maze_a"))` với **vòng lặp điều khiển tối giản giả lập bộ não** chỉ dùng `advance_segment`, `closed_loop_turn`, `dock_target` (không dùng `explore_maze`/`approach_target`), khẳng định chạm `junction`/`dead_end` thật, quay bằng tay, cuối cùng `dock_target` trả `reached=True` và `debug.contact_count == 0`.
5. `test_agent_brain_skills_cross_dynamic_a_lane_by_lane_and_dynamic_b_s_curve`:
   - Trên `dynamic_a`: lặp tối đa 8 bước (`advance_segment` -> khi `"dynamic_gate_ahead"` gọi `cross_dynamic_gate`, khẳng định phải gọi `cross_dynamic_gate` **>= 2 lần** cho các làn tuần tra khác nhau chứ không nuốt trọn 3 làn trong 1 lệnh; khi `"target_ready"` gọi `dock_target`), kết quả `reached == True` và `debug["contact_count"] == 0`.
   - Trên `dynamic_b`: chuỗi `advance_segment` + `cross_dynamic_gate` + góc rẽ tường theo `open_branches` + `dock_target`, kết quả `reached == True` và `debug["contact_count"] == 0`.

- [ ] **Step 2: Run test to verify failure**

Run: `.venv\Scripts\python.exe -m pytest tests/test_agent_brain_skills.py -v`
Expected: FAIL with `ImportError: cannot import name 'advance_segment' from 'robot_sim.skills'`.

- [ ] **Step 3: Implement `compute_open_branches`, `advance_segment`, `cross_dynamic_gate`, `dock_target` in `robot_sim/skills.py`**

Yêu cầu cài đặt trong `robot_sim/skills.py` (dùng chung helper phân loại tĩnh/động `0.42 s`, tính khoảng cách pinhole `_estimate_obs_cam_dist_m`, điều khiển PD giữ tâm hành lang và nép hốc bất đối xứng để tránh trùng lặp với `explore_maze`/`approach_target` mà không làm đổi hành vi của hai hàm cũ):
1. `compute_open_branches(rf: dict[str, float]) -> list[str]`: trả các hướng theo thứ tự chữ cái hoặc cố định `["left", "straight", "right"]` thỏa: `"left"` nếu `rf_l90 > 0.55`, `"straight"` nếu `rf_c > 0.62 and min(rf_l, rf_r) > 0.34`, `"right"` nếu `rf_r90 > 0.55` (chuyển sang kiểm thử thống nhất tập/thứ tự `["left", "straight", "right"]`).
2. `_probe_front_obstacle_motion(call_fn, initial_st, z_limit=0.75) -> tuple[str, dict, dict | None]`: helper chung vô trạng thái đứng yên `<= 0.42 s` so biến thiên `offset_x >= 0.018` hoặc `min_front_rf >= 0.015` (cộng xuất hiện/biến mất obstacle key), trả `("dynamic" | "static" | "none", latest_st, obstacle_summary)`.
3. `advance_segment(call_fn, max_dist=1.8, speed_m_s=0.55, step_dt=0.10) -> dict`:
   - Kiểm tra `_require_finite`, `max_dist > 0`, `0.05 <= speed_m_s <= MAX_WHEEL_RAD_S * WHEEL_RADIUS`, `0.04 <= step_dt <= 1.0`.
   - **Không lưu bất kỳ state nào giữa các lần gọi.**
   - Thực hiện kiểm tra đầu lệnh (bao gồm chờ `2.2 s` sim-time cho `pocket_piston_corridor` nếu đứng đầu nhánh rộng có tầm nhìn xa mà chưa thấy piston), kiểm tra `target_ready`, kiểm tra vật cản phía trước qua `_probe_front_obstacle_motion` (lùi `-0.10 m` nếu `min_front_rf < 0.34` hoặc `z_est < 0.26`), vòng lặp 20 Hz giữ tâm hành lang (`left90 < 0.65 and right90 < 0.65`) hoặc giữ góc IMU lúc xuất phát (nếu đang trong hành lang hẹp và lệch `<= 15°` khỏi bội `90°` thì bám trục bội `90°`), phát hiện `junction` sau `moved >= 0.22 m` (tiến tới tâm giao lộ `~0.32 m` nhưng dừng ngay nếu phía trước `<= 0.36 m` hoặc thấy vật cản), xử lý chạm tường trước mặt (`junction` nếu có `left90/right90 > 0.52`, ngược lại `dead_end`), hoặc `max_dist`.
   - Cuối lệnh luôn gọi `_stop_and_settle(call_fn, include_vision=True)` (khi chưa dừng) và trả `{"event", "moved_m", "open_branches", "obstacle", "flank_clearance_m", "after_status"}` với `after_status = _clean_status(...)`.
4. `cross_dynamic_gate(call_fn, timeout_s=12.0) -> dict`:
   - Kiểm tra `_require_finite`, `timeout_s > 0.0`.
   - Trong không gian mở (`min(l90, r90) > 0.75`) nếu chưa thấy obstacle phía trước, liếc nhanh `±26°` (mỗi bên `0.12 s`) để thu mẫu cánh giống `approach_target`, rồi trả mũi về `yaw0`.
   - Nếu thấy obstacle phía trước, kiểm tra `_probe_front_obstacle_motion` (`<= 0.42 s`); nếu đứng yên và `min_front_rf < 0.70` → trả `{"crossed": False, "reason": "static_obstacle", "moved_m": 0.0, "open_branches": ..., "after_status": _clean_status(st)}`.
   - Đứng chờ (lùi `-0.10 m` nếu quá sát `< 0.34 m`) tối đa `timeout_s` sim-time với điều kiện mở cổng 2 nhịp liên tiếp theo hướng rút lui (`center > 0.78`, `min(left, right) > 0.38`, và không có obstacle `z <= 0.75` đang trong vùng tâm `|ox| < 0.78` hoặc tiến vào tâm `(ox * d_ox) <= 0.004`). Hết hạn thì trả `{"crossed": False, "reason": "timeout", ...}`.
   - Khi mở cổng: đi thẳng giữ hướng `commit_dist = min(1.05, max(0.72, nearest_gate_z + 0.48))` trong không gian tối đa 1 cổng; dừng ngay với `reason = "wall_ahead"` nếu `center <= 0.34`, hoặc dừng an toàn với `reason = "next_obstacle_ahead"` sau khi qua `moved >= nearest_gate_z + 0.36` mà có vật cản tiếp theo trong `z_est <= 0.52` hoặc `min_front_rf < 0.44`. Nếu không bị ngắt sớm thì dừng tại `commit_dist` với `reason = "cleared"`.
   - Sau khi dừng, nếu nằm tại miệng nối bất đối xứng (`center > 0.90 and open_side_90 > 0.85 and wall_side_90 > 0.29`), thực hiện nép giữa hốc bằng helper chung với `_finish_target_seen`.
5. `dock_target(call_fn, stop_distance=0.25, max_steps=80, step_dt=0.12) -> dict`:
   - Kiểm tra `_require_finite`, `0.12 <= stop_distance <= 1.5`, `max_steps >= 1`, `0.04 <= step_dt <= 1.0`.
   - Từ chối sớm (`not_visible` / `too_far` / `obstacle_blocking`) theo Review Focus #4; điều khiển servo bám tâm `offset_x` và giảm tốc êm về `stop_distance`, kiểm tra khớp cả 3 điều kiện đỗ và trả `{"reached", "reason", "steps", "after_status"}` với `_clean_status`.

- [ ] **Step 4: Run tests to verify pass**

Run: `.venv\Scripts\python.exe -m pytest tests/test_agent_brain_skills.py -v`
Expected: PASS (toàn bộ test mới xanh 100%, `contact_count == 0` trên `maze_a`, `dynamic_a`, `dynamic_b`).

- [ ] **Step 5: Commit**

```powershell
git add robot_sim/skills.py tests/test_agent_brain_skills.py
git commit -m "feat(skills): add stateless event-driven advance_segment, cross_dynamic_gate, and dock_target"
```

---

### Task 3: Giao diện CLI mới (`advance`, `cross-gate`, `dock`) trong `robot_ctl.py`

**Files:**
- Modify: `robot_ctl.py:12-20,233-248,389-407`
- Test: `tests/test_cli.py`

**Interfaces:**
- Consumes: `advance_segment`, `cross_dynamic_gate`, `dock_target` từ `robot_sim/skills.py`.
- Produces: CLI subcommands trên `robot_ctl.py`:
  - `advance [--max-dist 1.8] [--speed 0.55] [--step-dt 0.10]`
  - `cross-gate [--timeout 12.0]`
  - `dock [--stop-distance 0.25] [--max-steps 80] [--step-dt 0.12]`

- [ ] **Step 1: Write the failing tests in `tests/test_cli.py`**

Thêm vào cuối `tests/test_cli.py` (bao phủ Review Focus #5):
```python
def test_cli_agent_brain_subcommands_run_strip_debug_and_validate_args(capsys):
    app = SimulationApp()
    host, port = app.start_http_server(host="127.0.0.1", port=0)
    url = f"http://{host}:{port}"
    stop_evt = threading.Event()
    res: dict = {}
    err_holder: list[BaseException] = []

    def worker():
        try:
            assert robot_ctl.main(["--url", url, "reset", "--target-x", "0.95", "--target-y", "0.0"]) == 0
            capsys.readouterr()

            assert robot_ctl.main(["--url", url, "advance", "--max-dist", "0.50", "--speed", "0.40"]) == 0
            res["advance"] = json.loads(capsys.readouterr().out)

            assert robot_ctl.main(["--url", url, "dock", "--stop-distance", "0.25"]) == 0
            res["dock"] = json.loads(capsys.readouterr().out)

            code_bad_adv = robot_ctl.main(["--url", url, "advance", "--max-dist", "-1.0"])
            err_bad_adv = capsys.readouterr().err
            res["bad_adv"] = (code_bad_adv, err_bad_adv)

            code_bad_gate = robot_ctl.main(["--url", url, "cross-gate", "--timeout", "0"])
            err_bad_gate = capsys.readouterr().err
            res["bad_gate"] = (code_bad_gate, err_bad_gate)
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

    assert res["advance"]["event"] in {"target_ready", "max_dist"}
    assert "debug" not in res["advance"]["after_status"]
    assert res["dock"]["reached"] is True
    assert "debug" not in res["dock"]["after_status"]
    assert res["bad_adv"][0] == 1 and "Lỗi điều khiển robot" in res["bad_adv"][1]
    assert res["bad_gate"][0] == 1 and "Lỗi điều khiển robot" in res["bad_gate"][1]
```

- [ ] **Step 2: Run test to verify failure**

Run: `.venv\Scripts\python.exe -m pytest tests/test_cli.py::test_cli_agent_brain_subcommands_run_strip_debug_and_validate_args -v`
Expected: FAIL (`SystemExit` / invalid choice `'advance'`).

- [ ] **Step 3: Add `advance`, `cross-gate`, and `dock` subparsers and handlers to `robot_ctl.py`**

1. Import `advance_segment`, `cross_dynamic_gate`, `dock_target` từ `robot_sim.skills` ở đầu `robot_ctl.py`.
2. Trong `build_parser()`: thêm 3 subparser `advance` (`--max-dist` default `1.8`, `--speed` default `0.55`, `--step-dt` default `0.10`), `cross-gate` (`--timeout` default `12.0`), và `dock` (`--stop-distance` default `0.25`, `--max-steps` default `80`, `--step-dt` default `0.12`).
3. Trong `main()`: gọi `advance_segment`, `cross_dynamic_gate`, `dock_target` qua `call_fn` chuẩn; bảo đảm mọi ngoại lệ `ValueError` trả mã thoát `1` kèm `[Lỗi điều khiển robot]`.

- [ ] **Step 4: Run CLI tests to verify pass**

Run: `.venv\Scripts\python.exe -m pytest tests/test_cli.py -v`
Expected: PASS (tất cả test trong `test_cli.py` đều xanh).

- [ ] **Step 5: Commit**

```powershell
git add robot_ctl.py tests/test_cli.py
git commit -m "feat(cli): expose advance, cross-gate, and dock commands for agent-brain control"
```

---

### Task 4: Prompt Bộ não (`driver-brain-prompt.md`), Dual-Mode `SKILL.md`, Tài liệu & Kiểm thử Toàn bộ

**Files:**
- Create: `.agents/skills/robot-multi-agent-rd/references/driver-brain-prompt.md`
- Modify: `.agents/skills/robot-multi-agent-rd/SKILL.md`
- Modify: `.agents/skills/robot-multi-agent-rd/references/engineer-prompt.md`
- Modify: `README.md`
- Modify: `tests/test_agent_skill.py`

**Interfaces:**
- Consumes: CLI `advance`, `cross-gate`, `dock`, `scan --no-recover-occlusion`, `turn`, `move`, `camera --name front_cam`, `status --vision` và telemetry metrics ở Task 1–3.
- Produces: Template `driver-brain-prompt.md` cho subagent `robot-driver-brain`, tài liệu Mode A (default) / Mode B (`agent-brain`) trong `SKILL.md` và `engineer-prompt.md`.

- [ ] **Step 1: Extend `tests/test_agent_skill.py` with Mode B assertions**

Thêm kiểm tra trong `tests/test_agent_skill.py`:
- File `.agents/skills/robot-multi-agent-rd/references/driver-brain-prompt.md` tồn tại; yêu cầu `.venv\Scripts\python.exe robot_ctl.py`; cho phép `advance`, `cross-gate`, `dock`, `scan --no-recover-occlusion`, `camera --name front_cam`; cấm `--debug`, `arenas`, `explore`, `approach`, `overview_cam`; mô tả Sổ tay Đồ thị (`Node`, `odom_x_m`, `odom_y_m`, `gyro_yaw_deg`).
- `SKILL.md` và `engineer-prompt.md` vẫn giữ nguyên toàn bộ từ khóa kiểm tra cũ, đồng thời đề cập `Mode A` / `auto-skill`, `Mode B` / `agent-brain`, `driver-brain-prompt.md`, `redundant_revisit_count`, và lệnh cấm sửa `models/robot_2wheel.xml` hay thêm bộ nhớ giải mê cung vào `skills.py` khi chạy Mode B.

- [ ] **Step 2: Run skill-doc test to verify failure**

Run: `.venv\Scripts\python.exe -m pytest tests/test_agent_skill.py -v`
Expected: FAIL (`AssertionError: driver-brain-prompt.md does not exist`).

- [ ] **Step 3: Create `driver-brain-prompt.md` and update `SKILL.md`, `engineer-prompt.md`, `README.md`**

1. Tạo `.agents/skills/robot-multi-agent-rd/references/driver-brain-prompt.md` theo đúng Mục 5.3 của spec:
   - Quy tắc Sim-to-Real & giới hạn tool (cấm `--debug`, `arenas`, `reset`, `reload`, `demo`, `telemetry`, `explore`, `approach`, `overview_cam`; chỉ dùng `front_cam` chọn lọc khi nhập nhằng).
   - Giao thức **Sổ tay Đồ thị (Topological Scratchpad)**: định nghĩa nút `<= 0.40 m`, 4 hướng chuẩn `0°/+90°/±180°/-90°`, ngăn xếp quay lui `dead_end`, nhận diện vòng lặp trùng nút `Node #k` (`maze_c`), lách vật cản tĩnh qua `flank_clearance_m` (`2wheel`), vượt từng làn `cross-gate` -> `advance` (`dynamic_a`, `dynamic_b`), và `dock --stop-distance 0.25`.
   - Format phản hồi cuối gồm chuỗi quyết định topo, số liệu dừng cuối, và tóm tắt nút/vòng lặp đã phát hiện.
2. Cập nhật `.agents/skills/robot-multi-agent-rd/SKILL.md`:
   - Đặt **Mode A (`auto-skill` — `robot-driver`)** là mặc định; bổ sung phần **Mode B (`agent-brain` — `robot-driver-brain`, `telemetry record --hz 10 --max-duration 900`)** với tiêu chí nghiệm thu không phụ thuộc `wall_time_s` (`goal_reached`, `contact_count == 0`, `near_collision_count == 0`, `error_count == 0`, `redundant_revisit_count == 0`, `path_efficiency`, `active_drive_time_s` vs `agent_think_time_s`, `command_burst_count`) và phạm vi R&D tối đa 3 vòng của `robot-engineer`.
3. Cập nhật `.agents/skills/robot-multi-agent-rd/references/engineer-prompt.md`:
   - Thêm mục **Mode B (`agent-brain`) Red Lines**: cho phép sửa `driver-brain-prompt.md` và ngưỡng phản xạ vô trạng thái ở `advance_segment`/`cross_dynamic_gate`/`dock_target`, nhưng **cấm** sửa `models/robot_2wheel.xml`, `models/arena_*.xml` hay thêm luật giải mê cung/trạng thái dài hạn vào `robot_sim/skills.py`.
4. Cập nhật `README.md`: ghi chú ngắn gọn các lệnh mới `advance`, `cross-gate`, `dock` và chế độ thử nghiệm `agent-brain` dùng `robot-driver-brain`.

- [ ] **Step 4: Run full `pytest -v` suite on dedicated CPU**

Run (khi không có `sim_server.py` nền chạy song song): `.venv\Scripts\python.exe -m pytest -v`
Expected: PASS 100% toàn bộ suite cũ (94) + toàn bộ test mới, `0 failed`.

- [ ] **Step 5: Commit**

```powershell
git add .agents/skills/robot-multi-agent-rd/SKILL.md .agents/skills/robot-multi-agent-rd/references/driver-brain-prompt.md .agents/skills/robot-multi-agent-rd/references/engineer-prompt.md README.md tests/test_agent_skill.py
git commit -m "feat(agent-brain): add driver-brain prompt, dual-mode SKILL.md, and docs"
```

---

## Live Acceptance Verification (Sau khi hoàn tất cả 4 Task)

Theo Mục 6.3 của spec, sau khi commit đủ 4 Task, chạy nghiệm thu **Mode B (`agent-brain`)** trên `sim_server.py --headless --port 8766` (10 Hz blackbox `--max-duration 900`) với subagent mù `robot-driver-brain` lần lượt trên **`2wheel`**, **`maze_b`**, **`maze_c`**, **`dynamic_a`**, **`dynamic_b`**, và lập bảng so sánh trực diện giữa **`auto-skill` (Mode A)** vs **`agent-brain` (Mode B)** theo các chỉ số: `goal_reached`, `contact_count`, `near_collision_count`, `path_length_m`, `path_efficiency`, `backtrack_revisit_count`, `redundant_revisit_count`, `command_burst_count`, `active_drive_time_s`, `agent_think_time_s`.
