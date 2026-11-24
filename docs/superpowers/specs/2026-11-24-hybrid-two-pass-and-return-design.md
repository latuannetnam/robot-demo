# Thiết kế Điều hướng Hai Pha & Khứ hồi dựa trên Đồ thị Topo (`hybrid-brain` Mở rộng — Hướng A + B)

**Ngày thiết kế:** 2026-11-24  
**Quy mô công việc:** Dự án đầy đủ (Full Project Design)  
**Trạng thái:** Đã được người dùng phê duyệt (Approved)  
**Phạm vi:** Ghi nhận nút đích & chế độ tái sử dụng bản đồ ([robot_sim/topo.py](file:///d:/latuan/Programming/robot-demo/robot_sim/topo.py)), thực thi chuỗi lộ trình & dừng đúng tâm nút ([robot_sim/skills.py](file:///d:/latuan/Programming/robot-demo/robot_sim/skills.py), [robot_ctl.py](file:///d:/latuan/Programming/robot-demo/robot_ctl.py)), chấm điểm Hộp đen đa pha ([robot_sim/telemetry.py](file:///d:/latuan/Programming/robot-demo/robot_sim/telemetry.py)), nâng cấp nút `Reset (Keep Topo)` & Goal HUD trên MuJoCo Studio ([studio/](file:///d:/latuan/Programming/robot-demo/studio/viewer_plugin.py)), nâng cấp hướng dẫn điều khiển đa tác tử Mode C ([.agents/skills/robot-multi-agent-rd/](file:///d:/latuan/Programming/robot-demo/.agents/skills/robot-multi-agent-rd/SKILL.md)), và kiểm thử tự động ([tests/test_hybrid_brain.py](file:///d:/latuan/Programming/robot-demo/tests/test_hybrid_brain.py), [tests/test_telemetry.py](file:///d:/latuan/Programming/robot-demo/tests/test_telemetry.py), [tests/test_cli.py](file:///d:/latuan/Programming/robot-demo/tests/test_cli.py)).

---

## 1. Mục đích & Lý do (Intent & Why)

Thực nghiệm Mode C (`hybrid-brain`) trên toàn bộ 6 sa bàn chuẩn trong MuJoCo Studio đã đạt `100%` tiêu chí an toàn và nhận thức ở lượt khám phá đầu tiên (`contact_count == 0`, `near_collision_count == 0`, `redundant_revisit_count == 0`). Tuy nhiên, trên các bản đồ mê cung chưa biết trước (`maze_a`, `maze_b`, `maze_c`), hiệu suất đường đi `path_efficiency` ở lượt đầu tự nhiên chỉ đạt `~0.28` do robot buộc phải dò và quay lui khỏi các ngõ cụt.

Giá trị cốt lõi của việc xây dựng đồ thị `TopologicalOdometer` ở Lượt 1 là **khai thác lại trí nhớ không gian đó** cho các nhiệm vụ thực tế tiếp theo mà không phải dò lại từ đầu:
1. **Hướng A — Nhiệm vụ Khứ hồi 1 phiên (`Explore → Dock → Return-to-Home`):** Mô phỏng bài toán robot giao hàng / tuần tra thực tế ngoài đời: xuất phát từ trạm `Node #0`, dò đường mù tới trụ mục tiêu xanh lá (`dock`), sau đó tự suy luận đường ngắn nhất trên đồ thị `topo.nodes` vừa học để **quay về đỗ lại đúng trạm xuất phát `Node #0`** trong cùng một phiên vật lý liên tục (không dùng lệnh `reset`).
2. **Hướng B — Thi đấu Hai lượt kiểu Micromouse (`Pass 1 Explore → reset --keep-topo → Pass 2 Speed-run`):** Cho phép đặt xe về lại vạch xuất phát nhưng bảo toàn cấu trúc bản đồ `topo.nodes` đã học (xóa sạch lịch sử đếm lượt `visits`), giúp Lượt 2 phóng thẳng theo đường ngắn nhất từ `Node #0` tới `Node #G` và đo kiểm trực tiếp bước nhảy vọt của `path_efficiency` (từ `~0.28` lên `> 0.80`) trên Hộp đen `telemetry`.

---

## 2. Mục tiêu, Phi mục tiêu & Phản mục tiêu

### Mục tiêu (Goals)
1. **Chốt nút đích (`kind: "goal"`) vào đồ thị Topo:** Khi robot cập đích thành công (`dock` đạt `reached == true`), đồ thị `topo.nodes` ghi nhận chính xác nút đích và cạnh hai chiều nối về nút xuất phát của chặng cuối, bất kể `dock` có xoay chéo thân xe khi căn tâm trụ xanh hay không.
2. **Hỗ trợ cả hai chế độ vận hành độc lập (Hướng A & Hướng B):**
   - **Hướng A:** Trong 1 phiên duy nhất, robot dò tới đích (`dock`), rồi tự tra đồ thị đi đường ngắn nhất quay về đỗ tại `Node #0` (sai số vị trí cuối so với điểm xuất phát `<= 0.35 m`, `contact_count == 0`, `near_collision_count == 0`).
   - **Hướng B:** Cung cấp `robot_ctl.py reset --keep-topo` để đặt lại mô phỏng về vạch xuất phát nhưng giữ nguyên bản đồ `topo.nodes` đã khám phá, cho phép Lượt 2 chạy tối ưu thẳng tới đích mà không vào ngõ cụt nào (`path_efficiency >= 0.80` trên `maze_a/b/c`).
3. **Thực thi chuỗi lộ trình nhanh (`go --route` + `--stop-at-node`):** Cho phép System 2 phát nguyên chuỗi hướng đã tự lập kế hoạch trong 1 lệnh CLI (`go --route W,S,N,N` hoặc `go --route W,S,N,E --stop-at-node 0`), giúp System 1 lướt liên hoàn qua các nút trung gian đã biết ở 20 Hz mà không tốn `~20s/nút` chờ LLM, nhưng vẫn phanh dừng tức thì nếu gặp cổng động, vật cản tĩnh, hoặc nhánh bị chắn.
4. **Hộp đen `telemetry` chấm điểm chính xác cả Đơn chặng, Khứ hồi (A) và Lượt 2 (B):** Bảo toàn 100% các trường JSON cũ (`goal_reached`, `path_efficiency`, ...) và bổ sung nhóm chỉ số khứ hồi (`goal_docked`, `returned_to_start`, `start_to_end_m`, `outbound_path_length_m`, `return_path_length_m`, `return_efficiency`).
5. **Tương thích ngược 100%:** Toàn bộ bài kiểm thử hiện có (Mode A, Mode B, Mode C 1-chiều, Studio HUD, Integrity) tiếp tục chạy xanh không đổi hành vi mặc định.

### Phi mục tiêu (Non-Goals)
- Không sửa [models/robot_2wheel.xml](file:///d:/latuan/Programming/robot-demo/models/robot_2wheel.xml) và tuyệt đối không chạm vào `models/arena_*.xml`.
- Không thay đổi hành vi mặc định của `robot_ctl.py reset` (khi không truyền `--keep-topo`, `.topo_state.json` vẫn được xóa trắng như cũ).
- Không tự động lưu vĩnh viễn bản đồ qua lệnh `reload --arena <khác>` (đổi địa hình luôn xóa trắng `.topo_state.json`).

### Phản mục tiêu (Anti-Goals)
1. **Nhúng thuật toán tìm đường tự động vào System 1:** Tuyệt đối **không** cài BFS/Dijkstra/A* trong `topo.py` hay `skills.py` (không có lệnh kiểu `go --to-node 0` tự tìm chuỗi cạnh). **System 2 (AI Agent)** bắt buộc phải tự đọc `topo.nodes` và tự suy luận danh sách hướng cần đi để truyền vào `--route` (hoặc `--cardinal`).
2. **Vi phạm kỷ luật Sim-to-Real:** System 1 và System 2 tuyệt đối không đọc `debug.robot_xy`, `debug.target_xy`, không gọi `/arenas`; việc căn dừng tại `Node #0` chỉ được dùng tọa độ Odometry tương đối onboard (`sensors.odom_x_m`, `sensors.odom_y_m`) so với `topo.nodes[stop_at_node]["xy"]`.
3. **Phóng mù qua chướng ngại vật trong `--route`:** Chuỗi `--route` không bao giờ được bỏ qua sự kiện an toàn (`dynamic_gate_ahead`, `static_obstacle_ahead`, `dead_end`, hoặc nhánh tiếp theo thực tế bị tường chắn).

---

## 3. Ràng buộc Kỹ thuật & Hình học (Constraints)

- **Lệch góc khi `dock`:** Visual servoing trong `dock_target` hướng mũi xe vào tâm trụ xanh nên góc dừng cuối có thể lệch chéo khỏi trục hành lang (`19°` trên `maze_a`, `-43°` trên `dynamic_a`). Do đó hướng cổng vào của nút `"goal"` phải được suy ra từ **góc hành lang tiếp cận trước khi `dock`** (hoặc vector dịch chuyển khi ở sân mở), không lấy mù quáng góc dừng cuối cùng của `dock`.
- **An toàn xoay tại chỗ trước trụ xanh:** Ở khoảng cách đỗ `rf_center ∈ [0.15, 0.35] m` (tâm trụ xanh cách tâm xe `~0.38–0.48 m`, bán kính góc thân xe `hypot(0.12, 0.10) = 0.156 m`), xe xoay tại chỗ thoát khỏi trụ xanh hoàn toàn không va chạm vật lý và giữ mọi tia rangefinder `>= 0.15 m > 0.12 m` (không gây `near_collision_count`).
- **Tự xóa nhầm bản đồ (`_check_auto_reset`):** `TopologicalOdometer._check_auto_reset` phát hiện reset vật lý khi `sim_time` giảm hoặc `odom_dist` về `0` trong khi `last_odom_dist_m > 0`. Khi dùng `reset --keep-topo`, con trỏ lịch sử cảm biến trong phiên Topo phải được đồng bộ lại về trạng thái khởi đầu mới (`0.0`) để lệnh tiếp theo không kích hoạt xóa nhầm đồ thị.

---

## 4. Các Phương án Đã Cân nhắc & Lý do Chọn (Approaches)

1. **Phương án 1 — Đi từng cạnh rời rạc (`Step-by-Step Hop`):** Giữ nguyên `go --cardinal <C>`, chỉ thêm `--stop-at-node`. *Loại bỏ* vì ở Lượt 2 / Lượt về trên mê cung 4–5 nút trung gian, xe vẫn phải dừng chờ LLM ở mọi ngã giao (`~80–100s` thời gian nghĩ thừa), làm mất ý nghĩa "Speed-run".
2. **Phương án 2 — System 2 cấp chuỗi lộ trình, System 1 thực thi liên hoàn có phanh sự kiện (`go --route C1,C2,... [--stop-at-node ID]`) [ĐƯỢC CHỌN]:** System 2 tự giải bài toán tìm đường ngắn nhất trên `topo.nodes` rồi truyền danh sách hướng chính (`E/N/W/S`) trong một lệnh `go --route`. System 1 lướt qua các ngã giao trung gian ở 20 Hz không dừng chờ LLM, tự dừng sớm an toàn nếu gặp sự kiện bất thường/cổng động, và căn dừng chính xác vào tâm nút đích khi có `--stop-at-node`. Giữ nguyên 100% ranh giới công bằng System 1 / System 2 đồng thời đạt tốc độ tối ưu.
3. **Phương án 3 — System 1 tự tìm đường (`go --to-node <ID>` chạy Dijkstra ngầm):** *Loại bỏ* vì chuyển phần suy luận đồ thị từ AI sang code cứng Python, vi phạm mục tiêu đánh giá nhận thức của Mode C.

---

## 5. Thiết kế Chi tiết Các Thành phần (Detailed Component Design)

### 5.1. Ghi nhận Nút Đích (`kind: "goal"`) & Tái sử dụng Bản đồ trong [robot_sim/topo.py](file:///d:/latuan/Programming/robot-demo/robot_sim/topo.py)

1. **Bảo toàn `pending_leg` và góc hành lang khi `finish_go` gặp `target_ready`:**
   - Trong `TopologicalOdometer.finish_go(event, moved_m, status, *, in_corridor)`:
     - Khi `event == "target_ready"`:
       - Nếu `self.pending_leg is None` và `self.current_node is not None`: khởi tạo `self.pending_leg` từ `self.current_node` với `depart_port = yaw_to_cardinal(yaw_deg)` và `dist_m = round(step_m, 4)`.
       - Ghi thêm trường `"approach_yaw_deg": round(yaw_deg, 3)` vào `self.pending_leg` (lưu góc dọc trục hành lang ngay trước khi `dock`).
       - Nếu vị trí hiện tại đã khớp một nút có sẵn mang `kind == "goal"`, hoàn tất cạnh vào nút đó như bình thường; ngược lại **giữ nguyên `self.pending_leg`** (đặt `self.current_node = None`, `self.loop_closed = False`) để chờ `dock` chốt tọa độ cuối cùng trước trụ xanh.

2. **Phương thức mới `record_dock(moved_m: float, status: dict, *, reached: bool) -> dict`:**
   - Nếu `not reached`: chỉ gọi `return self.observe_status(status)` (giữ nguyên `pending_leg` để xe có thể căn chỉnh và `dock` tiếp).
   - Nếu `reached == True`:
     - Trích xuất `(sim_time, x, y, odom_dist, yaw_deg, rf)`, chạy `_check_auto_reset` và `_record_telemetry_cursor`, đảm bảo có `Node #0` nếu đồ thị đang rỗng.
     - Cộng `step_m = max(0.0, float(moved_m))` vào `self.pending_leg["dist_m"]`. Nếu `self.pending_leg is None` và `self.current_node is not None`: tạo `self.pending_leg` từ `self.current_node` với `depart_port = yaw_to_cardinal(yaw_deg)` và `dist_m = round(step_m, 4)`.
     - **Xác định hoặc tạo nút `goal_id`:**
       - Nếu đã tồn tại nút có `kind == "goal"` trong bán kính `0.55 m` quanh `(x, y)`: dùng lại `goal_id` đó và cập nhật nhẹ `xy = [round(x, 3), round(y, 3)]`.
       - Ngược lại, nếu `nearest_existing_node(x, y, radius_m=NODE_MATCH_RADIUS_M)` khớp một nút có sẵn:
         - Nếu nút đó khác `from_node` (và không phải `Node #0`), nâng `kind` của nút đó thành `"goal"` và dùng `goal_id = matched`.
         - Nếu không khớp (hoặc trùng `from_node` với tổng quãng đường cạnh `>= 0.45 m`), tạo nút mới `goal_id = len(self.nodes)` với `{"id": goal_id, "xy": [round(x, 3), round(y, 3)], "kind": "goal", "exits": {}}`.
     - **Chốt cạnh nối hai chiều vào `goal_id`:**
       - Nếu `self.pending_leg is None` nhưng `self.is_open_space_start()` (trường hợp xe lách vật cản tĩnh trên sân mở `2wheel` bằng `turn` + `move` khiến `cancel_pending_leg` xóa `pending_leg` trước khi `dock`) và `goal_id != 0`: tự khôi phục cạnh thẳng từ `Node #0` tới `goal_id` với `dist_m = round(math.hypot(x - float(self.nodes[0]["xy"][0]), y - float(self.nodes[0]["xy"][1])), 3)` và hướng suy từ `math.degrees(math.atan2(y - float(self.nodes[0]["xy"][1]), x - float(self.nodes[0]["xy"][0])))` để `Node #G` không bao giờ bị cô lập trên sân mở.
       - Lấy góc đến `arr_yaw = float(self.pending_leg.get("approach_yaw_deg", yaw_deg))` nếu có `pending_leg`, rồi gọi `_complete_pending_leg(goal_id, arr_yaw, dest_existed=...)`.
       - Đặt `self.current_node = goal_id`. Chỉ gọi `_update_node_ports(goal_id, rf, yaw_deg)` nếu `not is_open_space_start()` **và** `is_cardinal_aligned(yaw_deg)` — nhưng **không thêm cổng `"open"` hướng thẳng vào trụ xanh** (`rf_center < 0.62 m`).
     - Lưu đĩa (`_persist()`) và trả về `self.export_dict()`.

3. **Phương thức mới `reset_for_repass(status: dict | None = None, *, target_changed: bool = False) -> dict` (và classmethod tiện ích):**
   - Dùng cho Hướng B (`robot_ctl.py reset --keep-topo`):
     - Nếu đồ thị đang rỗng: hoạt động giống hệt khởi tạo mới từ `status`.
     - Nếu đồ thị đã có `self.nodes`:
       - Giữ nguyên danh sách nút `id`, `xy`, `kind` và cấu trúc `exits` (`state`, `to_node`, `dist_m`).
       - Đặt lại toàn bộ `out_visits = 0` và `in_visits = 0` trên mọi cổng của mọi nút.
       - Nếu `target_changed == True` (người dùng truyền `--target-x/--target-y` mới khi `reset --keep-topo`): hạ các nút có `kind == "goal"` về `"junction"` (nếu số cổng `>= 2`) hoặc `"dead_end"` (nếu số cổng `<= 1`) vì trụ đích đã dời chỗ.
       - Đặt `self.current_node = 0`, `self.pending_leg = None`, `self.loop_closed = False`.
       - Đồng bộ lại con trỏ cảm biến: nếu có `status`, ghi `_record_telemetry_cursor` theo `status` mới (`sim_time = 0.0`, `odom_dist = 0.0`, `yaw = 0.0`, `xy = [0.0, 0.0]`); nếu `status is None`, đặt `last_sim_time = None`, `last_odom_dist_m = None`, `last_gyro_yaw_deg = 0.0`, `last_odom_xy = [0.0, 0.0]`. Nhờ đó lệnh `topo show` / `go` kế tiếp ở Lượt 2 **không** bị `_check_auto_reset` xóa bản đồ.
       - Lưu đĩa và trả về `self.export_dict()`.

---

### 5.2. Thực thi Chuỗi Lộ trình & Dừng tại Nút (`go --route` & `--stop-at-node`) trong [robot_sim/skills.py](file:///d:/latuan/Programming/robot-demo/robot_sim/skills.py) & [robot_ctl.py](file:///d:/latuan/Programming/robot-demo/robot_ctl.py)

1. **Mở rộng chữ ký `go_to_decision`:**
   ```python
   def go_to_decision(
       call_fn: BridgeCall,
       cardinal: str | None = None,
       heading_deg: float | None = None,
       max_dist: float = 6.0,
       speed_m_s: float = 0.55,
       step_dt: float = 0.08,
       max_chain: int = 6,
       auto_chain: bool = True,
       topo=None,
       *,
       route: list[str] | tuple[str, ...] | str | None = None,
       stop_at_node: int | None = None,
   ) -> dict:
   ```
   - **Kiểm tra đầu vào (Validation):**
     - Chỉ được truyền tối đa **một** trong ba tham số hướng: `cardinal`, `heading_deg`, hoặc `route` (truyền `>= 2` tham số → `ValueError`).
     - Nếu `route` là `str`: tách theo dấu phẩy `","`, cắt khoảng trắng, bỏ phần tử rỗng; nếu là `list`/`tuple`: ép từng phần tử. Danh sách sau chuẩn hóa phải có `1 <= len(steps) <= 16`, mọi phần tử thuộc `("E", "N", "W", "S")` (sai → `ValueError`).
     - Nếu `stop_at_node is not None`: phải là `int` (không phải `bool`) và `>= 0` (sai → `ValueError`); sau khi `observe_status` lần đầu, nếu `stop_at_node >= len(active_topo.nodes)` → báo `ValueError` rõ ràng rằng nút chưa tồn tại trong đồ thị.

2. **Hỗ trợ dừng đúng tâm nút đích (`stop_at_node`) trong `advance_segment` / `go_to_decision`:**
   - Mở rộng `_out_meta` (hoặc tham số keyword nội bộ `_target_odom_xy: tuple[float, float] | None = None`, `_ignore_target_ready: bool = False` của `advance_segment` — giữ 100% tương thích ngược với Mode B):
     - **Chống trụ xanh hút ngược khi đang đi về nút khác (`_ignore_target_ready = True` khi `stop_at_node is not None` và `active_topo.nodes[stop_at_node]["kind"] != "goal"`):** Khi robot đang thực hiện Return-to-Home (`--stop-at-node 0`) hoặc quay về một nút trung gian, `advance_segment` bỏ qua kiểm tra dừng sớm `target_ready` để xe không bị kẹt lại trước trụ xanh vừa rời đi.
     - **Bắt điểm dừng theo Odometry tương đối (`_target_odom_xy = (tx, ty)` ở chặng cuối của `route` hoặc lệnh `go` đơn có `stop_at_node`):**
       - Trong vòng lặp 20 Hz của `advance_segment` (sau khi đã rời điểm xuất phát của đoạn `moved >= 0.12` hoặc khi vốn đã đứng trong bán kính `<= 0.25 m` của `(tx, ty)`): tính `d_goal_node = math.hypot(cur_odom_x - tx, cur_odom_y - ty)`.
       - Khi `d_goal_node <= 0.18` (đã lọt sát tâm nút) **hoặc** (`d_goal_node <= 0.32` và `d_goal_node > prev_d_goal_node + 0.015`, tức vừa đi qua điểm gần tâm nút nhất): phanh dừng êm (`_stop_and_settle`) và trả về `event = "arrived_node"`.
       - Ngoài ra, nếu xe dừng tự nhiên do `junction` hoặc `dead_end` ở cuối chặng và sau `finish_go` có `active_topo.current_node == stop_at_node` (hoặc `hypot(odom_x - tx, odom_y - ty) <= NODE_MATCH_RADIUS_M`):
         - Nếu độ lệch dọc trục hiện tại tới `(tx, ty)` nằm trong `[0.20, 0.42] m` và hướng đó thoáng (`rf_center > 0.45 m`), tinh chỉnh nhẹ `closed_loop_move` (`<= 0.22 m`) để kéo xe vào sát tâm nút (`<= 0.20 m`), cập nhật `observe_status`, và trả về `event = "arrived_node"`.
     - **Tự căn góc về `(tx, ty)` trên sân mở (`open_space`):** Nếu `stop_at_node is not None`, `route is None`, `cardinal is None`, `heading_deg is None`, và `active_topo.is_open_space_start()` (như `2wheel`, `dynamic_a`), `go_to_decision` tự tính góc hướng về `(tx, ty)` từ `(odom_x, odom_y)` hiện tại bằng `math.degrees(math.atan2(ty - cur_y, tx - cur_x))` (100% Odometry tương đối của chính xe!).

3. **Vòng lặp thực thi chuỗi `route` có Phanh Sự kiện An toàn:**
   - Khi `route = [C_0, C_1, ..., C_{K-1}]` (`K >= 1`):
     - Theo dõi tổng `total_moved_m`, tổng `segments_chained`, danh sách gộp `all_auto_l_turns`, và `route_steps_completed = 0`.
     - Với `idx` từ `0` đến `K - 1`:
       - Nếu đã đứng sẵn tại `stop_at_node` (`idx == 0`, `hypot(odom_xy - target_xy) <= 0.22 m` và `K == 0`): dừng ngay.
       - Chỉ bật `_target_odom_xy` của `stop_at_node` ở chặng cuối cùng (`idx == K - 1` và `stop_at_node is not None`), nhưng bật `_ignore_target_ready` xuyên suốt mọi chặng nếu `stop_at_node` được truyền và không phải nút `"goal"`.
       - **Mốc hãm dự phòng chống trượt lố ngã rẽ trung gian (`next_node_xy`):** Khi ở chặng `idx < K - 1` xuất phát từ `current_node = u`, nếu cổng `C_idx` trên `active_topo.nodes[u]["exits"]` đã là `"explored"` nối tới `next_id = exit_info["to_node"]` **và** hướng kế tiếp `C_{idx+1} != C_idx` (cần rẽ tại `next_id`), truyền luôn `active_topo.nodes[next_id]["xy"]` làm mốc hãm Odometry cho chặng `idx` (đổi sự kiện `"arrived_node"` nội bộ của mốc trung gian về `"junction"` để vòng lặp `route` rẽ mượt sang `C_{idx+1}` mà không bao giờ trượt lố qua ngã ba có tường hông ngắn như trên `maze_b`).
       - Chạy 1 chặng `go` đơn lẻ với `cardinal = C_idx` và ngân sách quãng đường còn lại `remaining_route_dist = max_dist - total_moved_m` (mặc định nếu dùng `--route` mà người dùng giữ `--max-dist` mặc định `6.0`, tự nâng trần mặc định của `--route` lên `max(max_dist, 6.0 * K)` hoặc truyền `--max-dist` tùy ý để chuỗi 4 chặng dài ~12 m không bị cắt cụt giữa chừng bởi `6.0 m`!). *(Quyết định của người thiết kế: khi `route` có `K > 1` và `max_dist == 6.0` mặc định, trần tổng cho toàn tuyến là `6.0 * K`, mỗi chặng vẫn bị giới hạn `<= 6.0 m` để giữ phanh an toàn).*
       - Sau khi chặng `idx` kết thúc với `step_event`:
         - Nếu `idx == K - 1`: tăng `route_steps_completed = K` và kết thúc vòng lặp.
         - Nếu `idx < K - 1`:
           - Nếu `step_event in ("dynamic_gate_ahead", "static_obstacle_ahead", "target_ready", "arrived_node", "dead_end", "max_dist")` hoặc `topo_snap["current_node"] is None`: **dừng chuỗi ngay lập tức**, giữ `route_steps_completed = idx` (nếu dừng giữa cạnh `current_node is None`) hoặc `idx + 1` (nếu đã tới nút mới), và trả về danh sách hướng chưa đi trong `route_remaining`.
           - Nếu `step_event == "junction"` và `topo_snap["current_node"] is not None`:
             - Tăng `route_steps_completed = idx + 1`.
             - Kiểm tra an toàn trước khi sang `C_{idx+1}`: xoay đầu xe sang `C_{idx+1}` (bước đầu của chặng kế tiếp); nếu ngay sau khi xoay sang `C_{idx+1}` mà phía trước bị tường chắn sát (`_is_front_blocked(rf)` với `rf_center < 0.42 m` và không có `target_ready`), **dừng ngay** và trả về `event = "dead_end"` (hoặc `"route_blocked"`? Để không làm vỡ tập sự kiện chuẩn mà System 2 biết xử lý, trả về `"route_blocked"` kèm `open_cardinals` thực tế tại nút đó để System 2 lập lại lộ trình!).
     - JSON kết quả của `go_to_decision` luôn giữ đầy đủ các khóa hiện tại (`event`, `moved_m`, `segments_chained`, `auto_l_turns`, `open_branches`, `open_cardinals`, `obstacle`, `flank_clearance_m`, `topo`, `after_status`) và khi dùng `route` hoặc `stop_at_node` có thêm trường gọn:
       ```json
       "route_progress": {
         "requested": ["W", "S", "N", "N"],
         "completed": 4,
         "remaining": [],
         "stop_at_node": null
       }
       ```
       (Khi không truyền `route` và không truyền `stop_at_node`, không cần bắt buộc đọc `route_progress`, giữ 100% tương thích ngược).

4. **Cập nhật CLI trong [robot_ctl.py](file:///d:/latuan/Programming/robot-demo/robot_ctl.py):**
   - `robot_ctl.py go`: thêm `--route <E,N,W,S,...>` và `--stop-at-node <INT>`.
   - `robot_ctl.py dock`: khi `TopologicalOdometer.has_active_session(...)`: gọi `output["topo"] = topo_inst.record_dock(output.get("moved_m", 0.0), output["after_status"], reached=bool(output.get("reached")))` (bổ sung trường `"moved_m"` chuẩn xác từ `odom_dist_m` trong `dock_target` nếu chưa có!). *(Kiểm tra: hiện `dock_target` trả `{"reached", "reason", "steps", "after_status"}` — bổ sung thêm `"moved_m": round(max(0.0, end_odom - start_odom), 4)` vào kết quả `dock_target` hoàn toàn không phá vỡ test nào mà giúp `record_dock` cộng chính xác quãng đường cập đích!).*
   - `robot_ctl.py reset`: thêm cờ `--keep-topo` (`action="store_true"`).
     - Mặc định (`not args.keep_topo`): xóa phiên `.topo_state.json` như cũ.
     - Khi có `--keep-topo` và đang có phiên active: gọi `topo_inst.reset_for_repass(clean_status, target_changed=(target_xy is not None))` và đính kèm `output["topo"] = topo_snap`.

---

### 5.3. Chấm điểm Đa pha trong Hộp đen [robot_sim/telemetry.py](file:///d:/latuan/Programming/robot-demo/robot_sim/telemetry.py)

Giữ nguyên 100% ý nghĩa và cách tính mọi trường hiện có, đồng thời theo dõi thêm chuyển pha **Đi (`outbound`) → Về (`return`)** hoàn toàn tự động (0 tham số bắt buộc):

1. **Phát hiện mốc cập đích giữa phiên (`goal_docked`):**
   - Tại mỗi mẫu 10 Hz, định nghĩa mẫu đang đứng hợp lệ trước trụ xanh:
     `docked_now = (not is_active) and t_vis and (t_off is not None) and (abs(t_off) < 0.25) and (0.15 <= rf_c <= 0.35)`.
   - Khi `docked_now` đúng lần đầu tiên (`not goal_docked`):
     - `goal_docked = True`
     - `outbound_path_length_m = path_length_m`
     - `outbound_redundant_revisits = redundant_revisit_count`
   - Nếu sau khi `goal_docked == True`, robot **bắt đầu di chuyển rời đi** (`is_active == True` và `not return_phase_started` và `path_length_m - outbound_path_length_m >= 0.15`):
     - `return_phase_started = True`
     - Xóa `cell_last_path.clear()`, `dir_cell_last_path.clear()`, `last_cell = None`, `last_dir_cell = None` để chặng về được đo lặp hướng độc lập (tránh đếm oan các ô ngã giao mà chặng đi từng quay lui qua).
   - Nếu `reset_jumped == True` (bước nhảy vị trí `> 0.45 m` do gọi `/reset` khi Hộp đen đang mở): đặt lại cả `goal_docked = False`, `outbound_path_length_m = None`, `return_phase_started = False`.

2. **Các trường bổ sung trong JSON tóm tắt (`< 2 KB`):**
   - `goal_docked`: `bool` (`goal_docked or goal_reached`).
   - `start_to_end_m`: `round(math.hypot(end_xy[0] - start_xy[0], end_xy[1] - start_xy[1]), 4)` (hoặc `None` nếu chưa có mẫu).
   - `outbound_path_length_m`: `round(outbound_path_length_m, 4)` nếu `goal_docked` else `None`.
   - `return_path_length_m`: `round(max(0.0, path_length_m - outbound_path_length_m), 4)` nếu `return_phase_started` else `0.0` (nếu `goal_docked`) else `None`.
   - `returned_to_start`: `bool(goal_docked and return_phase_started and (not last_is_active) and start_to_end_m is not None and start_to_end_m <= 0.35)`.
   - `return_efficiency`: `round(min(1.0, reference_path_m / return_path_length_m), 3)` nếu `(returned_to_start and reference_path_m is not None and return_path_length_m and return_path_length_m > 0.0)` else `None`.

---

### 5.4. Chiến thuật System 2 (`robot-driver-hybrid`) & Quy trình Multi-Agent R&D

Cập nhật [driver-hybrid-prompt.md](file:///d:/latuan/Programming/robot-demo/.agents/skills/robot-multi-agent-rd/references/driver-hybrid-prompt.md) và [SKILL.md](file:///d:/latuan/Programming/robot-demo/.agents/skills/robot-multi-agent-rd/SKILL.md) để `robot-driver-hybrid` tự thích ứng mượt với cả 3 dạng nhiệm vụ chỉ dựa vào lời giao bài và trạng thái `topo show`:

1. **Nhiệm vụ Chuẩn 1 chiều (như hiện tại):** Khám phá → `dock` → dừng và báo cáo. (Có thể tận dụng `go --route C1,C2,...` khi cần quay lui nhiều chặng liên tiếp qua các cổng `out_visits == 0` để giảm lượt gọi LLM!).
2. **Nhiệm vụ Hướng A — Khứ hồi (`Explore → Dock → Return-to-Home`):**
   - Chặng đi: khám phá và gọi `dock --stop-distance 0.25`. Đọc ngay `output["topo"]` trả về từ `dock` (lúc này `current_node` là `Node #G`, `kind == "goal"`).
   - Chặng về:
     - Trên mê cung / hành lang (`Node #0` có `kind == "start"`): tìm đường ngắn nhất theo tổng `dist_m` nhỏ nhất trên các cạnh `"explored"` trong `topo.nodes` từ `Node #G` về `Node #0`, trích xuất chuỗi hướng cổng ra `[C_1, C_2, ..., C_k]`, và phát **1 lệnh** `go --route C_1,C_2,...,C_k --stop-at-node 0`. Nếu trên đường về dừng ở `dynamic_gate_ahead`, gọi `cross-gate --timeout 12.0` rồi tiếp tục phần lộ trình còn lại cho tới khi nhận `event == "arrived_node"` (hoặc `topo.current_node == 0` và `hypot(odom_x, odom_y) <= 0.30`).
     - Trên sân mở (`Node #0` có `kind == "open_space"`): gọi `go --stop-at-node 0` (tự căn hướng Odometry về `(0, 0)`), kết hợp `cross-gate` nếu gặp làn tuần tra động hoặc lách nếu gặp `static_obstacle_ahead`, cho tới khi về tâm `Node #0`.
3. **Nhiệm vụ Hướng B — Speed-run Lượt 2 (sau `reset --keep-topo`, khởi tạo subagent mới độc lập):**
   - **Quy trình điều phối khách quan của Main Agent:** Sau khi Lượt 1 hoàn tất và dừng Hộp đen (`telemetry stop`), Main Agent chạy `robot_ctl.py reset --keep-topo --vision`, bật phiên Hộp đen mới, và **khởi tạo một subagent `robot-driver-hybrid` hoàn toàn mới** (không dùng lại lịch sử chat của Lượt 1) để chứng minh 100% hiệu suất Speed-run đến từ đồ thị `.topo_state.json`.
   - Khi mở đầu bằng `topo show`, nếu thấy trong `topo.nodes` **đã có sẵn nút `kind == "goal"`** (bản đồ đã học từ Lượt 1!):
     - **Bỏ qua hoàn toàn mọi nhánh `"open"` hay ngõ cụt:** tìm đường ngắn nhất theo tổng `dist_m` nhỏ nhất trên các cạnh `"explored"` từ `Node #0` tới `Node #G`.
     - Nếu `Node #0` là `"start"`: gọi ngay `go --route C_1,C_2,...,C_k` (tự đi thẳng qua mọi ngã giao, gặp cổng động thì `cross-gate`, gặp `target_ready` thì gọi `dock --stop-distance 0.25`).
     - Nếu `Node #0` là `"open_space"`: căn hướng về `Node #G.xy` hoặc gọi `go` / `cross-gate` / `dock` thẳng tới đích.

---

### 5.5. Hiển thị `Goal` HUD & Nút `Reset (Keep Topo)` trên MuJoCo Studio ([studio/](file:///d:/latuan/Programming/robot-demo/studio/viewer_plugin.py))

1. **`studio/messages.py`:**
   - Mở rộng `ResetRobotEvent(keep_topo: bool = False)` (mặc định `False` tương thích ngược).
   - Mở rộng `RobotTelemetry` thêm trường `topo_goal_node: int = -1`.
2. **`robot_sim/topo.py` & `studio/sim_plugin.py`:**
   - Bổ sung hàm `read_topo_studio_details(base_url, state_path) -> tuple[int, int, int, int]` trả `(node_count, cur_id, open_exits, goal_id)` (giữ nguyên `read_topo_studio_summary` 3 phần tử cho các test cũ).
   - Khi nhận `ResetRobotEvent(keep_topo=False)` hoặc `ArenaSelectEvent`: xóa phiên `.topo_state.json` như reset mới hoàn toàn.
   - Khi nhận `ResetRobotEvent(keep_topo=True)`: sau khi đặt lại mô phỏng ở `on_post_model`, gọi `TopologicalOdometer.reset_for_repass(...)` để giữ nguyên bản đồ và nút `"goal"` nhưng xóa sạch `visits = 0`.
3. **`studio/viewer_plugin.py`:**
   - Đặt nút **`Reset (Keep Topo)`** nằm ngay cạnh nút **`Reset Robot Spawn`** (`imgui.SameLine()`).
   - Trên dòng `Topo Graph:`, nếu `t.topo_goal_node >= 0`, nối thêm ` | Goal: #{t.topo_goal_node}`.

---

## 6. Phần Giao quyền cho Người Triển khai (Left to the Builder)

- Cấu trúc hàm phụ nội bộ trong `robot_sim/skills.py` để tách vòng lặp 1 chặng và vòng lặp đa chặng `route` sao cho không lặp code.
- Định dạng chi tiết thông báo lỗi tiếng Việt khi truyền sai cú pháp `--route` hoặc `--stop-at-node`.
- Bố cục các bài kiểm thử đơn vị và kiểm thử tích hợp MuJoCo headless trong `tests/test_hybrid_brain.py`, `tests/test_cli.py`, và `tests/test_telemetry.py` (tối thiểu kiểm tra: chốt `kind: "goal"` sau `dock` lệch góc, `reset --keep-topo` giữ đồ thị & xóa `visits`, `go --route` đi nhiều chặng + phanh sớm khi gặp `dynamic_gate_ahead` / `route_blocked`, `--stop-at-node 0` trên cả mê cung lẫn sân mở, và Hộp đen `telemetry` chấm `returned_to_start` + `return_efficiency` không đếm oan `redundant_revisit_count`).
