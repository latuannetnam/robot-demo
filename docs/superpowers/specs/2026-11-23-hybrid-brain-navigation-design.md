# Thiết kế Hệ thống Điều hướng Lai Phân tầng System 1 / System 2 (`hybrid-brain` — Mode C)

**Ngày thiết kế:** 2026-11-23  
**Quy mô công việc:** Dự án đầy đủ (Full Project Design)  
**Trạng thái:** Đã được người dùng phê duyệt (Approved)  
**Phạm vi:** Bộ định vị đồ thị khách quan ([robot_sim/topo.py](file:///d:/latuan/Programming/robot-demo/robot_sim/topo.py)), kỹ năng vận động gộp System 1 ([robot_sim/skills.py](file:///d:/latuan/Programming/robot-demo/robot_sim/skills.py), [robot_ctl.py](file:///d:/latuan/Programming/robot-demo/robot_ctl.py)), cấu hình đa tác tử Mode C ([.agents/skills/robot-multi-agent-rd/](file:///d:/latuan/Programming/robot-demo/.agents/skills/robot-multi-agent-rd/SKILL.md)), tích hợp hiển thị trên MuJoCo Studio ([studio/](file:///d:/latuan/Programming/robot-demo/studio/sim_plugin.py)), và bộ kiểm thử tự động ([tests/test_hybrid_brain.py](file:///d:/latuan/Programming/robot-demo/tests/test_hybrid_brain.py)).

---

## 1. Mục đích & Lý do (Intent & Why)

Dự án hiện có hai chế độ điều khiển đại diện cho hai thái cực:
1. **Mode A (`auto-skill` — `explore_maze`, `approach_target`):** Chạy khép kín ở 20 Hz rất nhanh (`< 60s`), nhưng nhồi hơn 2.000 dòng thuật toán cứng (bám tường trái/phải, đánh dấu Trémaux, tự quay đầu ngõ cụt, tự né vật cản, tự vượt chuỗi cổng động) vào Python. AI Agent gần như không thực sự tư duy điều khiển.
2. **Mode B (`agent-brain` — `advance`, `cross-gate`, `dock` vô trạng thái):** Giao 100% trí nhớ và quyết định từng góc cua cho LLM. Thực nghiệm 3 vòng R&D trên `maze_a` cho thấy hai nút thắt lớn làm che mờ năng lực thật của AI:
   - **Độ trễ cơ học vô ích:** Xe mất `732s` đứng chờ LLM suy nghĩ so với `168s` lăn bánh (`~30` lượt CLI), bao gồm cả việc phải dừng hỏi LLM 2 lượt (`turn` rồi `advance`) tại các góc chữ L chỉ có duy nhất 1 lối đi.
   - **Trôi số học khi tính nhẩm tọa độ bằng chữ:** Bắt LLM tự nhẩm `hypot(odom_x - nx, odom_y - ny) <= 0.40` và tự đổi hướng tương đối/tuyệt đối trong văn bản gây nhầm nút giao (`redundant_revisit_count = 10..13`), dù phản xạ vật lý an toàn tuyệt đối (`contact_count = 0`).

**Mục đích của Mode C (`hybrid-brain`):** Đo lường **năng lực điều khiển robot thực chất của AI Agent** trong vòng lặp thời gian thực (`Sense → Think → Act`) thông qua kiến trúc **System 1 / System 2** sát với robot tự hành ngoài đời:
- **System 1 (Python 20 Hz — Tay chân & Định vị đo đạc):** Đảm nhận điều khiển chuyển động gộp (`xoay khóa trục + tiến`), tự lướt qua các khúc cua hành lang bắt buộc **chỉ có 1 lối đi duy nhất**, và ghi nhận khách quan **Đồ thị Odometry Topo** hai chiều của những nơi xe *đã thực sự đặt bánh tới* (tương tự module SLAM/Odometry trên mạch nhúng).
- **System 2 (AI Agent — Bộ não chiến thuật bị chấm điểm):** Chịu trách nhiệm **100% mọi quyết định có lựa chọn**: chọn nhánh tại mọi ngã 3/ngã 4, suy luận cây ngõ cụt để lùi về nút tổ tiên, phát hiện & thoát vòng lặp đảo chữ U (`maze_c`), nhìn ảnh `front_cam` chọn đường lách vật cản tĩnh (`2wheel`), canh nhịp vượt từng làn/cổng động (`dynamic_a`, `dynamic_b`) và chốt cập đích (`dock`).

---

## 2. Mục tiêu, Phi mục tiêu & Phản mục tiêu

### Mục tiêu (Goals)
1. Xây dựng **Mode C (`hybrid-brain`)** gồm lệnh gộp `go`, bộ định vị khách quan `TopologicalOdometer` (`topo`), và subagent **`robot-driver-hybrid`** giải quyết trọn vẹn **cả 6 sa bàn chuẩn** (`2wheel`, `maze_a`, `maze_b`, `maze_c`, `dynamic_a`, `dynamic_b`) trên cả [sim_server.py](file:///d:/latuan/Programming/robot-demo/sim_server.py) lẫn **MuJoCo Studio** ([studio/](file:///d:/latuan/Programming/robot-demo/studio/sim_plugin.py)).
2. Giảm mạnh số lượt dừng chờ LLM trên mê cung (từ `~30` lượt ở Mode B xuống `~4–10` lượt quyết định thực chất trên `maze_a`), đưa `redundant_revisit_count` về `0` và giữ `contact_count == 0`, `near_collision_count == 0`.
3. **Bảo toàn 100%** Mode A (`auto-skill`), Mode B (`agent-brain`) và toàn bộ 106 bài kiểm thử hiện có để làm bộ đối chuẩn 3 chế độ (`Mode A` vs `Mode B` vs `Mode C`).

### Phi mục tiêu (Non-Goals)
- Không chỉnh sửa phần cứng [models/robot_2wheel.xml](file:///d:/latuan/Programming/robot-demo/models/robot_2wheel.xml) hay bất kỳ địa hình nào trong `models/arena_*.xml`.
- Không thay thế hay xóa bỏ Mode A hoặc Mode B.
- Không nhúng bộ lập lịch tìm đường (BFS/Dijkstra/A*/DFS tự động) vào Python.

### Phản mục tiêu (Anti-Goals — coi là thất bại thiết kế dù xe về đích)
1. **Gian lận hộ AI ở System 1:** Tự chọn nhánh khi có `>= 2` hướng mở; tự quay đầu `180°` ở ngõ cụt mà không có lệnh của AI; tự lan truyền nhãn `dead_end` ngược lên nút cha hoặc gợi ý `recommended_branch` trong JSON trả về; tự lách vật cản tĩnh hoặc tự vượt nhiều cổng động liên tiếp.
2. **Vi phạm kỷ luật Sim-to-Real:** Đọc `debug` (`robot_xy`, `target_xy`, `arena`), gọi `/arenas`, dùng `overview_cam`, hoặc gán cứng kích thước lưới mê cung của từng bản đồ.
3. **Đóng băng thời gian vật lý:** Mô phỏng vẫn chạy liên tục `1.0x realtime` (500 Hz) trong suốt thời gian System 2 (AI Agent) suy luận.

---

## 3. Ràng buộc & Ranh giới Công bằng (Fairness Boundary)

| Hạng mục | System 1 (Python 20 Hz — Cơ học & Đo đạc) | System 2 (`robot-driver-hybrid` — Trí khôn bị chấm điểm) |
| :--- | :--- | :--- |
| **Chuyển động cơ bản** | Xoay khóa góc chính xác bằng Gyro + chạy giữ tâm hành lang (`go`). | Quyết định hướng hệ quy chiếu khởi hành (`E/N/W/S`) hoặc góc xoay cần đi. |
| **Khúc cua 1 lối đi (Góc chữ L)** | Tự xoay 90° và chạy tiếp nếu đang ở hành lang chỉ có **đúng 1 hướng rẽ ngang**, không thấy đích, không có vật cản. | Không phải dừng chờ vô ích ở đoạn ống cong không có ngã rẽ. |
| **Ngã giao `>= 2` nhánh mở** | Phanh dừng êm giữa tâm nút, báo danh sách nhánh mở + bản đồ đo đạc. | **Tự chọn nhánh khám phá** dựa trên lịch sử đồ thị & cảm biến. |
| **Ngõ cụt (`dead_end`)** | Phanh dừng an toàn, ghi nhận thực tế nút hiện tại là `dead_end`. **Không tự quay đầu.** | **Tự suy luận nút tổ tiên còn nhánh mở** và phát lệnh `go --cardinal <hướng_thoát>`. |
| **Vòng lặp (`maze_c`)** | Báo cờ khách quan `loop_closed = true` + số lượt đi/về (`out_visits`, `in_visits`) từng đầu cổng. | **Tự nhận diện mạch vòng** và rút lui/đổi nhánh mà không đi lặp cùng chiều cũ. |
| **Vật cản tĩnh (`2wheel`)** | Phanh dừng trước vật cản, báo `static_obstacle_ahead` + khoảng hở 2 sườn. | Tự xem `flank_clearance_m` / ảnh `front_cam` và điều khiển vòng lách (`turn` + `move` + `scan`/`go`). |
| **Vật cản động (`dynamic_a/b`)** | Phanh ngoài vùng quét piston, báo `dynamic_gate_ahead`; thực thi vượt **1 cổng** khi được lệnh `cross-gate`. | Tự điều phối trình tự vượt từng cổng một kết hợp rẽ hành lang chữ S. |

---

## 4. Các Phương án Đã Cân nhắc & Lý do Chọn (Approaches)

### 4.1. Kiến trúc tổng thể
1. **Phương án 1 — System 1 / System 2 Phân tầng theo Sự kiện Quyết định (`Mode C: hybrid-brain`) [ĐƯỢC CHỌN]:**
   - *Cơ chế:* Gộp `xoay + tiến + lướt góc chữ L 1 lựa chọn` vào lệnh `go`, kèm sổ cái định vị khách quan `TopologicalOdometer` lưu ở phía client theo phiên chạy; AI Agent trực tiếp quyết định ở mọi nút phân nhánh, ngõ cụt, vật cản và cổng động.
   - *Đánh đổi:* Giữ AI ngồi trực tiếp trong buồng lái thời gian thực để đánh giá đúng năng lực điều khiển, giảm 70–85% độ trễ dừng chờ so với Mode B, loại bỏ lỗi nhẩm tọa độ mà không vi phạm ranh giới công bằng.
2. **Phương án 2 — Code-as-Policies (LLM sinh kịch bản Python 20 Hz đầu phiên):**
   - *Lý do loại:* Kiểm tra năng lực *viết code offline* của LLM chứ không kiểm tra năng lực *điều khiển & phản ứng trực tiếp theo luồng cảm biến sống* của AI Agent; kém linh hoạt trước bất ngờ thị giác giữa đường và đòi hỏi sandbox thực thi code tự sinh.
3. **Phương án 3 — Chỉ tối ưu đường truyền trên Mode B thuần túy (giữ dừng ở mọi góc L, không có `TopologicalOdometer`):**
   - *Lý do loại:* Vẫn bắt xe dừng chờ ở mọi góc chữ L không có lựa chọn và vẫn để LLM tự nhẩm khoảng cách Euclid bằng văn bản, không giải quyết tận gốc hiện tượng trôi nút giao trên mê cung dài (`maze_b`, `maze_c`).

### 4.2. Đo đạc lịch sử cổng tại các nút đồ thị (Đã chốt cùng người dùng)
1. **Phương án A — Đo đạc hai chiều trung thực (`out_visits` / `in_visits`) + cập nhật cảm biến ngay sau pha xoay tại chỗ [ĐƯỢC CHỌN]:**
   - *Đánh đổi:* Mỗi cổng đã đi lưu riêng số lần rời nút (`out_visits`) và số lần tiến vào nút (`in_visits`), đồng thời cập nhật tia khoảng cách ngay sau mỗi lệnh xoay tại chỗ (`go` bước 1 / `turn --wait` / `scan`). Vẫn 100% là lịch sử đo đạc bánh xe khách quan, nhưng giúp AI thấy rõ chiều nào chưa từng đi xuôi (`out_visits == 0`) để bảo đảm `redundant_revisit_count == 0` trên vòng lặp `maze_c`, và giải mã sớm góc mù `180°` ở `Node #0` (`maze_b`).
2. **Phương án B — Giữ 1 số đếm `"visits"` gộp và chỉ cập nhật cuối lệnh `go`:**
   - *Lý do loại:* Khi khép vòng lặp trên `maze_c`, cả hai nhánh nối vòng đều hiển thị `"visits": 1` như nhau khiến AI dễ đi lặp lại cùng chiều cũ và bị `telemetry.py` phạt `redundant_revisit_count > 0`.

---

## 5. Thiết kế Chi tiết Kiến trúc Mode C (`hybrid-brain`)

### 5.1. Module Định vị Đồ thị Khách quan — `robot_sim/topo.py` (`TopologicalOdometer`)

`[Quyết định đã chốt cùng người dùng]` Đặt sổ cái định vị ở phía công cụ điều khiển robot (`captures/.topo_state.json`, cách ly phiên theo `base_url` chuẩn hóa dưới khóa `sessions[base_url]`) thay vì sửa máy chủ vật lý [sim_server.py](file:///d:/latuan/Programming/robot-demo/sim_server.py), tương thích tức thì với cả `sim_server.py` lẫn MuJoCo Studio và giữ đúng mô hình máy tính nhúng trên xe tự tích hợp Odometry từ cảm biến.

1. **Nguồn dữ liệu đầu vào & Hệ quy chiếu khởi hành (100% Sim-to-Real):**
   - Chỉ đọc từ `status["sensors"]`: `odom_x_m`, `odom_y_m`, `odom_dist_m`, `gyro_yaw_deg`, `rangefinders_m` và `sim_time`. Tuyệt đối không đọc `debug`.
   - `[Quyết định kỹ thuật của thiết kế]` Các nhãn hướng `E = 0°`, `N = +90°`, `W = ±180°`, `S = -90°` được định nghĩa hoàn toàn trong **hệ quy chiếu IMU/Odometry tương đối theo lúc khởi hành** (`t = 0`: `odom_x_m = 0`, `odom_y_m = 0`, `gyro_yaw_deg = 0°` hướng dọc trục `+odom_x_m`), không phụ thuộc hướng địa lý thực của bản đồ (ví dụ trên `maze_c` xe xuất phát quay mặt hướng Nam thế giới nhưng IMU khởi tạo `0°`, hệ quy chiếu nội tại vẫn nhất quán tuyệt đối giữa `gyro_yaw_deg`, `odom_x_m/odom_y_m` và `E/N/W/S`).
   - Quy đổi lượng tử hóa góc sang 4 hướng chính: `idx = int(((yaw_deg + 45.0) % 360.0) // 90.0)` ứng với `{0: "E", 1: "N", 2: "W", 3: "S"}`. Chỉ ánh xạ tia cảm biến (`left90`, `center`, `right90`) vào cổng `E/N/W/S` của một nút mê cung khi xe đang căn thẳng trong phạm vi `<= 15.0°` so với trục `90°` gần nhất (tránh gán nhầm hướng khi xe xoay chéo né vật cản trên sân mở).

2. **Tự động xóa sạch khi sang lượt chạy mới (`Auto-Reset Detection`):**
   - Khởi tạo lại phiên đồ thị mới bất cứ khi nào:
     - Gọi `robot_ctl.py reset`, `robot_ctl.py reload`, hoặc `robot_ctl.py topo reset`; **hoặc**
     - Phát hiện `cur_sim_time < last_sim_time - 1e-3`, hoặc `cur_odom_dist_m < last_odom_dist_m - 1e-3`; **hoặc**
     - Xe quay về đúng gốc xuất phát sau khi đã vận hành (`abs(cur_odom_dist_m) < 0.03` và `hypot(cur_odom_x, cur_odom_y) < 0.03` trong khi `last_odom_dist_m > 0.10`, hoặc `abs(cur_odom_dist_m) < 1e-3` và `abs(cur_gyro_yaw) < 0.5` trong khi trước đó `abs(last_odom_dist_m) > 1e-3` hoặc `abs(last_gyro_yaw) > 2.0` — bắt được cả trường hợp người dùng bấm `Reset Robot Spawn` trực tiếp trong cửa sổ MuJoCo Studio).

3. **Quy tắc Sinh Nút (`Node`), Nối Mạch Tạm (`pending_leg`) & Cạnh Hai Đầu Cổng (`Port-Accurate Bidirectional Edge`):**
   - **Nút khởi đầu `Node #0`:** Tự động tạo khi mở phiên (`topo show` hoặc đầu lệnh `go`):
     - Nếu `left90 > 0.75`, `right90 > 0.75` và `center > 0.85` -> `kind = "open_space"` (`2wheel`, `dynamic_a`); ngược lại -> `kind = "start"`.
   - **Khớp nút hiện có (`<= 0.42 m`):** Mọi điểm dừng cách tâm một `Node #k` đã biết khoảng cách Euclid `<= 0.42 m` đều gán `current_node = k` và giữ cố định tọa độ neo ban đầu `xy` của `Node #k` để chống trôi tích lũy.
   - **Khi nào tạo `Node #N` mới vs giữ `current_node = null` (Đã chốt cùng người dùng):**
     - Chỉ tạo `Node #N` mới (`> 0.42 m` so với mọi nút cũ) khi sự kiện dừng là `junction` hoặc `dead_end` trong không gian hành lang/tường chắn (`kind = "dead_end"` nếu `event == "dead_end"`, ngược lại `"junction"`).
     - Nếu dừng giữa chừng vì `dynamic_gate_ahead`, `static_obstacle_ahead`, `target_ready` hoặc `max_dist` (và cách mọi `Node` cũ `> 0.42 m`), **không tạo nút giả**: đặt `current_node = null`.
   - **Nối mạch hành trình `pending_leg` qua cổng động / điểm dừng lửng (Đã chốt cùng người dùng):**
     - Khi rời `Node #u` trong lệnh `go` (ghi nhận **sau** bước xoay khóa hướng đầu lệnh để lấy đúng cổng xuất phát `depart_port`), lưu `pending_leg = {"from_node": u, "depart_port": depart_port, "dist_m": 0.0}`.
     - Nếu xe dừng lưng chừng (`dynamic_gate_ahead`, `max_dist`) rồi gọi tiếp `cross-gate` và `go`, `pending_leg.dist_m` tiếp tục cộng dồn quãng đường cho tới khi chạm một `Node #v` thực sự (`junction` / `dead_end` hoặc khớp lại `Node` cũ).
     - Nếu xe thực hiện lách tự do trên sân mở (`move`, `wheels`, hoặc `turn` lệch khỏi lưới `90°` quá `20°` khi `current_node == null`), hủy `pending_leg = null` để không nối nhầm đường vòng sân mở thành hành lang mê cung.
   - **Cạnh hai đầu cổng với bộ đếm `out_visits` / `in_visits` (Đã chốt cùng người dùng):**
     - Khi hoàn tất `pending_leg` từ `(Node #u, p_out)` tiến vào `Node #v` theo hướng `arrival_cardinal` (nên hướng nhìn ngược lại từ `Node #v` vào hành lang đó là `p_in = opposite(arrival_cardinal)`):
       - Trên `Node #u`, cổng `p_out` cập nhật: `{"state": "explored", "to_node": v, "out_visits": prev_out + 1, "in_visits": prev_in, "dist_m": d}`.
       - Trên `Node #v`, cổng `p_in` cập nhật: `{"state": "explored", "to_node": u, "out_visits": prev_out_v, "in_visits": prev_in_v + 1, "dist_m": d}`.
       - Nhờ lưu đúng hướng cửa ở từng đầu nút (`p_out` ở `#u`, `p_in` ở `#v`), hành lang gấp khúc chữ L/U được đi xuôi và đi ngược chính xác 100% chỉ bằng `go --cardinal <cổng>`.
     - **Cờ khách quan `loop_closed`:** Chỉ bật `loop_closed = true` khi `Node #v` đã tồn tại từ trước (`v != u` hoặc `d >= 0.80 m`) **và** cạnh `(u:p_out <-> v:p_in)` trước đó chưa được ghi nhận `"explored"` nối với nhau (tức vừa khép một dây cung vòng lặp mới; đi lùi bình thường trên cạnh đã `"explored"` thì `loop_closed = false`).

4. **Điểm mù `180°` lúc khởi hành & Phân giải tức thì khi xoay tại chỗ (Đã chốt cùng người dùng):**
   - Phần cứng [models/robot_2wheel.xml](file:///d:/latuan/Programming/robot-demo/models/robot_2wheel.xml) chỉ có tia khoảng cách ở `0°, ±30°, ±90°, ±120°` (không có cảm biến đuôi `180°`). Tại `t = 0` ở `Node #0` (`kind == "start"`), cổng `180°` ngay sau lưng xe được ghi nhận trung thực là `"state": "uninspected"`.
   - Bất cứ khi nào xe đứng tại một `Node #k` (`<= 0.42 m`) và hoàn tất pha xoay tại chỗ (`go` bước 1 `closed_loop_turn`, `turn --wait`, `scan`, hoặc `topo show`) với góc lệch `<= 15°` so với trục `90°`, `TopologicalOdometer` đối chiếu ngay 3 hướng trước/trái/phải vừa lọt vào vùng quét laser:
     - Cổng đang `"uninspected"` nếu thông thoáng (`in open_cardinals`) -> nâng lên `"state": "open"`; nếu bị tường chắn -> xóa khỏi `exits`.
     - Cổng thông thoáng mới phát hiện (chưa có trong `exits` và `kind != "open_space"`) -> thêm `"state": "open"`.
     - Cổng đã `"explored"` giữ nguyên lịch sử.

5. **Schema JSON `topo` xuất cho AI Agent (thuần quan sát, không gợi ý đường):**
   ```json
   {
     "current_node": 1,
     "heading_cardinal": "N",
     "gyro_yaw_deg": 90.12,
     "odom_xy": [0.98, 0.02],
     "loop_closed": false,
     "pending_from_node": null,
     "nodes": [
       {
         "id": 0,
         "xy": [0.0, 0.0],
         "kind": "start",
         "exits": {
           "E": {"state": "explored", "to_node": 1, "out_visits": 1, "in_visits": 0, "dist_m": 1.94},
           "N": {"state": "open", "to_node": null, "out_visits": 0, "in_visits": 0, "dist_m": null},
           "W": {"state": "uninspected", "to_node": null, "out_visits": 0, "in_visits": 0, "dist_m": null}
         }
       }
     ]
   }
   ```
   *(Tuyệt đối không có `parent`, không tự đánh dấu nhánh cụt hộ AI, không có `suggested_exit`.)*

---

### 5.2. Kỹ năng Vận động Gộp System 1 — `go_to_decision` (`robot_ctl.py go`)

Thêm hàm `go_to_decision` vào [robot_sim/skills.py](file:///d:/latuan/Programming/robot-demo/robot_sim/skills.py) và lệnh CLI `.venv\Scripts\python.exe robot_ctl.py go`:

```powershell
.venv\Scripts\python.exe robot_ctl.py go [--cardinal E|N|W|S | --heading-deg <DEG>] [--max-dist 6.0] [--speed 0.55] [--step-dt 0.10] [--max-chain 6] [--no-chain]
```

1. **Bước 1 — Quan sát nút hiện tại & Xoay khóa hướng đầu lệnh:**
   - Đọc `/status?vision=1`, đồng bộ `topo.observe_status(st)` (đảm bảo `Node #0` được ghi nhận trước mọi chuyển động).
   - Nếu có `--cardinal` (`E=0°, N=+90°, W=180°, S=-90°`) hoặc `--heading-deg`: tính `turn_deg = wrap180(target_deg - gyro_yaw_deg)`. Nếu `abs(turn_deg) > 3.0°`, gọi `closed_loop_turn(..., fast_settle=True, include_vision=True)` rồi gọi lại `topo.observe_status(post_turn_st)` để phân giải ngay mọi góc mù vừa quét qua tại nút đứng.
   - Bắt đầu ghi nhận `topo.begin_leg(cur_st)` nếu đang đứng tại một `Node`.

2. **Bước 2 — Chạy phản xạ 20 Hz & Lướt góc chữ L 1 lựa chọn (`Auto-Chain`) (Đã chốt cùng người dùng):**
   - **Ngân sách quãng đường từng chặng (`seg_max`):**
     - Mặc định cấp `seg_max = min(3.5, remaining_dist)` kể cả khi xe xuất phát từ giữa tâm ngã 3/ngã 4 mở hai bên (`left90 > 0.75` và `right90 > 0.75`), giúp đi trọn các hành lang thẳng dài `2.0–3.2 m` (`maze_a`, `maze_c`) không bị cắt lửng lơ.
     - Chỉ hạ `seg_max = min(1.8, remaining_dist)` khi không gian xung quanh thực sự là sân trống rộng (`min(left90, right90, left120, right120) > 1.35 m` hoặc `Node #0.kind == "open_space"` và hai bên trống `> 0.85 m`) để kiểm tra định kỳ an toàn cánh bên.
   - **5 điều kiện bắt buộc để tự động lướt góc chữ L (`auto-chain`)** (khi `auto_chain == True`, số đoạn đã nối `< max_chain`, quãng đường còn lại `> 0.15 m`, và ở đoạn nối thứ 2 trở đi phải đi được `moved_m >= 0.20 m` để chống lặp tại chỗ):
     1. `advance_segment` vừa dừng với `event == "junction"`.
     2. Vị trí dừng hiện tại **cách mọi `Node` đã đăng ký trong `TopologicalOdometer` trên `0.42 m`** (đảm bảo khi xe lùi/vòng về lại một nút đã biết như `Node #0` trên `maze_b` hoặc khép vòng trên `maze_c`, xe bắt buộc dừng ngay tại nút đó và trả quyền cho AI).
     3. `open_branches` chỉ có **đúng 1 hướng rẽ ngang duy nhất**: `open_branches == ["left"]` hoặc `open_branches == ["right"]` (phía trước bị tường chặn, sườn đối diện bị tường chặn).
     4. Không thấy trụ đích (`vision.target_visible == false`) và không thấy bất kỳ vật cản màu nào trong khung hình (`vision.obstacles == []`).
     5. **Xác nhận ngữ cảnh hành lang (không xét cứng lúc mới xuất phát):** Trong quá trình chạy chặng vừa rồi cảm biến sườn đã ghi nhận vách hành lang (`min(left90, right90) <= 0.60 m` ở ít nhất một mẫu trên đường đi), **hoặc** tại điểm dừng góc cua sườn bị chặn có `rf_90 <= 0.55 m` kèm `min(left120, right120) < 1.35 m` (loại trừ trường hợp xe đâm chéo sát góc tường bao sân mở `4m x 4m`).
   - Khi thỏa đủ 5 điều kiện: khóa góc vuông `90°` kế tiếp `next_yaw = round((cur_yaw + (±90.0)) / 90.0) * 90.0`, xoay bằng `closed_loop_turn(..., fast_settle=True)`, ghi `"left"` / `"right"` vào `auto_l_turns`, và chạy tiếp chặng `advance_segment` kế tiếp.
   - Khi dừng tại bất kỳ **Điểm Quyết định Thực sự** nào (ngã giao `>= 2` nhánh, khớp `Node` cũ, `dead_end`, `dynamic_gate_ahead`, `static_obstacle_ahead`, `target_ready`, hoặc chạm `max_dist`/`max_chain`), dừng êm, cập nhật `TopologicalOdometer.finish_go(...)` và trả JSON về cho System 2.

3. **Định dạng JSON trả về của `go` (luôn lọc sạch `debug`):**
   ```json
   {
     "event": "junction | dead_end | dynamic_gate_ahead | static_obstacle_ahead | target_ready | max_dist",
     "moved_m": 1.94,
     "segments_chained": 2,
     "auto_l_turns": ["left"],
     "open_branches": ["left", "right"],
     "open_cardinals": ["W", "E"],
     "obstacle": null,
     "flank_clearance_m": {"left": 0.82, "right": 0.84},
     "topo": { ... },
     "after_status": { ... }
   }
   ```

4. **Lệnh `robot_ctl.py topo` & Đồng bộ CLI (`cross-gate`, `dock`, `turn`, `move`, `scan`, `reset`, `reload`):**
   - `robot_ctl.py topo show`: đọc `/status?vision=1`, khởi tạo/cập nhật `TopologicalOdometer` cho `--url`, trả `{ "topo": ..., "open_branches": ..., "open_cardinals": ..., "after_status": ... }` (lọc sạch `debug`).
   - `robot_ctl.py topo reset`: xóa phiên `topo` của `--url` trong `captures/.topo_state.json`.
   - `[Đã chốt cùng người dùng]` Ở tầng CLI [robot_ctl.py](file:///d:/latuan/Programming/robot-demo/robot_ctl.py) (giữ nguyên chữ ký và giá trị trả về của các hàm lõi Mode A/B trong `skills.py`):
     - `cross-gate`: bổ sung trường `"open_cardinals"` vào JSON đầu ra CLI, và nếu đang có phiên `topo` hoạt động cho `--url` thì cộng dồn `moved_m` vào `pending_leg` + đính kèm `"topo"`.
     - `turn --wait`, `scan`, `dock`: nếu đang có phiên `topo` hoạt động cho `--url`, đồng bộ trạng thái cảm biến cuối lệnh vào `TopologicalOdometer` (xóa phiên khi gọi `reset` / `reload`, và hủy `pending_leg` nếu gọi `move` / `wheels` tự do ngoài nút). Mode A và Mode B không gọi `go` / `topo show` nên không tạo file `.topo_state.json`.

---

### 5.3. Subagent `robot-driver-hybrid` & Quy trình Đa Tác Tử Mode C

1. **Tạo tệp `.agents/skills/robot-multi-agent-rd/references/driver-hybrid-prompt.md`:**
   - **Bộ công cụ được phép dùng:** `topo show`, `status --vision`, `go`, `cross-gate`, `dock`, `turn`, `move`, `scan --no-recover-occlusion`, `camera --name front_cam`, `stop`.
   - **Cấm tuyệt đối:** `--debug`, `arenas`, `topo reset`, `reset`, `reload`, `demo`, `telemetry`, `explore`, `approach`, `overview_cam`, và cấm đọc mã nguồn dự án hay `captures/`.
   - **Chiến thuật tư duy của System 2 (`robot-driver-hybrid`):**
     - **Khởi động:** Gọi `topo show` (nhận đồng thời `after_status` kèm `vision`, `open_cardinals` và `Node #0`). Nếu `Node #0.kind == "open_space"` và chưa thấy đích, gọi `scan --max-degrees 360 --direction left --no-recover-occlusion` rồi `go`; nếu ở hành lang hoặc đã thấy hướng đi, gọi `go` (hoặc `go --cardinal <C>` theo nhánh mở).
     - **Duyệt đồ thị mê cung & Chống đi lặp cùng chiều (`maze_a`, `maze_b`, `maze_c`):**
       - Tại `current_node` có cổng `"state": "open"`: chọn một cổng `"open"` và gọi `go --cardinal <C>`.
       - Nếu không còn cổng `"open"` nhưng tại `current_node` có cổng `"uninspected"`, gọi `go --cardinal <C>` (bước xoay đầu lệnh sẽ tự kiểm tra hướng đó và chạy tiếp nếu mở, hoặc dừng ngay nếu là tường).
       - Tại `dead_end` hoặc khi `current_node` hết sạch cổng `"open"` / `"uninspected"` (bao gồm cả khi `loop_closed == true` trên `maze_c`): tìm đường đi ngắn nhất trên `topo.nodes` về nút gần nhất còn cổng `"open"` (ưu tiên `"open"` trước, chỉ xét `"uninspected"` khi toàn đồ thị không còn cổng `"open"` nào), **trong đó chỉ đi qua các cổng `"explored"` có `out_visits == 0`** (tức lùi ngược chiều đã đến, tuyệt đối tránh đi lần 2 theo cùng một chiều `out_visits >= 1` trừ khi không tồn tại đường nào khác) — đảm bảo **`redundant_revisit_count == 0`** trên cả cây ngõ cụt lẫn mạch vòng chữ U.
     - **Lách vật cản tĩnh (`2wheel`):** Khi `go` báo `static_obstacle_ahead`, đọc `flank_clearance_m` (chỉ chụp `camera --name front_cam` khi cần quan sát bố cục cụm vật cản sân mở), lách qua sườn thoáng bằng `turn ±65 --wait --vision` → `move 0.55 --speed 0.35 --wait --vision` → `scan --max-degrees 180 --no-recover-occlusion` → `go` / `dock`.
     - **Vượt làn & cổng động (`dynamic_a`, `dynamic_b`):** Khi `go` báo `dynamic_gate_ahead`, gọi `cross-gate --timeout 12.0` để vượt đúng 1 cổng, sau đó đọc `open_cardinals` / `after_status`: nếu đã `target_ready` thì `dock`, nếu còn đường/góc cua chữ S thì gọi tiếp `go` (hoặc `go --cardinal <C>` nếu phía trước bị tường chặn).
     - **Cập đích (`target_ready`):** Gọi `dock --stop-distance 0.25` và xác nhận `vision.target_visible == true`, `abs(offset_x) < 0.25`, `0.15 <= rf_center <= 0.35`.

2. **Cập nhật [SKILL.md](file:///d:/latuan/Programming/robot-demo/.agents/skills/robot-multi-agent-rd/SKILL.md) & [engineer-prompt.md](file:///d:/latuan/Programming/robot-demo/.agents/skills/robot-multi-agent-rd/references/engineer-prompt.md):**
   - Bổ sung **Mode C — `hybrid-brain`** (kích hoạt khi người dùng yêu cầu `hybrid-brain` / `Mode C`, giữ Mode A là mặc định và bảo toàn nguyên vẹn Mode B):
     - Đăng ký subagent `robot-driver-hybrid` từ `references/driver-hybrid-prompt.md`.
     - Chạy Hộp đen `telemetry record --hz 10 --max-duration 480`.
     - **Tiêu chí chấm điểm Mode C:**
       - Bắt buộc: `goal_reached == true`, `contact_count == 0`, `near_collision_count == 0`, `error_count == 0`.
       - Chất lượng tư duy & hiệu năng lai: `redundant_revisit_count == 0`, báo cáo đầy đủ `path_efficiency`, `wall_time_s`, `active_drive_time_s`, `agent_think_time_s`, `command_burst_count` để so sánh trực tiếp `Mode A` vs `Mode B` vs `Mode C`.
     - **Ranh giới của `robot-engineer` ở Mode C:** Được tinh chỉnh `driver-hybrid-prompt.md`, ngưỡng phản xạ an toàn trong `go_to_decision` và dung sai ghép nút trong `topo.py`; **tuyệt đối cấm** sửa `models/*.xml` hoặc nhúng thuật toán chọn nhánh/tìm đường tự động vào `topo.py` hay `go_to_decision`.

---

## 6. Tích hợp Hiển thị trên MuJoCo Studio (`studio/`)

`[Đã chốt cùng người dùng]` Không thay đổi giao thức HTTP (`127.0.0.1:8765`) trong [studio/sim_plugin.py](file:///d:/latuan/Programming/robot-demo/studio/sim_plugin.py). Để hiển thị trực tiếp tiến độ bản đồ Topo trên bảng điều khiển MuJoCo Studio mà **không đọc đĩa 60 FPS trong hàm vẽ UI và không gây lỗi khóa file Windows (`PermissionError / WinError 5`)**:
- Trong [studio/messages.py](file:///d:/latuan/Programming/robot-demo/studio/messages.py): thêm các trường mặc định nhẹ vào `RobotTelemetry` (`topo_node_count: int = 0`, `topo_current_node: int = -1`, `topo_open_exits: int = 0`).
- Trong [studio/sim_plugin.py](file:///d:/latuan/Programming/robot-demo/studio/sim_plugin.py): kiểm tra `mtime` của `captures/.topo_state.json` tối đa `2 Hz` (bọc `try/except (OSError, ValueError)` an toàn, đồng thời ghi tệp bên `topo.py` có retry ngắn `3 x 15ms` khi `os.replace` gặp `PermissionError` trên Windows) để cập nhật bộ nhớ đệm gửi qua `RobotTelemetry`.
- Trong [studio/viewer_plugin.py](file:///d:/latuan/Programming/robot-demo/studio/viewer_plugin.py): khi `t.topo_node_count > 0`, hiển thị thêm 1 dòng trạng thái gọn dưới mục Telemetry (`Topo Graph: N nodes | Current: #k | Open exits: m`).

---

## 7. Phần Dành Cho Người Lập Kế Hoạch Thi Công (What's Left to the Builder)

1. Chi tiết hóa cấu trúc lớp `TopologicalOdometer` trong `robot_sim/topo.py` (ghi file nguyên tử `.tmp -> replace` kèm retry `PermissionError` trên Windows, hàm tiện ích quy đổi góc/cardinal dùng chung giữa `topo.py`, `skills.py` và `robot_ctl.py`).
2. Triển khai `go_to_decision` trong [robot_sim/skills.py](file:///d:/latuan/Programming/robot-demo/robot_sim/skills.py) tái sử dụng sạch `closed_loop_turn`, `advance_segment`, `compute_open_branches` mà không làm thay đổi hành vi của Mode A hay Mode B.
3. Xây dựng bộ kiểm thử tự động `tests/test_hybrid_brain.py` và cập nhật `tests/test_agent_skill.py` / `tests/test_cli.py`:
   - Kiểm thử `TopologicalOdometer`: hợp nhất nút `<= 0.42 m`, cạnh hai đầu cổng `out_visits`/`in_visits` qua hành lang chữ L/U, nối tiếp `pending_leg` qua cổng động, phân giải `"uninspected"` ngay sau khi xoay tại chỗ, cờ `loop_closed` (chỉ bật cho dây cung mới, không bật khi đi lùi cạnh cũ), tự động reset phiên, và kiểm định không lộ `debug` hay gợi ý nhánh.
   - Kiểm thử `go_to_decision` & CLI `go`/`topo`: xoay khóa hướng đầu lệnh, xuất phát từ ngã 4 mở đi trọn hành lang dài `> 2.0 m`, tự lướt góc chữ L 1 lối đi, dừng ngay ở ngã giao `>= 2` nhánh hoặc `Node` đã biết, dừng an toàn trước vật cản tĩnh/động và trụ đích (`contact_count == 0`), kèm kịch bản tích hợp headless giải `maze_a`, `maze_b`, `maze_c`, `dynamic_b`.
   - Bảo đảm toàn bộ 106 bài kiểm thử cũ + bộ kiểm thử mới đều PASS 100%.
