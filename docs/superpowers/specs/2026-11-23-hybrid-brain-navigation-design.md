# Thiết kế Hệ thống Điều hướng Lai Phân tầng System 1 / System 2 (`hybrid-brain` — Mode C)

**Ngày thiết kế:** 2026-11-23  
**Trạng thái:** Bản thảo chờ người dùng phê duyệt (trước bước lập kế hoạch thi công `writing-plans`)  
**Phạm vi:** Bộ định vị đồ thị khách quan (`robot_sim/topo.py`), kỹ năng vận động gộp System 1 (`robot_sim/skills.py`, `robot_ctl.py`), cấu hình đa tác tử Mode C (`.agents/skills/robot-multi-agent-rd/`), tích hợp theo dõi trên MuJoCo Studio (`studio/`), và bộ kiểm thử tự động (`tests/`).

---

## 1. Mục đích & Lý do (Intent & Why)

Dự án hiện có hai chế độ điều khiển đại diện cho hai thái cực:
1. **Mode A (`auto-skill` — `explore_maze`, `approach_target`):** Chạy khép kín ở 20 Hz rất nhanh (`< 60s`), nhưng nhồi hơn 2.000 dòng thuật toán cứng (bám tường trái/phải, đánh dấu Trémaux, tự quay đầu ngõ cụt, tự né vật cản, tự vượt chuỗi cổng động) vào Python. AI Agent gần như không thực sự tư duy điều khiển.
2. **Mode B (`agent-brain` — `advance`, `cross-gate`, `dock` vô trạng thái):** Giao 100% trí nhớ và quyết định từng góc cua cho LLM. Thực nghiệm 3 vòng R&D trên `maze_a` cho thấy hai nút thắt lớn làm che mờ năng lực thật của AI:
   - **Độ trễ cơ học vô ích:** Xe mất `732s` đứng chờ LLM suy nghĩ so với `168s` lăn bánh (`~30` lượt CLI), bao gồm cả việc phải dừng hỏi LLM 2 lượt (`turn` rồi `advance`) tại các góc chữ L chỉ có duy nhất 1 lối đi.
   - **Trôi số học khi tính nhẩm tọa độ bằng chữ:** Bắt LLM tự nhẩm `hypot(odom_x - nx, odom_y - ny) <= 0.40` và tự đổi hướng tương đối/tuyệt đối trong văn bản gây nhầm nút giao (`redundant_revisit_count = 10..13`), dù phản xạ vật lý an toàn tuyệt đối (`contact_count = 0`).

**Mục đích của Mode C (`hybrid-brain`):** Đo lường **năng lực điều khiển robot thực chất của AI Agent** trong vòng lặp thời gian thực (`Sense → Think → Act`) thông qua kiến trúc **System 1 / System 2** sát với robot tự hành ngoài đời:
- **System 1 (Python 20 Hz — Tay chân & Định vị đo đạc):** Đảm nhận điều khiển chuyển động gộp (`xoay khóa trục + tiến`), tự lướt qua các khúc cua bắt buộc **chỉ có 1 lối đi duy nhất**, và ghi nhận khách quan **Đồ thị Odometry Topo** của những nơi xe *đã thực sự đặt bánh tới* (tương tự module SLAM/Odometry trên mạch nhúng).
- **System 2 (AI Agent — Bộ não chiến thuật bị chấm điểm):** Chịu trách nhiệm **100% mọi quyết định có lựa chọn**: chọn nhánh tại mọi ngã 3/ngã 4, suy luận cây ngõ cụt để lùi về nút tổ tiên, phát hiện & thoát vòng lặp đảo chữ U (`maze_c`), nhìn ảnh `front_cam` chọn đường lách vật cản tĩnh (`2wheel`), canh nhịp vượt từng làn/cổng động (`dynamic_a`, `dynamic_b`) và chốt cập đích (`dock`).

---

## 2. Mục tiêu, Phi mục tiêu & Phản mục tiêu

### Mục tiêu (Goals)
1. Xây dựng **Mode C (`hybrid-brain`)** gồm lệnh gộp `go`, bộ định vị khách quan `TopologicalOdometer` (`topo`), và subagent **`robot-driver-hybrid`** giải quyết trọn vẹn **cả 6 sa bàn chuẩn** (`2wheel`, `maze_a`, `maze_b`, `maze_c`, `dynamic_a`, `dynamic_b`) trên cả `sim_server.py` lẫn **MuJoCo Studio** (`studio/`).
2. Giảm mạnh số lượt dừng chờ LLM trên mê cung (từ `~30` lượt ở Mode B xuống `~4–10` lượt quyết định thực chất trên `maze_a`), đưa `redundant_revisit_count` về `0` và giữ `contact_count == 0`, `near_collision_count == 0`.
3. **Bảo toàn 100%** Mode A (`auto-skill`), Mode B (`agent-brain`) và toàn bộ 106 bài kiểm thử hiện có để làm bộ đối chuẩn 3 chế độ (`Mode A` vs `Mode B` vs `Mode C`).

### Phi mục tiêu (Non-Goals)
- Không chỉnh sửa phần cứng `models/robot_2wheel.xml` hay bất kỳ địa hình nào trong `models/arena_*.xml`.
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
| **Chuyển động cơ bản** | Xoay khóa góc chính xác bằng Gyro + chạy giữ tâm hành lang (`go`). | Quyết định hướng tuyệt đối (`E/N/W/S`) hoặc góc xoay cần đi. |
| **Khúc cua 1 lối đi (Góc chữ L)** | Tự xoay 90° và chạy tiếp nếu chỉ có **đúng 1 hướng mở**, không thấy đích, không có vật cản. | Không phải dừng chờ vô ích ở đoạn ống cong không có ngã rẽ. |
| **Ngã giao `>= 2` nhánh mở** | Phanh dừng êm giữa tâm nút, báo danh sách nhánh mở + bản đồ đo đạc. | **Tự chọn nhánh khám phá** dựa trên lịch sử đồ thị & cảm biến. |
| **Ngõ cụt (`dead_end`)** | Phanh dừng an toàn, ghi nhận thực tế nút hiện tại là `dead_end`. **Không tự quay đầu.** | **Tự suy luận nút cha/ông còn nhánh mở** và phát lệnh `go --cardinal <hướng_thoát>`. |
| **Vòng lặp (`maze_c`)** | Chỉ báo cờ khách quan `loop_closed = true` khi khớp lại tọa độ `Node #k` cũ từ cạnh mới. | **Tự nhận diện mạch vòng** trên đồ thị và đổi sang nhánh chưa khám phá. |
| **Vật cản tĩnh (`2wheel`)** | Phanh dừng trước vật cản, báo `static_obstacle_ahead` + khoảng hở 2 sườn. | Tự xem `flank_clearance_m` / ảnh `front_cam` và điều khiển vòng lách (`turn` + `move` + `scan`/`go`). |
| **Vật cản động (`dynamic_a/b`)** | Phanh ngoài vùng quét piston, báo `dynamic_gate_ahead`; thực thi vượt **1 cổng** khi được lệnh `cross-gate`. | Tự điều phối trình tự vượt từng cổng một kết hợp rẽ hành lang chữ S. |

---

## 4. Các Phương án Đã Cân nhắc & Lý do Chọn

1. **Phương án 1 — System 1 / System 2 Phân tầng theo Sự kiện Quyết định (`Mode C: hybrid-brain`) [ĐƯỢC CHỌN]:**
   - *Cơ chế:* Gộp `xoay + tiến + lướt góc chữ L 1 lựa chọn` vào lệnh `go`, kèm sổ cái định vị khách quan `TopologicalOdometer` lưu ở phía client theo phiên chạy; AI Agent trực tiếp quyết định ở mọi nút phân nhánh, ngõ cụt, vật cản và cổng động.
   - *Đánh đổi:* Giữ AI ngồi trực tiếp trong buồng lái thời gian thực để đánh giá đúng năng lực điều khiển, giảm 70–85% độ trễ dừng chờ so với Mode B, loại bỏ lỗi nhẩm tọa độ mà không vi phạm ranh giới công bằng.
2. **Phương án 2 — Code-as-Policies (LLM sinh kịch bản Python 20 Hz đầu phiên):**
   - *Cơ chế:* LLM viết một hàm Python điều khiển robot trước lúc chạy rồi để Python thực thi toàn bộ.
   - *Lý do loại:* Kiểm tra năng lực *viết code offline* của LLM chứ không kiểm tra năng lực *điều khiển & phản ứng trực tiếp theo luồng cảm biến sống* của AI Agent; kém linh hoạt trước bất ngờ thị giác giữa đường và đòi hỏi sandbox thực thi code tự sinh.
3. **Phương án 3 — Chỉ tối ưu đường truyền trên Mode B thuần túy (giữ nguyên dừng ở mọi góc L, không có `TopologicalOdometer`):**
   - *Lý do loại:* Vẫn bắt xe dừng chờ ở mọi góc chữ L không có lựa chọn và vẫn để LLM tự nhẩm khoảng cách Euclid bằng văn bản, không giải quyết tận gốc hiện tượng trôi nút giao trên mê cung dài (`maze_b`, `maze_c`).

---

## 5. Thiết kế Chi tiết Kiến trúc Mode C (`hybrid-brain`)

### 5.1. Module Định vị Đồ thị Khách quan — `robot_sim/topo.py` (`TopologicalOdometer`)

`[Quyết định kỹ thuật đề xuất]` Đặt sổ cái định vị ở phía công cụ điều khiển robot (`captures/.topo_state.json`, phân tách theo `base_url`) thay vì sửa máy chủ vật lý `sim_server.py`, giúp tương thích tức thì với cả `sim_server.py` lẫn MuJoCo Studio và giữ đúng mô hình phần cứng thật (máy tính nhúng trên xe tự tích hợp SLAM từ luồng cảm biến trả về).

1. **Nguồn dữ liệu đầu vào hợp lệ duy nhất (100% Sim-to-Real):**
   - Chỉ đọc từ `status["sensors"]`: `odom_x_m`, `odom_y_m`, `odom_dist_m`, `gyro_yaw_deg`, `rangefinders_m` và `sim_time`. Tuyệt đối không đọc `debug`.
2. **Tự động xóa sạch khi sang lượt chạy mới (`Auto-Reset Detection`):**
   - Tự động khởi tạo lại đồ thị rỗng với `Node #0 (0.0, 0.0)` bất cứ khi nào:
     - Người dùng / Main Agent gọi `robot_ctl.py reset`, `robot_ctl.py reload`, hoặc `robot_ctl.py topo reset`; **hoặc**
     - Phát hiện `sim_time` giảm (`cur_sim_time < last_sim_time - 1e-3`), `odom_dist_m` giảm (`cur_odom_dist_m < last_odom_dist_m - 1e-3`), hoặc xe vừa xuất phát lại tại gốc (`cur_odom_dist_m < 0.03` trong khi `last_odom_dist_m > 0.15`).
3. **Cấu trúc Nút (`Node`) & Cạnh hai đầu cổng (`Port-Accurate Edge`):**
   - `[Quyết định kỹ thuật đề xuất]` Hợp nhất vị trí dừng hiện tại `(odom_x_m, odom_y_m)` vào `Node #k` đã biết nếu khoảng cách Euclid `<= 0.42 m` (giữ cố định tọa độ neo ban đầu của `Node #k` để tránh trôi tích lũy); nếu lớn hơn, tạo `Node #N` mới.
   - ** Xử lý chính xác hành lang gấp khúc chữ L/U:** Khi một hành lang có lướt qua góc chữ L (ví dụ rời `Node #1` theo hướng Đông **`E`**, ôm góc chữ L sang Bắc, rồi tiến vào `Node #2` theo hướng Bắc nên hướng nhìn ngược lại tại `Node #2` là Nam **`S`**), hai đầu của cùng một hành lang có hướng cửa khác nhau (`E` ở `Node #1`, nhưng `S` ở `Node #2`). Do đó, mỗi cạnh đã đi được lưu chính xác theo **cổng thực tế tại từng đầu nút**:
     - Trên `Node #1`: cổng `"E"` ghi `{"state": "explored", "to_node": 2, "visits": 1, "dist_m": 1.62}`.
     - Trên `Node #2`: cổng `"S"` ghi `{"state": "explored", "to_node": 1, "visits": 1, "dist_m": 1.62}`.
     - Nhờ vậy, khi AI muốn đi ngược từ `Node #2` về `Node #1`, AI nhìn cổng `"S"` trên `Node #2` và gọi `go --cardinal S` là System 1 tự lướt ngược qua góc chữ L về đúng `Node #1` mà không bao giờ lệch hướng!
4. **Ghi nhận trung thực điểm mù `180°` phía sau lúc khởi hành (`Rear Blind-Spot Honesty`):**
   - `[Quyết định kỹ thuật đề xuất]` Phần cứng `models/robot_2wheel.xml` chỉ có tia khoảng cách ở `0°, ±30°, ±90°, ±120°` (không có cảm biến đuôi `180°`). Tại `t = 0` ở `Node #0` (`kind = "start"`), nếu không gian xung quanh là hành lang hẹp, hướng `180°` ngay sau lưng xe thực tế **chưa được bất kỳ cảm biến nào nhìn thấy** (ví dụ trên `maze_b`, lối duy nhất tới đích nằm đúng hướng Tây `180°` sau lưng điểm xuất phát `(0, 0)`).
   - Thay vì bỏ trống hướng sau lưng (vốn ngầm khẳng định sai rằng phía sau có tường kín), `Node #0` ghi nhận cổng sau lưng với trạng thái trung thực `"state": "uninspected"`. Ngay khi xe xoay mặt hoặc đi lùi/quay về `Node #0` khiến hướng đó lọt vào vùng quét `0° / ±90°` của laser, cổng đó lập tức được phân giải thành `"open"` (nếu thông thoáng) hoặc bị xóa khỏi `exits` (nếu là tường chắn).
5. **Dữ liệu JSON `topo` xuất cho AI Agent (thuần quan sát, không gợi ý đường):**
   - `current_node`: mã số nút hiện tại (`int`, hoặc `null` nếu xe đang dừng tạm lưng chừng do chạm `max_dist` hoặc tránh vật cản trên sân mở — không sinh nút giả giữa hành lang thẳng).
   - `heading_cardinal`: hướng 90° gần nhất hiện tại (`"E" | "N" | "W" | "S"`) kèm `gyro_yaw_deg`.
   - `loop_closed`: `true` nếu bước di chuyển vừa rồi nối vào một `Node` cũ đã tồn tại từ một cổng trước đó còn `"open"` hoặc `"uninspected"` (báo hiệu khách quan vừa khép một vòng lặp).
   - `nodes`: danh sách gọn các nút đã biết, mỗi nút gồm `id`, `xy`, `kind` (`"start" | "junction" | "dead_end" | "open_space"`), và từ điển `exits` theo các hướng `E/N/W/S` hiện diện tại nút đó (`state`: `"open"` nếu cảm biến thấy thông thoáng nhưng xe chưa đi vào, `"uninspected"` cho góc mù `180°` sau lưng `Node #0` khi chưa quét tới, hoặc `"explored"` kèm `to_node`, `visits`, `dist_m` nếu xe đã đi qua).
   - **Không có trường `parent`, không tự đánh dấu nhánh của nút cha là `dead_end`, không có `suggested_exit`.** AI Agent phải tự đọc `nodes` để suy luận nhánh nào dẫn tới nút cụt hoặc nhánh nào còn `"open"` / `"uninspected"` sâu bên trong.

---

### 5.2. Kỹ năng Vận động Gộp System 1 — `go_to_decision` (`robot_ctl.py go`)

Thêm hàm `go_to_decision` vào [robot_sim/skills.py](file:///c:/Users/latuan/Programming/robot-demo/robot_sim/skills.py) và lệnh CLI `.venv\Scripts\python.exe robot_ctl.py go`:

```powershell
.venv\Scripts\python.exe robot_ctl.py go [--cardinal E|N|W|S | --heading-deg <DEG>] [--max-dist 6.0] [--speed 0.55] [--max-chain 6] [--no-chain]
```

1. **Bước 1 — Xoay khóa hướng đầu lệnh (nếu có `--cardinal` hoặc `--heading-deg`):**
   - Quy đổi `E = 0°`, `N = +90°`, `W = 180°`, `S = -90°`.
   - Tính góc bù chính xác `turn_deg = wrap180(target_deg - gyro_yaw_deg)`. Nếu `abs(turn_deg) > 3.0°`, gọi `closed_loop_turn` để khóa thẳng trục trước khi lăn bánh (nếu `<= 3.0°`, bỏ qua bước xoay để tiết kiệm thời gian, để bộ giữ hướng của `advance_segment` tự chỉnh mượt).
2. **Bước 2 — Chạy phản xạ 20 Hz & Lướt góc chữ L 1 lựa chọn (`Auto-Chain`):**
   - `[Quyết định kỹ thuật đề xuất]` **Phân bổ ngân sách quãng đường (`--max-dist 6.0` tổng hành trình lệnh `go`):**
     - Trong hành lang có tường hai bên, mỗi chặng `advance_segment` con được cấp `seg_max = min(3.5, remaining_dist)` (đủ đi trọn một mạch các hành lang thẳng dài `2.0–2.3 m` trên `maze_a` và `maze_c` mà không bị ngắt lửng lơ giữa hành lang; đồng thời tổng hành trình `6.0 m` với `max_chain = 6` đủ đi trọn nhánh ống cụt gấp khúc `4.4 m` trên `maze_b` chỉ trong 1 lệnh `go`).
     - Ở sân mở không có tường hai bên (`left90 > 0.75` và `right90 > 0.75`), giới hạn một chặng tiến đơn lẻ ở `min(1.8, remaining_dist)` để xe dừng kiểm tra an toàn định kỳ.
     - Nếu dừng vì chạm ngưỡng `max_dist` lưng chừng hành lang (ví dụ người gọi chủ động truyền `--max-dist` nhỏ), `TopologicalOdometer` chỉ cập nhật quãng đường tạm thời (`current_node = null`) chứ không sinh nút giao ảo giữa ống thẳng.
   - `[Quyết định kỹ thuật đề xuất]` **Điều kiện duy nhất được phép lướt tiếp góc chữ L (`auto-chain`)** mà không dừng hỏi AI (tối đa `max_chain = 6` đoạn và tổng quãng đường `<= max_dist`):
     1. `advance_segment` vừa dừng với `event == "junction"`.
     2. Vị trí dừng hiện tại **không trùng khớp (`> 0.42 m`) với bất kỳ `Node` nào đã đăng ký trong `TopologicalOdometer`** (bảo đảm khi xe quay về lại một nút cũ như `Node #0` trên `maze_b`, xe bắt buộc dừng ngay tại nút đó, cập nhật các cổng vừa lọt vào tầm nhìn cảm biến và trả quyền cho AI thay vì tự lướt xuyên qua nút gốc).
     3. `open_branches` chỉ có **đúng 1 hướng duy nhất** và hướng đó là rẽ ngang: `open_branches == ["left"]` hoặc `open_branches == ["right"]` (tức phía trước bị tường chặn, một bên bị tường chặn, chỉ còn đúng 1 ngả rẽ vuông góc bắt buộc).
     4. Không thấy trụ đích (`after_status.vision.target_visible == false`) và không thấy bất kỳ vật cản màu nào trong khung hình (`after_status.vision.obstacles == []`).
     5. Không gian đầu và cuối đoạn đều là hành lang có vách (không tự bẻ lái liên hoàn trên sân mở).
   - Khi thỏa đủ 5 điều kiện trên: System 1 xoay khóa `±90°` theo hướng mở duy nhất đó (dùng công thức khóa bội số `90°` chống trôi góc tích lũy), ghi nhận `auto_turns_count += 1`, rồi gọi tiếp `advance_segment` cho đoạn kế tiếp.
   - Ngay khi chạm bất kỳ **Điểm Quyết định Thực sự** nào — khớp lại một `Node` đã biết, ngã giao có `>= 2` nhánh mở (VD `["left", "straight"]`, `["left", "right"]`, `["left", "straight", "right"]`), `dead_end`, `dynamic_gate_ahead`, `static_obstacle_ahead`, `target_ready`, hoặc hết hạn mức `max_dist`/`max_chain` — System 1 phanh dừng êm, cập nhật `TopologicalOdometer` và trả quyền về ngay cho System 2 (AI Agent).
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
   *(Cung cấp sẵn `open_cardinals` quy đổi từ `open_branches` + `gyro_yaw_deg` hiện tại bằng toán học chính xác để AI không bao giờ nhầm Trái/Phải khi xe đang quay mặt về hướng Nam/Tây.)*

4. **Lệnh phụ trợ đọc/xóa bản đồ đo đạc (`robot_ctl.py topo`):**
   - `robot_ctl.py topo show` — đọc trạng thái cảm biến hiện tại + xuất bản đồ `topo` (cập nhật nút khởi đầu nếu chưa có).
   - `robot_ctl.py topo reset` — xóa file `.topo_state.json` (dùng trong kiểm thử hoặc khi muốn xóa sổ cái thủ công).
   - Ngoài ra, khi gọi `cross-gate` hoặc `dock` thành công, `robot_ctl.py` cũng đồng bộ vị trí dừng mới vào `TopologicalOdometer` nếu đang có phiên `topo` hoạt động để đồ thị không bị đứt quãng trên `dynamic_b` (hành lang chữ S có cổng piston).

---

### 5.3. Subagent `robot-driver-hybrid` & Quy trình Đa Tác Tử Mode C

1. **Thêm tệp `.agents/skills/robot-multi-agent-rd/references/driver-hybrid-prompt.md`:**
   - **Bộ công cụ được phép dùng:** `status --vision`, `topo show`, `go`, `cross-gate`, `dock`, `turn`, `move`, `scan --no-recover-occlusion`, `camera --name front_cam`, `stop`.
   - **Cấm tuyệt đối:** `--debug`, `arenas`, `reset`, `reload`, `demo`, `telemetry`, `explore`, `approach`, `overview_cam`, và cấm đọc mã nguồn dự án.
   - **Chiến thuật tư duy của System 2 (AI Agent):**
     - **Khởi động:** Gọi `status --vision` (hoặc `topo show`). Nếu ở sân mở chưa thấy đích, gọi `scan --max-degrees 360 --direction left --no-recover-occlusion`; nếu ở hành lang hoặc đã thấy hướng đi, gọi `go`.
     - **Duyệt đồ thị & Thoát ngõ cụt / vòng lặp (`maze_a`, `maze_b`, `maze_c`):** Đọc trực tiếp `topo.nodes` và `open_cardinals` từ kết quả `go`:
       - Tại nút có cổng `"open"`: chọn một cổng `"open"` và gọi `go --cardinal <C>`. (Nếu tại nút hiện tại chỉ còn cổng `"uninspected"` ở `Node #0`, gọi `turn 180 --wait --vision` hoặc `go --cardinal <C>` để kiểm tra và đi vào hướng sau lưng đó.)
       - Tại `dead_end` hoặc nút mà mọi cổng đều dẫn sang các vùng đã cạn sạch `"open"` / `"uninspected"` (do chính AI kiểm tra trên `topo.nodes`): tìm đường ngắn nhất trên các cạnh `"explored"` của `topo.nodes` về nút gần nhất còn cổng `"open"` hoặc `"uninspected"`, rồi đi từng bước `go --cardinal <cổng_nối>` để quay về đó — bảo đảm **`redundant_revisit_count == 0`**, kể cả khi điểm xuất phát có nhánh sau lưng trên `maze_b` hoặc đảo vòng lặp chữ U trên `maze_c` (`loop_closed == true`).
     - **Lách vật cản tĩnh (`2wheel`):** Khi `go` báo `static_obstacle_ahead`, đọc `flank_clearance_m` (và chụp `camera --name front_cam` nếu cần xem bố cục), chủ động lách qua sườn thoáng bằng `turn ±65 --wait --vision` → `move 0.55 --speed 0.35 --wait --vision` → `scan --max-degrees 180 --no-recover-occlusion` → `go` / `dock`.
     - **Vượt làn & cổng động (`dynamic_a`, `dynamic_b`):** Khi `go` báo `dynamic_gate_ahead`, gọi `cross-gate --timeout 12.0` để vượt qua 1 cổng, sau đó gọi tiếp `go` (nếu ở góc cua sau cổng không có hướng đi thẳng thì chọn hướng trong `open_cardinals` truyền vào `go --cardinal <C>`).
     - **Cập đích (`target_ready`):** Gọi `dock --stop-distance 0.25` và kiểm tra đủ 3 điều kiện dừng chuẩn trước khi kết thúc.

2. **Cập nhật `.agents/skills/robot-multi-agent-rd/SKILL.md` & `engineer-prompt.md`:**
   - Bổ sung **Mode C — `hybrid-brain`** bên cạnh Mode A (mặc định) và Mode B:
     - Đăng ký subagent `robot-driver-hybrid` từ `references/driver-hybrid-prompt.md`.
     - Hộp đen chạy nền `--max-duration 360` (tương đương ngưỡng thời gian thực tế như Mode A, nhanh hơn nhiều so với `900s` của Mode B!).
     - **Tiêu chí chấm điểm Mode C:**
       - Bắt buộc: `goal_reached == true`, `contact_count == 0`, `near_collision_count == 0`, `error_count == 0`.
       - Chất lượng bộ não & tốc độ lai: `redundant_revisit_count == 0`, ghi nhận đầy đủ `path_efficiency`, `wall_time_s`, `active_drive_time_s`, `agent_think_time_s`, `command_burst_count` để lập bảng so sánh 3 cột (`Mode A` vs `Mode B` vs `Mode C`).
     - **Ranh giới sửa chữa của `robot-engineer` trong Mode C:** Được tinh chỉnh `driver-hybrid-prompt.md`, ngưỡng phản xạ an toàn trong `go_to_decision`/`advance_segment`/`cross_dynamic_gate`/`dock_target` và sai số ghép nút trong `topo.py`; **tuyệt đối cấm** sửa `models/*.xml` hoặc đưa thuật toán chọn nhánh/giải mê cung tự động vào `topo.py` hay `go_to_decision`.

---

## 6. Tích hợp Hiển thị trên MuJoCo Studio (`studio/`)

`[Quyết định kỹ thuật đề xuất]` Không cần thay đổi giao thức HTTP (`127.0.0.1:8765`) trong `studio/sim_plugin.py` vì `go` và `topo` hoạt động hoàn toàn qua các endpoint cảm biến/bánh xe hiện có (`/status?vision=1`, `/wheels`, `/stop`, `/camera`). Để việc quan sát trực tiếp trong MuJoCo Studio sinh động hơn, bảng `viewer_plugin.py` có thể hiển thị thêm dòng tóm tắt gọn số `Node` đã mở trong `captures/.topo_state.json` (chỉ đọc hiển thị thụ động nếu file tồn tại, không ảnh hưởng luồng mô phỏng).

---

## 7. Phần Dành Cho Người Lập Kế Hoạch Thi Công (What's Left to the Builder)

1. Thiết kế chi tiết schema lưu trữ JSON nguyên tử (`.tmp` -> `replace`) cho `robot_sim/topo.py` (`TopologicalOdometer`), xử lý trường hợp xe dừng giữa sân mở (`open_space`, không gộp bừa các điểm lách vật cản thành nút mê cung) so với nút giao hành lang (`junction` / `dead_end`).
2. Viết hàm `go_to_decision` trong [robot_sim/skills.py](file:///c:/Users/latuan/Programming/robot-demo/robot_sim/skills.py) tái sử dụng tối đa `closed_loop_turn`, `advance_segment` và `compute_open_branches` hiện có mà không phá vỡ chữ ký hay hành vi của Mode A & Mode B.
3. Xây dựng bộ kiểm thử đơn vị & tích hợp mới `tests/test_hybrid_brain.py`:
   - Kiểm thử `TopologicalOdometer`: hợp nhất nút `<= 0.42 m`, cạnh hai đầu cổng qua hành lang chữ L/U, cờ `loop_closed` trên mạch vòng, tự động reset khi `sim_time`/`odom_dist_m` quay về 0, và xác nhận không rò rỉ `debug` hay trường gợi ý nhánh.
   - Kiểm thử `go_to_decision`: xoay khóa hướng đầu lệnh, tự lướt qua góc chữ L 1 lối đi, dừng ngay khi gặp ngã 3 (`>= 2` hướng), dừng ngay khi thấy đích hoặc vật cản màu (`contact_count == 0`), và chạy kịch bản mô phỏng System 2 giải `maze_a`, `maze_c`, `dynamic_b`.
   - Đảm bảo toàn bộ 106 bài kiểm thử cũ + bộ kiểm thử mới đều PASS 100%.
