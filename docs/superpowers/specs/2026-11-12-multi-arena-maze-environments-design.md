# Thiết kế: Kiến trúc Đa Địa Hình (Multi-Arena) & Bộ 3 Mê Cung Thử Thách cho Robot 2 Bánh MuJoCo

- **Ngày:** 2026-11-12
- **Thư mục dự án:** `robot-demo`
- **Tài liệu nền tảng trước đó:**
  - `docs/superpowers/specs/2026-11-11-mujoco-2wheel-robot-design.md`
  - `docs/superpowers/specs/2026-11-11-multi-agent-robot-control-design.md`

---

## 1. Mục đích & Lý do (Intent & Why)

Hệ thống Multi-Agent R&D (`Main Agent` + `robot-driver` + `robot-engineer`) đã giải quyết trọn vẹn bài toán sa bàn cơ bản 3 vật cản màu (`models/arena_2wheel.xml`), đạt đích bị che khuất chỉ với 2 lệnh CLI (`scan` → `approach`) và `0` va chạm.

Tuy nhiên, sa bàn đơn lẻ hiện tại chỉ che khuất cục bộ bởi các khối trụ/hộp nhỏ: robot chỉ cần nhích ngang `0.55m–0.85m` tạo thị sai là thấy ngay cột đích xanh lá. Để nâng độ khó mô phỏng lên sát bài toán robot tự hành thực tế trong không gian kín, người dùng muốn:
1. Mở rộng dự án sang **kiến trúc nhiều file địa hình XML độc lập** (`models/arena_*.xml`), cho phép chọn và chuyển đổi linh hoạt từng loại địa hình.
2. Xây dựng **bộ 3 bản đồ địa hình mê cung tăng dần độ khó** (`maze_a`, `maze_b`, `maze_c`) có vách tường cao che kín hoàn toàn tầm nhìn tới cột đích xanh cho tới tận khúc cua cuối cùng, kèm các nhánh ngõ cụt đánh lừa.
3. **Không lập trình sẵn kỹ năng giải mê cung từ trước:** giữ nguyên bộ kỹ năng và cảm biến hiện có của xe, sau đó đưa bản đồ **Mê cung A (`maze_a`)** vào thử nghiệm trước để đội Multi-Agent (`robot-driver` + `robot-engineer`) tự vấp ngõ cụt, tự chẩn đoán điểm mù qua Hộp đen 10 Hz và tự nâng cấp cảm biến/thuật toán tìm đường ra.

---

## 2. Mục tiêu, Phi mục tiêu & Phản mục tiêu (Goals, Non-goals, Anti-goals)

### Mục tiêu (Goals)
1. **Tách riêng phần cứng Robot dùng chung và các file Địa hình độc lập (`models/`):**
   - Tách `<body name="robot">`, `<actuator>` và `<sensor>` sang file dùng chung `models/robot_2wheel.xml` (thẻ gốc `<mujocoinclude>`), nhúng vào mọi file địa hình qua `<include file="robot_2wheel.xml"/>`.
   - Giữ nguyên địa hình gốc `models/arena_2wheel.xml` và bổ sung 3 file địa hình mê cung mới:
     - `models/arena_maze_a.xml` (bí danh `maze_a`, đồng thời `maze` trỏ mặc định vào `maze_a`): Mê cung lưới `4×4`, hành lang rộng `1.0m` (`~0.96m` thông thủy), 2 nhánh ngõ cụt, xuất phát góc Tây-Nam `(-1.5, -1.5)`, đích Đông-Bắc `(+1.5, +1.5)` — **dùng để thử nghiệm đầu tiên**.
     - `models/arena_maze_b.xml` (`maze_b`): Mê cung lưới `5×5` phân nhánh chuẩn, hành lang rộng `0.8m` (`~0.76m` thông thủy), 3 nhánh ngõ cụt (có ngõ cụt sát vách dưới đích), xuất phát tâm `(0.0, 0.0)`, đích Đông-Bắc `(+1.6, +1.6)`.
     - `models/arena_maze_c.xml` (`maze_c`): Mê cung lưới `5×5` có **đảo tường vòng lặp chữ U tách rời ở tâm** kèm 3 nhánh cụt vòng ngoài, xuất phát tâm `(0.0, 0.0)`, đích Đông-Bắc `(+1.6, +1.6)` — đánh bại thuật toán bám một bên tường ngây thơ nếu bám trúng đảo giữa.
2. **Chuyển đổi nóng địa hình & Tự động nhận diện cảm biến mới không cần tắt Server (`robot_sim/core.py`, `sim_server.py`, `robot_ctl.py`):**
   - Hỗ trợ khởi chạy `sim_server.py --arena <name>` / `--model <path>` và chuyển đổi nóng ngay trên cửa sổ 3D đang chạy qua `POST /reload` / `robot_ctl.py reload --arena <name>` (hoặc `--model <path>`), đồng thời có lệnh tra cứu danh sách địa hình `robot_ctl.py arenas` (`GET /arenas`).
   - Hỗ trợ điểm xuất phát riêng theo từng bản đồ qua `<site name="robot_spawn" pos="x y 0" euler="0 0 yaw"/>` trong `<worldbody>` của mỗi `arena_*.xml` (không dùng `<keyframe>` cố định độ dài `nq` để không làm gãy mô hình nếu `robot-engineer` thêm khớp phụ vào `robot_2wheel.xml`).
   - Tự động quét và xuất **mọi cảm biến `<rangefinder>`** có trong mô hình biên dịch (`rf_<key>` → `sensors.rangefinders_m["<key>"]` và `sensors.rangefinder_hits["<key>"]`, đọc đúng `sensor_cutoff` riêng của từng tia) khi khởi tạo và khi `POST /reload`. Nhờ đó, nếu `robot-engineer` lắp thêm tia cảm biến hông/sau (ví dụ `rf_side_left`, `rf_side_right`) vào `models/robot_2wheel.xml` và gọi `reload`, dữ liệu cảm biến mới lập tức xuất hiện trong `/status` mà không cần khởi động lại `sim_server.py`.
3. **Bảo vệ chống gian lận Đa Bản Đồ (`robot_sim/integrity.py`, `robot_sim/arenas.py`):**
   - Khóa cứng toàn bộ cấu trúc sàn, tường bao, vách ngăn mê cung / vật cản, `robot_spawn` và `target` của cả 4 bản đồ (`2wheel`, `maze_a`, `maze_b`, `maze_c`) trong bộ kiểm định `verify_arena_integrity`.
   - Cho phép `robot-engineer` chỉ được phép tinh chỉnh cơ khí / cảm biến / động cơ bên trong `models/robot_2wheel.xml` trong khuôn khổ giới hạn vật lý Sim-to-Real (`|x| <= 0.25m`, `|y| <= 0.20m`, `0 <= z <= 0.30m` tính theo hệ quy chiếu thân `robot`).
4. **Cập nhật Tài liệu & Quy ước Tổ đội Multi-Agent (`AGENTS.md`, `.agents/skills/robot-multi-agent-rd/`, `README.md`):**
   - Cập nhật hướng dẫn cho `Main Agent`, `robot-driver` và `robot-engineer` để hỗ trợ chọn bản đồ (`--arena`), chỉnh sửa `models/robot_2wheel.xml` thay vì sửa trực tiếp `arena_*.xml`, và nâng giới hạn thời gian Hộp đen hợp lý cho quãng đường dài trong mê cung.

### Phi mục tiêu (Non-goals — Cố ý KHÔNG làm trong đợt dựng môi trường này)
- **KHÔNG lập trình sẵn kỹ năng giải mê cung** (như wall-following, Tremaux, SLAM hay cảm biến tia ngang lắp sẵn) trước khi cho Agent thử sức. Giữ nguyên 3 tia phía trước (`rf_left`, `rf_center`, `rf_right`) và các kỹ năng hiện có (`scan`, `approach`, `move`, `turn`, `wheels`) ở trạng thái xuất phát để chính đội Multi-Agent R&D tự phát hiện giới hạn và tự nâng cấp.
- Không xây dựng bộ sinh mê cung ngẫu nhiên (procedural maze generator) trong giai đoạn này; dùng 3 bản đồ cố định có kiểm định để dễ tái lập và so sánh điểm số R&D qua các vòng.

### Phản mục tiêu (Anti-goals — Tuyệt đối cấm dù về mặt kỹ thuật chạy được)
- **Cấm lộ tên bản đồ hoặc sơ đồ tường cho `robot-driver` và `robot_sim/skills.py`:** Trường nhận diện địa hình (`arena`) chỉ được đặt bên trong nhánh `"debug"` của `GET /status` (bị cắt bỏ khi không có `--debug`). Các hàm trong `robot_sim/skills.py` tuyệt đối không được hardcode tọa độ ngã rẽ hay trình tự rẽ cố định cho riêng `maze_a`/`maze_b`/`maze_c`.
- **Cấm hạ thấp tường mê cung hoặc làm tường trong suốt:** Mọi vách mê cung phải cao `0.28m` (cao hơn cả `front_cam` ở `z=0.09m` lẫn đỉnh cột `target` ở `z=0.24m`), đục hoàn toàn (`alpha=1.0`), bật va chạm vật lý đầy đủ (`contype`/`conaffinity` không bằng `0`) và dùng màu trung tính không gây nhiễu bộ lọc màu của `analyze_vision`.
- **Cấm phá vỡ tương thích ngược:** Mọi lệnh CLI, API HTTP và 38 bài kiểm thử hiện có trên bản đồ mặc định `models/arena_2wheel.xml` phải tiếp tục hoạt động đúng như cũ.

---

## 3. Ràng buộc Kỹ thuật & Kỷ luật Sim-to-Real (Constraints)

1. **CLI tốc độ cao trên Windows:** Mọi lệnh kiểm thử và tự động hóa dùng trực tiếp `.venv\Scripts\python.exe`.
2. **An toàn `<include>` chống leo thang đường dẫn:** `verify_arena_integrity` chỉ chấp nhận thẻ `<include file="robot_2wheel.xml"/>` nằm cùng thư mục với file arena XML (từ chối đường dẫn tuyệt đối hoặc `..`), đồng thời vẫn tương thích với file XML độc lập chứa trực tiếp `<body name="robot">` (phục vụ kiểm thử đơn vị trong `tmp_path`).
3. **Kiểm tra khung bao 3D tương đối theo thân xe:** Hàm `_check_robot_compiled_geom_extents` tính độ lệch của từng geom trên cây con `robot` so với tâm thân `robot` (`mj_data.xpos[robot_body_id]`), đảm bảo chính xác ngay cả khi `robot` đặt tại `robot_spawn` khác `(0, 0, 0)`.
4. **Hộp đen Telemetry không nhầm bước nhảy khi Reset sang điểm xuất phát xa:** `robot_sim/telemetry.py` đã lọc bỏ bước nhảy `> 0.45m` khi `reset`; cần đảm bảo `near_collision_count` trong `telemetry.py` chỉ xét va chạm quá gần (`< 0.12m`) hoặc cho phép đọc tất cả các tia trong `sensors.rangefinders_m` một cách an toàn mà không phạt oan khi xe đi dọc hành lang cách tường `0.20m–0.35m` (`0.12m` hiện tại hoàn toàn an toàn vì hành lang hẹp nhất rộng `0.76m`, tâm xe cách tường `~0.38m`, mép xe cách tường `~0.25m`).

---

## 4. Chi tiết Thiết kế Đã Chốt (Agreed Design Decisions)

### 4.1. Cấu trúc Thư mục `models/` & Danh bạ Địa hình (`robot_sim/arenas.py`)
*(Quyết định của người dùng: nhiều file arena XML riêng biệt, lấy đủ cả 3 bố cục A, B, C và thử nghiệm A trước. Quyết định kỹ thuật của người thiết kế: tách `models/robot_2wheel.xml` dùng chung và tạo danh bạ `robot_sim/arenas.py`.)*

| Tên ngắn (`arena_id`) | Bí danh hỗ trợ | File mô hình XML | Kích thước lưới & Hành lang | Xuất phát (`robot_spawn`) | Đích mặc định (`target`) | Đặc điểm thử thách |
|---|---|---|---|---|---|---|
| `2wheel` *(mặc định)* | `default`, `basic` | `models/arena_2wheel.xml` | Sân mở `4m × 4m`, 3 vật cản màu | `(0.0, 0.0)`, `yaw=0°` | `(1.4, 0.0)` | Sa bàn cơ bản né 3 khối màu |
| `maze_a` | `maze` | `models/arena_maze_a.xml` | Lưới `4×4`, hành lang `1.0m` (`~0.96m` thông thủy) | `(-1.5, -1.5)`, `yaw=0°` | `(1.5, 1.5)` | **Ưu tiên thử nghiệm đầu tiên:** 2 nhánh ngõ cụt dài, lộ trình chữ S xuyên chéo sân |
| `maze_b` | — | `models/arena_maze_b.xml` | Lưới `5×5`, hành lang `0.8m` (`~0.76m` thông thủy) | `(0.0, 0.0)`, `yaw=0°` | `(1.6, 1.6)` | 3 nhánh ngõ cụt (có ngõ cụt sát vách dưới đích), 9 khúc cua vuông góc |
| `maze_c` | — | `models/arena_maze_c.xml` | Lưới `5×5`, hành lang `0.8m` (`~0.76m` thông thủy) | `(0.0, 0.0)`, `yaw=-90°` *(nhìn thẳng ra cửa mở phía Nam của phòng tâm)* | `(1.6, 1.6)` | Đảo tường chữ U tách rời ở tâm tạo vòng lặp kín + 3 nhánh cụt vòng ngoài |

### 4.2. Tọa độ Hình học Chính xác của 3 Bản đồ Mê cung (MuJoCo SI: mét)
- **Quy ước chung cho cả 3 mê cung:**
  - Mặt sàn `floor`: `type="plane" size="2.0 2.0 0.05"` (`x, y ∈ [-2.0, +2.0]`), họa tiết bàn cờ `grid_mat`, `friction="0.9 0.005 0.0001"`.
  - Tường bao 4 phía (`wall_north`, `wall_south`, `wall_east`, `wall_west`): cao `0.28m` (`pos z = 0.14`, `size z = 0.14`), `rgba="0.35 0.38 0.42 1"`.
    - `wall_north`: `pos="0 2.025 0.14" size="2.05 0.025 0.14"`
    - `wall_south`: `pos="0 -2.025 0.14" size="2.05 0.025 0.14"`
    - `wall_east`: `pos="2.025 0 0.14" size="0.025 2.00 0.14"`
    - `wall_west`: `pos="-2.025 0 0.14" size="0.025 2.00 0.14"`
  - Tường vách ngăn mê cung bên trong (`maze_w*`): `type="box"`, dày `0.04m` (`half-thickness = 0.02m`, cộng bù `0.02m` ở hai đầu mút nối góc để kín khít hoàn toàn với tia rangefinder), cao `0.28m` (`pos z = 0.14`, `size z = 0.14`), màu xám đá trung tính `rgba="0.48 0.52 0.57 1"` (trong `maze_c`, 3 đoạn đảo trung tâm dùng xám tro nhạt `rgba="0.58 0.60 0.64 1"` — cả hai đều có `|R-G| <= 9`, `|B-R| <= 23`, tuyệt đối không kích hoạt nhầm mặt nạ đỏ/xanh/vàng/lá của `analyze_vision`).
  - Camera toàn cảnh `overview_cam`: đặt cao hơn (`pos="0 -3.3 4.2" xyaxes="1 0 0 0 0.78 0.62" fovy="52"`) để người dùng chụp ảnh `camera --name overview_cam` nhìn rõ toàn bộ lòng mê cung từ trên cao.
  - Cột đích `target`: `<body name="target" mocap="true" pos="..."><geom name="target_geom" type="cylinder" size="0.10 0.12" rgba="0.10 0.90 0.20 1" contype="1" conaffinity="1"/></body>`.

- **Chi tiết vách ngăn `models/arena_maze_a.xml` (7 vách hộp gộp từ lưới 4×4, ô `1.0m`):**
  - `robot_spawn`: `pos="-1.5 -1.5 0" euler="0 0 0"`
  - `target`: `pos="1.5 1.5 0.12"`
  - Ngang (`y = const`):
    - `maze_w01`: `pos="-0.5 1.0 0.14" size="0.52 0.02 0.14"` (`x ∈ [-1.0, 0.0], y = 1.0`)
    - `maze_w02`: `pos="1.5 1.0 0.14" size="0.52 0.02 0.14"` (`x ∈ [1.0, 2.0], y = 1.0` — chắn kín phía Nam ô đích)
    - `maze_w03`: `pos="0.5 0.0 0.14" size="0.52 0.02 0.14"` (`x ∈ [0.0, 1.0], y = 0.0`)
  - Dọc (`x = const`):
    - `maze_w04`: `pos="-1.0 0.0 0.14" size="0.02 1.02 0.14"` (`x = -1.0, y ∈ [-1.0, 1.0]`)
    - `maze_w05`: `pos="0.0 1.5 0.14" size="0.02 0.52 0.14"` (`x = 0.0, y ∈ [1.0, 2.0]`)
    - `maze_w06`: `pos="0.0 -1.0 0.14" size="0.02 1.02 0.14"` (`x = 0.0, y ∈ [-2.0, 0.0]`)
    - `maze_w07`: `pos="1.0 -0.5 0.14" size="0.02 0.52 0.14"` (`x = 1.0, y ∈ [-1.0, 0.0]`)

- **Chi tiết vách ngăn `models/arena_maze_b.xml` (12 vách hộp gộp từ lưới 5×5, ô `0.8m`):**
  - `robot_spawn`: `pos="0.0 0.0 0" euler="0 0 0"`
  - `target`: `pos="1.6 1.6 0.12"`
  - Ngang:
    - `maze_w01`: `pos="-0.8 1.2 0.14" size="0.42 0.02 0.14"`
    - `maze_w02`: `pos="1.6 1.2 0.14" size="0.42 0.02 0.14"`
    - `maze_w03`: `pos="-1.2 0.4 0.14" size="0.82 0.02 0.14"`
    - `maze_w04`: `pos="0.0 -0.4 0.14" size="0.42 0.02 0.14"`
    - `maze_w05`: `pos="1.6 -0.4 0.14" size="0.42 0.02 0.14"`
    - `maze_w06`: `pos="-0.8 -1.2 0.14" size="0.42 0.02 0.14"`
    - `maze_w07`: `pos="0.8 -1.2 0.14" size="0.42 0.02 0.14"`
  - Dọc:
    - `maze_w08`: `pos="-1.2 -0.8 0.14" size="0.02 0.42 0.14"`
    - `maze_w09`: `pos="-0.4 0.8 0.14" size="0.02 0.42 0.14"`
    - `maze_w10`: `pos="-0.4 -1.6 0.14" size="0.02 0.42 0.14"`
    - `maze_w11`: `pos="0.4 0.4 0.14" size="0.02 1.62 0.14"`
    - `maze_w12`: `pos="1.2 0.8 0.14" size="0.02 0.42 0.14"`

- **Chi tiết vách ngăn `models/arena_maze_c.xml` (11 vách hộp gộp từ lưới 5×5, ô `0.8m`):**
  - `robot_spawn`: `pos="0.0 0.0 0" euler="0 0 -90"`
  - `target`: `pos="1.6 1.6 0.12"`
  - Ngang:
    - `maze_w01`: `pos="-0.4 1.2 0.14" size="0.82 0.02 0.14"`
    - `maze_w02`: `pos="1.6 1.2 0.14" size="0.42 0.02 0.14"`
    - `maze_w03`: `pos="0.0 0.4 0.14" size="0.42 0.02 0.14"` *(đảo chữ U trung tâm — cạnh Bắc)*
    - `maze_w04`: `pos="-0.4 -1.2 0.14" size="0.82 0.02 0.14"`
  - Dọc:
    - `maze_w05`: `pos="-1.2 0.8 0.14" size="0.02 0.42 0.14"`
    - `maze_w06`: `pos="-1.2 -0.8 0.14" size="0.02 0.42 0.14"`
    - `maze_w07`: `pos="-0.4 0.0 0.14" size="0.02 0.42 0.14"` *(đảo chữ U trung tâm — cạnh Tây)*
    - `maze_w08`: `pos="0.4 1.6 0.14" size="0.02 0.42 0.14"`
    - `maze_w09`: `pos="0.4 0.0 0.14" size="0.02 0.42 0.14"` *(đảo chữ U trung tâm — cạnh Đông)*
    - `maze_w10`: `pos="0.4 -1.6 0.14" size="0.02 0.42 0.14"`
    - `maze_w11`: `pos="1.2 0.0 0.14" size="0.02 1.22 0.14"`

### 4.3. Giao diện Lõi Mô phỏng, HTTP Server & CLI *(Quyết định kỹ thuật của người thiết kế)*
1. **Giải phân giải tên bản đồ (`resolve_arena_model(arena_or_path)` trong `robot_sim/arenas.py`):**
   - Nhận tên ngắn (`2wheel`, `maze`, `maze_a`, `maze_b`, `maze_c`) hoặc đường dẫn tới một file XML hợp lệ.
   - Hàm `list_arenas()` trả danh sách metadata các bản đồ chuẩn (`id`, `aliases`, `model_path`, `description`, `spawn_xy`, `default_target_xy`).
2. **Đặt tư thế xuất phát (`_apply_robot_spawn` trong `RobotSimulation`):**
   - Đọc `<site name="robot_spawn">` từ `self.model` (nếu không có như file test tối giản thì mặc định `(0.0, 0.0, yaw=0.0)`).
   - Trong `__init__`, `reset()` và `reload()`, sau `mj_resetData`, gán `qpos[0:2] = [spawn_x, spawn_y]` và quaternion `qpos[3:7] = [cos(yaw/2), 0, 0, sin(yaw/2)]`, sau đó gọi `mj_forward`.
3. **Hành vi `preserve_target` thông minh & kiểm tra va chạm mục tiêu khi đổi địa hình (`RobotSimulation.reload`, `validate_target_xy` & `POST /reload`):**
   - Nếu nạp lại **cùng một file địa hình** (`resolved == self.model_path`, ví dụ `robot-engineer` vừa sửa `models/robot_2wheel.xml`): mặc định `preserve_target=True` để giữ nguyên tọa độ đích của bài thi hiện tại.
   - Nếu chuyển sang **file địa hình khác** (`resolved != self.model_path`, ví dụ chuyển từ `2wheel` sang `maze_a`): mặc định đặt `target` về vị trí đích chuẩn của địa hình mới (`_default_target_pos`), trừ khi người gọi truyền tường minh `target_xy` (hoặc `preserve_target=True` rõ ràng).
   - *(Quyết định của người thiết kế để giữ tương thích ngược 100% với `tests/test_cli.py`:)* Hàm `validate_target_xy` giữ giới hạn vùng an toàn `[-1.7, 1.7]m`, đồng thời kiểm tra trên bản đồ đang mở: (a) từ chối đặt hình trụ `target` (`r = 0.10m`) chồng lấn vào bất kỳ vách tường bao `wall_*` hoặc vách ngăn mê cung `maze_w*` nào, và (b) từ chối đặt tâm `(tx, ty)` nằm lọt bên trong khối vật cản tĩnh `obs_*` (nhờ vậy tọa độ kiểm thử cũ `(1.0, -0.8)` sát cạnh `obs_blue` vẫn hợp lệ).
4. **Cập nhật CLI & HTTP:**
   - `sim_server.py`: thêm cờ `--arena` và `--model`; thêm endpoint `GET /arenas` (trả danh bạ các bản đồ hỗ trợ; chỉ kèm `"active_arena"` khi truyền `?debug=1` để giữ kín thông tin bản đồ với `robot-driver`); mở rộng `POST /reload` nhận tùy chọn `"arena"` hoặc `"model"` (chỉ cho phép chọn tên địa hình trong danh bạ `list_arenas()` hoặc file XML nằm trực tiếp trong thư mục `models/` của dự án để đảm bảo an toàn đường dẫn qua HTTP).
   - `robot_ctl.py`:
     - Lệnh mới `robot_ctl.py arenas [--debug]`: liệt kê các địa hình có sẵn (chỉ hiện địa hình đang chạy khi có `--debug`).
     - Mở rộng `robot_ctl.py reload`: thêm `--arena <name>` và `--model <path>`.
   - `GET /status`: thêm `"arena": "<arena_id>"` và `"model": "<filename>"` **chỉ bên trong `"debug"`** để `Main Agent`, `robot-engineer` và `telemetry.py` ghi nhận rõ bản đồ trong báo cáo, trong khi `robot-driver` vẫn mù hoàn toàn về bản đồ.

---

## 5. Phần Để Mở Cho Người Lập Kế Hoạch & Triển Khai (Left to the Builder)

- Cách tổ chức cấu trúc dữ liệu đặc tả địa hình trong `robot_sim/arenas.py` và chia sẻ nó với `robot_sim/integrity.py` sao cho gọn sạch, không lặp lại hằng số tọa độ tường hai nơi.
- Chi tiết thông báo lỗi tiếng Việt khi người dùng truyền sai tên `--arena` hoặc đặt `--target-x/--target-y` đè lên tường mê cung.
- Bổ sung trường `"arena"` vào JSON đầu ra của `robot_sim/telemetry.py` (lấy từ `st["debug"]["arena"]`) để báo cáo hộp đen tự ghi nhớ vòng chạy thuộc địa hình nào.
- Cấu trúc bộ kiểm thử đơn vị (`tests/test_arenas.py`, mở rộng `tests/test_integrity.py`, `tests/test_core.py`, `tests/test_server.py`, `tests/test_cli.py`) bao gồm kiểm chứng bằng tìm kiếm đồ thị / raycast rằng trên cả 3 bản đồ `maze_a`, `maze_b`, `maze_c`:
  1. Từ `robot_spawn` ở góc quay bất kỳ `0..360°`, `analyze_vision("front_cam")["target_visible"]` luôn bằng `False` (đích bị che khuất 100% lúc xuất phát),
  2. Tồn tại hành lang thông suốt đủ rộng (`>= 0.70m`) từ `robot_spawn` tới `target`,
  3. Khi xe đứng ở khúc cua cuối trước ô `target` và hướng về đích, `analyze_vision("front_cam")["target_visible"]` bằng `True`.
