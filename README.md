# Robot 2 Bánh Tự Hành với MuJoCo & Antigravity

Dự án nhập môn thực hành mô phỏng vật lý **MuJoCo** dành cho người mới bắt đầu từ con số 0, hướng tới hai mục tiêu:
1. **Hiểu bản chất mô phỏng vật lý robot:** Thấy rõ khung thân, khớp bánh xe, động cơ điều tốc, ma sát, quán tính, va chạm vật cản, cảm biến đo khoảng cách và camera góc nhìn thứ nhất hoạt động cùng nhau như thế nào.
2. **Thử nghiệm dùng AI Agent (Antigravity) lái robot trực tiếp:** Mở cửa sổ 3D trên màn hình và giao nhiệm vụ bằng lời để Antigravity tự chụp ảnh qua mắt camera của xe, đọc cảm biến khoảng cách rồi phát lệnh quay bánh đưa xe về đích — không "ăn gian" dịch chuyển tọa độ.

---

## 1. Cấu trúc Dự án

- `models/robot_2wheel.xml` — Phần cứng dùng chung của xe robot 2 bánh vi sai (khung xe, 2 bánh chủ động, cặp bánh cầu đỡ trước/sau, camera mũi xe `front_cam`, các cảm biến khoảng cách `<rangefinder>` và IMU), được nhúng chung vào mọi sa bàn.
- `models/arena_2wheel.xml`, `models/arena_maze_a.xml`, `models/arena_maze_b.xml`, `models/arena_maze_c.xml`, `models/arena_dynamic_a.xml`, `models/arena_dynamic_b.xml` — 6 bản đồ địa hình chuẩn (sân mở 3 vật cản màu `2wheel`, 3 mê cung nhiều tầng độ khó `maze_a`/`maze_b`/`maze_c`, và 2 bài toán vật cản di động `dynamic_a` 3 làn tuần tra cắt ngang & `dynamic_b` hành lang chữ S với 3 cổng piston đóng/mở theo chu kỳ).
- `robot_sim/arenas.py` — Danh bạ thông số địa hình chuẩn, giải phân giải bí danh (`2wheel`/`default`/`basic`, `maze_a`/`maze`, `maze_b`, `maze_c`, `dynamic_a`/`dynamic`, `dynamic_b`) và tra cứu bản đồ.
- `robot_sim/core.py` — Lõi điều khiển vật lý, điều khiển khớp trượt vật cản di động 500 Hz, tích phân IMU Gyro + Odometry 2D tương đối 500 Hz (`odom_x_m`, `odom_y_m`), tự động dò cảm biến khoảng cách động, nạp nóng/chuyển đổi bản đồ nguyên tử (`reload`) và phân tích `Edge Vision` trong RAM.
- `robot_sim/skills.py` — Các kỹ năng phản xạ khép kín 20 Hz chuẩn Sim-to-Real (`scan`, `approach`, `explore` cho Mode A; `advance`, `cross-gate`, `dock` cho Mode B; và lệnh gộp System 1 `go_to_decision` cho Mode C `hybrid-brain`).
- `robot_sim/topo.py` — Bộ định vị đồ thị Odometry hai chiều khách quan `TopologicalOdometer` (`out_visits`/`in_visits`, `pending_leg`, `loop_closed`) cho Mode C `hybrid-brain`.
- `robot_sim/telemetry.py` — Hộp đen giám sát nền 10 Hz (0 token LLM) xuất báo cáo tóm tắt JSON `< 2 KB` (ghi nhận va chạm `contact_count` và cận va chạm với tường bao, vách mê cung lẫn vật cản tĩnh/di động).
- `robot_sim/integrity.py` — Hàng rào chống gian lận khóa cứng tất cả bản đồ `models/arena_*.xml` và kiểm tra biên dạng cơ khí Sim-to-Real của `models/robot_2wheel.xml`.
- `sim_server.py` — Chương trình mở **cửa sổ 3D MuJoCo trực tiếp** kèm máy chủ HTTP cục bộ (`127.0.0.1:8765`, hỗ trợ `--arena`).
- `robot_ctl.py` — Công cụ dòng lệnh cho người dùng và Antigravity điều khiển, liệt kê/đổi địa hình (`arenas`, `reload --arena`), quét mục tiêu và thu thập hộp đen.
- `.agents/skills/robot-multi-agent-rd/` & `AGENTS.md` — Quy trình Tổ đội Multi-Agent R&D 3 vai trò tích hợp sẵn cho Antigravity.

---

## 2. Khởi chạy Nhanh

### Bước 1: Mở cửa sổ 3D MuJoCo (Desktop Viewer hoặc MuJoCo Studio)

- **Cách A — Desktop Viewer (`sim_server.py`):** Mở một cửa sổ PowerShell tại thư mục dự án và chạy:

```powershell
uv run python sim_server.py
# Hoặc mở thẳng vào một bản đồ mê cung:
# uv run python sim_server.py --arena maze_a
```

- **Cách B — MuJoCo Studio Web Sidecar (`studio/`):** Stream 3D trực tiếp vào Antigravity aux pane (`http://127.0.0.1:8080`) kèm bảng điều khiển ImGui + Topo HUD và tự mở cùng cầu HTTP `http://127.0.0.1:8765` (không chạy song song với `sim_server.py`). Xem hướng dẫn đầy đủ tại [`docs/mujoco-studio-runbook.md`](docs/mujoco-studio-runbook.md).

Cửa sổ 3D MuJoCo sẽ hiện lên:
- **Chuột trái + kéo:** Xoay góc nhìn camera quan sát.
- **Cuộn chuột:** Phóng to / thu nhỏ.
- **Chuột phải + kéo:** Di chuyển ngang khung nhìn.
- **Nhấp đúp vào trụ xanh lá + giữ `Ctrl` + kéo chuột phải:** Kéo mục tiêu sang vị trí mới ngay khi xe đang chạy!

### Bước 2: Tự tay điều khiển thử từ cửa sổ PowerShell thứ hai

```powershell
# 1. Xem cảm biến khoảng cách KÈM tóm tắt thị giác tại biên (Edge Vision JSON siêu nhẹ, không tốn ảnh!)
uv run python robot_ctl.py status --vision

# 2. Chỉ đọc riêng bản phân tích thị giác trong RAM (đích xanh, hộp cản màu, hướng lệch ngang)
uv run python robot_ctl.py vision

# 3. Chụp ảnh từ camera mũi xe (mặc định nén JPEG 320x240 chỉ ~3-6 KB tại captures/front_cam.jpg)
uv run python robot_ctl.py camera
# (Tùy chọn xuất ảnh nét PNG 640x480 nếu cần lưu tư liệu: uv run python robot_ctl.py camera --format png --width 640 --height 480)

# 4. Cho xe tiến thẳng 0.5 mét, chờ dừng hẳn rồi báo lại cả cảm biến lẫn Edge Vision
uv run python robot_ctl.py move 0.5 --wait --vision

# 5. Cho xe xoay trái 45 độ (số âm như -45 để xoay phải)
uv run python robot_ctl.py turn 45 --wait --vision

# 6. Điều khiển trực tiếp vận tốc góc 2 bánh trái/phải (rad/s) để đi đường cong
uv run python robot_ctl.py wheels 4.0 7.0 --duration 1.5 --wait

# 7. Chạy thử vòng lặp tự hành tốc độ cao (xử lý Edge Vision trực tiếp trong RAM)
uv run python robot_ctl.py demo --steps 30

# 8. Đặt lại vị trí ban đầu (có thể đổi tọa độ đích nằm trong [-1.7, 1.7])
uv run python robot_ctl.py reset --target-x -1.2 --target-y 1.0 --vision

# 9. Quét vòng tròn 20Hz tìm trụ xanh lá và tự căn giữa khung hình (chuẩn Sim-to-Real)
.venv\Scripts\python.exe robot_ctl.py scan --max-degrees 360 --direction left

# 10. Tiếp cận khép kín 20Hz bám tâm trụ xanh kết hợp né vật cản gần
.venv\Scripts\python.exe robot_ctl.py approach --stop-distance 0.25

# 10b. Các công cụ phản xạ vô trạng thái cho chế độ thử nghiệm Agent-Brain (Mode B)
.venv\Scripts\python.exe robot_ctl.py advance --max-dist 1.8 --speed 0.55
.venv\Scripts\python.exe robot_ctl.py cross-gate --timeout 12.0
.venv\Scripts\python.exe robot_ctl.py dock --stop-distance 0.25

# 10c. Các công cụ System 1 + Đồ thị Odometry khách quan cho chế độ Hybrid-Brain (Mode C — Đơn chặng, Khứ hồi & Hai lượt)
.venv\Scripts\python.exe robot_ctl.py topo show
.venv\Scripts\python.exe robot_ctl.py go --cardinal E --max-dist 6.0 --speed 0.55
.venv\Scripts\python.exe robot_ctl.py go --route W,S,N,N --speed 0.55
.venv\Scripts\python.exe robot_ctl.py go --route W,S,N,E --stop-at-node 0 --speed 0.55
.venv\Scripts\python.exe robot_ctl.py reset --keep-topo --vision
.venv\Scripts\python.exe robot_ctl.py topo reset

# 11. Xem danh sách 6 bản đồ địa hình chuẩn và chuyển đổi nóng ngay trên cửa sổ 3D đang mở!
.venv\Scripts\python.exe robot_ctl.py arenas
.venv\Scripts\python.exe robot_ctl.py reload --arena maze_a --vision
# (Thử thêm các mê cung và vật cản di động: --arena maze_b, --arena maze_c, --arena dynamic_a, --arena dynamic_b, hoặc --arena 2wheel)

# 12. Nạp nóng lại phần cứng models/robot_2wheel.xml sau khi tinh chỉnh thiết kế thân xe (giữ nguyên đích hiện tại)
.venv\Scripts\python.exe robot_ctl.py reload --vision
```

---

## 3. Dùng Antigravity Điều Khiển Robot

Khi `sim_server.py` đang mở cửa sổ 3D trên màn hình, bạn chỉ cần nhắn trực tiếp vào khung chat của **Antigravity**, ví dụ:

> *"Hãy đặt mục tiêu sang tọa độ `(-1.2, 1.1)`, sau đó chỉ dùng camera mũi xe (`front_cam`) và cảm biến khoảng cách trong `robot_ctl.py` để tự tìm và lái xe tới sát trụ màu xanh lá nhé!"*

Antigravity sẽ đóng vai "bộ não" trên robot theo đúng vòng lặp **Cảm nhận → Suy luận → Hành động (Sense → Think → Act)** tối ưu tốc độ:
1. Gọi `robot_ctl.py status --vision` để đọc khoảng cách + **bản tóm tắt thị giác tại biên (Edge Vision JSON < 300 bytes)** phản hồi tức thì mà không làm đầy bộ nhớ ảnh; chỉ khi gặp tình huống phức tạp mới chụp thêm `captures/front_cam.jpg` (`320x240` JPEG siêu nhẹ `~4 KB`).
2. Nhận biết các khối hộp cản, tường hành lang và vị trí trụ đích màu xanh lá trong khung hình.
3. Gửi từng lệnh `turn` / `move` / `wheels` (hoặc kỹ năng `scan` / `approach`) để bạn xem xe xoay sở trực tiếp trên màn hình 3D.

### Chế độ Tổ đội Multi-Agent R&D Tự Cải Tiến (3 Vai Trò & Đa Địa Hình Mê Cung)

> **Lưu ý:** Nếu bạn đang mở phiên `sim_server.py` từ bản cũ, hãy nhấn `Ctrl+C` rồi chạy lại `uv run python sim_server.py` **một lần duy nhất** để kích hoạt cổng đa địa hình `/arenas` và chuyển đổi bản đồ nóng `/reload`.

Dự án tích hợp sẵn Workspace Skill `.agents/skills/robot-multi-agent-rd/SKILL.md` vận hành 3 vai trò độc lập để tối ưu tốc độ và bảo vệ Context Window của Antigravity:
1. **Main Agent (Tổng công trình sư):** Chọn sa bàn thử thách (`reload --arena maze_a` / `maze_b` / `maze_c` / `dynamic_a` / `dynamic_b` / `2wheel`), bật **Hộp đen Python 10 Hz** (`telemetry record --max-duration 360` tốn 0 token LLM) và chỉ đọc bảng điểm tóm tắt `~1 KB` (`telemetry stop`) sau mỗi vòng.
2. **`robot-driver` (Subagent Lái mù Sim-to-Real):** Gọi trực tiếp `.venv\Scripts\python.exe robot_ctl.py` với các kỹ năng khép kín 20 Hz, hoàn toàn mù tọa độ và không biết mình đang ở bản đồ nào (tuyệt đối cấm `--debug`, cấm `arenas` và cấm đọc mã nguồn). Dự án hỗ trợ đầy đủ 3 chế độ đối chuẩn: **Mode A (`auto-skill`, mặc định)**, **Mode B (`agent-brain`, `robot-driver-brain`)** và **Mode C (`hybrid-brain`, `robot-driver-hybrid` phân tầng System 1 `go`/`topo` + System 2 AI quyết định mọi ngã giao, ngõ cụt, vòng lặp & cổng động)**.
3. **`robot-engineer` (Subagent Mổ băng & Tinh chỉnh):** Khi chuyến chạy chưa đạt chuẩn, tự đối chiếu Hộp đen với mã nguồn, nâng cấp thuật toán điều hướng tổng quát trong `robot_sim/skills.py` hoặc gắn thêm/tinh chỉnh cảm biến & cơ khí trên `models/robot_2wheel.xml` (trong khi mọi bản đồ `models/arena_*.xml` bị khóa bất biến bởi `robot_sim/integrity.py`), chạy `pytest` và gọi `reload` để kiểm chứng ở vòng tiếp theo (tối đa 3 vòng).

Bạn có thể kích hoạt ngay bằng câu lệnh trong khung chat:
> *"Hãy nạp bản đồ mê cung `maze_a`, dùng cơ chế Multi-Agent R&D cho robot tự dò đường tìm trụ xanh lá, giám sát qua Hộp đen và tự cải tiến cảm biến/kỹ năng nếu cần!"*

---

## 4. Bảng Đối Chiếu Mô Phỏng → Robot Phần Cứng Thật (Sim-to-Real)

Mọi thành phần trong dự án này đều tương ứng 1-1 với linh kiện khi bạn lắp xe robot thật sau này (ví dụ dùng ESP32 hoặc Raspberry Pi):

| Thành phần trong MuJoCo | Linh kiện ngoài đời thật | Vai trò thực tế |
| :--- | :--- | :--- |
| `<body name="robot">` & `<geom name="chassis">` | Khung xe mica / nhôm 2 tầng | Nâng đỡ pin, mạch điều khiển và cảm biến |
| `<joint type="hinge">` (`left/right_wheel_joint`) | Trục bánh xe gắn lốp cao su | Cho phép hai bánh quay độc lập quanh trục ngang |
| `<geom name="caster">` (ma sát thấp) | Bánh mắt trâu (ball caster) | Đỡ đuôi xe giữ thăng bằng mà không cản hướng rẽ |
| `<velocity name="left_motor" ...>` | Động cơ DC giảm tốc + Encoder + Mạch cầu H (L298N / TB6612) | Giữ tốc độ quay bánh xe theo lệnh điều khiển |
| `<rangefinder>` (`rf_left`, `rf_center`, `rf_right`) | Cảm biến siêu âm `HC-SR04` hoặc laser `VL53L0X` (ToF) | Đo khoảng cách tới vật cản phía trước (mét) |
| `<velocimeter>` & `<gyro>` | Cảm biến quán tính IMU (`MPU6050` / `BNO055`) | Đo tốc độ di chuyển và tốc độ xoay thân xe |
| `<camera name="front_cam">` | Camera Raspberry Pi CSI hoặc USB Webcam | Thu hình ảnh góc nhìn thứ nhất phía trước mũi xe |
| `sim_server.py` (HTTP JSON + Watchdog) | Firmware chạy trên ESP32 / Raspberry Pi | Nhận lệnh qua Wi-Fi/UART và tự phanh nếu mất kết nối |
| `robot_ctl.py` / Antigravity | Máy tính trạm hoặc AI Agent điều hành | Đọc cảm biến + ảnh camera và ra quyết định điều hướng |

---

## 5. Chạy Kiểm Thử Tự Động

```powershell
uv run pytest -v
```
