# Robot 2 Bánh Tự Hành với MuJoCo & Antigravity

Dự án nhập môn thực hành mô phỏng vật lý **MuJoCo** dành cho người mới bắt đầu từ con số 0, hướng tới hai mục tiêu:
1. **Hiểu bản chất mô phỏng vật lý robot:** Thấy rõ khung thân, khớp bánh xe, động cơ điều tốc, ma sát, quán tính, va chạm vật cản, cảm biến đo khoảng cách và camera góc nhìn thứ nhất hoạt động cùng nhau như thế nào.
2. **Thử nghiệm dùng AI Agent (Antigravity) lái robot trực tiếp:** Mở cửa sổ 3D trên màn hình và giao nhiệm vụ bằng lời để Antigravity tự chụp ảnh qua mắt camera của xe, đọc cảm biến khoảng cách rồi phát lệnh quay bánh đưa xe về đích — không "ăn gian" dịch chuyển tọa độ.

---

## 1. Cấu trúc Dự án

- `models/arena_2wheel.xml` — Mô hình vật lý MJCF (có chú thích tiếng Việt từng thẻ) của xe 2 bánh vi sai, bánh cầu đỡ tự do, camera mũi xe, 3 cảm biến khoảng cách, tường bao, hộp cản màu và trụ đích màu xanh lá.
- `robot_sim/core.py` — Lõi điều khiển vật lý, chuẩn hóa cảm biến, cơ chế tự ngắt an toàn (watchdog) và kết xuất ảnh PNG từ camera.
- `sim_server.py` — Chương trình mở **cửa sổ 3D MuJoCo trực tiếp** kèm máy chủ HTTP cục bộ (`127.0.0.1:8765`).
- `robot_ctl.py` — Công cụ dòng lệnh để bạn hoặc Antigravity đọc cảm biến, chụp ảnh camera và gửi lệnh lái xe.

---

## 2. Khởi chạy Nhanh

### Bước 1: Mở cửa sổ 3D MuJoCo
Mở một cửa sổ PowerShell tại thư mục dự án và chạy:

```powershell
uv run python sim_server.py
```

Cửa sổ 3D MuJoCo sẽ hiện lên:
- **Chuột trái + kéo:** Xoay góc nhìn camera quan sát.
- **Cuộn chuột:** Phóng to / thu nhỏ.
- **Chuột phải + kéo:** Di chuyển ngang khung nhìn.
- **Nhấp đúp vào trụ xanh lá + giữ `Ctrl` + kéo chuột phải:** Kéo mục tiêu sang vị trí mới ngay khi xe đang chạy!

### Bước 2: Tự tay điều khiển thử từ cửa sổ PowerShell thứ hai

```powershell
# 1. Xem các cảm biến trên xe đang đo được gì (khoảng cách trái/giữa/phải, tốc độ bánh)
uv run python robot_ctl.py status

# 2. Chụp ảnh góc nhìn thứ nhất từ camera gắn ở mũi xe (lưu tại captures/front_cam.png)
uv run python robot_ctl.py camera

# 3. Cho xe tiến thẳng 0.5 mét và chờ chạy xong
uv run python robot_ctl.py move 0.5 --wait

# 4. Cho xe xoay trái 45 độ (số âm như -45 để xoay phải)
uv run python robot_ctl.py turn 45 --wait

# 5. Điều khiển trực tiếp vận tốc góc 2 bánh trái/phải (rad/s) để đi đường cong
uv run python robot_ctl.py wheels 4.0 7.0 --duration 1.5 --wait

# 6. Chạy thử vòng lặp tự hành bằng Python thuần (tự nhìn màu xanh + né hộp cản)
uv run python robot_ctl.py demo --steps 30

# 7. Đặt lại vị trí ban đầu (có thể đổi tọa độ đích nằm trong [-1.7, 1.7])
uv run python robot_ctl.py reset --target-x -1.2 --target-y 1.0
```

---

## 3. Dùng Antigravity Điều Khiển Robot

Khi `sim_server.py` đang mở cửa sổ 3D trên màn hình, bạn chỉ cần nhắn trực tiếp vào khung chat của **Antigravity**, ví dụ:

> *"Hãy đặt mục tiêu sang tọa độ `(-1.2, 1.1)`, sau đó chỉ dùng camera mũi xe (`front_cam`) và cảm biến khoảng cách trong `robot_ctl.py` để tự tìm và lái xe tới sát trụ màu xanh lá nhé!"*

Antigravity sẽ đóng vai "bộ não" trên robot theo đúng vòng lặp **Cảm nhận → Suy luận → Hành động (Sense → Think → Act)**:
1. Gọi `robot_ctl.py status` và `robot_ctl.py camera` rồi dùng khả năng thị giác đọc file `captures/front_cam.png`.
2. Nhận biết các khối hộp cản và vị trí trụ đích màu xanh lá trong khung hình.
3. Gửi từng lệnh `turn` / `move` / `wheels` để bạn xem xe xoay sở trực tiếp trên màn hình 3D.

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
