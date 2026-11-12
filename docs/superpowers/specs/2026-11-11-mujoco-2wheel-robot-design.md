# Thiết kế Dự án: Mô phỏng Xe Robot 2 Bánh trong MuJoCo & Điều khiển qua Antigravity

- **Ngày:** 2026-11-11
- **Thư mục dự án:** `robot-demo`

---

## 1. Mục đích & Lý do (Intent & Why)

Người dùng mới bắt đầu tìm hiểu về robot từ con số 0 và muốn có nền tảng trực quan trước khi bước sang lắp ráp, thử nghiệm robot phần cứng thật trong tương lai.

Dự án này giải quyết hai câu hỏi thực tế ngay từ bước đầu:
1. **MuJoCo làm được gì trong mô phỏng?** Cho thấy rõ cách một hệ thống vật lý mô phỏng khung xe, khớp xoay, động cơ bánh xe, ma sát, quán tính, va chạm với vật cản, cảm biến khoảng cách và camera gắn trên xe.
2. **Agent harness như Antigravity có điều khiển được robot không?** Chứng minh và trải nghiệm trực tiếp việc dùng AI Agent (Antigravity) đọc dữ liệu cảm biến + hình ảnh từ mắt camera của xe, tự suy luận và phát lệnh lái xe về đích theo thời gian thực ngay trước mắt người dùng trên cửa sổ 3D.

---

## 2. Mục tiêu, Phi mục tiêu & Phản mục tiêu (Goals, Non-goals, Anti-goals)

### Mục tiêu (Goals)
1. **Mô hình xe robot 2 bánh & sa bàn 3D dễ đọc (`models/arena_2wheel.xml`):**
   - Xe vi sai 2 bánh (differential drive): thân xe, bánh trái & bánh phải gắn động cơ độc lập, 1 bánh cầu tự do (caster) đỡ phía sau để giữ thăng bằng.
   - Cảm biến sát thực tế phần cứng:
     - 01 camera góc nhìn thứ nhất (`front_cam`) gắn ở mũi xe nhìn về phía trước.
     - 03 cảm biến đo khoảng cách tia (`rf_left`, `rf_center`, `rf_right` — tương đương cảm biến siêu âm / ToF / LiDAR đơn tia trên xe thật).
     - Cảm biến vận tốc khớp bánh xe và odometry cơ bản.
   - Sa bàn 3D có sàn kẻ ô, tường bao quanh, vài khối hộp chướng ngại vật khác màu và 01 mục tiêu (quả cầu/trụ màu nổi bật, ví dụ xanh lá sáng) để xe tìm tới.
   - File XML được chú thích tiếng Việt rõ ràng từng phần (`body`, `joint`, `geom`, `actuator`, `sensor`) để người mới đọc hiểu ngay cấu trúc robot.
2. **Trình chạy mô phỏng trực tiếp kèm cổng điều khiển cục bộ (`sim_server.py`):**
   - Mở cửa sổ 3D tương tác của MuJoCo (`mujoco.viewer.launch_passive`) để người dùng quan sát trực tiếp, xoay góc nhìn, hoặc kéo thả vật thể.
   - Chạy kèm một HTTP JSON server gọn nhẹ tại `127.0.0.1:8765` ngay trong cùng tiến trình để nhận lệnh điều khiển và trả dữ liệu cảm biến / xuất ảnh camera mà không làm giật cửa sổ 3D.
3. **Công cụ dòng lệnh cho Người dùng & Antigravity (`robot_ctl.py`):**
   - Cung cấp các lệnh trực quan để cả người dùng lẫn Antigravity gọi qua terminal:
     - `status`: Đọc khoảng cách 3 tia cảm biến phía trước, vận tốc bánh xe, trạng thái chuyển động (kèm thông tin vị trí thực trong trường `debug` riêng phục vụ kiểm tra/đặt lại).
     - `camera`: Chụp ảnh hiện tại từ camera mũi xe (`front_cam`) hoặc camera toàn cảnh (`overview_cam`) ra file PNG để người dùng hoặc Antigravity mở xem trực tiếp.
     - `wheels`: Đặt tốc độ góc bánh trái / bánh phải trong một khoảng thời gian (hoặc duy trì liên tục).
     - `move` & `turn`: Lệnh tiện ích tiến/lùi một quãng ngắn hoặc xoay một góc dựa trên điều khiển vận tốc hai bánh (giống tầng firmware trên mạch điều khiển xe thật).
     - `stop` & `reset`: Dừng khẩn cấp hoặc đưa sa bàn về vị trí ban đầu (hỗ trợ đặt lại vị trí mục tiêu).
     - `demo`: Chạy thử một kịch bản điều khiển tự động ngắn bằng Python thuần (tự né vật cản / hướng về đích) để người dùng thấy cách viết vòng lặp điều khiển cổ điển bên cạnh cách dùng AI Agent.
4. **Tài liệu hướng dẫn cho người mới (`README.md`):**
   - Giải thích ngắn gọn bằng tiếng Việt cách khởi chạy, cách tự lái thử, cách giao bài toán cho Antigravity lái xe, và bảng đối chiếu từng thành phần trong MuJoCo sang linh kiện phần cứng ngoài đời thật (khung gầm, motor DC + encoder, mạch cầu H, cảm biến khoảng cách, camera CSI/USB, giao thức lệnh).

### Phi mục tiêu (Non-goals — chưa làm ở phiên bản đầu)
- Chưa huấn luyện Học tăng cường (Reinforcement Learning / PPO / Gymnasium).
- Chưa dùng ROS 2 (cài đặt nặng và phức tạp không cần thiết trên Windows cho người mới).
- Chưa làm bản đồ SLAM 2D/3D phức tạp hay cánh tay gắp vật.
- Chưa bắt buộc cài đặt MCP Server toàn cục vào cấu hình Antigravity (vì CLI + HTTP cục bộ kết hợp công cụ chạy lệnh và đọc ảnh sẵn có của Antigravity đã hoạt động mượt ngay lập tức; có thể bọc thêm lớp MCP sau nếu muốn).

### Phản mục tiêu (Anti-goals — điều cần tránh tuyệt đối)
- **"Ăn gian" dịch chuyển tức thời (God-mode teleportation):** Không điều khiển robot bằng cách ghi đè trực tiếp tọa độ `qpos` trong không gian, và khi Antigravity đóng vai bộ não robot thì phải dựa vào camera mũi xe + cảm biến khoảng cách + chuyển động bánh xe chứ không đọc tọa độ đích tuyệt đối để tính đường đi sẵn. Nếu ăn gian, bài học sẽ không áp dụng được lên robot thật.
- **Đóng gói quá phức tạp thành "hộp đen":** Code phải ngắn gọn, sáng sủa, ít phụ thuộc để người mới mở file `.xml` và `.py` ra là thấy ngay vòng lặp vật lý `mj_step` hoạt động thế nào.
- **Chỉ chạy ngầm không có hình ảnh 3D trực tiếp:** Người dùng phải nhìn thấy cửa sổ MuJoCo 3D sống động trên màn hình trong lúc gõ lệnh hoặc trong lúc Antigravity đang lái xe.

---

## 3. Ràng buộc Kỹ thuật (Constraints)

- **Hệ điều hành & Môi trường:** Windows, PowerShell (`pwsh`), Python 3.12 và trình quản lý gói `uv` (đã cài sẵn trên máy).
- **Thư viện phụ thuộc tối thiểu:** Chỉ dùng `mujoco`, `numpy`, và `pillow` (để lưu ảnh PNG từ mảng pixel RGB của MuJoCo); phần HTTP server và CLI dùng thư viện chuẩn của Python (`http.server`, `urllib.request`, `threading`, `queue`, `argparse`, `json`).
- **An toàn luồng OpenGL trên Windows (Quan trọng):** Trên Windows, ngữ cảnh đồ họa OpenGL của `mujoco.Renderer` và `mujoco.viewer` gắn chặt với luồng chính (main thread). Do đó:
  - Luồng chính giữ toàn quyền gọi `mujoco.mj_step`, `viewer.sync()`, và kết xuất ảnh `Renderer.render()` thông qua hàng đợi yêu cầu (`queue.Queue`).
  - Luồng phụ chỉ chạy HTTP server để nhận/trả JSON và đẩy yêu cầu vào hàng đợi cho luồng chính xử lý. Kiến trúc này đảm bảo không bao giờ xảy ra lỗi xung đột OpenGL hay đơ cửa sổ trên Windows.
- **Giới hạn thời gian lệnh chuyển động (Watchdog/Timeout):** Giống xe robot thật, mọi lệnh quay bánh đều có thời gian hiệu lực (ví dụ mặc định tự dừng sau thời gian chỉ định hoặc tối đa vài giây nếu không có lệnh mới) để xe không lao vô tận vào tường khi Agent đang suy nghĩ giữa hai bước.

---

## 4. Các Quyết định Thiết kế

### Do người dùng quyết định
- **Loại robot khởi đầu:** Xe tự hành 2 bánh vi sai (differential drive), ưu tiên sự đơn giản cho người chưa có kinh nghiệm về robot.
- **Định hướng học tập:** Vừa hiểu khả năng mô phỏng vật lý của MuJoCo, vừa thử nghiệm dùng Antigravity điều khiển robot như bước đệm thực tế tiến tới làm việc với phần cứng thật.
- **Kiến trúc tổng thể đã thống nhất:** Xe 2 bánh + bánh đỡ tự do, có cảm biến khoảng cách và camera góc nhìn thứ nhất trong sa bàn có tường/vật cản/mục tiêu; dùng Python + `uv`; mở cửa sổ 3D trực tiếp kèm cầu nối cục bộ để cả người dùng và Antigravity điều khiển.

### Do người xây dựng chủ động chọn (Builder's calls)
- **Kiểu cơ cấu chấp hành (Actuator):** Dùng `<velocity>` servo có giới hạn lực kéo (`forcerange`) cho hai bánh trái/phải thay vì đặt mô-men xoắn trần (`<motor>`), vì trên robot thật ở tầng cơ bản, vi điều khiển thường nhận lệnh tốc độ bánh xe mong muốn và tự giữ tốc độ đó.
- **Cổng giao tiếp cục bộ:** HTTP JSON trên `127.0.0.1:8765` (có thể đổi qua tham số `--port`), lưu ảnh chụp camera mặc định vào thư mục tạm trong dự án (`captures/front.png`, `captures/overview.png`) và trả về đường dẫn tuyệt đối trong JSON để Antigravity dùng công cụ đọc ảnh xem ngay.
- **Kích thước tham khảo của xe & sân:** Sân vuông khoảng `4m x 4m` có tường thấp bao quanh; thân xe cỡ `24cm x 20cm` (tương đương khung xe robot 2 bánh phổ thông ngoài đời), bán kính bánh xe `5cm`, cảm biến khoảng cách quét tối đa `3.0m`.

---

## 5. Những phần để ngỏ cho bước Lập kế hoạch & Triển khai (Left to the Builder)

- Tinh chỉnh hệ số ma sát mặt sàn/bánh xe, khối lượng quán tính, độ lợi `kv` của actuator và giới hạn lực để xe tăng/giảm tốc mượt, không bị trượt bánh bất thường hay lật khi phanh gấp.
- Bố trí cụ thể màu sắc, vị trí các khối hộp cản và điểm đích sao cho vừa đẹp mắt trên cửa sổ 3D vừa dễ nhận diện qua camera góc nhìn thứ nhất (`640x480`).
- Công thức ước lượng thời gian quay bánh cho lệnh tiện ích `move --meters <d>` và `turn --degrees <deg>` dựa trên bán kính bánh xe và khoảng cách hai bánh (kinematics vi sai cơ bản), có ghi chú rõ trong code rằng kết quả thực tế vẫn chịu ảnh hưởng ma sát/trượt bánh đúng bản chất vật lý.
- Kịch bản mẫu trong lệnh `demo` (ví dụ: xoay tìm hướng thoáng bằng cảm biến khoảng cách hoặc phản xạ né chướng ngại vật đơn giản).
