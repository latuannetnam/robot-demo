# Thiết kế: Cơ chế Điều khiển, Giám sát & Tự Cải tiến Robot với Multi-Agent trên Antigravity

- **Ngày:** 2026-11-11
- **Thư mục dự án:** `robot-demo`
- **Tài liệu nền tảng trước đó:** `docs/superpowers/specs/2026-11-11-mujoco-2wheel-robot-design.md`

---

## 1. Mục đích & Lý do (Intent & Why)

Từ kết quả đo kiểm thực nghiệm khi chạy thử một Subagent đơn lẻ điều khiển xe về đích bị che khuất hoàn toàn tại `(-1.4, 1.15)`:
- **Điểm mạnh đã chứng minh:** Server vật lý MuJoCo + `Edge Vision JSON` hoạt động ổn định 100% (`0` lỗi qua 628 mẫu đo, độ trễ trung bình `38.6 ms`, tốn `0` lần đọc ảnh JPEG).
- **Ba vấn đề thực tế lộ ra:**
  1. **Tốc độ phản xạ chậm (`Duty Cycle = 3.5%`):** Tổng thời gian hoàn thành mất ~11 phút 50 giây (20 bước), trong đó xe đứng yên chờ 96.5% thời gian do LLM phải gọi các lệnh rời rạc mức thấp (`turn 60` × 6 lần để quét 360°), mất 15–40 giây suy luận hình học mỗi bước, cộng thêm ~2.5–3.5 giây chi phí khởi tạo `uv run python` trên Windows.
  2. **Tiêu hao Context Window lớn (~44.6k tokens/chuyến):** Mỗi bước điều khiển sinh log JSON cảm biến và chuỗi suy nghĩ dài; nếu chạy trực tiếp hoặc lặp nhiều vòng trên Main Agent sẽ nhanh chóng làm đầy bộ nhớ ngữ cảnh.
  3. **Sai lệch vật lý Sim-to-Real cần góc nhìn kỹ sư độc lập:** Xe bị hụt ~10% góc xoay vòng hở do quán tính/ma sát, và bị **"chúi mũi khi phanh gấp" (~5.5° trong 40–110 ms)** khiến tia laser bắn xuống mặt sàn (`0.78 m`), làm người lái (Driver) hiểu nhầm rằng bán kính bánh xe bị sai lệch `1.46×`. Chỉ khi đối chiếu với dữ liệu giám sát độc lập từ bên ngoài mới thấy đúng bản chất cơ học.

Thiết kế này xây dựng **Tổ đội R&D 3 vai trò (Main Agent – Driver Subagent – Engineer Subagent)** kết hợp **Tầng phản xạ khép kín 20 Hz & Hộp đen Python** nhằm:
1. Rút ngắn thời gian hoàn thành nhiệm vụ từ hàng chục phút xuống ~1–2 phút.
2. Giữ context window của **Main Agent** luôn sạch và gọn (~1–2 KB tóm tắt mỗi vòng) dù chạy nhiều vòng thử nghiệm liên tiếp.
3. Tạo quy trình khép kín để các Agent vừa lái xe đúng kỷ luật Sim-to-Real, vừa tự giám sát, mổ băng và cải tiến cả phần mềm điều khiển lẫn thiết kế cơ khí/cảm biến của robot.

---

## 2. Mục tiêu, Phi mục tiêu & Phản mục tiêu (Goals, Non-goals, Anti-goals)

### Mục tiêu (Goals)

1. **Tầng Phản xạ Khép kín 20 Hz & Ổn định Cảm biến (`robot_sim/`, `robot_ctl.py`, `sim_server.py`):**
   - **Chờ tắt dao động sau lệnh (`Settled Wait`):** Khi gọi lệnh kèm `--wait`, chỉ trả về `after_status` sau khi lệnh kết thúc (`moving == False`) **và** thân xe đã thực sự đứng yên (`|forward_speed_m_s|` và `|yaw_rate_rad_s|` giảm xuống dưới ngưỡng tĩnh trong một khoảng thời gian ngắn, có timeout bảo vệ) để không đọc cảm biến giữa đỉnh pha chúi mũi.
   - **Các kỹ năng điều khiển cấp trung (Mid-level Skills) chuẩn Sim-to-Real** (chỉ dùng cảm biến trên xe `chassis_gyro`, `rangefinders_m`, `Edge Vision`, không chạm vào `debug`):
     - Xoay khép kín theo con quay hồi chuyển (`chassis_gyro`) để đạt sát góc yêu cầu thay vì cắt theo thời gian vòng hở thuần túy.
     - `scan`: Tự xoay quét (theo chiều và góc tối đa chỉ định) ở tần số cao và phanh êm ngay khi phát hiện mục tiêu xanh (`target_visible == true`) ở vùng nhìn rõ.
     - `approach`: Tự căn tâm mục tiêu xanh (`offset_x`) kết hợp né chướng ngại vật gần qua 3 tia `rangefinder` cho tới khi cách mục tiêu một cự ly dừng an toàn (`stop_distance`, mặc định `0.25 m`).
   - **Nạp nóng mô hình (`POST /reload` & `robot_ctl.py reload`):** Cho phép nạp lại `models/arena_2wheel.xml` ngay trên tiến trình `sim_server.py` đang chạy (giữ nguyên hoặc cho phép truyền lại `target_xy` của bài thi) để các tinh chỉnh cơ khí/cảm biến của Engineer Subagent có hiệu lực tức thì. *(Lưu ý: Người dùng chỉ cần khởi động lại `sim_server.py` một lần sau khi hoàn tất bản nâng cấp này để server có endpoint `/reload`).*

2. **Hộp đen Giám sát Chạy nền tốn 0 Token LLM (`telemetry_recorder.py` hoặc lệnh con trong `robot_ctl.py`):**
   - Tiến trình Python độc lập lấy mẫu `/status?vision=1` (bao gồm cả `debug` ở phía server) với tần số ~10 Hz trong suốt chuyến chạy của Driver.
   - Khi kết thúc lượt chạy (qua cờ dừng file hoặc thời lượng tối đa), tự động xuất ra một bản tóm tắt JSON ~1–2 KB (ví dụ `captures/telemetry_latest.json`) gồm:
     - `wall_time_s`, `sim_time_s`, `duty_cycle_pct` (tỷ lệ thời gian bánh xe thực sự hoạt động).
     - Quỹ đạo tóm tắt (điểm đầu, điểm cuối, tổng quãng đường thực `path_length_m`, sai số vị trí/góc cuối cùng đối với mục tiêu).
     - Chỉ số an toàn & bất thường vật lý: khoảng cách cảm biến nhỏ nhất `min_rangefinder_m`, số lần cảnh báo sát vật cản (`< 0.15 m`), biên độ sụt giảm đột ngột của `rf_center` ngay lúc phanh (dấu hiệu chúi mũi bắn trúng sàn), hệ số đạt góc xoay thực tế và độ lệch ngang cuối cùng.

3. **Hàng rào Kiểm định Chống Gian lận (`Arena Integrity & Sim-to-Real Guard`):**
   - Có bài kiểm thử tự động trong `tests/` kiểm tra tính toàn vẹn của đề bài và kỷ luật Sim-to-Real:
     - **Cho phép Engineer chỉnh sửa:** Nội dung bên trong `<body name="robot">`, `<actuator>`, và `<sensor>` gắn trên xe trong `models/arena_2wheel.xml` (ví dụ: thêm đệm/bánh cầu phụ chống chúi mũi, chỉnh vị trí/góc cảm biến hợp lý trên khung xe, tinh chỉnh `kv`, `forcerange`, `damping`, ma sát bánh xe), cùng mã nguồn điều khiển trong `robot_sim/` và `robot_ctl.py`.
     - **Cấm tuyệt đối:** Sửa kích thước mặt sàn/tường (`floor`, `wall_*`), thay đổi vị trí/kích thước/màu sắc 3 vật cản (`obs_red`, `obs_blue`, `obs_yellow`), phóng to hoặc đổi tính chất `target`, ghi đè trực tiếp `qpos` để dịch chuyển xe, hoặc để lộ trường `debug` ra đầu ra mặc định của các lệnh Driver sử dụng.

4. **Cơ chế Tổ đội 3 Vai trò & Skill Dự án (`.agents/skills/robot-multi-agent-rd/SKILL.md` & `AGENTS.md`):**
   - Lưu quy trình điều phối vào `.agents/skills/robot-multi-agent-rd/SKILL.md` (kèm hướng dẫn ngắn trong `AGENTS.md` ở gốc dự án) để bất kỳ phiên Antigravity mới nào cũng tự động nhận diện và vận hành đúng 3 vai trò:
     1. **`robot-driver` (Driver Subagent — Người lái mù Sim-to-Real):**
        - Khởi tạo bằng `define_subagent` / `invoke_subagent` (chỉ cấp quyền chạy lệnh điều khiển qua `.venv\Scripts\python.exe robot_ctl.py`, không sửa file).
        - Chỉ được dùng cảm biến gắn trên xe (`status --vision`, `vision`, `scan`, `approach`, `move`, `turn`, `wheels`, `stop`, và `camera` khi thật sự cần). Cấm `--debug`, `reset`, `reload`, `demo`, cấm đọc mã nguồn hay file Hộp đen.
        - Khi hoàn thành (hoặc hết giới hạn bước), gửi về Main Agent bản báo cáo ngắn gọn (< 250 từ): chuỗi lệnh đã gọi, trạng thái cảm biến cuối cùng và cảm nhận điều khiển.
     2. **`robot-engineer` (Engineer Subagent — Kỹ sư Mổ băng & Tự Cải tiến):**
        - Chỉ được Main Agent triệu tập khi chuyến chạy thất bại hoặc chưa đạt ngưỡng hiệu năng/độ mượt mong muốn.
        - Đọc `captures/telemetry_latest.json` + báo cáo của Driver + mã nguồn dự án (`models/arena_2wheel.xml`, `robot_sim/`, `robot_ctl.py`, `tests/`).
        - Tự chẩn đoán nguyên nhân gốc rễ (đối chiếu *"Driver nghĩ gì"* với *"Hộp đen ghi nhận gì"*), trực tiếp sửa code hoặc tinh chỉnh cơ khí thân xe trong `models/arena_2wheel.xml`, bổ sung/chạy `.venv\Scripts\python.exe -m pytest` cho tới khi xanh 100%, gọi `reload` nếu có sửa mô hình/server, và trả về tóm tắt thay đổi ngắn gọn (< 300 từ).
     3. **Main Agent (Tổng công trình sư / Điều phối viên):**
        - Giữ context window siêu nhẹ: không đọc `transcript.jsonl` dài của các Subagent khi không có sự cố đặc biệt; chỉ làm việc trên bảng điểm ~1 KB của từng vòng.
        - Điều phối chu trình tối đa **3 vòng R&D** cho một bài toán (`Vòng i`: Đặt bài toán `reset` -> Bật Hộp đen nền -> Chạy `robot-driver` -> Dừng Hộp đen & chấm điểm -> Nếu đạt chuẩn thì báo cáo kết thúc; nếu chưa đạt thì gọi `robot-engineer` cải tiến + test -> Sang `Vòng i+1` chạy kiểm chứng lại).
        - Nếu bản sửa của Engineer làm `pytest` thất bại không khắc phục được hoặc làm kết quả vòng sau tệ đi rõ rệt (ví dụ từ về đích thành đâm vật cản/thất bại), hoàn tác thay đổi không hiệu quả đó trước khi thử hướng khác hoặc kết thúc báo cáo.

5. **Tài liệu Hướng dẫn (`README.md`):**
   - Bổ sung mục giải thích kiến trúc Multi-Agent R&D, các lệnh kỹ năng mới (`scan`, `approach`, `reload`, Hộp đen), và cách người dùng giao bài kiểm tra/tối ưu cho Antigravity.

### Phi mục tiêu (Non-goals)
- Không chia nhỏ từng chu kỳ điều khiển mili-giây cho nhiều Subagent nhắn tin qua lại với nhau trong lúc xe đang lăn bánh (vì độ trễ LLM liên-agent hàng giây sẽ phá hỏng điều khiển thời gian thực).
- Không triển khai huấn luyện Học tăng cường (RL) hay bản đồ SLAM 2D/3D phức tạp.
- Không tự ý tắt/bật cửa sổ PowerShell bên ngoài của người dùng.

### Phản mục tiêu (Anti-goals — coi là thất bại dù bài chạy vượt qua)
- **Gian lận đề bài hoặc gian lận cảm biến:** Sửa vị trí/kích thước vật cản và đích trong `arena_2wheel.xml`, dịch chuyển tức thời `qpos`, hoặc truyền tọa độ thực `debug` cho `robot-driver` hay cho các hàm `scan` / `approach`.
- **Phình to Context Window của Main Agent:** Để Main Agent tự chạy hàng chục bước điều khiển rời rạc hoặc nhồi toàn bộ chuỗi mẫu thô của Hộp đen vào cuộc hội thoại chính.
- **Tự sửa code không có lưới an toàn:** Cho phép Engineer chỉnh sửa code/XML mà không chạy bộ kiểm thử `pytest` (bao gồm kiểm tra toàn vẹn sa bàn) trước khi đưa vào chạy vòng tiếp theo.

---

## 3. Ràng buộc Kỹ thuật (Constraints)

- **Môi trường thực thi:** Windows + PowerShell (`pwsh`), Python 3.12 trong `.venv` có sẵn của dự án (`mujoco`, `numpy`, `pillow`, `pytest`, không cài thêm gói ngoài mới).
- **Tối ưu tốc độ gọi CLI trên Windows:** Mọi lệnh gọi lặp lại từ Subagent hoặc script đều ưu tiên gọi trực tiếp `.venv\Scripts\python.exe` thay vì `uv run python` (giảm thời gian khởi động từ ~2.5 giây xuống ~0.3 giây mỗi lệnh), trong khi vẫn tương thích hoàn toàn khi người dùng gõ `uv run python ...` thủ công.
- **Cách ly Desktop trên Windows (`exebox`):** Cửa sổ 3D `sim_server.py` chạy trên màn hình chính của người dùng, còn Antigravity chạy lệnh trong môi trường cách ly và giao tiếp hoàn toàn qua HTTP cục bộ `http://127.0.0.1:8765`.
- **An toàn luồng OpenGL & MuJoCo:** Mọi thao tác đọc/ghi `MjModel`, `MjData`, `Renderer` (bao gồm cả `reload` và các bước điều khiển khép kín nếu thực hiện phía server) phải đi qua hàng đợi lệnh trên luồng chính của `SimulationApp` hoặc ghép bằng các lệnh HTTP ngắn an toàn luồng từ phía client.

---

## 4. Các Quyết định Thiết kế

### Do người dùng quyết định
- **Mô hình Multi-Agent:** Chọn kiến trúc **Tổ đội R&D Tự Cải tiến 3 vai trò** (**Main Agent** điều phối + **Driver Subagent** lái mù Sim-to-Real + **Engineer Subagent** chẩn đoán & cải tiến, kết hợp nền tảng phản xạ Python 20 Hz và Hộp đen chạy nền).
- **Quyền tự quyết của Engineer Subagent:** Được quyền **tự động sửa code và tự chạy test** ngay trong vòng lặp cải tiến mà không cần dừng lại xin phép từng dòng sửa.
- **Phạm vi được phép cải tiến:** Ngoài phần mềm điều khiển Python, **Engineer Subagent** được phép tinh chỉnh cả **thiết kế cơ khí và cảm biến trên thân xe** trong `models/arena_2wheel.xml`.
- **Hàng rào an toàn & giới hạn vòng lặp (đã đồng thuận):** Giữ nguyên vẹn sa bàn/vật cản/đích (chống ăn gian), hỗ trợ nạp lại mô hình `reload`, giới hạn tối đa **3 vòng R&D** mỗi bài toán và hoàn tác nếu bản sửa làm hỏng test hoặc làm kết quả thụt lùi.

### Do người xây dựng chủ động chọn (Builder's calls)
- **Đóng gói quy trình thành Workspace Skill chuẩn Antigravity:** Đặt tại `.agents/skills/robot-multi-agent-rd/SKILL.md` kèm tệp `AGENTS.md` ngắn ở thư mục gốc để Antigravity khám phá theo cơ chế *Progressive Disclosure* (chỉ nạp tên & mô tả ngắn khi bình thường, chỉ bung đầy đủ hướng dẫn khi người dùng yêu cầu chạy/tối ưu robot — tiết kiệm tối đa context window).
- **Cơ chế dừng Hộp đen bằng File Tín hiệu + Thời gian trần:** Hộp đen chạy nền qua `run_command` (background) và tự động kết thúc sạch để ghi file JSON ngay khi phát hiện file cờ dừng (ví dụ `captures/.telemetry_stop`) hoặc chạm `--max-duration` (mặc định `300s`), giúp Main Agent thu hồi báo cáo tin cậy mà không cần giết tiến trình thô bạo làm mất dữ liệu.
- **Kiến trúc thực thi các lệnh kỹ năng cấp trung (`scan`, `approach`, xoay theo Gyro):** Chạy vòng lặp điều khiển 20 Hz ở phía client (`robot_ctl.py` / module phụ trợ trong `robot_sim/`) giao tiếp qua HTTP cục bộ (`~5–40 ms`/nhịp) với `sim_server.py`. Cách này có ưu điểm lớn cho vòng lặp R&D: khi Engineer tinh chỉnh thuật toán `scan`/`approach`/PID trong Python, lần gọi CLI tiếp theo áp dụng code mới **ngay lập tức** mà không cần khởi động lại tiến trình `sim_server.py` của người dùng; riêng thay đổi cơ khí `arena_2wheel.xml` sẽ dùng `POST /reload`.

---

## 5. Những phần để ngỏ cho bước Lập kế hoạch & Triển khai (Left to the Builder)

- Chi tiết công thức điều khiển tỷ lệ/giảm tốc cho `approach`, `scan`, và lệnh `turn` hồi tiếp `chassis_gyro` (sao cho xe xoay đúng góc mục tiêu với sai số nhỏ, tự động giảm tốc khi gần tới góc đích và né vật cản mượt).
- Ngưỡng vận tốc tĩnh và cửa sổ thời gian cụ thể trong `_wait_for_completion` (ví dụ `|v| < 0.015 m/s`, `|ω| < 0.03 rad/s` liên tiếp 2–3 mẫu thăm dò) cùng giải pháp cơ khí/điều khiển khởi điểm cho hiện tượng chúi mũi (hoặc để dành một phần tinh chỉnh thực tế cho chính Engineer Subagent thể hiện trong buổi chạy kiểm chứng).
- Cách cài đặt chi tiết `POST /reload` trong `SimulationApp` sao cho vừa hoạt động mượt ở chế độ `--headless` (phục vụ `pytest`), vừa nạp lại `MjModel`/`MjData` an toàn trên cửa sổ `mujoco.viewer` đang mở (giữ lại tọa độ `target_xy` hiện hành của bài thi trừ khi người gọi yêu cầu đặt lại).
- Cấu trúc chi tiết các trường trong `captures/telemetry_latest.json` và tiêu chí chấm điểm Đạt / Cần tối ưu thêm giữa các vòng R&D (ví dụ: về đích đúng vùng `[0.15m, 0.35m]`, `0` va chạm, hoàn thành dưới `90s` thời gian thực và `duty_cycle_pct` cải thiện rõ rệt so với mức nền `3.5%`).
