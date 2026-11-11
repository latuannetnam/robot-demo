# Quy tắc Dự án Robot 2 Bánh MuJoCo (`robot-demo`)

- **Tốc độ CLI trên Windows:** Khi gọi lệnh tự động từ Agent hoặc kiểm thử, luôn dùng trực tiếp `.venv\Scripts\python.exe` (ví dụ `.venv\Scripts\python.exe robot_ctl.py ...` và `.venv\Scripts\python.exe -m pytest -v`) thay vì `uv run python` để tiết kiệm ~2 giây khởi tạo mỗi lệnh.
- **Kỷ luật Sim-to-Real:** Agent đóng vai người lái (`robot-driver`) hoặc các hàm kỹ năng trong `robot_sim/skills.py` **tuyệt đối không** dùng cờ `--debug`, không đọc tọa độ thực `robot_xy`/`target_xy` và không dịch chuyển trực tiếp `qpos`.
- **Toàn vẹn Sa bàn:** Không bao giờ sửa kích thước sàn, tường, 3 vật cản (`obs_red`, `obs_blue`, `obs_yellow`) hoặc mục tiêu (`target`) trong `models/arena_2wheel.xml` (được kiểm tra tự động bởi `robot_sim/integrity.py`).
- **Cửa sổ 3D trực tiếp:** `sim_server.py` chạy trên cửa sổ PowerShell riêng của người dùng và giao tiếp qua `http://127.0.0.1:8765`. Khi sửa `models/arena_2wheel.xml`, dùng `.venv\Scripts\python.exe robot_ctl.py reload` để nạp nóng mô hình.
- **Điều khiển & Tối ưu Multi-Agent:** Khi người dùng yêu cầu chạy điều khiển đa tác tử, giám sát hiệu năng hoặc tự động cải tiến robot, hãy kích hoạt skill `.agents/skills/robot-multi-agent-rd/SKILL.md`.
