# Runbook: Chạy Mô Phỏng Robot 2 Bánh với MuJoCo Studio

Tài liệu hướng dẫn khởi chạy, vận hành và xử lý sự cố khi chạy dự án `robot-demo` trên **MuJoCo Studio Web Sidecar** (stream 3D trực tiếp vào Antigravity aux pane hoặc trình duyệt tại `http://127.0.0.1:8080`, đồng thời mở cầu HTTP `http://127.0.0.1:8765` tương thích 100% với `robot_ctl.py` và các chế độ Mode A / B / C).

---

## 1. Điều kiện tiên quyết (Prerequisites)

Dự án dùng tách biệt 2 môi trường Python:
1. **Môi trường điều khiển CLI & kiểm thử (`.venv` của dự án):**
   ```powershell
   uv sync
   .\.venv\Scripts\python.exe -m pytest -v
   ```
2. **Môi trường MuJoCo Studio (`~\.venvs\mujoco-studio`):**
   Cần có `mujoco>=4.0.1`, `mujoco_studio_wasm`, và `pillow` (`PIL` dùng cho `studio/sim_plugin.py`). Kiểm tra nhanh:
   ```powershell
   & "$env:USERPROFILE\.venvs\mujoco-studio\Scripts\python.exe" -c "import mujoco, mujoco.studio, mujoco_studio_wasm, PIL; print('OK', mujoco.__version__)"
   ```
   Nếu thiếu `pillow`:
   ```powershell
   uv pip install --python "$env:USERPROFILE\.venvs\mujoco-studio\Scripts\python.exe" pillow
   ```

> **Lưu ý cổng `8765`:** `studio/sim_plugin.py` tự mở cầu HTTP tại `http://127.0.0.1:8765`. Hãy đảm bảo **không chạy đồng thời** `sim_server.py` và MuJoCo Studio cùng lúc.

---

## 2. Khởi chạy MuJoCo Studio Daemon (`studiod`)

### Bước 1: Chạy `studiod.py` trỏ vào thư mục `studio/`
Trong PowerShell (hoặc chạy nền `IsDaemon: true` khi gọi từ Agent):
```powershell
& "$env:USERPROFILE\.venvs\mujoco-studio\Scripts\python.exe" `
  "$env:USERPROFILE\.gemini\config\plugins\robotics\skills\mujoco-studio-sidecar\scripts\studiod.py" `
  --workspace_dir="d:\latuan\Programming\robot-demo\studio"
```
Khi sẵn sàng, log sẽ in:
```text
[studiod] Ready: WebViewer=http://127.0.0.1:8080 Control=http://127.0.0.1:<control_port> Workspace=D:\latuan\Programming\robot-demo\studio ...
```

### Bước 2: Xác minh trạng thái plugin (`/_studio/status`)
Kiểm tra `control_port` trong log (hoặc trong `$env:TEMP\mujoco_studio_session_*.json`) và xác nhận toàn bộ `scene`, `messages`, `sim_plugin`, `viewer_plugin` đều `"ok": true`:
```powershell
curl.exe --noproxy "*" -s http://127.0.0.1:<control_port>/_studio/status
```

### Bước 3: Mở khung nhìn 3D
- **Trong Antigravity:** Nhấp vào pill `[MuJoCo Studio](sidecar://robotics/mujoco_studio/)`.
- **Trình duyệt ngoài:** Mở `http://127.0.0.1:8080/`.

---

## 3. Chuyển bản đồ & Chạy điều khiển (Mode A / B / C)

Mọi lệnh `robot_ctl.py` đều chạy qua `.\.venv\Scripts\python.exe` và kết nối trực tiếp tới `studio/sim_plugin.py` ở `http://127.0.0.1:8765`:

```powershell
# 1. Nạp nóng sa bàn mục tiêu (2wheel | maze_a | maze_b | maze_c | dynamic_a | dynamic_b)
.\.venv\Scripts\python.exe robot_ctl.py reload --arena maze_a --vision

# 2. (Tùy chọn) Bật hộp đen giám sát nền 10 Hz (0 token LLM)
#    Mode A: --max-duration 360 | Mode C: --max-duration 480 | Mode B: --max-duration 900
.\.venv\Scripts\python.exe robot_ctl.py telemetry record --hz 10 --max-duration 480

# 3. Chạy thử lệnh System 1 (Mode C hybrid-brain)
.\.venv\Scripts\python.exe robot_ctl.py topo show
.\.venv\Scripts\python.exe robot_ctl.py go --cardinal E --max-dist 6.0 --speed 0.55

# 4. Dừng hộp đen và xuất bảng điểm JSON
.\.venv\Scripts\python.exe robot_ctl.py telemetry stop
```

> **Studio Topo HUD:** Khi Mode C hoạt động (`captures/.topo_state.json` được ghi bởi `robot_ctl.py`), panel UI bên phải trong MuJoCo Studio tự động hiển thị dòng `Topo Graph: N nodes | current=#id | open=K` (đồng bộ thụ động `<= 2 Hz` qua `RobotTelemetry`, tự xóa về `0` khi reset/reload).

---

## 4. Xử lý sự cố thường gặp (Troubleshooting)

| Triệu chứng | Nguyên nhân | Cách khắc phục |
| :--- | :--- | :--- |
| `/_studio/status` báo `sim_plugin.ok: false` với `ModuleNotFoundError: No module named 'PIL'` | Môi trường `~\.venvs\mujoco-studio` chưa cài `pillow` | Chạy `uv pip install --python "$env:USERPROFILE\.venvs\mujoco-studio\Scripts\python.exe" pillow` rồi khởi động lại `studiod.py`. |
| `robot_ctl.py` báo `Không kết nối được tới http://127.0.0.1:8765` | `sim_plugin.py` lỗi khi load hoặc cổng `8765` đang bị `sim_server.py` chiếm | Kiểm tra `/_studio/status` xem traceback ở `sim_plugin.error`, hoặc tắt `sim_server.py` đang chạy rồi khởi động lại `studiod.py`. |
| WebViewer hiện `DISCONNECTED` / `stalled` khi robot đang chạy | Khi `sim_plugin.py` xử lý HTTP/render nặng trên luồng mô phỏng, nhịp WebSocket tới trình duyệt có thể trễ thoáng qua | `studio/sim_plugin.py` đã giới hạn `_drain_http_queue(max_items=2)`, bật `_vision_cache` theo tư thế `1 mm` và giảm telemetry IPC xuống `12.5 Hz` (`http_latency_ms.max ~ 23 ms`). Nếu vẫn gặp, đợi lệnh kết thúc hoặc tải lại tab WebViewer — mô phỏng vẫn chạy bình thường ở nền. |
| Góc xoay (`gyro_yaw_deg`) hoặc odometry lệch trong Studio | `PostStepEvent` trong MuJoCo Studio có thể gọi lại khi `data.time` chưa tăng nếu tích phân theo `model.opt.timestep` cố định | `studio/sim_plugin.py` tích phân theo `dt = data.time - self._last_odom_time` (chỉ tích phân khi `dt > 1e-9`), bảo đảm sai số góc `< 0.1°` và odometry khớp tuyệt đối với vật lý. |
| MuJoCo Studio đang hiển thị workspace dự án khác | Mỗi user chỉ có 1 daemon `studiod` tại một thời điểm | Khởi chạy lại `studiod.py --workspace_dir="d:\latuan\Programming\robot-demo\studio"` — daemon mới sẽ tự thay thế phiên cũ. |
