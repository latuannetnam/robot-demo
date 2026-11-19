# Thiết kế Hệ thống Điều hướng Thử nghiệm "Agent Làm Não, Skill Làm Tay Chân" (`agent-brain`)

**Ngày thiết kế:** 2026-11-19  
**Trạng thái:** Chờ người dùng phê duyệt (trước bước lập kế hoạch thi công `writing-plans`)  
**Phạm vi:** Tầng điều khiển kỹ năng (`robot_sim/skills.py`, `robot_ctl.py`), prompt & quy trình đa tác tử (`.agents/skills/robot-multi-agent-rd/`), chỉ số hộp đen (`robot_sim/telemetry.py`, `robot_sim/arenas.py`), và bộ test tương ứng.

---

## 1. Mục đích & Lý do (Intent & Why)

Trong kiến trúc hiện tại (`explore_maze` và `approach_target`), AI Agent chỉ đóng vai trò người điều phối cấp cao (gọi 2–3 lệnh trọn gói), còn gần như toàn bộ trí thông minh tìm đường — bám tường, chọn nhánh, ghi dấu chân odometry 2D kiểu Trémaux để thoát vòng lặp, tự quay đầu ở ngõ cụt, tự né vật cản và canh vượt nhiều làn động liên tiếp — đều được lập trình cứng bằng thuật toán trong [robot_sim/skills.py](file:///C:/Users/latuan/Programming/robot-demo/robot_sim/skills.py).

Thử nghiệm này tạo ra một chế độ song song mang tên **`agent-brain`** nhằm đảo ngược vai trò đó:
- **AI Agent là bộ não duy nhất:** Agent tự quan sát cảm biến, tự dựng và duy trì trí nhớ không gian (nút giao, nhánh đã thử, ngõ cụt, vòng lặp), tự lập kế hoạch tránh vật cản tĩnh và tự quyết định vượt từng làn/cổng vật cản động.
- **Skill chỉ là công cụ thực thi ("tay chân"):** Không trạng thái dài hạn (stateless), không lưu bản đồ hay lịch sử đi lại, không có quy tắc bám tường trái/phải, không tự quay đầu ở ngõ cụt hay tự giải mê cung.
- **Tái định nghĩa tiêu chí đánh giá:** Thời gian hoàn thành tổng cộng (`wall_time_s`) **không phải** tiêu chí đánh giá chính; thay vào đó hệ thống đo độ an toàn, tỷ lệ về đích, trí nhớ không gian, hiệu suất quãng đường và tách bạch thời gian suy nghĩ của Agent so với thời gian xe chạy thực tế.

---

## 2. Mục tiêu, Phi mục tiêu & Phản mục tiêu

### Mục tiêu (Goals)
1. Bổ sung bộ công cụ skill trung gian theo sự kiện (`advance`, `cross-gate`, `dock`) cùng cấu hình subagent **`robot-driver-brain`** để Agent tự giải quyết **cả 6 sa bàn chuẩn** (`2wheel`, `maze_a`, `maze_b`, `maze_c`, `dynamic_a`, `dynamic_b`).
2. Giữ nguyên toàn bộ chế độ **`auto-skill`** (`explore`, `approach`, `robot-driver`) và toàn bộ 94 bài kiểm thử hiện có để làm đường cơ sở (baseline) so sánh trực tiếp.
3. Mở rộng [telemetry.py](file:///C:/Users/latuan/Programming/robot-demo/robot_sim/telemetry.py) và [arenas.py](file:///C:/Users/latuan/Programming/robot-demo/robot_sim/arenas.py) nhằm đo được các chỉ số nhận thức & trí nhớ không gian của Agent (vẫn giữ tương thích ngược hoàn toàn với schema JSON cũ).

### Phi mục tiêu (Non-Goals)
- Không tối ưu rút ngắn `wall_time_s` trong chế độ `agent-brain`.
- Không chỉnh sửa hình học robot hoặc bất kỳ địa hình nào trong `models/robot_2wheel.xml` và `models/arena_*.xml`.
- Không loại bỏ hoặc phá vỡ hành vi hiện tại của `explore` / `approach` / `scan`.

### Phản mục tiêu (Anti-Goals — coi là thất bại thiết kế dù xe tới đích)
- Lén nhét bộ nhớ `breadcrumbs`, thuật toán Trémaux, quy tắc `left`/`right`, tự động quay đầu 180° ở ngõ cụt, tự động lách quanh nhiều vật cản, hoặc tự động vượt nhiều làn/cổng động liên tiếp vào trong một lệnh skill mới.
- Đóng băng vật lý mô phỏng trong lúc LLM đang suy nghĩ (phải giữ nguyên `realtime 1.0x` 500 Hz để kiểm chứng kỹ năng dừng an toàn của xe và chiến thuật của Agent).
- Vi phạm kỷ luật Sim-to-Real (rò rỉ `--debug`, `arenas`, tọa độ ground-truth `robot_xy`/`target_xy` hay bản đồ sa bàn cho Agent lái).

---

## 3. Ràng buộc Vật lý & Kỷ luật Sim-to-Real

1. **Vật lý thời gian thực khi LLM suy nghĩ:** Khi Agent mất 2–10 giây suy luận giữa hai lệnh CLI, mô phỏng 500 Hz và các khối piston `obs_red`, `obs_blue`, `obs_yellow` vẫn chuyển động liên tục. Mọi skill vận động khi trả kết quả **bắt buộc** đã phanh đứng yên ở vị trí an toàn bên ngoài biên quét của piston gần nhất (có lùi nhẹ hoặc nép hốc nếu đang sát đường ray).
2. **Đầu vào thuần cảm biến trên xe:** Agent chỉ được đọc JSON của 7 tia laser (`left`, `center`, `right`, `left90`, `right90`, `left120`, `right120`), odometry 2D xuất phát từ vị trí spawn (`odom_dist_m`, `odom_x_m`, `odom_y_m`, `gyro_yaw_deg`, `forward_speed_m_s`, `yaw_rate_rad_s`), `front_cam` Edge Vision JSON (`target_visible`, `target`, `obstacles`), và chủ động chụp/xem ảnh JPEG qua `camera --name front_cam`.
3. **Tốc độ CLI trên Windows:** Luôn gọi qua `.venv\Scripts\python.exe robot_ctl.py ...` (nhanh hơn `uv run python` ~2 giây mỗi lệnh).

---

## 4. Các Phương án Đã Cân nhắc & Phương án Được Chọn

| Tiêu chí | PA1: Hình học Sơ cấp (`move`, `turn`, `cross-gate`, `dock`) | PA2: Theo Sự kiện Địa hình (`advance`, `cross-gate`, `dock`) **[ĐƯỢC CHỌN]** | PA3: Điểm mốc Cục bộ (`goto_rel <dx> <dy>`) |
| :--- | :--- | :--- | :--- |
| **Cách vận hành** | Agent ra lệnh tiến từng đoạn mét rời rồi tự căn lại giữa hành lang. | Skill giữ làn phản xạ ngắn hạn 20 Hz, **dừng trả quyền ngay khi chạm sự kiện đầu tiên** (ngã rẽ, ngõ cụt, vật cản tĩnh/động, gần đích). | Agent ước lượng tọa độ đích cục bộ `(dx, dy)` rồi giao bộ điều khiển tự lái tới đó. |
| **Vai trò não của Agent** | Agent tự kiểm soát mét & độ nhưng bị ngập trong tác vụ chỉnh làn nhỏ nhặt. | **Agent làm 100% não điều hướng:** tự vẽ đồ thị nút giao, chọn nhánh, nhớ ngõ cụt, thoát lặp, quyết định đường lách vật cản tĩnh và ra lệnh vượt từng làn động. | Mờ ranh giới não–tay chân vì bộ lái tới `(dx, dy)` buộc phải tự né tường/vách khuất. |
| **Độ an toàn 1.0x** | Dễ lệch tâm hành lang tích lũy hoặc dừng ngay miệng cua dẫn tới quẹt góc (`near_collision`). | Cao: xe luôn phanh thẳng giữa tâm nút hoặc ngoài tầm piston trước khi chờ LLM. | Trung bình: sai số ước lượng `(dx, dy)` từ ảnh 2D dễ đâm cạnh tường. |
| **Mức độ phù hợp** | Thô, hao 40–70 bước chỉnh cơ học trên `maze_c`. | **Tối ưu cho thử nghiệm:** ~8–25 lượt quyết định gọn gàng, mọi bước đều là tư duy chiến thuật thực chất. | Không tách bạch rạch ròi Agent vs Skill. |

**Kết luận:** Chọn **Phương án 2** (vẫn giữ sẵn `move`, `turn`, `scan --no-recover-occlusion` ở tầng sơ cấp để Agent tự chủ động tính góc/khoảng cách khi cần lách vật cản tĩnh trên sân mở).

---

## 5. Thiết kế Chi tiết Kiến trúc `agent-brain`

### 5.1. Giác quan đầu vào cho Agent (Cấu trúc hóa + Trực quan Chọn lọc)
- **Mặc định ở mọi bước:** Mọi skill trả về `after_status` chứa JSON cảm biến + Edge Vision nhẹ (~0.5–0.8 KB), không chứa `debug`, giúp Agent giữ trọn ngữ cảnh lịch sử dài hàng chục nút giao mà không tràn context.
- **Trực quan chọn lọc (Multimodal on demand):** Khi gặp tình huống nhập nhằng (ví dụ vật cản tĩnh chắn giữa sân mở `2wheel`, ngã giao nhiều nhánh khuất góc, hoặc `cross-gate` trả về thất bại), Agent chủ động gọi:
  ```powershell
  .venv\Scripts\python.exe robot_ctl.py camera --name front_cam --out captures/brain_view.jpg
  ```
  rồi dùng công cụ đọc tệp hình ảnh (`read_file`) quan sát trực tiếp góc nhìn mũi xe trước khi ra quyết định hướng lách hoặc hướng rẽ. *(Ghi chú Sim-to-Real: chỉ được phép dùng `--name front_cam`, tuyệt đối không dùng `--name overview_cam` vì trong thực tế xe không có camera vệ tinh trên trần.)*

### 5.2. Hợp đồng 3 Công cụ Skill Stateless Mới (`robot_sim/skills.py` & `robot_ctl.py`)

Tất cả skill mới đều **không nhận tham số `rule`**, không lưu bất kỳ biến trạng thái toàn cục hay giữa các lần gọi, và luôn gọi `_clean_status` để tước bỏ trường `debug`.

#### A. `advance_segment` — CLI: `.venv\Scripts\python.exe robot_ctl.py advance [--max-dist 1.8] [--speed 0.55] [--step-dt 0.10]`
- **Trách nhiệm phản xạ (20 Hz):**
  - Chạy thẳng dọc hành lang/hướng hiện tại cho tới khi hết đoạn hoặc chạm **sự kiện đầu tiên**.
  - `[Quyết định kỹ thuật đề xuất]` Khi thấy tường hai bên (`left90 < 0.65` và `right90 < 0.65`), dùng điều khiển PD giữ xe ở chính giữa hành lang (theo `left90 - right90`) và bám trục hướng gần bội số `90°` nhất nếu góc lệch hiện tại trong vòng `±15°`. Ở không gian mở, giữ nguyên góc `gyro_yaw_deg` lúc bắt đầu lệnh (bộ giữ thẳng hướng).
- **Thứ tự ưu tiên ngắt & trả quyền (dừng ngay ở điều kiện thỏa đầu tiên):**
  1. **Kiểm tra đầu lệnh (`t = 0`, trước khi lăn bánh):**
     - Nếu đang ở trong hành lang rộng `>= 0.72 m` (`min(rf_l90, rf_r90) >= 0.24 and rf_l90 + rf_r90 >= 0.72`), thấy đích thẳng tâm ở xa (`target_visible`, `|offset_x| < 0.25`, `rf_center > 1.05`, `min(rf_l, rf_r) > 0.50`) và `obstacles` đang rỗng: đứng yên quan sát tối đa `2.2 s` sim-time để phát hiện piston có thể đang ẩn trong hốc bên (`obs_yellow` trên `dynamic_b`). Nếu thấy vật cản xuất hiện → nhảy ngay sang bước phân loại vật cản bên dưới.
     - Nếu `target_visible == true`, không có vật cản đứng gần hơn mục tiêu, và `rf_center <= 0.95 m` (đích đã ở cự ly gần, thoáng trước mặt): dừng trả ngay `event = "target_ready"`.
     - Nếu thấy vật cản màu trong ngưỡng kích hoạt (`z_est <= 0.65 m` khi ở hành lang hoặc `z_est <= 0.75 m` ở không gian mở, với `|offset_x| < 1.01`): chạy vòng thăm dò đứng yên `<= 0.42 s` (so độ dịch chuyển `offset_x` và `min_front_rf`). Nếu dịch chuyển (`>= 0.018` hoặc `>= 0.015 m`) → trả `event = "dynamic_gate_ahead"`; nếu đứng yên sau `0.42 s` → trả `event = "static_obstacle_ahead"`.
  2. **Đích ở cự ly gần trên đường đi (`"target_ready"`):** `target_visible == true`, không có vật cản nào nằm gần giữa hơn mục tiêu (`closer_center_obs == false`), và `rf_center <= 0.95 m` → phanh dừng và trả về để Agent tự quyết định gọi `dock`.
  3. **Vật cản phía trước trên đường đi (`"dynamic_gate_ahead"` hoặc `"static_obstacle_ahead"`):**
     - Phát hiện sớm qua camera (`0.16 <= z_est <= 0.65 m` ở hành lang hoặc `<= 0.75 m` ở không gian mở, `|offset_x| < 1.01`) hoặc qua tia laser phía trước (`min_front_rf < 0.44 m` trong không gian mở khi không phải trụ đích gần).
     - Phanh dừng hoàn toàn (nếu đang quá sát `< 0.34 m` hoặc `z_est < 0.26 m` thì chủ động lùi `-0.10 m` ra ngoài vùng quét piston trước khi dừng hẳn).
     - Thăm dò tĩnh `<= 0.42 s` để báo cho Agent biết vật cản đang nằm trước mặt là **động** hay **tĩnh** (`event = "dynamic_gate_ahead"` hoặc `"static_obstacle_ahead"`), kèm thông tin `obstacle` (`color`, `est_dist_m`, `offset_x`, `v_span`) và khoảng hở hai sườn (`flank_clearance_m`) để Agent tự chọn rẽ trái hay phải lách qua nếu là vật cản tĩnh.
  4. **Ngã rẽ mới mở (`"junction"`):**
     - Chỉ kích hoạt sau khi xe đã đi được tối thiểu `0.22 m` kể từ lúc bắt đầu lệnh `advance` (tránh báo lặp lại chính ngã giao mà xe vừa rời đi).
     - Khi phát hiện ít nhất một bên (`left` hoặc `right`) chuyển từ có vách tường sang mở thoáng (`rf_90 > 0.78 m` và `rf_120 > 0.75 m`, hoặc `rf_90 > 0.85 m` nếu không có tia 120°): cho xe tiến thêm tối đa `~0.32 m` tính từ mép vách để thân xe lọt đúng vào tâm giao lộ (tự cắt ngắn sớm nếu phía trước gặp tường `< 0.36 m` hoặc thấy vật cản phía trước), rồi phanh dừng và trả `event = "junction"`.
  5. **Chạm mặt tường cuối đoạn (`rf_center < 0.40 m` hoặc `min(rf_left, rf_right) < 0.28 m`):**
     - Phanh dừng (lùi nhẹ `-0.08 m` nếu quá sát `< 0.25 m` để giữ đủ bán kính xoay an toàn).
     - Không tự quay đầu: kiểm tra `open_branches` theo laser 90°. Nếu còn ít nhất `"left"` (`left90 > 0.52 m`) hoặc `"right"` (`right90 > 0.52 m`) → trả về **`event = "junction"`** (góc L hoặc ngã T cụt đầu); nếu cả hai bên đều kín → trả về **`event = "dead_end"`**.
  6. **Hết đoạn tối đa (`"max_dist"`):** Đã đi đủ `max_dist` mét mà không gặp sự kiện nào.
- **Định dạng trả về JSON (luôn lọc bỏ `debug`):**
  ```json
  {
    "event": "junction | dead_end | dynamic_gate_ahead | static_obstacle_ahead | target_ready | max_dist",
    "moved_m": 0.84,
    "open_branches": ["left", "straight"],
    "obstacle": {"color": "red", "motion": "dynamic", "est_dist_m": 0.46, "offset_x": -0.12},
    "flank_clearance_m": {"left": 0.92, "right": 0.88, "left90": 1.90, "right90": 1.90},
    "after_status": { ... }
  }
  ```
  *(Quy tắc tính `open_branches` chỉ dùng cảm biến tại chỗ lúc đã dừng: `"straight"` khi `center > 0.62` và `min(left, right) > 0.34`; `"left"` khi `left90 > 0.55`; `"right"` khi `right90 > 0.55`.)*

#### B. `cross_dynamic_gate` — CLI: `.venv\Scripts\python.exe robot_ctl.py cross-gate [--timeout 12.0]`
- **Trách nhiệm phản xạ (20 Hz):** Vượt qua **duy nhất 1 làn/cổng vật cản động** ngay trước mặt khi Agent ra lệnh, rồi lập tức dừng an toàn.
- **Cơ chế an toàn:**
  1. Nếu ở không gian mở (`min(left90, right90) > 0.75 m`) và vật cản vừa lọt ra ngoài góc nhìn camera ngay trước khi Agent gọi `cross-gate`: `[Quyết định kỹ thuật đề xuất]` thực hiện liếc nhanh hai cánh `±26°` (mỗi bên dừng `0.12 s`) để khóa pha vật cản nếu nó đang chuẩn bị lao từ rìa vào giữa làn. Nếu kiểm tra `0.42 s` thấy vật cản hoàn toàn đứng yên → không lao bừa, trả ngay `{"crossed": false, "reason": "static_obstacle", ...}`.
  2. Chờ tối đa `timeout` giây sim-time cho tới khi **cửa sổ an toàn mở 2 nhịp liên tiếp**: `center > 0.78 m`, `min(left, right) > 0.38 m`, và vật cản trước cổng hoặc đã đi ra ngoài FOV theo chiều thuận, hoặc đang ở mép `|offset_x| >= 0.78` và dịch chuyển ra xa tâm. Nếu hết `timeout` mà không thấy nhịp mở → trả `{"crossed": false, "reason": "timeout", ...}`.
  3. Tiến thẳng dứt khoát tối đa `commit_dist = min(1.05, max(0.72, nearest_gate_z + 0.48))` mét để qua khỏi đường ray hiện tại. **Bắt buộc ngắt sớm lệnh tiến** nếu một trong hai điều kiện xảy ra:
     - Đang tiến tới tường chắn (`rf_center <= 0.34 m`), hoặc
     - Sau khi đã qua vùng tâm của cổng hiện tại (`moved >= nearest_gate_z + 0.36 m`), phát hiện **vật cản kế tiếp** xuất hiện ở phía trước trong khoảng `z_est <= 0.50 m` hoặc `min(rf_left, rf_center, rf_right) < 0.44 m` (bảo đảm xe phanh dừng an toàn ở khoảng đệm giữa các làn của `dynamic_a`).
  4. **Bảo vệ đỗ sau khi vượt cổng bất đối xứng:** Nếu điểm dừng nằm ngay miệng nối hai hành lang (`center > 0.90 m`, `max(l90, r90) > 0.85 m`, `min(l90, r90) > 0.29 m` — trường hợp vừa vượt qua `obs_blue` sang nhánh cuối trong `dynamic_b`), tự động nép chéo `±30°/45°` một đoạn ngắn vào giữa nhánh hẹp rồi xoay lại hướng ban đầu để bánh sau không thò vào vùng quét của `obs_blue` trong lúc chờ Agent nghĩ.
- **Trả về JSON:** `{"crossed": bool, "reason": "cleared | next_obstacle_ahead | wall_ahead | static_obstacle | timeout", "moved_m": float, "open_branches": list[str], "after_status": dict}`.

#### C. `dock_target` — CLI: `.venv\Scripts\python.exe robot_ctl.py dock [--stop-distance 0.25]`
- **Trách nhiệm phản xạ (20 Hz):** Servo thị giác cự ly gần (`stop_distance` hợp lệ `0.15..0.35 m`, mục tiêu là đưa tâm xe về `0.15 <= center <= 0.35 m` và `|offset_x| < 0.20`).
- **Giới hạn trách nhiệm chặt chẽ (không tự giải vật cản):**
  - Chỉ chạy khi mục tiêu đang thấy được và ở cự ly gần thoáng (`area_pct >= 0.9%` hoặc `rf_center <= 1.25 m`). Nếu quá xa (`too_far`) hoặc mất dấu ban đầu (`not_visible`) → từ chối chạy ngay, trả về cho Agent quyết định gọi `advance` hoặc `scan`.
  - Không tự bẻ lái né vật cản, không tự đứng chờ nhiều nhịp piston: nếu xuất hiện vật cản đứng gần giữa đường ngắm hơn đích hoặc `min_front_rf < 0.38 m` khi chưa áp sát đích → phanh dừng và trả `{"reached": false, "reason": "obstacle_blocking", ...}` để Agent tự xử lý.

#### D. Điều chỉnh nhỏ công cụ hiện có trong `robot_ctl.py`
- Cho phép truyền `robot_ctl.py scan --max-degrees 360 --direction left --no-recover-occlusion` (đã có cờ `--no-recover-occlusion` trong code) và chỉ dẫn trong `driver-brain-prompt.md` **luôn gắn `--no-recover-occlusion`** để `scan` chỉ làm công cụ xoay quan sát tại chỗ, không tự động chạy tới `0.85 m`.
- Bổ sung 3 subcommand `advance`, `cross-gate`, `dock` vào [robot_ctl.py](file:///C:/Users/latuan/Programming/robot-demo/robot_ctl.py) và xuất các hàm tương ứng từ [robot_sim/skills.py](file:///C:/Users/latuan/Programming/robot-demo/robot_sim/skills.py).

---

### 5.3. Kiến trúc Bộ não Agent (`driver-brain-prompt.md` & `SKILL.md`)

Tạo tệp prompt mới `.agents/skills/robot-multi-agent-rd/references/driver-brain-prompt.md` cho subagent **`robot-driver-brain`**:

1. **Giới hạn lệnh nghiêm ngặt (Sim-to-Real + Không Dùng Skill Giải Mê Cung):**
   - **Được dùng:** `status --vision`, `vision`, `camera --name front_cam`, `advance`, `cross-gate`, `dock`, `turn`, `move`, `scan --no-recover-occlusion`, `wheels`, `stop`.
   - **Cấm tuyệt đối:** `--debug`, `arenas`, `reset`, `reload`, `demo`, `telemetry`, và **CẤM gọi `explore` hay `approach`**.

2. **Giao thức Trí nhớ Không gian — Sổ tay Đồ thị (Topological Scratchpad):**
   - Trước mỗi quyết định rẽ hoặc sau mỗi lệnh `advance` / `turn` / `cross-gate`, Agent bắt buộc cập nhật trong suy nghĩ bảng Sổ tay Đồ thị gồm:
     - **Danh sách Nút (`Node #1, #2, ...`):** hợp nhất hai lần dừng vào cùng 1 nút nếu khoảng cách Euclid `hypot(odom_x - nx, odom_y - ny) <= 0.40 m`.
     - **4 hướng tuyệt đối theo IMU (`gyro_yaw_deg`):** quy chuẩn theo bội 90° gần nhất (`Đông ≈ 0°`, `Bắc ≈ +90°`, `Tây ≈ ±180°`, `Nam ≈ -90°`). Với mỗi nút, ghi trạng thái từng hướng mở: `chưa_đi` | `đã_đi` | `đã_dẫn_vào_cụt`.
     - **Lịch sử ngăn xếp nút (Stack / Backtrack):** để khi chạm `dead_end`, Agent biết chính xác phải `turn 180`, `advance` quay lại nút cha gần nhất, đánh dấu nhánh vừa thoát là `đã_dẫn_vào_cụt`, và chọn nhánh `chưa_đi` tiếp theo (hoặc lùi tiếp về nút ông nếu nút cha không còn hướng `chưa_đi`).

3. **Chiến thuật 4 lớp kịch bản hoàn toàn do Agent suy luận:**
   - **Lớp 1 — Mê cung có ngõ cụt (`maze_a`, `maze_b`):** Dùng `advance` + bảng Nút giao; luôn ưu tiên hướng `chưa_đi`. Nếu quay lại nút có nhánh đã đánh dấu `đã_dẫn_vào_cụt`, tuyệt đối không rẽ lại nhánh đó (`redundant_revisit_count == 0`).
   - **Lớp 2 — Mê cung có đảo vòng lặp (`maze_c`):** Nếu `advance` đưa xe tới một tọa độ trùng trong bán kính `0.40 m` của một nút cũ qua một hướng mới (nhưng đích chưa thấy), Agent **tự phát hiện vòng lặp** ("Tọa độ `(x, y)` trùng `Node #k`"), đánh dấu cạnh vừa đi thành vòng lặp đã khám phá hết và chọn nhánh thay thế trên `Node #k` (hoặc quay ngược thoát đảo theo Sổ tay Đồ thị) thay vì chạy vòng vèo theo 1 quy tắc bám tường cố định.
   - **Lớp 3 — Sân mở & Vật cản tĩnh (`2wheel`):** Khi `advance` dừng với `event = "static_obstacle_ahead"` (hoặc phát hiện đích bị che khuất sau khi `scan --no-recover-occlusion`), Agent đọc `flank_clearance_m` (và có thể gọi `camera --name front_cam` nếu cần), tự tính bước vòng lách bằng `turn ±60..90` → `move 0.45..0.65` → `scan --no-recover-occlusion` (xoay nhìn lại hướng đích) → `advance`, hoàn toàn tự chủ quỹ đạo né.
   - **Lớp 4 — Vật cản di động theo làn & cổng piston (`dynamic_a`, `dynamic_b`):**
     - Bất cứ khi nào `advance` trả về `event = "dynamic_gate_ahead"` (hoặc `status --vision` ban đầu cho thấy khối màu chuyển động chắn ngay trước mặt): Agent gọi `cross-gate` để vọt qua **đúng 1 làn/cổng**, rồi gọi tiếp `advance` để đánh giá nút giao hoặc làn tiếp theo.
     - Trên `dynamic_a` (3 làn tuần tra cắt ngang), quá trình diễn ra theo đúng chu kỳ nhận thức: `advance` → `cross-gate` (làn 1) → `advance` → `cross-gate` (làn 2) → `advance` → `cross-gate` (làn 3) → `dock`.
   - **Kết thúc nhiệm vụ:** Chỉ gọi `dock --stop-distance 0.25` khi `target_ready` và không có vật cản chắn giữa; xác nhận 3 điều kiện dừng chuẩn trong `after_status` rồi báo cáo.

4. **Cập nhật [SKILL.md](file:///C:/Users/latuan/Programming/robot-demo/.agents/skills/robot-multi-agent-rd/SKILL.md) và `engineer-prompt.md` (`[Quyết định kỹ thuật đề xuất]`):**
   - **Chế độ mặc định:** Khi người dùng gọi skill mà không chỉ định chế độ, **Mode A (`auto-skill`, `robot-driver`)** vẫn là mặc định để giữ nguyên toàn bộ luồng vận hành & kiểm thử cơ sở.
   - **Điều phối Mode B (`agent-brain`, `robot-driver-brain`, blackbox `--max-duration 900`):**
     - Vẫn chạy vòng lặp auto-R&D tối đa 3 vòng (không áp ngưỡng `wall_time_s`). Vòng R&D kích hoạt `robot-engineer` khi vi phạm **bất kỳ tiêu chí bắt buộc nào** (`goal_reached == false`, `contact_count > 0`, `near_collision_count > 0`, `error_count > 0`) **HOẶC** thất bại về trí nhớ không gian (`redundant_revisit_count > 0`).
     - **Phạm vi sửa của `robot-engineer` ở Mode B (cần ghi rõ trong `engineer-prompt.md`):** Chỉ được phép chỉnh sửa (a) chiến thuật suy luận / sổ tay topo trong `driver-brain-prompt.md`, và (b) ngưỡng an toàn phản xạ vô trạng thái trong `advance_segment` / `cross_dynamic_gate` / `dock_target` (`robot_sim/skills.py`); **cấm tuyệt đối** sửa `models/robot_2wheel.xml` (Phi mục tiêu #2), sửa `models/arena_*.xml`, hoặc thêm bộ nhớ/`breadcrumbs`/luật giải mê cung vào `robot_sim/skills.py` (Phản mục tiêu #1).
     - **Regression guard Mode B:** Nếu vòng `i + 1` tụt hạng, khôi phục qua `git checkout -- robot_sim/skills.py .agents/skills/robot-multi-agent-rd/references/driver-brain-prompt.md`.

---

### 5.4. Bộ Chỉ số Đánh giá Mới trong `robot_sim/arenas.py` & `robot_sim/telemetry.py`

1. **Thêm `reference_path_m: float` vào `ArenaSpec` ([robot_sim/arenas.py](file:///C:/Users/latuan/Programming/robot-demo/robot_sim/arenas.py)):**
   `[Quyết định kỹ thuật đề xuất]` Định nghĩa **`reference_path_m`** thống nhất là **độ dài đường trắc địa (geodesic) ngắn nhất không va chạm tường/vật cản tĩnh** từ `spawn_xy` mặc định tới vùng đỗ chuẩn (`hypot(x - tx, y - ty) <= 0.44 m` ứng với `stop_distance = 0.25 m`, tính bằng tìm đường lưới `0.04 m` với bán kính an toàn thân xe `0.16 m`):
   - `2wheel`: `0.96 m`
   - `maze_a`: `4.22 m`
   - `maze_b`: `6.50 m`
   - `maze_c`: `3.39 m`
   - `dynamic_a`: `2.88 m`
   - `dynamic_b`: `3.16 m`
   *(Hằng số này ẩn hoàn toàn khỏi Agent lái. Nếu người dùng đổi mục tiêu tùy chỉnh qua `reset --target-x <X> --target-y <Y>` làm `debug.target_xy` lệch `> 0.05 m` so với `default_target_xy`, hoặc `arena_id` không nằm trong `ARENAS`, `telemetry.py` giữ nhẹ bằng cách xuất `reference_path_m = None` và `path_efficiency = None` mà không chạy lại planner 2D trong hộp đen. Trong bảng đối chiếu Mục 6.3, kết quả `agent-brain` sẽ được so với cả mốc trắc địa `reference_path_m` lẫn kết quả chạy sạch thực tế của `auto-skill`.)*

2. **Mở rộng tóm tắt của `record_telemetry` trong [robot_sim/telemetry.py](file:///C:/Users/latuan/Programming/robot-demo/robot_sim/telemetry.py):**
   Giữ nguyên 100% các trường JSON cũ và bổ sung nhóm trường mới:
   - **`active_drive_time_s` & `agent_think_time_s`:**
     Mỗi chu kỳ mẫu telemetry (`dt_sample`, khoảng `0.10 s`, tính theo chênh lệch đồng hồ giữa hai lần poll thực tế hoặc tổng chuẩn hóa khớp `wall_time_s`), nếu xe đang chạy (`moving or fwd_spd > 0.02 or yaw_spd > 0.04`) thì cộng vào `active_drive_time_s`; ngược lại (xe dừng hẳn chờ LLM suy luận giữa các lệnh CLI) cộng vào `agent_think_time_s`. Như vậy `active_drive_time_s + agent_think_time_s ≈ wall_time_s`.
   - **`command_burst_count`:** Đếm số lần chuyển pha `moving` từ `False -> True` (tương ứng số nhịp lệnh vận động thực tế Agent đã phát xuống xe).
   - **Đo trí nhớ không gian theo lưới có hướng `(cell, dir_q)` + lưới ô vô hướng (`[Quyết định kỹ thuật đề xuất]`):**
     - Rời rạc hóa vị trí (`debug.robot_xy`) và góc hướng (`debug.robot_yaw_deg`) thành ô lưới `cell = (round(x / 0.50), round(y / 0.50))` và góc phần tư hướng `dir_q = int(((yaw_deg + 45.0) % 360.0) // 90.0)` (4 hướng chính: Đông/Bắc/Tây/Nam); reset sạch lịch sử khi `reset_jumped == true`.
     - Một lần quay lại ô vô hướng `cell` sau khi rời `cell` và đi thêm `>= 0.80 m` (bất kể hướng) làm tăng **`backtrack_revisit_count`** (đo tổng lượt đi ngược lại địa hình cũ khi thoát ngõ cụt).
     - Một lần quay lại đúng cặp có hướng **`(cell, dir_q)`** sau khi đã rời `(cell, dir_q)` và đi thêm `>= 0.80 m` (`lần ghé thứ 2+` theo cùng một chiều đi qua cùng một ô) làm tăng **`redundant_revisit_count`**.
     - *Lý do thiết kế:* Trên ngã tư bậc 3 có 2 nhánh cụt liên tiếp (như nút `(-1.6, 0.0)` phía Tây của `maze_c`), ngay cả thuật toán DFS mù tối ưu cũng phải đi qua nút `(-1.6, 0.0)` **3 lần** theo 3 hướng khác nhau (vào hướng Tây → từ Bắc về hướng Nam → từ Nam về hướng Bắc). Đếm theo `(cell, dir_q)` giúp trường hợp DFS hoàn hảo vẫn đạt **`redundant_revisit_count == 0`**, trong khi rẽ nhầm lại đúng nhánh cụt cũ hoặc chạy quẩn vòng lặp cùng chiều sẽ lập tức bị ghi nhận `redundant_revisit_count >= 1`.
     - Xuất thêm **`unique_cells_visited`** (số ô `0.50 m` phân biệt đã đi qua).
   - **`reference_path_m` & `path_efficiency`:**
     - Tra `spec = ARENAS.get(arena_id)`; nếu `target_xy` khớp `spec.default_target_xy` trong sai số `<= 0.05 m` thì gán `reference_path_m = spec.reference_path_m`, ngược lại `None`.
     - `path_efficiency = round(min(1.0, reference_path_m / path_length_m), 3)` khi `reference_path_m is not None` và `goal_reached == true` và `path_length_m > 0`, ngược lại là `None`.

3. **Bảng tiêu chí đánh giá mới cho chế độ `agent-brain` (không đánh giá theo `wall_time_s`):**
   - **Điều kiện Đạt (Bắt buộc):** `goal_reached == true`, `contact_count == 0`, `near_collision_count == 0`, `error_count == 0`.
   - **Chỉ số Chất lượng "Bộ não":**
     - `redundant_revisit_count == 0` (trí nhớ không gian chuẩn, không quay lại ngõ cụt hay luẩn quẩn vòng lặp quá 1 lượt backtrack).
     - `path_efficiency` (tiến gần `1.0` nhất có thể, đối chiếu với baseline `auto-skill`).
     - `command_burst_count` (ít lệnh thừa / sửa sai).
     - `active_drive_time_s` so với `agent_think_time_s` (kiểm chứng thời gian thực tế xe chạy mượt mà dù thời gian LLM suy nghĩ kéo dài).

---

## 6. Phần Dành Cho Người Lập Kế Hoạch Thi Công (What's Left to the Builder)

1. Chọn chi tiết hằng số điều khiển giữ tâm hành lang & điều kiện chuyển trạng thái trong `advance_segment` / `cross_dynamic_gate` / `dock_target`, đồng thời tách các hàm hỗ trợ chung (như `classify_front_obstacle`, `compute_open_branches`, `side_shaft_tuck`) trong [robot_sim/skills.py](file:///C:/Users/latuan/Programming/robot-demo/robot_sim/skills.py) để không trùng lặp logic với `explore_maze` và `approach_target`.
2. Xây dựng bộ test đơn vị & tích hợp mới (mock HTTP + MuJoCo headless) cho:
   - Từng điều kiện ngắt của `advance_segment`, `cross_dynamic_gate`, `dock_target` (bảo đảm stateless: không giữ bộ nhớ giữa các lần gọi).
   - Các chỉ số telemetry mới (`active_drive_time_s`, `agent_think_time_s`, `command_burst_count`, `backtrack_revisit_count`, `redundant_revisit_count`, `path_efficiency`, reset sạch khi `reset_jumped`) và giữ 100% pass cho toàn bộ 94 test cũ.
3. Nghiệm thu thực tế subagent `robot-driver-brain` với blackbox telemetry 10 Hz trên các bản đồ chuẩn (`2wheel`, `maze_b` / `maze_c`, `dynamic_a`, `dynamic_b`) và lập bảng đối chiếu giữa 2 chế độ **`auto-skill`** vs **`agent-brain`**.
