from __future__ import annotations

import math
from pathlib import Path
import mujoco
import numpy as np
from PIL import Image

from robot_sim.integrity import verify_arena_integrity

WHEEL_RADIUS = 0.05
TRACK_WIDTH = 0.23
MAX_WHEEL_RAD_S = 15.0
MAX_COMMAND_DURATION = 10.0
RANGEFINDER_MAX = 3.0
ARENA_SAFE_LIMIT = 1.7
DEFAULT_CAMERA_WIDTH = 320
DEFAULT_CAMERA_HEIGHT = 240
MAX_CAMERA_WIDTH = 640
MAX_CAMERA_HEIGHT = 480
DEFAULT_JPEG_QUALITY = 75

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL_PATH = PROJECT_ROOT / "models" / "arena_2wheel.xml"
DEFAULT_CAPTURES_DIR = PROJECT_ROOT / "captures"


def _require_finite(**kwargs: float) -> None:
    for name, val in kwargs.items():
        if isinstance(val, bool) or not isinstance(val, (int, float)) or not math.isfinite(float(val)):
            raise ValueError(f"Tham số '{name}' phải là số hữu hạn hợp lệ (nhận được: {val!r}).")


def compute_move_command(meters: float, speed_m_s: float = 0.25) -> tuple[float, float, float]:
    """Tính vận tốc góc 2 bánh (rad/s) và thời gian (s) để tiến/lùi một quãng đường ước lượng."""
    _require_finite(meters=meters, speed_m_s=speed_m_s)
    if speed_m_s <= 0.0:
        raise ValueError("Tốc độ di chuyển 'speed_m_s' phải lớn hơn 0.")
    if abs(meters) < 1e-6:
        raise ValueError("Quãng đường 'meters' phải khác 0.")

    omega = min(abs(speed_m_s) / WHEEL_RADIUS, MAX_WHEEL_RAD_S)
    effective_speed = omega * WHEEL_RADIUS
    duration = min(abs(meters) / effective_speed, MAX_COMMAND_DURATION)
    direction = 1.0 if meters > 0.0 else -1.0
    return direction * omega, direction * omega, duration


def compute_turn_command(degrees: float, angular_deg_s: float = 60.0) -> tuple[float, float, float]:
    """Tính vận tốc góc 2 bánh (rad/s) và thời gian (s) để xoay tại chỗ (+degrees = quay trái ngược chiều kim đồng hồ)."""
    _require_finite(degrees=degrees, angular_deg_s=angular_deg_s)
    if angular_deg_s <= 0.0:
        raise ValueError("Tốc độ góc 'angular_deg_s' phải lớn hơn 0.")
    if abs(degrees) < 1e-6:
        raise ValueError("Góc xoay 'degrees' phải khác 0.")

    target_yaw_rate = math.radians(abs(angular_deg_s))
    omega = min((target_yaw_rate * TRACK_WIDTH) / (2.0 * WHEEL_RADIUS), MAX_WHEEL_RAD_S)
    effective_yaw_rate = (2.0 * WHEEL_RADIUS * omega) / TRACK_WIDTH
    duration = min(math.radians(abs(degrees)) / effective_yaw_rate, MAX_COMMAND_DURATION)

    if degrees > 0.0:
        # Quay trái (CCW): bánh phải tiến (+), bánh trái lùi (-)
        return -omega, omega, duration
    return omega, -omega, duration


class RobotSimulation:
    """Lõi mô phỏng vật lý MuJoCo cho xe robot 2 bánh vi sai."""

    def __init__(self, model_path: Path | str | None = None) -> None:
        resolved = Path(model_path) if model_path is not None else DEFAULT_MODEL_PATH
        self.model_path = resolved.resolve()
        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.data = mujoco.MjData(self.model)
        self._default_target_pos = self.data.mocap_pos[0].copy()
        self._command_deadline: float | None = None
        self._renderer: mujoco.Renderer | None = None
        self._renderer_size: tuple[int, int] | None = None
        self._clean_vis_opt = mujoco.MjvOption()
        self._clean_vis_opt.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = 0

        self._sensor_adr = {
            name: int(self.model.sensor_adr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, name)])
            for name in (
                "rf_left",
                "rf_center",
                "rf_right",
                "left_wheel_vel",
                "right_wheel_vel",
                "chassis_vel",
                "chassis_gyro",
            )
        }
        mujoco.mj_forward(self.model, self.data)

    def close(self) -> None:
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
            self._renderer_size = None

    def reload(self, model_path: Path | str | None = None, preserve_target: bool = True) -> dict:
        """Kiểm định tính toàn vẹn sa bàn và nạp lại mô hình vật lý an toàn (giữ nguyên mục tiêu nếu cần)."""
        if not isinstance(preserve_target, bool):
            raise ValueError("Tham số 'preserve_target' phải là kiểu boolean (true/false).")
        resolved = (Path(model_path) if model_path is not None else self.model_path).resolve()
        verify_arena_integrity(resolved)

        prev_target_pos = self.data.mocap_pos[0].copy()
        new_model = mujoco.MjModel.from_xml_path(str(resolved))
        new_data = mujoco.MjData(new_model)
        new_sensor_adr = {
            name: int(new_model.sensor_adr[mujoco.mj_name2id(new_model, mujoco.mjtObj.mjOBJ_SENSOR, name)])
            for name in (
                "rf_left",
                "rf_center",
                "rf_right",
                "left_wheel_vel",
                "right_wheel_vel",
                "chassis_vel",
                "chassis_gyro",
            )
        }

        self.close()
        self.model_path = resolved
        self.model = new_model
        self.data = new_data
        self._default_target_pos = self.data.mocap_pos[0].copy()
        self._sensor_adr = new_sensor_adr
        self._command_deadline = None
        if preserve_target:
            self.data.mocap_pos[0] = [
                float(prev_target_pos[0]),
                float(prev_target_pos[1]),
                float(self._default_target_pos[2]),
            ]
        mujoco.mj_forward(self.model, self.data)
        return self.get_status()

    def _get_renderer(self, width: int, height: int) -> mujoco.Renderer:
        if self._renderer is not None and self._renderer_size != (width, height):
            self._renderer.close()
            self._renderer = None
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, height=height, width=width)
            self._renderer_size = (width, height)
        return self._renderer

    def set_wheels(self, left_rad_s: float, right_rad_s: float, duration: float = 1.0) -> dict:
        _require_finite(left_rad_s=left_rad_s, right_rad_s=right_rad_s, duration=duration)
        if duration <= 0.0:
            raise ValueError("Thời gian thực thi 'duration' phải lớn hơn 0 giây.")

        clamped_left = max(-MAX_WHEEL_RAD_S, min(MAX_WHEEL_RAD_S, float(left_rad_s)))
        clamped_right = max(-MAX_WHEEL_RAD_S, min(MAX_WHEEL_RAD_S, float(right_rad_s)))
        capped_duration = min(float(duration), MAX_COMMAND_DURATION)

        self.data.ctrl[0] = clamped_left
        self.data.ctrl[1] = clamped_right
        self._command_deadline = float(self.data.time) + capped_duration
        return {
            "left_rad_s": round(clamped_left, 4),
            "right_rad_s": round(clamped_right, 4),
            "duration_s": round(capped_duration, 4),
        }

    def move(self, meters: float, speed_m_s: float = 0.25) -> dict:
        left, right, duration = compute_move_command(meters=meters, speed_m_s=speed_m_s)
        cmd = self.set_wheels(left, right, duration=duration)
        cmd["requested_meters"] = round(float(meters), 4)
        return cmd

    def turn(self, degrees: float, angular_deg_s: float = 60.0) -> dict:
        left, right, duration = compute_turn_command(degrees=degrees, angular_deg_s=angular_deg_s)
        cmd = self.set_wheels(left, right, duration=duration)
        cmd["requested_degrees"] = round(float(degrees), 4)
        return cmd

    def stop(self) -> dict:
        self.data.ctrl[0] = 0.0
        self.data.ctrl[1] = 0.0
        self._command_deadline = None
        return {"stopped": True, "sim_time": round(float(self.data.time), 4)}

    def reset(self, target_xy: tuple[float, float] | list[float] | None = None) -> dict:
        if target_xy is not None:
            if not isinstance(target_xy, (list, tuple)) or len(target_xy) != 2:
                raise ValueError("'target_xy' phải là danh sách gồm đúng 2 số [x, y].")
            _require_finite(target_x=target_xy[0], target_y=target_xy[1])
            tx, ty = float(target_xy[0]), float(target_xy[1])
            if abs(tx) > ARENA_SAFE_LIMIT or abs(ty) > ARENA_SAFE_LIMIT:
                raise ValueError(
                    f"Tọa độ mục tiêu ({tx}, {ty}) nằm ngoài vùng an toàn của sân [-{ARENA_SAFE_LIMIT}, {ARENA_SAFE_LIMIT}]m."
                )
        else:
            tx, ty = float(self._default_target_pos[0]), float(self._default_target_pos[1])

        mujoco.mj_resetData(self.model, self.data)
        self.data.mocap_pos[0] = [tx, ty, float(self._default_target_pos[2])]
        self._command_deadline = None
        mujoco.mj_forward(self.model, self.data)
        return self.get_status()

    def step_once(self) -> None:
        if self._command_deadline is not None and self.data.time >= self._command_deadline:
            self.stop()
        mujoco.mj_step(self.model, self.data)
        if self._command_deadline is not None and self.data.time >= self._command_deadline:
            self.stop()

    def step_seconds(self, seconds: float) -> None:
        _require_finite(seconds=seconds)
        if seconds < 0.0:
            raise ValueError("Số giây mô phỏng không được âm.")
        steps = int(round(seconds / self.model.opt.timestep))
        for _ in range(steps):
            self.step_once()

    def _read_rangefinder(self, sensor_name: str) -> tuple[float, bool]:
        raw = float(self.data.sensordata[self._sensor_adr[sensor_name]])
        if raw < 0.0 or raw >= RANGEFINDER_MAX - 1e-6:
            return RANGEFINDER_MAX, False
        return round(raw, 4), True

    def get_status(self, include_vision: bool = False) -> dict:
        mujoco.mj_forward(self.model, self.data)
        rf_l, hit_l = self._read_rangefinder("rf_left")
        rf_c, hit_c = self._read_rangefinder("rf_center")
        rf_r, hit_r = self._read_rangefinder("rf_right")

        left_vel = float(self.data.sensordata[self._sensor_adr["left_wheel_vel"]])
        right_vel = float(self.data.sensordata[self._sensor_adr["right_wheel_vel"]])
        forward_speed = float(self.data.sensordata[self._sensor_adr["chassis_vel"]])
        yaw_rate = float(self.data.sensordata[self._sensor_adr["chassis_gyro"] + 2])

        quat = self.data.qpos[3:7]
        yaw_rad = float(
            np.arctan2(
                2.0 * (quat[0] * quat[3] + quat[1] * quat[2]),
                1.0 - 2.0 * (quat[2] ** 2 + quat[3] ** 2),
            )
        )
        remaining = (
            max(0.0, float(self._command_deadline - self.data.time))
            if self._command_deadline is not None
            else 0.0
        )

        result = {
            "sim_time": round(float(self.data.time), 4),
            "moving": remaining > 0.0,
            "command_remaining_s": round(remaining, 4),
            "sensors": {
                "rangefinders_m": {"left": rf_l, "center": rf_c, "right": rf_r},
                "rangefinder_hits": {"left": hit_l, "center": hit_c, "right": hit_r},
                "wheel_vel_rad_s": {"left": round(left_vel, 4), "right": round(right_vel, 4)},
                "forward_speed_m_s": round(forward_speed, 4),
                "yaw_rate_rad_s": round(yaw_rate, 4),
            },
            "debug": {
                "robot_xy": [round(float(self.data.qpos[0]), 4), round(float(self.data.qpos[1]), 4)],
                "robot_yaw_deg": round(math.degrees(yaw_rad), 2),
                "target_xy": [
                    round(float(self.data.mocap_pos[0][0]), 4),
                    round(float(self.data.mocap_pos[0][1]), 4),
                ],
            },
        }
        if include_vision:
            result["vision"] = self.analyze_vision("front_cam")
        return result

    def _validate_camera_name(self, camera_name: str) -> None:
        if not isinstance(camera_name, str) or not camera_name.strip():
            raise ValueError("Tên camera phải là chuỗi hợp lệ ('front_cam' hoặc 'overview_cam').")
        cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
        if cam_id < 0:
            raise ValueError(f"Không tìm thấy camera '{camera_name}'. Chọn 'front_cam' hoặc 'overview_cam'.")

    @staticmethod
    def _summarize_mask(mask: np.ndarray, min_area_pct: float = 0.12) -> dict | None:
        total = mask.shape[0] * mask.shape[1]
        count = int(mask.sum())
        area_pct = (100.0 * count) / float(total)
        if area_pct < min_area_pct:
            return None
        xs = np.where(mask)[1]
        half_w = mask.shape[1] / 2.0
        offset_x = (float(xs.mean()) - half_w) / half_w
        if offset_x < -0.20:
            side = "left"
        elif offset_x > 0.20:
            side = "right"
        else:
            side = "center"
        return {
            "offset_x": round(offset_x, 3),
            "side": side,
            "area_pct": round(area_pct, 2),
        }

    def analyze_vision(self, camera_name: str = "front_cam") -> dict:
        """Phân tích trực tiếp khung hình camera trong RAM (không ghi đĩa) để trích xuất đích xanh & vật cản."""
        self._validate_camera_name(camera_name)
        width, height = DEFAULT_CAMERA_WIDTH, DEFAULT_CAMERA_HEIGHT
        mujoco.mj_forward(self.model, self.data)
        renderer = self._get_renderer(width=width, height=height)
        renderer.update_scene(self.data, camera=camera_name, scene_option=self._clean_vis_opt)
        arr = renderer.render().astype(np.int16)

        r, g, b = arr[:, :, 0], arr[:, :, 1], arr[:, :, 2]
        green_mask = (g > 110) & ((g - r) > 50) & ((g - b) > 45)
        red_mask = (r > 110) & ((r - g) > 45) & ((r - b) > 45)
        blue_mask = (b > 95) & (r < 120) & ((b - r) > 50) & ((b - g) > 25)
        yellow_mask = (r > 115) & (g > 90) & (b < 85) & ((r - b) > 60) & ((g - b) > 45)

        target_info = self._summarize_mask(green_mask)
        obstacles: list[dict] = []
        for color_name, c_mask in (("yellow", yellow_mask), ("red", red_mask), ("blue", blue_mask)):
            summary = self._summarize_mask(c_mask)
            if summary is not None:
                obstacles.append({"color": color_name, **summary})
        obstacles.sort(key=lambda item: item["area_pct"], reverse=True)

        return {
            "camera": camera_name,
            "width": width,
            "height": height,
            "target_visible": target_info is not None,
            "target": target_info,
            "obstacles": obstacles,
        }

    def capture_camera(
        self,
        camera_name: str = "front_cam",
        out_path: Path | str | None = None,
        width: int = DEFAULT_CAMERA_WIDTH,
        height: int = DEFAULT_CAMERA_HEIGHT,
        img_format: str | None = None,
        quality: int = DEFAULT_JPEG_QUALITY,
    ) -> dict:
        self._validate_camera_name(camera_name)
        if isinstance(width, bool) or not isinstance(width, int) or not (64 <= width <= MAX_CAMERA_WIDTH):
            raise ValueError(f"Chiều rộng ảnh 'width' phải là số nguyên trong [64, {MAX_CAMERA_WIDTH}].")
        if isinstance(height, bool) or not isinstance(height, int) or not (48 <= height <= MAX_CAMERA_HEIGHT):
            raise ValueError(f"Chiều cao ảnh 'height' phải là số nguyên trong [48, {MAX_CAMERA_HEIGHT}].")
        if isinstance(quality, bool) or not isinstance(quality, int) or not (10 <= quality <= 95):
            raise ValueError("Chất lượng nén JPEG 'quality' phải là số nguyên trong [10, 95].")

        fmt: str
        if img_format is not None:
            if not isinstance(img_format, str):
                raise ValueError("Định dạng 'format' phải là 'jpg' hoặc 'png'.")
            norm_fmt = img_format.strip().lower()
            if norm_fmt in ("jpg", "jpeg"):
                fmt = "JPEG"
            elif norm_fmt == "png":
                fmt = "PNG"
            else:
                raise ValueError(f"Định dạng ảnh '{img_format}' không hỗ trợ (chọn 'jpg' hoặc 'png').")
        elif out_path is not None and str(out_path).strip().lower().endswith(".png"):
            fmt = "PNG"
        else:
            fmt = "JPEG"

        ext = "png" if fmt == "PNG" else "jpg"
        if out_path is not None:
            if not isinstance(out_path, (str, Path)) or (isinstance(out_path, str) and not out_path.strip()):
                raise ValueError("Đường dẫn lưu ảnh 'path' phải là chuỗi tên file hợp lệ.")
            target_path = Path(out_path).resolve()
        else:
            target_path = (DEFAULT_CAPTURES_DIR / f"{camera_name}.{ext}").resolve()

        if target_path.exists() and target_path.is_dir():
            raise ValueError(f"Đường dẫn '{target_path}' là thư mục; vui lòng truyền đường dẫn tới file .{ext}.")
        target_path.parent.mkdir(parents=True, exist_ok=True)

        mujoco.mj_forward(self.model, self.data)
        renderer = self._get_renderer(width=width, height=height)
        renderer.update_scene(self.data, camera=camera_name)
        rgb = renderer.render()

        pil_img = Image.fromarray(rgb)
        if fmt == "JPEG":
            pil_img.save(target_path, format="JPEG", quality=quality, optimize=True)
        else:
            pil_img.save(target_path, format="PNG")

        return {
            "camera": camera_name,
            "path": str(target_path),
            "width": width,
            "height": height,
            "format": fmt,
            "bytes": int(target_path.stat().st_size),
        }
