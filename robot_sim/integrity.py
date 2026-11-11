from __future__ import annotations

import math
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco

DEFAULT_MODEL_PATH = Path(__file__).resolve().parents[1] / "models" / "arena_2wheel.xml"

_EXPECTED_WALLS = {
    "wall_north": ([0.0, 2.025, 0.125], [2.05, 0.025, 0.125]),
    "wall_south": ([0.0, -2.025, 0.125], [2.05, 0.025, 0.125]),
    "wall_east": ([2.025, 0.0, 0.125], [0.025, 2.00, 0.125]),
    "wall_west": ([-2.025, 0.0, 0.125], [0.025, 2.00, 0.125]),
}

_EXPECTED_OBSTACLES = {
    "obs_red": ([0.70, 0.95, 0.12], "obs_red_geom", "box", [0.12, 0.12, 0.12], [0.90, 0.25, 0.20, 1.0]),
    "obs_blue": ([0.85, -0.75, 0.12], "obs_blue_geom", "box", [0.12, 0.12, 0.12], [0.20, 0.48, 0.92, 1.0]),
    "obs_yellow": ([-0.85, 0.75, 0.12], "obs_yellow_geom", "cylinder", [0.12, 0.12], [0.95, 0.78, 0.15, 1.0]),
}

_REQUIRED_SENSORS = (
    "rf_left",
    "rf_center",
    "rf_right",
    "left_wheel_vel",
    "right_wheel_vel",
    "chassis_vel",
    "chassis_gyro",
)


def _parse_vec(raw: str | None, expected_len: int, label: str) -> list[float]:
    if raw is None:
        raise ValueError(f"Vi phạm toàn vẹn sa bàn: thiếu thuộc tính của '{label}'.")
    parts = raw.strip().split()
    if len(parts) != expected_len:
        raise ValueError(f"Vi phạm toàn vẹn sa bàn: '{label}' cần {expected_len} số, nhận được '{raw}'.")
    try:
        vals = [float(x) for x in parts]
    except ValueError as exc:
        raise ValueError(f"Vi phạm toàn vẹn sa bàn: giá trị số không hợp lệ ở '{label}' ('{raw}').") from exc
    if not all(math.isfinite(v) for v in vals):
        raise ValueError(f"Vi phạm toàn vẹn sa bàn: giá trị không hữu hạn ở '{label}'.")
    return vals


def _assert_vec_close(actual: list[float], expected: list[float], label: str, tol: float = 1e-4) -> None:
    if len(actual) != len(expected) or any(abs(a - b) > tol for a, b in zip(actual, expected)):
        raise ValueError(
            f"Vi phạm toàn vẹn sa bàn: không được phép thay đổi '{label}' (nhận {actual}, chuẩn {expected})."
        )


def _assert_collision_enabled(geom_elem: ET.Element, label: str) -> None:
    for attr in ("contype", "conaffinity"):
        val = geom_elem.get(attr)
        if val is not None and int(val) == 0:
            raise ValueError(f"Vi phạm toàn vẹn sa bàn: không được tắt va chạm '{attr}' của '{label}'.")


def _check_robot_bounds(elem: ET.Element, parent_pos: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> None:
    for child in elem:
        pos_str = child.get("pos")
        if pos_str is not None:
            local = _parse_vec(pos_str, 3, f"robot/{child.get('name', child.tag)}.pos")
            cum = (parent_pos[0] + local[0], parent_pos[1] + local[1], parent_pos[2] + local[2])
        else:
            cum = parent_pos

        if child.tag in ("geom", "site", "camera", "body"):
            if abs(cum[0]) > 0.25 or abs(cum[1]) > 0.20 or cum[2] < -1e-4 or cum[2] > 0.30:
                raise ValueError(
                    f"Vi phạm giới hạn cơ khí Sim-to-Real: phần tử '{child.get('name', child.tag)}' "
                    f"nằm tại tọa độ cục bộ {cum} vượt ngoài khung robot cho phép (|x|<=0.25, |y|<=0.20, 0<=z<=0.30m)."
                )
        if child.tag == "body":
            _check_robot_bounds(child, cum)


def verify_arena_integrity(model_path: Path | str | None = None) -> dict:
    """Kiểm tra chống gian lận đề bài sa bàn và đảm bảo giới hạn vật lý hợp lệ của thân xe robot."""
    resolved = (Path(model_path) if model_path is not None else DEFAULT_MODEL_PATH).resolve()
    if not resolved.is_file():
        raise ValueError(f"Không tìm thấy file mô hình XML tại '{resolved}'.")

    try:
        tree = ET.parse(resolved)
    except (ET.ParseError, OSError) as exc:
        raise ValueError(f"File XML mô hình không hợp lệ ('{resolved}'): {exc}") from exc

    root = tree.getroot()
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("File XML thiếu thẻ <worldbody>.")

    # 1. Floor
    floor = worldbody.find("./geom[@name='floor']")
    if floor is None or floor.get("type") != "plane":
        raise ValueError("Vi phạm toàn vẹn sa bàn: thiếu hoặc sai kiểu mặt sàn 'floor'.")
    _assert_vec_close(_parse_vec(floor.get("size"), 3, "floor.size"), [2.0, 2.0, 0.05], "floor.size")
    _assert_collision_enabled(floor, "floor")

    # 2. Four arena walls
    for wall_name, (exp_pos, exp_size) in _EXPECTED_WALLS.items():
        wall = worldbody.find(f"./geom[@name='{wall_name}']")
        if wall is None or wall.get("type") != "box":
            raise ValueError(f"Vi phạm toàn vẹn sa bàn: thiếu hoặc sai kiểu tường '{wall_name}'.")
        _assert_vec_close(_parse_vec(wall.get("pos"), 3, f"{wall_name}.pos"), exp_pos, f"{wall_name}.pos")
        _assert_vec_close(_parse_vec(wall.get("size"), 3, f"{wall_name}.size"), exp_size, f"{wall_name}.size")
        _assert_collision_enabled(wall, wall_name)

    # 3. Three colored obstacles
    for body_name, (exp_pos, geom_name, exp_type, exp_size, exp_rgba) in _EXPECTED_OBSTACLES.items():
        obs_body = worldbody.find(f"./body[@name='{body_name}']")
        if obs_body is None:
            raise ValueError(f"Vi phạm toàn vẹn sa bàn: thiếu vật cản '{body_name}'.")
        _assert_vec_close(_parse_vec(obs_body.get("pos"), 3, f"{body_name}.pos"), exp_pos, f"{body_name}.pos")
        obs_geom = obs_body.find(f"./geom[@name='{geom_name}']")
        if obs_geom is None or obs_geom.get("type") != exp_type:
            raise ValueError(f"Vi phạm toàn vẹn sa bàn: thiếu hoặc sai kiểu hình học '{geom_name}'.")
        _assert_vec_close(
            _parse_vec(obs_geom.get("size"), len(exp_size), f"{geom_name}.size"),
            exp_size,
            f"{geom_name}.size",
        )
        _assert_vec_close(
            _parse_vec(obs_geom.get("rgba"), 4, f"{geom_name}.rgba"),
            exp_rgba,
            f"{geom_name}.rgba",
            tol=0.05,
        )
        _assert_collision_enabled(obs_geom, geom_name)

    # 4. Green mocap target
    target_body = worldbody.find("./body[@name='target']")
    if target_body is None or target_body.get("mocap") != "true":
        raise ValueError("Vi phạm toàn vẹn sa bàn: thiếu mục tiêu mocap 'target'.")
    target_geom = target_body.find("./geom[@name='target_geom']")
    if target_geom is None or target_geom.get("type") != "cylinder":
        raise ValueError("Vi phạm toàn vẹn sa bàn: thiếu hoặc sai kiểu 'target_geom'.")
    _assert_vec_close(_parse_vec(target_geom.get("size"), 2, "target_geom.size"), [0.10, 0.12], "target_geom.size")
    _assert_vec_close(
        _parse_vec(target_geom.get("rgba"), 4, "target_geom.rgba"),
        [0.10, 0.90, 0.20, 1.0],
        "target_geom.rgba",
        tol=0.05,
    )
    if target_geom.get("contype") != "1" or target_geom.get("conaffinity") != "1":
        raise ValueError("Vi phạm toàn vẹn sa bàn: 'target_geom' phải bật contype='1' và conaffinity='1'.")

    # 5. Robot body & Sim-to-Real physical envelope
    robot_body = worldbody.find("./body[@name='robot']")
    if robot_body is None:
        raise ValueError("Mô hình thiếu <body name='robot'>.")
    if robot_body.find("./freejoint") is None:
        raise ValueError("Robot bắt buộc phải di chuyển vật lý qua <freejoint>.")
    if robot_body.find(".//camera[@name='front_cam']") is None:
        raise ValueError("Camera 'front_cam' bắt buộc phải gắn trên thân 'robot'.")
    _check_robot_bounds(robot_body)

    # 6. MuJoCo compilation + required sensors/actuators/cameras
    try:
        mj_model = mujoco.MjModel.from_xml_path(str(resolved))
    except Exception as exc:
        raise ValueError(f"MuJoCo không biên dịch được mô hình '{resolved}': {exc}") from exc

    for cam in ("front_cam", "overview_cam"):
        if mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_CAMERA, cam) < 0:
            raise ValueError(f"Mô hình thiếu camera bắt buộc '{cam}'.")
    for act in ("left_motor", "right_motor"):
        if mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_ACTUATOR, act) < 0:
            raise ValueError(f"Mô hình thiếu động cơ bắt buộc '{act}'.")
    for sensor in _REQUIRED_SENSORS:
        if mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_SENSOR, sensor) < 0:
            raise ValueError(f"Mô hình thiếu cảm biến bắt buộc '{sensor}'.")

    return {"valid": True, "model_path": str(resolved)}
