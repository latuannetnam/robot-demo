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

_ALLOWED_WORLDBODY_GEOMS = {"floor", *_EXPECTED_WALLS.keys()}
_ALLOWED_WORLDBODY_BODIES = {*_EXPECTED_OBSTACLES.keys(), "target", "robot"}



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


def _robot_subtree_body_ids(mj_model: mujoco.MjModel, robot_body_id: int) -> set[int]:
    body_ids = {robot_body_id}
    changed = True
    while changed:
        changed = False
        for bid in range(mj_model.nbody):
            if bid not in body_ids and int(mj_model.body_parentid[bid]) in body_ids:
                body_ids.add(bid)
                changed = True
    return body_ids


def _geom_world_half_extents(mj_model: mujoco.MjModel, mj_data: mujoco.MjData, gid: int) -> tuple[float, float, float]:
    gtype = int(mj_model.geom_type[gid])
    if gtype in (int(mujoco.mjtGeom.mjGEOM_PLANE), int(mujoco.mjtGeom.mjGEOM_HFIELD)):
        gname = mujoco.mj_id2name(mj_model, mujoco.mjtObj.mjOBJ_GEOM, gid) or f"geom#{gid}"
        raise ValueError(f"Vi phạm giới hạn cơ khí Sim-to-Real: không cho phép kiểu hình học vô hạn/địa hình trên robot ('{gname}').")

    size = mj_model.geom_size[gid]
    rmat = mj_data.geom_xmat[gid].reshape(3, 3)

    if gtype == int(mujoco.mjtGeom.mjGEOM_SPHERE):
        r = float(size[0])
        return r, r, r
    if gtype in (int(mujoco.mjtGeom.mjGEOM_BOX), int(mujoco.mjtGeom.mjGEOM_ELLIPSOID)):
        sx, sy, sz = float(size[0]), float(size[1]), float(size[2])
        return tuple(
            abs(float(rmat[axis, 0])) * sx + abs(float(rmat[axis, 1])) * sy + abs(float(rmat[axis, 2])) * sz
            for axis in range(3)
        )
    if gtype == int(mujoco.mjtGeom.mjGEOM_CYLINDER):
        r, h = float(size[0]), float(size[1])
        return tuple(
            r * math.sqrt(max(0.0, 1.0 - float(rmat[axis, 2]) ** 2)) + h * abs(float(rmat[axis, 2]))
            for axis in range(3)
        )
    if gtype == int(mujoco.mjtGeom.mjGEOM_CAPSULE):
        r, h = float(size[0]), float(size[1])
        return tuple(h * abs(float(rmat[axis, 2])) + r for axis in range(3))

    rb = float(mj_model.geom_rbound[gid])
    return rb, rb, rb


def _check_robot_compiled_geom_extents(mj_model: mujoco.MjModel, robot_body_id: int) -> None:
    mj_data = mujoco.MjData(mj_model)
    mujoco.mj_forward(mj_model, mj_data)
    robot_bodies = _robot_subtree_body_ids(mj_model, robot_body_id)

    for gid in range(mj_model.ngeom):
        if int(mj_model.geom_bodyid[gid]) not in robot_bodies:
            continue
        gname = mujoco.mj_id2name(mj_model, mujoco.mjtObj.mjOBJ_GEOM, gid) or f"geom#{gid}"
        xpos = mj_data.geom_xpos[gid]
        hx, hy, hz = _geom_world_half_extents(mj_model, mj_data, gid)
        if (
            abs(float(xpos[0])) + hx > 0.25 + 1e-4
            or abs(float(xpos[1])) + hy > 0.20 + 1e-4
            or float(xpos[2]) - hz < -0.002
            or float(xpos[2]) + hz > 0.30 + 1e-4
        ):
            raise ValueError(
                f"Vi phạm giới hạn cơ khí Sim-to-Real: kích thước hình học '{gname}' "
                f"(tâm={tuple(round(float(v), 4) for v in xpos)}, bán kính mở rộng={(round(hx, 4), round(hy, 4), round(hz, 4))}) "
                f"vượt ngoài khung robot cho phép (|x|<=0.25, |y|<=0.20, 0<=z<=0.30m)."
            )



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

    # 0. Whitelist top-level worldbody elements so unauthorized bodies/geoms cannot be injected
    seen_geoms: set[str] = set()
    seen_bodies: set[str] = set()
    for child in worldbody:
        if child.tag in ("light", "camera"):
            continue
        if child.tag == "geom":
            gname = child.get("name")
            if gname not in _ALLOWED_WORLDBODY_GEOMS or gname in seen_geoms:
                raise ValueError(f"Vi phạm toàn vẹn sa bàn: phát hiện geom không được phép '{gname}' trong worldbody.")
            seen_geoms.add(gname)
        elif child.tag == "body":
            bname = child.get("name")
            if bname not in _ALLOWED_WORLDBODY_BODIES or bname in seen_bodies:
                raise ValueError(f"Vi phạm toàn vẹn sa bàn: phát hiện body không được phép '{bname}' trong worldbody.")
            seen_bodies.add(bname)
        else:
            raise ValueError(f"Vi phạm toàn vẹn sa bàn: thẻ '<{child.tag}>' không được phép trực tiếp trong worldbody.")


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
        if len(list(obs_body)) != 1:
            raise ValueError(f"Vi phạm toàn vẹn sa bàn: '{body_name}' không được chứa phần tử phụ.")

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
    if len(list(target_body)) != 1:
        raise ValueError("Vi phạm toàn vẹn sa bàn: 'target' không được chứa phần tử phụ.")

    # 5. Robot body & Sim-to-Real physical envelope
    robot_body = worldbody.find("./body[@name='robot']")
    if robot_body is None:
        raise ValueError("Mô hình thiếu <body name='robot'>.")
    _assert_vec_close(_parse_vec(robot_body.get("pos", "0 0 0"), 3, "robot.pos"), [0.0, 0.0, 0.0], "robot.pos")
    if robot_body.find("./freejoint") is None:
        raise ValueError("Robot bắt buộc phải di chuyển vật lý qua <freejoint>.")
    if robot_body.find(".//camera[@name='front_cam']") is None:
        raise ValueError("Camera 'front_cam' bắt buộc phải gắn trên thân 'robot'.")
    _check_robot_bounds(robot_body)

    # 6. MuJoCo compilation + required sensors/actuators/cameras + 3D geom extents
    try:
        mj_model = mujoco.MjModel.from_xml_path(str(resolved))
    except Exception as exc:
        raise ValueError(f"MuJoCo không biên dịch được mô hình '{resolved}': {exc}") from exc

    robot_body_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_BODY, "robot")
    if robot_body_id < 0:
        raise ValueError("Mô hình biên dịch thiếu body 'robot'.")
    _check_robot_compiled_geom_extents(mj_model, robot_body_id)


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
