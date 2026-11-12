from __future__ import annotations

import math
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco
import numpy as np

from robot_sim.arenas import (
    ARENAS,
    MODEL_NAME_TO_ARENA_ID,
    MODELS_DIR,
    ROBOT_INCLUDE_FILENAME,
)

DEFAULT_MODEL_PATH = MODELS_DIR / ARENAS["2wheel"].filename

_REQUIRED_SENSORS = (
    "rf_left",
    "rf_center",
    "rf_right",
    "left_wheel_vel",
    "right_wheel_vel",
    "chassis_vel",
    "chassis_gyro",
)

_ALLOWED_ROOT_TAGS = {
    "compiler",
    "option",
    "visual",
    "asset",
    "default",
    "worldbody",
    "include",
    "actuator",
    "sensor",
}
_ALLOWED_INCLUDE_ROOT_TAGS = {"default", "worldbody", "actuator", "sensor"}
_ROTATION_ATTRS = ("euler", "quat", "axisangle", "xyaxes", "zaxis")


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


def _assert_no_rotation_attrs(elem: ET.Element, label: str, *, allow_euler: bool = False) -> None:
    for attr in _ROTATION_ATTRS:
        if allow_euler and attr == "euler":
            continue
        if elem.get(attr) is not None:
            raise ValueError(
                f"Vi phạm toàn vẹn sa bàn: không được phép gắn thuộc tính xoay '{attr}' trên '{label}'."
            )


def _assert_collision_enabled(geom_elem: ET.Element, label: str) -> None:
    for attr in ("contype", "conaffinity"):
        val = geom_elem.get(attr)
        if val is not None:
            try:
                parsed = int(val)
            except ValueError as exc:
                raise ValueError(f"Vi phạm toàn vẹn sa bàn: '{attr}' không hợp lệ ở '{label}'.") from exc
            if parsed == 0:
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


def _geom_half_extents(mj_model: mujoco.MjModel, gid: int, rmat: np.ndarray) -> tuple[float, float, float]:
    gtype = int(mj_model.geom_type[gid])
    if gtype in (int(mujoco.mjtGeom.mjGEOM_PLANE), int(mujoco.mjtGeom.mjGEOM_HFIELD)):
        gname = mujoco.mj_id2name(mj_model, mujoco.mjtObj.mjOBJ_GEOM, gid) or f"geom#{gid}"
        raise ValueError(
            f"Vi phạm giới hạn cơ khí Sim-to-Real: không cho phép kiểu hình học vô hạn/địa hình trên robot ('{gname}')."
        )

    size = mj_model.geom_size[gid]

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
    spawn_site_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_SITE, "robot_spawn")
    if spawn_site_id >= 0 and int(mj_model.body_jntnum[robot_body_id]) >= 1:
        jnt_id = int(mj_model.body_jntadr[robot_body_id])
        if int(mj_model.jnt_type[jnt_id]) == int(mujoco.mjtJoint.mjJNT_FREE):
            qadr = int(mj_model.jnt_qposadr[jnt_id])
            mj_data.qpos[qadr : qadr + 2] = mj_model.site_pos[spawn_site_id][:2]
            mj_data.qpos[qadr + 3 : qadr + 7] = mj_model.site_quat[spawn_site_id]

    mujoco.mj_forward(mj_model, mj_data)
    robot_bodies = _robot_subtree_body_ids(mj_model, robot_body_id)
    robot_origin = mj_data.xpos[robot_body_id]
    robot_rmat = mj_data.xmat[robot_body_id].reshape(3, 3)

    for gid in range(mj_model.ngeom):
        if int(mj_model.geom_bodyid[gid]) not in robot_bodies:
            continue
        gname = mujoco.mj_id2name(mj_model, mujoco.mjtObj.mjOBJ_GEOM, gid) or f"geom#{gid}"
        rel_pos = robot_rmat.T @ (mj_data.geom_xpos[gid] - robot_origin)
        rel_rmat = robot_rmat.T @ mj_data.geom_xmat[gid].reshape(3, 3)
        hx, hy, hz = _geom_half_extents(mj_model, gid, rel_rmat)
        dx, dy, dz = float(rel_pos[0]), float(rel_pos[1]), float(rel_pos[2])
        if (
            abs(dx) + hx > 0.25 + 1e-4
            or abs(dy) + hy > 0.20 + 1e-4
            or dz - hz < -0.002
            or dz + hz > 0.30 + 1e-4
        ):
            raise ValueError(
                f"Vi phạm giới hạn cơ khí Sim-to-Real: kích thước hình học '{gname}' "
                f"(tâm cục bộ={(round(dx, 4), round(dy, 4), round(dz, 4))}, "
                f"bán kính mở rộng={(round(hx, 4), round(hy, 4), round(hz, 4))}) "
                f"vượt ngoài khung robot cho phép (|x|<=0.25, |y|<=0.20, 0<=z<=0.30m)."
            )


def _extract_included_robot_body(resolved_arena: Path, inc_elem: ET.Element) -> ET.Element:
    inc_file = inc_elem.get("file")
    if (
        not inc_file
        or inc_file != ROBOT_INCLUDE_FILENAME
        or ".." in inc_file
        or "/" in inc_file
        or "\\" in inc_file
        or Path(inc_file).is_absolute()
        or Path(inc_file).name != inc_file
    ):
        raise ValueError(
            f"Vi phạm an toàn <include>: chỉ chấp nhận '<include file=\"{ROBOT_INCLUDE_FILENAME}\"/>' cùng thư mục "
            f"(nhận được: {inc_file!r})."
        )

    inc_path = resolved_arena.parent / inc_file
    if inc_path.is_symlink() or not inc_path.is_file() or inc_path.resolve().parent != resolved_arena.parent:
        raise ValueError(
            f"Vi phạm an toàn <include>: không tìm thấy hoặc đường dẫn không hợp lệ cho '{inc_path}'."
        )

    try:
        inc_tree = ET.parse(inc_path)
    except (ET.ParseError, OSError) as exc:
        raise ValueError(f"File XML nhúng '{inc_path}' không hợp lệ: {exc}") from exc

    inc_root = inc_tree.getroot()
    if inc_root.tag != "mujocoinclude":
        raise ValueError(f"File '{inc_file}' phải có thẻ gốc <mujocoinclude>.")
    if inc_root.findall(".//include"):
        raise ValueError(f"Không cho phép lồng thẻ <include> bên trong '{inc_file}'.")

    inc_worldbodies = [c for c in inc_root if c.tag == "worldbody"]
    for child in inc_root:
        if child.tag not in _ALLOWED_INCLUDE_ROOT_TAGS:
            raise ValueError(f"Thẻ '<{child.tag}>' không được phép trong '{inc_file}'.")
    if len(inc_worldbodies) != 1:
        raise ValueError(f"File '{inc_file}' phải chứa đúng 1 khối <worldbody>.")

    wb_children = list(inc_worldbodies[0])
    if len(wb_children) != 1 or wb_children[0].tag != "body" or wb_children[0].get("name") != "robot":
        raise ValueError(
            f"Khối <worldbody> trong '{inc_file}' chỉ được phép chứa duy nhất <body name='robot'>."
        )
    return wb_children[0]


def verify_arena_integrity(model_path: Path | str | None = None) -> dict:
    """Kiểm tra chống gian lận đề bài đa sa bàn và đảm bảo giới hạn vật lý hợp lệ của thân xe robot."""
    resolved = (Path(model_path) if model_path is not None else DEFAULT_MODEL_PATH).resolve()
    if not resolved.is_file():
        raise ValueError(f"Không tìm thấy file mô hình XML tại '{resolved}'.")

    try:
        tree = ET.parse(resolved)
    except (ET.ParseError, OSError) as exc:
        raise ValueError(f"File XML mô hình không hợp lệ ('{resolved}'): {exc}") from exc

    root = tree.getroot()
    if root.tag != "mujoco":
        raise ValueError("File XML mô hình phải có thẻ gốc <mujoco>.")

    model_attr = root.get("model")
    arena_id = MODEL_NAME_TO_ARENA_ID.get(model_attr or "")
    if arena_id is None:
        raise ValueError(
            f"Vi phạm toàn vẹn sa bàn: thuộc tính <mujoco model={model_attr!r}> không thuộc danh bạ địa hình chuẩn."
        )
    spec = ARENAS[arena_id]

    if resolved.parent == MODELS_DIR.resolve() and resolved.name != spec.filename:
        raise ValueError(
            f"Vi phạm toàn vẹn sa bàn: tên file '{resolved.name}' trong models/ không khớp với model='{model_attr}' "
            f"(yêu cầu '{spec.filename}')."
        )

    for child in root:
        if child.tag not in _ALLOWED_ROOT_TAGS:
            raise ValueError(f"Vi phạm toàn vẹn sa bàn: thẻ gốc '<{child.tag}>' không được phép trong mô hình.")

    all_includes = root.findall(".//include")
    root_includes = root.findall("./include")
    if len(all_includes) != len(root_includes):
        raise ValueError("Vi phạm an toàn <include>: thẻ <include> chỉ được phép đặt trực tiếp dưới <mujoco>.")
    if len(root_includes) > 1:
        raise ValueError("Vi phạm an toàn <include>: chỉ cho phép tối đa 1 thẻ <include>.")

    included_robot_body: ET.Element | None = None
    if len(root_includes) == 1:
        included_robot_body = _extract_included_robot_body(resolved, root_includes[0])

    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("File XML thiếu thẻ <worldbody>.")

    # 0. Whitelist top-level worldbody elements against the matched ArenaSpec
    allowed_geoms = {"floor", *spec.walls.keys(), *spec.maze_walls.keys()}
    allowed_bodies = {*spec.obstacles.keys(), "target"}
    if included_robot_body is None:
        allowed_bodies.add("robot")

    seen_geoms: set[str] = set()
    seen_bodies: set[str] = set()
    seen_sites: set[str] = set()
    for child in worldbody:
        if child.tag in ("light", "camera"):
            continue
        if child.tag == "site":
            sname = child.get("name")
            if sname != "robot_spawn" or sname in seen_sites:
                raise ValueError(f"Vi phạm toàn vẹn sa bàn: phát hiện site không được phép '{sname}' trong worldbody.")
            seen_sites.add(sname)
        elif child.tag == "geom":
            gname = child.get("name")
            if gname not in allowed_geoms or gname in seen_geoms:
                raise ValueError(f"Vi phạm toàn vẹn sa bàn: phát hiện geom không được phép '{gname}' trong worldbody.")
            seen_geoms.add(gname)
        elif child.tag == "body":
            bname = child.get("name")
            if bname not in allowed_bodies or bname in seen_bodies:
                raise ValueError(f"Vi phạm toàn vẹn sa bàn: phát hiện body không được phép '{bname}' trong worldbody.")
            seen_bodies.add(bname)
        else:
            raise ValueError(
                f"Vi phạm toàn vẹn sa bàn: thẻ '<{child.tag}>' không được phép trực tiếp trong worldbody."
            )

    # 1. Spawn site validation (required on maze arenas; validated whenever present)
    spawn_site = worldbody.find("./site[@name='robot_spawn']")
    if spawn_site is None:
        if spec.arena_id != "2wheel":
            raise ValueError(f"Vi phạm toàn vẹn sa bàn: thiếu site 'robot_spawn' trên bản đồ '{spec.arena_id}'.")
    else:
        _assert_no_rotation_attrs(spawn_site, "robot_spawn", allow_euler=True)
        _assert_vec_close(
            _parse_vec(spawn_site.get("pos"), 3, "robot_spawn.pos"),
            [spec.spawn_xy[0], spec.spawn_xy[1], 0.0],
            "robot_spawn.pos",
        )
        _assert_vec_close(
            _parse_vec(spawn_site.get("euler", "0 0 0"), 3, "robot_spawn.euler"),
            [0.0, 0.0, spec.spawn_yaw_deg],
            "robot_spawn.euler",
        )

    # 2. Floor
    floor = worldbody.find("./geom[@name='floor']")
    if floor is None or floor.get("type") != "plane":
        raise ValueError("Vi phạm toàn vẹn sa bàn: thiếu hoặc sai kiểu mặt sàn 'floor'.")
    _assert_no_rotation_attrs(floor, "floor")
    _assert_vec_close(_parse_vec(floor.get("size"), 3, "floor.size"), list(spec.floor_size), "floor.size")
    _assert_collision_enabled(floor, "floor")

    # 3. Four outer arena walls
    for wall_name, (exp_pos, exp_size) in spec.walls.items():
        wall = worldbody.find(f"./geom[@name='{wall_name}']")
        if wall is None or wall.get("type") != "box":
            raise ValueError(f"Vi phạm toàn vẹn sa bàn: thiếu hoặc sai kiểu tường '{wall_name}'.")
        _assert_no_rotation_attrs(wall, wall_name)
        _assert_vec_close(_parse_vec(wall.get("pos"), 3, f"{wall_name}.pos"), exp_pos, f"{wall_name}.pos")
        _assert_vec_close(_parse_vec(wall.get("size"), 3, f"{wall_name}.size"), exp_size, f"{wall_name}.size")
        if wall.get("rgba") is not None:
            wall_rgba = _parse_vec(wall.get("rgba"), 4, f"{wall_name}.rgba")
            if wall_rgba[3] < 0.99:
                raise ValueError(f"Vi phạm toàn vẹn sa bàn: tường '{wall_name}' không được làm trong suốt.")
        _assert_collision_enabled(wall, wall_name)

    # 4. Internal maze walls (if any)
    for mw_name, (exp_pos, exp_size, exp_rgba) in spec.maze_walls.items():
        mw = worldbody.find(f"./geom[@name='{mw_name}']")
        if mw is None or mw.get("type") != "box":
            raise ValueError(f"Vi phạm toàn vẹn sa bàn: thiếu hoặc sai kiểu vách mê cung '{mw_name}'.")
        _assert_no_rotation_attrs(mw, mw_name)
        _assert_vec_close(_parse_vec(mw.get("pos"), 3, f"{mw_name}.pos"), exp_pos, f"{mw_name}.pos")
        _assert_vec_close(_parse_vec(mw.get("size"), 3, f"{mw_name}.size"), exp_size, f"{mw_name}.size")
        mw_rgba = _parse_vec(mw.get("rgba"), 4, f"{mw_name}.rgba")
        _assert_vec_close(mw_rgba, exp_rgba, f"{mw_name}.rgba", tol=0.05)
        if mw_rgba[3] < 0.99:
            raise ValueError(f"Vi phạm toàn vẹn sa bàn: vách mê cung '{mw_name}' phải đục hoàn toàn (alpha=1.0).")
        _assert_collision_enabled(mw, mw_name)

    # 5. Colored obstacles (on basic arena)
    for body_name, (exp_pos, geom_name, exp_type, exp_size, exp_rgba) in spec.obstacles.items():
        obs_body = worldbody.find(f"./body[@name='{body_name}']")
        if obs_body is None:
            raise ValueError(f"Vi phạm toàn vẹn sa bàn: thiếu vật cản '{body_name}'.")
        _assert_no_rotation_attrs(obs_body, body_name)
        _assert_vec_close(_parse_vec(obs_body.get("pos"), 3, f"{body_name}.pos"), exp_pos, f"{body_name}.pos")
        obs_geom = obs_body.find(f"./geom[@name='{geom_name}']")
        if obs_geom is None or obs_geom.get("type") != exp_type:
            raise ValueError(f"Vi phạm toàn vẹn sa bàn: thiếu hoặc sai kiểu hình học '{geom_name}'.")
        _assert_no_rotation_attrs(obs_geom, geom_name)
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

    # 6. Green mocap target
    target_body = worldbody.find("./body[@name='target']")
    if target_body is None or target_body.get("mocap") != "true":
        raise ValueError("Vi phạm toàn vẹn sa bàn: thiếu mục tiêu mocap 'target'.")
    _assert_vec_close(
        _parse_vec(target_body.get("pos"), 3, "target.pos"),
        [spec.default_target_xy[0], spec.default_target_xy[1], 0.12],
        "target.pos",
    )
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

    # 7. Robot body & Sim-to-Real physical envelope
    robot_body = included_robot_body if included_robot_body is not None else worldbody.find("./body[@name='robot']")
    if robot_body is None:
        raise ValueError("Mô hình thiếu <body name='robot'>.")
    robot_pos = _parse_vec(robot_body.get("pos", "0 0 0"), 3, "robot.pos")
    valid_origin = all(abs(a - b) <= 1e-4 for a, b in zip(robot_pos, [0.0, 0.0, 0.0]))
    valid_spawn = all(abs(a - b) <= 1e-4 for a, b in zip(robot_pos, [spec.spawn_xy[0], spec.spawn_xy[1], 0.0]))
    if not (valid_origin or valid_spawn):
        raise ValueError(
            f"Vi phạm toàn vẹn sa bàn: không được phép thay đổi 'robot.pos' thành {robot_pos}."
        )
    if robot_body.find("./freejoint") is None:
        raise ValueError("Robot bắt buộc phải di chuyển vật lý qua <freejoint>.")
    if robot_body.find(".//camera[@name='front_cam']") is None:
        raise ValueError("Camera 'front_cam' bắt buộc phải gắn trên thân 'robot'.")
    _check_robot_bounds(robot_body)

    # 8. MuJoCo compilation + required sensors/actuators/cameras + 3D geom extents relative to robot body
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

    return {"valid": True, "model_path": str(resolved), "arena": spec.arena_id}
