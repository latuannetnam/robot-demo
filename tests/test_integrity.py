from pathlib import Path
import shutil
import pytest

from robot_sim.arenas import MODELS_DIR, ROBOT_INCLUDE_FILENAME, resolve_arena_model
from robot_sim.core import DEFAULT_MODEL_PATH, RobotSimulation
from robot_sim.integrity import verify_arena_integrity


def _copy_arena_bundle(tmp_path: Path, arena_filename: str = "arena_2wheel.xml", dst_name: str = "tampered.xml") -> tuple[Path, Path]:
    arena_dst = tmp_path / dst_name
    robot_dst = tmp_path / ROBOT_INCLUDE_FILENAME
    shutil.copyfile(MODELS_DIR / arena_filename, arena_dst)
    shutil.copyfile(MODELS_DIR / ROBOT_INCLUDE_FILENAME, robot_dst)
    return arena_dst, robot_dst


def test_default_arena_passes_integrity():
    res = verify_arena_integrity(DEFAULT_MODEL_PATH)
    assert res["valid"] is True
    assert res["arena"] == "2wheel"


@pytest.mark.parametrize(
    "tamper_src,tamper_dst",
    [
        ('pos="-0.85 0.75 0.12"', 'pos="-1.50 0.75 0.12"'),  # moved obs_yellow
        ('size="0.12 0.12"', 'size="0.02 0.02"'),            # shrunk obs_yellow
        ('size="0.10 0.12"', 'size="0.40 0.12"'),            # enlarged green target
        ('pos="0 2.025 0.125"', 'pos="0 3.025 0.125"'),      # moved wall_north
        ('pos="0.12 0 0.09"', 'pos="0.12 0 1.50"'),          # illegal periscope front_cam height
    ],
)
def test_verify_arena_integrity_rejects_arena_or_periscope_tampering(
    tmp_path: Path, tamper_src: str, tamper_dst: str
):
    bad_arena, bad_robot = _copy_arena_bundle(tmp_path)
    arena_text = bad_arena.read_text(encoding="utf-8")
    robot_text = bad_robot.read_text(encoding="utf-8")
    if tamper_src in arena_text:
        bad_arena.write_text(arena_text.replace(tamper_src, tamper_dst, 1), encoding="utf-8")
    else:
        assert tamper_src in robot_text
        bad_robot.write_text(robot_text.replace(tamper_src, tamper_dst, 1), encoding="utf-8")

    with pytest.raises(ValueError):
        verify_arena_integrity(bad_arena)


def test_reload_rejects_tampered_or_corrupt_xml_without_breaking_sim(tmp_path: Path):
    sim = RobotSimulation()
    try:
        sim.reset(target_xy=(-1.4, 1.15))
        corrupt = tmp_path / "corrupt.xml"
        corrupt.write_text("<mujoco><broken></mujoco>", encoding="utf-8")
        with pytest.raises(ValueError):
            sim.reload(model_path=corrupt)

        # Simulation remains intact with previous target_xy preserved
        st = sim.get_status()
        assert st["debug"]["target_xy"] == [-1.4, 1.15]

        # Valid reload preserves current target_xy by default
        reloaded = sim.reload(preserve_target=True)
        assert reloaded["debug"]["target_xy"] == [-1.4, 1.15]
    finally:
        sim.close()


def test_verify_arena_integrity_rejects_oversized_robot_geom_extents(tmp_path: Path):
    oversized_arena, oversized_robot = _copy_arena_bundle(tmp_path, dst_name="oversized_geom.xml")
    robot_text = oversized_robot.read_text(encoding="utf-8")
    assert 'size="0.12 0.10 0.03"' in robot_text
    # Origin (0, 0, 0.06) is inside envelope, but box half-extents (1.5m x 1.5m) span the arena
    oversized_robot.write_text(
        robot_text.replace('size="0.12 0.10 0.03"', 'size="1.50 1.50 0.03"', 1),
        encoding="utf-8",
    )
    with pytest.raises(ValueError):
        verify_arena_integrity(oversized_arena)


def test_verify_arena_integrity_rejects_unauthorized_worldbody_elements(tmp_path: Path):
    injected_arena, _ = _copy_arena_bundle(tmp_path, dst_name="injected_worldbody.xml")
    xml_text = injected_arena.read_text(encoding="utf-8")
    injected_body = (
        '<body name="guide_rail" pos="0.5 0 0.05">'
        '<geom name="guide_geom" type="box" size="0.5 0.02 0.05"/>'
        "</body>\n  </worldbody>"
    )
    injected_arena.write_text(xml_text.replace("</worldbody>", injected_body, 1), encoding="utf-8")
    with pytest.raises(ValueError):
        verify_arena_integrity(injected_arena)


@pytest.mark.parametrize(
    "tamper_src,tamper_dst",
    [
        ('pos="1.5 1.0 0.14" size="0.52 0.02 0.14"', 'pos="1.5 1.0 0.14" size="0.10 0.02 0.14"'),  # shrunk maze_w02
        ('pos="1.5 1.0 0.14" size="0.52 0.02 0.14"', 'pos="0.5 1.0 0.14" size="0.52 0.02 0.14"'),  # moved maze_w02
        ('name="robot_spawn" pos="-1.5 -1.5 0"', 'name="robot_spawn" pos="1.0 1.5 0"'),             # moved robot_spawn
        ('name="robot_spawn" pos="-1.5 -1.5 0" euler="0 0 0"', 'name="robot_spawn" pos="-1.5 -1.5 0" euler="0 0 90"'),
    ],
)
def test_verify_arena_integrity_rejects_maze_wall_tampering(
    tmp_path: Path, tamper_src: str, tamper_dst: str
):
    bad_maze, _ = _copy_arena_bundle(tmp_path, arena_filename="arena_maze_a.xml", dst_name="tampered_maze_a.xml")
    maze_text = bad_maze.read_text(encoding="utf-8")
    assert tamper_src in maze_text
    bad_maze.write_text(maze_text.replace(tamper_src, tamper_dst, 1), encoding="utf-8")
    with pytest.raises(ValueError):
        verify_arena_integrity(bad_maze)


@pytest.mark.parametrize(
    "bad_include",
    [
        '../outside.xml',
        'C:/outside.xml',
        'subdir/robot_2wheel.xml',
    ],
)
def test_verify_arena_integrity_rejects_include_path_traversal(tmp_path: Path, bad_include: str):
    sub = tmp_path / "nested"
    sub.mkdir(parents=True, exist_ok=True)
    # Place a valid robot xml at ../outside.xml so traversal is rejected by policy, not file-not-found
    shutil.copyfile(MODELS_DIR / ROBOT_INCLUDE_FILENAME, tmp_path / "outside.xml")
    bad_arena, _ = _copy_arena_bundle(sub, arena_filename="arena_maze_a.xml", dst_name="traversal.xml")
    xml_text = bad_arena.read_text(encoding="utf-8")
    bad_arena.write_text(
        xml_text.replace(f'<include file="{ROBOT_INCLUDE_FILENAME}"/>', f'<include file="{bad_include}"/>', 1),
        encoding="utf-8",
    )
    with pytest.raises(ValueError):
        verify_arena_integrity(bad_arena)


def test_verify_arena_integrity_checks_robot_extents_relative_to_off_origin_spawn(tmp_path: Path):
    maze_copy, robot_copy = _copy_arena_bundle(
        tmp_path, arena_filename="arena_maze_a.xml", dst_name="arena_maze_a.xml"
    )
    res = verify_arena_integrity(maze_copy)
    assert res["valid"] is True
    assert res["arena"] == "maze_a"

    # Also verify explicit robot body at (-1.5, -1.5, 0) passes when valid
    robot_at_spawn = robot_copy.read_text(encoding="utf-8").replace(
        '<body name="robot" pos="0 0 0">', '<body name="robot" pos="-1.5 -1.5 0">', 1
    )
    robot_copy.write_text(robot_at_spawn, encoding="utf-8")
    assert verify_arena_integrity(maze_copy)["valid"] is True

    # Inflating chassis size to 1.50 x 1.50m must still fail relative to off-origin spawn
    robot_copy.write_text(
        robot_at_spawn.replace('size="0.12 0.10 0.03"', 'size="1.50 1.50 0.03"', 1),
        encoding="utf-8",
    )
    with pytest.raises(ValueError):
        verify_arena_integrity(maze_copy)


def test_verify_arena_integrity_supports_standalone_inline_robot_and_rejects_unknown_model_attr(tmp_path: Path):
    arena_text = resolve_arena_model("2wheel").read_text(encoding="utf-8")
    inc_root_text = (MODELS_DIR / ROBOT_INCLUDE_FILENAME).read_text(encoding="utf-8")
    # Strip <mujocoinclude> wrapper and inline <body name="robot"> into <worldbody>, <actuator>/<sensor> under <mujoco>
    robot_body_block = inc_root_text.split("<worldbody>", 1)[1].split("</worldbody>", 1)[0]
    rest_block = inc_root_text.split("</worldbody>", 1)[1].rsplit("</mujocoinclude>", 1)[0]
    inline_xml = arena_text.replace("</worldbody>", f"{robot_body_block}</worldbody>", 1).replace(
        f'<include file="{ROBOT_INCLUDE_FILENAME}"/>', rest_block, 1
    )
    standalone = tmp_path / "standalone_inline.xml"
    standalone.write_text(inline_xml, encoding="utf-8")
    res = verify_arena_integrity(standalone)
    assert res["valid"] is True
    assert res["arena"] == "2wheel"

    unknown_model = tmp_path / "unknown_model.xml"
    unknown_model.write_text(
        inline_xml.replace('model="arena_2wheel"', 'model="arena_hacked"', 1),
        encoding="utf-8",
    )
    with pytest.raises(ValueError):
        verify_arena_integrity(unknown_model)


def test_verify_arena_integrity_rejects_unclassed_default_collision_disable_and_rotated_camera_periscope(
    tmp_path: Path,
):
    # Case 1: Unclassed <default> in robot_2wheel.xml disabling geom collisions globally
    default_dir = tmp_path / "unclassed_default"
    default_dir.mkdir(parents=True, exist_ok=True)
    bad_arena_default, bad_robot_default = _copy_arena_bundle(
        default_dir, arena_filename="arena_maze_a.xml", dst_name="arena_maze_a.xml"
    )
    robot_xml = bad_robot_default.read_text(encoding="utf-8")
    bad_robot_default.write_text(
        robot_xml.replace(
            "<mujocoinclude>",
            '<mujocoinclude>\n  <default><geom contype="0" conaffinity="0"/></default>',
            1,
        ),
        encoding="utf-8",
    )
    with pytest.raises(ValueError):
        verify_arena_integrity(bad_arena_default)

    # Case 1b: Arena-root <default> disabling collision or alpha on non-robot compiled geoms
    root_def_dir = tmp_path / "root_default_collision"
    root_def_dir.mkdir(parents=True, exist_ok=True)
    bad_arena_root_def, _ = _copy_arena_bundle(
        root_def_dir, arena_filename="arena_maze_a.xml", dst_name="arena_maze_a.xml"
    )
    arena_xml = bad_arena_root_def.read_text(encoding="utf-8")
    bad_arena_root_def.write_text(
        arena_xml.replace(
            "<worldbody>",
            '<default><geom contype="0" conaffinity="0"/></default>\n  <worldbody>',
            1,
        ),
        encoding="utf-8",
    )
    with pytest.raises(ValueError):
        verify_arena_integrity(bad_arena_root_def)

    # Case 2: Rotated child body lifts front_cam to z=0.49m while unrotated XML pos sum (0.24, 0, 0.25) stays inside bounds
    cam_dir = tmp_path / "rotated_cam"
    cam_dir.mkdir(parents=True, exist_ok=True)
    bad_arena_cam, bad_robot_cam = _copy_arena_bundle(
        cam_dir, arena_filename="arena_maze_a.xml", dst_name="arena_maze_a.xml"
    )
    bad_robot_cam.write_text(
        robot_xml.replace(
            '<camera name="front_cam" pos="0.12 0 0.09" xyaxes="0 -1 0 0 0 1" fovy="65"/>',
            '<body name="cam_boom" pos="0 0 0.25" euler="0 -90 0">'
            '<camera name="front_cam" pos="0.24 0 0" xyaxes="0 -1 0 0 0 1" fovy="65"/>'
            "</body>",
            1,
        ),
        encoding="utf-8",
    )
    with pytest.raises(ValueError):
        verify_arena_integrity(bad_arena_cam)

    # Case 3: Rotated child body lifts a sensor site to z=0.49m while unrotated XML pos sum stays inside bounds
    site_dir = tmp_path / "rotated_site"
    site_dir.mkdir(parents=True, exist_ok=True)
    bad_arena_site, bad_robot_site = _copy_arena_bundle(
        site_dir, arena_filename="arena_maze_a.xml", dst_name="arena_maze_a.xml"
    )
    bad_robot_site.write_text(
        robot_xml.replace(
            '<site name="site_rf_center" pos="0.125 0.00 0.075" zaxis="1 0 0" size="0.008" rgba="1 0.2 0.2 1"/>',
            '<body name="site_boom" pos="0 0 0.25" euler="0 -90 0">'
            '<site name="site_rf_center" pos="0.24 0 0" zaxis="1 0 0" size="0.008" rgba="1 0.2 0.2 1"/>'
            "</body>",
            1,
        ),
        encoding="utf-8",
    )
    with pytest.raises(ValueError):
        verify_arena_integrity(bad_arena_site)


@pytest.mark.parametrize("arena_id,arena_file", [("dynamic_a", "arena_dynamic_a.xml"), ("dynamic_b", "arena_dynamic_b.xml")])
def test_verify_arena_integrity_validates_dynamic_obstacles_and_rejects_tampering(tmp_path: Path, arena_id: str, arena_file: str):
    res = verify_arena_integrity(resolve_arena_model(arena_id))
    assert res["valid"] is True
    assert res["arena"] == arena_id

    if arena_id == "dynamic_a":
        cases = [
            ('range="-1.35 1.35"', 'range="-0.35 0.35"'),
            ('pos="-0.92 0.0 0.12"', 'pos="-1.40 0.0 0.12"'),
            ('rgba="0.90 0.25 0.20 1"', 'rgba="0.90 0.25 0.20 0.2"'),
            ('mass="8.0"', 'mass="0.5"'),
        ]
    else:
        cases = [
            ('<joint name="obs_red_slide" type="slide" axis="0 1 0" range="-0.50 0.50"/>', ""),
            ('range="-0.45 0.45"', 'range="-0.90 0.90"'),
        ]

    for idx, (src, dst) in enumerate(cases):
        case_dir = tmp_path / f"{arena_id}_{idx}"
        case_dir.mkdir(parents=True, exist_ok=True)
        bad_arena, _ = _copy_arena_bundle(case_dir, arena_filename=arena_file, dst_name=arena_file)
        xml = bad_arena.read_text(encoding="utf-8")
        assert src in xml
        bad_arena.write_text(xml.replace(src, dst, 1), encoding="utf-8")
        with pytest.raises(ValueError):
            verify_arena_integrity(bad_arena)

