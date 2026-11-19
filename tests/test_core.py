from pathlib import Path
import math
import shutil
import pytest
from robot_sim.arenas import MODELS_DIR
from robot_sim.core import RobotSimulation, RANGEFINDER_MAX, MAX_WHEEL_RAD_S


def test_open_space_rangefinder_normalizes_minus_one():
    sim = RobotSimulation()
    try:
        sim.reset(target_xy=(-1.5, 1.5))
        # Place robot near (-1.6, -1.6) facing +45 deg diagonal (~5.1m > 3.0m cutoff) so raw rangefinder is -1.0
        sim.data.qpos[0:2] = [-1.6, -1.6]
        yaw = math.pi / 4.0
        sim.data.qpos[3:7] = [math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)]
        status = sim.get_status()
        assert status["sensors"]["rangefinder_hits"]["center"] is False
        assert status["sensors"]["rangefinders_m"]["center"] == RANGEFINDER_MAX

        # Also pitch nose straight up (-90 deg around Y) into open sky so raw sensordata is literally -1.0
        sim.data.qpos[3:7] = [math.cos(-math.pi / 4.0), 0.0, math.sin(-math.pi / 4.0), 0.0]
        status_sky = sim.get_status()
        assert sim.data.sensordata[0] == -1.0
        assert status_sky["sensors"]["rangefinder_hits"]["center"] is False
        assert status_sky["sensors"]["rangefinders_m"]["center"] == RANGEFINDER_MAX
    finally:
        sim.close()


def test_watchdog_stops_wheels_after_duration_and_clamps_speed():
    sim = RobotSimulation()
    try:
        res = sim.set_wheels(50.0, -50.0, duration=0.4)
        assert res["left_rad_s"] == MAX_WHEEL_RAD_S
        assert res["right_rad_s"] == -MAX_WHEEL_RAD_S
        sim.step_seconds(0.2)
        assert sim.get_status()["moving"] is True
        sim.step_seconds(0.3)
        st = sim.get_status()
        assert st["moving"] is False
        assert abs(sim.data.ctrl[0]) == 0.0 and abs(sim.data.ctrl[1]) == 0.0
    finally:
        sim.close()


@pytest.mark.parametrize(
    "bad_args",
    [
        (math.nan, 1.0, 1.0),
        (1.0, math.inf, 1.0),
        (2.0, 2.0, 0.0),
        (2.0, 2.0, -1.0),
    ],
)
def test_set_wheels_rejects_invalid_inputs(bad_args):
    sim = RobotSimulation()
    try:
        with pytest.raises(ValueError):
            sim.set_wheels(*bad_args)
    finally:
        sim.close()


def test_move_and_turn_kinematics_and_reset_bounds():
    sim = RobotSimulation()
    try:
        sim.reset(target_xy=(-1.2, 1.2))
        sim.move(0.30, speed_m_s=0.30)
        sim.step_seconds(1.2)
        x_after = sim.get_status()["debug"]["robot_xy"][0]
        assert 0.18 < x_after < 0.42

        sim.reset(target_xy=(-1.2, 1.2))
        sim.turn(90.0, angular_deg_s=90.0)
        sim.step_seconds(1.2)
        yaw_deg = sim.get_status()["debug"]["robot_yaw_deg"]
        assert 55.0 < yaw_deg < 125.0

        with pytest.raises(ValueError):
            sim.reset(target_xy=(5.0, 0.0))
    finally:
        sim.close()


def test_capture_camera_defaults_to_compact_jpeg_and_supports_png_override(tmp_path: Path):
    sim = RobotSimulation()
    try:
        # Default capture: 320x240 compact JPEG (< 12 KB)
        jpg_file = tmp_path / "compact" / "front.jpg"
        info_jpg = sim.capture_camera("front_cam", out_path=jpg_file)
        assert jpg_file.is_file()
        assert info_jpg["format"] == "JPEG"
        assert info_jpg["width"] == 320 and info_jpg["height"] == 240
        assert 300 < info_jpg["bytes"] < 12000

        # Explicit high-res PNG override remains supported
        png_file = tmp_path / "nested" / "front.png"
        info_png = sim.capture_camera("front_cam", out_path=png_file, width=640, height=480, img_format="png")
        assert png_file.is_file()
        assert info_png["format"] == "PNG"
        assert info_png["width"] == 640 and info_png["height"] == 480

        with pytest.raises(ValueError):
            sim.capture_camera("nonexistent_cam", out_path=jpg_file)
    finally:
        sim.close()


def test_analyze_vision_detects_green_target_and_colored_obstacles_in_ram():
    sim = RobotSimulation()
    try:
        # 1. Default reset places green target at (1.4, 0.0) directly ahead of robot nose (+X),
        #    with blue obstacle at (0.85, -0.75) visible on the right side of the frame
        sim.reset(target_xy=(1.2, 0.0))
        vis = sim.analyze_vision("front_cam")
        assert vis["target_visible"] is True
        assert vis["target"] is not None
        assert abs(vis["target"]["offset_x"]) < 0.15
        assert vis["target"]["side"] == "center"
        assert vis["target"]["area_pct"] > 0.5
        obs_colors = {o["color"]: o for o in vis["obstacles"]}
        assert "blue" in obs_colors
        assert obs_colors["blue"]["side"] == "right"

        # 2. Facing (-X, -Y) empty corner with target behind robot at (+1.4, +1.4)
        #    should see neither target nor false-positive yellow laser beams
        sim.reset(target_xy=(1.4, 1.4))
        yaw = -3.0 * math.pi / 4.0
        sim.data.qpos[3:7] = [math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)]
        vis_empty = sim.analyze_vision("front_cam")
        assert vis_empty["target_visible"] is False
        assert vis_empty["target"] is None
        assert vis_empty["obstacles"] == []
    finally:
        sim.close()


def test_braking_from_cruise_does_not_strike_floor_with_rangefinder():
    sim = RobotSimulation()
    try:
        sim.reset(target_xy=(-1.4, 1.15))
        cmd = sim.move(0.75, speed_m_s=0.35)
        sim.step_seconds(cmd["duration_s"])
        # Sample across the 150ms post-stop braking window where nose-dive previously hit 0.78m
        min_center = 3.0
        for _ in range(15):
            sim.step_seconds(0.01)
            min_center = min(min_center, sim.get_status()["sensors"]["rangefinders_m"]["center"])
        assert min_center > 1.05
    finally:
        sim.close()


def test_reload_switches_arena_spawn_and_resets_target_unless_same_arena():
    sim = RobotSimulation()
    try:
        st0 = sim.get_status()
        assert "arena" not in st0 and "model" not in st0
        assert st0["debug"]["arena"] == "2wheel"
        assert st0["debug"]["model"] == "arena_2wheel.xml"

        sim.reset(target_xy=(-1.2, 0.9))
        st_same = sim.reload()
        assert st_same["debug"]["arena"] == "2wheel"
        assert st_same["debug"]["robot_xy"] == pytest.approx([0.0, 0.0], abs=1e-3)
        assert st_same["debug"]["target_xy"] == pytest.approx([-1.2, 0.9], abs=1e-3)

        st_maze = sim.reload(model_path="maze_a")
        assert st_maze["debug"]["arena"] == "maze_a"
        assert st_maze["debug"]["model"] == "arena_maze_a.xml"
        assert st_maze["debug"]["robot_xy"] == pytest.approx([-1.5, -1.5], abs=1e-3)
        assert st_maze["debug"]["target_xy"] == pytest.approx([1.5, 1.5], abs=1e-3)

        st_back = sim.reload(model_path="2wheel")
        assert st_back["debug"]["arena"] == "2wheel"
        assert st_back["debug"]["model"] == "arena_2wheel.xml"
        assert st_back["debug"]["robot_xy"] == pytest.approx([0.0, 0.0], abs=1e-3)
        assert st_back["debug"]["target_xy"] == pytest.approx([1.4, 0.0], abs=1e-3)
    finally:
        sim.close()


def test_validate_target_xy_rejects_maze_wall_and_obstacle_center_overlap_while_preserving_near_obstacle_cli_coord():
    sim = RobotSimulation()
    try:
        # On 2wheel: legacy CLI coord (1.0, -0.8) near obs_blue remains valid, while obstacle centers fail
        assert sim.validate_target_xy((1.0, -0.8)) == pytest.approx((1.0, -0.8))
        with pytest.raises(ValueError):
            sim.validate_target_xy((0.85, -0.75))  # inside obs_blue box
        with pytest.raises(ValueError):
            sim.validate_target_xy((-0.85, 0.75))  # inside obs_yellow cylinder

        # Atomic reload failure: switching to maze_a with invalid target_xy on maze_w02 leaves 2wheel state untouched
        sim.reset(target_xy=(-1.1, 0.8))
        sim.move(0.20, speed_m_s=0.25)
        sim.step_seconds(0.30)
        before = sim.get_status()
        with pytest.raises(ValueError):
            sim.reload(model_path="maze_a", target_xy=(1.5, 1.0))
        after = sim.get_status()
        assert after["debug"]["arena"] == "2wheel"
        assert after["debug"]["target_xy"] == pytest.approx([-1.1, 0.8], abs=1e-3)
        assert after["sim_time"] == before["sim_time"]
        assert after["debug"]["robot_xy"] == before["debug"]["robot_xy"]

        # Once on maze_a, validate_target_xy rejects center and r=0.10m cylinder overlap with maze_w02
        sim.reload(model_path="maze_a")
        with pytest.raises(ValueError):
            sim.validate_target_xy((1.5, 1.0))
        with pytest.raises(ValueError):
            sim.validate_target_xy((1.5, 1.08))
    finally:
        sim.close()


def test_reload_discovers_extra_rangefinder_and_respects_per_sensor_cutoff(tmp_path: Path):
    arena_copy = tmp_path / "arena_2wheel.xml"
    robot_copy = tmp_path / "robot_2wheel.xml"
    shutil.copyfile(MODELS_DIR / "arena_2wheel.xml", arena_copy)
    robot_xml = (MODELS_DIR / "robot_2wheel.xml").read_text(encoding="utf-8")
    robot_xml = robot_xml.replace(
        '<site name="site_rf_right"',
        '<site name="site_rf_side_left" pos="0.10 0.09 0.075" zaxis="0 1 0" size="0.008"/>\n      <site name="site_rf_right"',
        1,
    ).replace(
        '<rangefinder name="rf_right"  site="site_rf_right"  cutoff="3.0"/>',
        '<rangefinder name="rf_right"  site="site_rf_right"  cutoff="3.0"/>\n    <rangefinder name="rf_side_left" site="site_rf_side_left" cutoff="1.5"/>',
        1,
    )
    robot_copy.write_text(robot_xml, encoding="utf-8")

    sim = RobotSimulation()
    try:
        st = sim.reload(model_path=arena_copy)
        assert st["sensors"]["rangefinders_m"]["side_left"] == 1.5
        assert st["sensors"]["rangefinder_hits"]["side_left"] is False

        # Move closer to wall_north (y=2.0 inner face) so distance (~0.91m) is within the 1.5m cutoff
        sim.data.qpos[1] = 1.0
        st_hit = sim.get_status()
        assert st_hit["sensors"]["rangefinder_hits"]["side_left"] is True
        assert 0.85 < st_hit["sensors"]["rangefinders_m"]["side_left"] < 0.96
    finally:
        sim.close()


def test_reload_cross_arena_explicit_preserve_target_rejects_wall_overlapping_prev_target_atomically(tmp_path: Path):
    sim = RobotSimulation()
    try:
        # (1.5, 1.0) is open floor on 2wheel, but sits directly inside maze_w02 on maze_a
        sim.reset(target_xy=(1.5, 1.0))
        sim.move(0.20, speed_m_s=0.25)
        sim.step_seconds(0.20)
        before = sim.get_status()
        assert before["debug"]["arena"] == "2wheel"
        assert before["debug"]["target_xy"] == pytest.approx([1.5, 1.0], abs=1e-3)

        with pytest.raises(ValueError):
            sim.reload(model_path="maze_a", preserve_target=True)

        after = sim.get_status()
        assert after["debug"]["arena"] == "2wheel"
        assert after["debug"]["model"] == "arena_2wheel.xml"
        assert after["debug"]["target_xy"] == pytest.approx([1.5, 1.0], abs=1e-3)
        assert after["sim_time"] == before["sim_time"]
        assert after["debug"]["robot_xy"] == before["debug"]["robot_xy"]

        # Duplicate stripped rangefinder keys (e.g. 'rf_center' and 'center') must also be rejected atomically
        arena_dup = tmp_path / "arena_2wheel.xml"
        robot_dup = tmp_path / "robot_2wheel.xml"
        shutil.copyfile(MODELS_DIR / "arena_2wheel.xml", arena_dup)
        robot_xml = (MODELS_DIR / "robot_2wheel.xml").read_text(encoding="utf-8")
        robot_dup.write_text(
            robot_xml.replace(
                '<rangefinder name="rf_center" site="site_rf_center" cutoff="3.0"/>',
                '<rangefinder name="rf_center" site="site_rf_center" cutoff="3.0"/>\n'
                '    <rangefinder name="center" site="site_rf_center" cutoff="3.0"/>',
                1,
            ),
            encoding="utf-8",
        )
        with pytest.raises(ValueError):
            sim.reload(model_path=arena_dup)
        assert sim.get_status()["debug"]["target_xy"] == pytest.approx([1.5, 1.0], abs=1e-3)
    finally:
        sim.close()


def test_analyze_vision_caches_same_state_and_invalidates_on_pose_change():
    sim = RobotSimulation()
    try:
        sim.reset(target_xy=(0.8, 0.0))
        v1 = sim.analyze_vision("front_cam")
        assert v1["target_visible"] is True

        # Mutating returned dict must not corrupt cached entry
        v1["target"]["offset_x"] = 999.0
        v2 = sim.analyze_vision("front_cam")
        assert abs(v2["target"]["offset_x"]) < 0.1

        # Direct mocap/qpos update invalidates cache immediately
        sim.data.mocap_pos[0] = [-0.8, 0.0, float(sim.data.mocap_pos[0][2])]
        v3 = sim.analyze_vision("front_cam")
        assert v3["target_visible"] is False
    finally:
        sim.close()


def test_spawn_relative_2d_odometry_tracks_turns_and_resets_cleanly():
    sim = RobotSimulation(model_path="maze_c")
    try:
        st0 = sim.get_status()
        assert st0["sensors"]["odom_x_m"] == 0.0
        assert st0["sensors"]["odom_y_m"] == 0.0
        assert "arena" not in st0["sensors"]
        sx, sy = st0["debug"]["robot_xy"]
        yaw0_rad = math.radians(st0["debug"]["robot_yaw_deg"])

        # Drive straight out of center room (~0.6m), pause, turn left ~90°, then drive ~0.45m
        for lw, rw, secs in [
            (8.0, 8.0, 1.6),
            (0.0, 0.0, 0.3),
            (-5.0, 5.0, 0.72),
            (0.0, 0.0, 0.3),
            (8.0, 8.0, 1.2),
            (0.0, 0.0, 0.3),
        ]:
            sim.set_wheels(lw, rw, duration=secs)
            sim.step_seconds(secs)

        st1 = sim.get_status()
        ox = float(st1["sensors"]["odom_x_m"])
        oy = float(st1["sensors"]["odom_y_m"])
        assert ox > 0.25
        assert oy > 0.20

        ex, ey = st1["debug"]["robot_xy"]
        dx_w, dy_w = ex - sx, ey - sy
        true_rx = dx_w * math.cos(-yaw0_rad) - dy_w * math.sin(-yaw0_rad)
        true_ry = dx_w * math.sin(-yaw0_rad) + dy_w * math.cos(-yaw0_rad)
        assert math.hypot(ox - true_rx, oy - true_ry) < 0.03

        st_reset = sim.reset()
        assert st_reset["sensors"]["odom_x_m"] == 0.0
        assert st_reset["sensors"]["odom_y_m"] == 0.0

        sim.set_wheels(6.0, 6.0, duration=0.5)
        sim.step_seconds(0.5)
        st_reload = sim.reload(model_path="maze_a")
        assert st_reload["sensors"]["odom_x_m"] == 0.0
        assert st_reload["sensors"]["odom_y_m"] == 0.0
    finally:
        sim.close()


def test_dynamic_obstacles_track_500hz_patrol_invalidate_vision_cache_and_count_contacts():
    from robot_sim.arenas import ARENAS

    spec = ARENAS["dynamic_a"]
    sim = RobotSimulation(model_path="dynamic_a")
    try:
        assert sim._bindings.robot_qpos_adr == 0
        assert sim.model.nq > 7
        st0 = sim.get_status(include_vision=True)
        assert st0["debug"]["arena"] == "dynamic_a"
        assert st0["debug"]["contact_count"] == 0
        assert st0["debug"]["in_contact"] is False
        assert "contact_count" not in st0["sensors"]
        assert "contact_count" not in st0
        assert st0["debug"]["robot_xy"] == pytest.approx([-1.65, 0.0], abs=1e-3)

        # Target placed inside a moving obstacle's sweep corridor must be rejected
        with pytest.raises(ValueError):
            sim.validate_target_xy((-0.92, 0.45))

        v_samples = []
        robot_q0 = sim.data.qpos[0:7].copy()
        for _ in range(3):
            sim.set_wheels(0.0, 0.0, duration=1.0)
            sim.step_seconds(1.0)
            t = float(sim.data.time)
            # Robot stayed stationary while obstacles moved at 500 Hz
            assert abs(float(sim.data.qpos[0] - robot_q0[0])) < 1e-3
            for name, mspec in spec.moving_obstacles.items():
                bid = sim.model.body(name).id
                exp_x = mspec.base_pos[0] + mspec.slide_axis[0] * mspec.amplitude_m * math.sin(
                    2.0 * math.pi * mspec.freq_hz * t + mspec.phase_rad
                )
                exp_y = mspec.base_pos[1] + mspec.slide_axis[1] * mspec.amplitude_m * math.sin(
                    2.0 * math.pi * mspec.freq_hz * t + mspec.phase_rad
                )
                actual_xy = sim.data.xpos[bid][:2]
                assert math.hypot(float(actual_xy[0]) - exp_x, float(actual_xy[1]) - exp_y) < 0.03
            vis = sim.analyze_vision("front_cam")
            for o in vis["obstacles"]:
                assert o["v_span"] > 0.0
            v_samples.append({o["color"]: o["offset_x"] for o in vis["obstacles"]})

        # Vision cache must invalidate when obstacles move even if robot pose qpos[0:7] is static
        common_colors = set(v_samples[0]).intersection(v_samples[1])
        if common_colors:
            c = next(iter(common_colors))
            assert abs(v_samples[0][c] - v_samples[1][c]) > 0.05
        else:
            assert v_samples[0] != v_samples[1]

        # Physical collision (backing straight into wall_west) increments debug.contact_count and resets cleanly
        sim.set_wheels(-10.0, -10.0, duration=1.5)
        sim.step_seconds(1.5)
        st_hit = sim.get_status()
        assert st_hit["debug"]["contact_count"] >= 1
        assert st_hit["debug"]["contact_steps"] >= 1

        st_reset = sim.reset()
        assert st_reset["debug"]["contact_count"] == 0
        assert st_reset["debug"]["contact_steps"] == 0
        assert st_reset["debug"]["in_contact"] is False
    finally:
        sim.close()




