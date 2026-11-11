from pathlib import Path
import math
import pytest
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


def test_capture_camera_creates_png_and_rejects_unknown_camera(tmp_path: Path):
    sim = RobotSimulation()
    try:
        out_file = tmp_path / "nested" / "front.png"
        info = sim.capture_camera("front_cam", out_path=out_file)
        assert out_file.is_file()
        assert out_file.stat().st_size > 1000
        assert info["width"] == 640 and info["height"] == 480

        with pytest.raises(ValueError):
            sim.capture_camera("nonexistent_cam", out_path=out_file)
    finally:
        sim.close()
