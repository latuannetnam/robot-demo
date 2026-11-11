from pathlib import Path
import mujoco
import numpy as np

MODEL_PATH = Path(__file__).resolve().parents[1] / "models" / "arena_2wheel.xml"


def test_model_loads_and_named_elements_exist():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "robot") >= 0
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "target") >= 0
    for cam in ("front_cam", "overview_cam"):
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, cam) >= 0
    for act in ("left_motor", "right_motor"):
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, act) >= 0
    for sensor in (
        "rf_left",
        "rf_center",
        "rf_right",
        "left_wheel_vel",
        "right_wheel_vel",
        "chassis_vel",
        "chassis_gyro",
    ):
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, sensor) >= 0


def test_robot_drives_forward_and_turns_cleanly():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    # Move target out of forward path so robot has clear runway
    data.mocap_pos[0] = [1.5, 1.2, 0.12]
    data.ctrl[:] = [6.0, 6.0]
    for _ in range(500):
        mujoco.mj_step(model, data)
    assert data.qpos[0] > 0.20  # moved forward along +X
    assert abs(data.qpos[1]) < 0.05  # stayed straight
    assert abs(data.qpos[2]) < 0.02  # did not tip or bounce

    # Differential spin in place (right wheel forward, left wheel backward => +Z CCW yaw)
    mujoco.mj_resetData(model, data)
    data.ctrl[:] = [-5.0, 5.0]
    for _ in range(300):
        mujoco.mj_step(model, data)
    quat = data.qpos[3:7]
    yaw = np.arctan2(
        2.0 * (quat[0] * quat[3] + quat[1] * quat[2]),
        1.0 - 2.0 * (quat[2] ** 2 + quat[3] ** 2),
    )
    assert yaw > 0.3


def test_rangefinder_points_forward_and_front_cam_renders():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    # Place mocap target directly 0.8m in front of robot nose (nose is at x=0.12)
    data.mocap_pos[0] = [0.92, 0.0, 0.10]
    mujoco.mj_forward(model, data)
    rf_center_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "rf_center")
    rf_center_adr = model.sensor_adr[rf_center_id]
    assert 0.55 < data.sensordata[rf_center_adr] < 0.90

    renderer = mujoco.Renderer(model, height=480, width=640)
    try:
        renderer.update_scene(data, camera="front_cam")
        img = renderer.render()
        assert img.shape == (480, 640, 3)
        # Green target in center of frame should produce strong green pixels
        center_patch = img[180:300, 260:380]
        assert int(center_patch[:, :, 1].max()) > 150
    finally:
        renderer.close()
