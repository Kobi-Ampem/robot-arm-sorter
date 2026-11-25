import mujoco
import numpy as np

from arm_sorter.model import ARM_JOINTS, arm_dof_indices, arm_qpos_indices, load_model


def test_model_loads_and_steps():
    model = load_model()
    data = mujoco.MjData(model)
    for _ in range(100):
        mujoco.mj_step(model, data)
    assert np.all(np.isfinite(data.qpos))


def test_arm_joints_and_actuators():
    model = load_model()
    for name in ARM_JOINTS:
        assert model.joint(name).type == mujoco.mjtJoint.mjJNT_HINGE
        assert model.actuator(name).trntype == mujoco.mjtTrn.mjTRN_JOINT
    assert model.nu == 5  # four arm motors and the gripper
    assert arm_qpos_indices(model) == [0, 1, 2, 3]
    assert arm_dof_indices(model) == [0, 1, 2, 3]


def test_fingers_move_together():
    model = load_model()
    data = mujoco.MjData(model)
    data.ctrl[model.actuator("gripper").id] = 0.03
    for _ in range(1000):
        mujoco.mj_step(model, data)
    left = data.joint("finger_left").qpos[0]
    right = data.joint("finger_right").qpos[0]
    assert abs(left - 0.03) < 2e-3
    assert abs(left - right) < 1e-3


def test_no_contacts_at_rest_in_free_space():
    from arm_sorter.ik import READY_POSE

    model = load_model()
    data = mujoco.MjData(model)
    data.qpos[:4] = READY_POSE
    mujoco.mj_forward(model, data)
    assert data.ncon == 0


def test_base_turns_freely():
    model = load_model()
    model.opt.gravity[:] = 0  # without a controller the arm would fall onto the table
    data = mujoco.MjData(model)
    data.qpos[:4] = [0, 1.2, -1.8, -0.97]
    data.ctrl[model.actuator("base_yaw").id] = 0.2
    for _ in range(250):  # 0.5 s
        mujoco.mj_step(model, data)
    assert data.joint("base_yaw").qpos[0] > 0.5
