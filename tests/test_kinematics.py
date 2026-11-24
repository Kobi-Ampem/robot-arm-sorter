import mujoco
import numpy as np
import pytest

from arm_sorter.kinematics import D1, JOINT_LIMITS, L2, L3, L4, jacobian, joint_frames
from arm_sorter.model import ARM_JOINTS, arm_dof_indices, arm_qpos_indices, load_model


@pytest.fixture(scope="module")
def model():
    return load_model()


def random_configs(n, seed=0):
    rng = np.random.default_rng(seed)
    return rng.uniform(JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1], size=(n, 4))


def mujoco_kinematics(model, q):
    data = mujoco.MjData(model)
    data.qpos[arm_qpos_indices(model)] = q
    mujoco.mj_kinematics(model, data)
    return data


def test_dimensions_match_the_xml(model):
    assert model.body("upper_arm").pos == pytest.approx([0, 0, D1])
    assert model.body("forearm").pos == pytest.approx([L2, 0, 0])
    assert model.body("hand").pos == pytest.approx([L3, 0, 0])
    assert model.site("tcp").pos == pytest.approx([L4, 0, 0])
    for name, (low, high) in zip(ARM_JOINTS, JOINT_LIMITS):
        assert model.joint(name).range == pytest.approx([low, high], abs=1e-4)


def test_zero_pose_points_straight_out(model):
    tcp = joint_frames(np.zeros(4))[-1]
    assert tcp[:3, 3] == pytest.approx([L2 + L3 + L4, 0, D1])
    assert tcp[:3, 0] == pytest.approx([1, 0, 0])


def test_frames_match_mujoco(model):
    bodies = ["upper_arm", "forearm", "hand"]
    for q in random_configs(200):
        data = mujoco_kinematics(model, q)
        frames = joint_frames(q)
        for frame, body in zip(frames[1:4], bodies):
            assert frame[:3, 3] == pytest.approx(data.body(body).xpos, abs=1e-9)
        assert frames[4][:3, 3] == pytest.approx(data.site("tcp").xpos, abs=1e-9)
        # The gripper points along the tool frame's x axis in both.
        assert frames[4][:3, 0] == pytest.approx(data.site("tcp").xmat.reshape(3, 3)[:, 0], abs=1e-9)


def test_dh_z_axes_are_the_joint_axes(model):
    for q in random_configs(50, seed=1):
        data = mujoco_kinematics(model, q)
        frames = joint_frames(q)
        for i, name in enumerate(ARM_JOINTS):
            # In DH, joint i+1 turns about the z axis of frame i.
            assert frames[i][:3, 2] == pytest.approx(data.joint(name).xaxis, abs=1e-9)


def test_jacobian_matches_mujoco(model):
    dofs = arm_dof_indices(model)
    for q in random_configs(200, seed=2):
        data = mujoco_kinematics(model, q)
        mujoco.mj_comPos(model, data)  # mj_jacSite needs the spatial quantities from this
        jacp = np.zeros((3, model.nv))
        jacr = np.zeros((3, model.nv))
        mujoco.mj_jacSite(model, data, jacp, jacr, model.site("tcp").id)
        J = jacobian(q)
        assert J[:3] == pytest.approx(jacp[:, dofs], abs=1e-9)
        assert J[3:] == pytest.approx(jacr[:, dofs], abs=1e-9)


def test_jacobian_matches_finite_differences():
    from arm_sorter.kinematics import tcp_position

    eps = 1e-6
    for q in random_configs(50, seed=3):
        numeric = np.column_stack([
            (tcp_position(q + eps * e) - tcp_position(q - eps * e)) / (2 * eps)
            for e in np.eye(4)
        ])
        assert jacobian(q)[:3] == pytest.approx(numeric, abs=1e-7)
