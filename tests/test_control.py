import numpy as np
import pytest

from arm_sorter.control import JointPID
from arm_sorter.ik import READY_POSE, solve_closed_form
from arm_sorter.model import load_model
from arm_sorter.sim import start_at, track
from arm_sorter.trajectory import joint_move


@pytest.fixture(scope="module")
def model():
    return load_model()


def test_torques_stay_within_motor_limits(model):
    controller = JointPID(model)
    data = start_at(model, READY_POSE)
    q1 = solve_closed_form([0.30, 0.12, 0.05]).q
    log = track(model, data, joint_move(READY_POSE, q1, model.opt.timestep), controller)
    limits = model.actuator_ctrlrange[:4]
    assert np.all(log.tau >= limits[:, 0] - 1e-12) and np.all(log.tau <= limits[:, 1] + 1e-12)


def test_pid_tracks_a_move(model):
    controller = JointPID(model)
    data = start_at(model, READY_POSE)
    q1 = solve_closed_form([0.30, 0.12, 0.05]).q
    log = track(model, data, joint_move(READY_POSE, q1, model.opt.timestep), controller)
    assert np.all(np.isfinite(log.q))
    # Plain PD sags under gravity, so only a loose bound here; gravity compensation tightens it.
    assert np.all(log.final_error() < 0.1)


def test_anti_windup_stops_integrating_when_saturated(model):
    from arm_sorter.control import PIDGains

    controller = JointPID(model, PIDGains(ki=np.full(4, 10.0)))
    data = start_at(model, READY_POSE)
    far = READY_POSE + np.array([3.0, 0, 0, 0])  # huge error: the base motor saturates
    for _ in range(50):
        controller.torque(data, far, np.zeros(4))
    assert controller.integral[0] == 0.0


def test_gravity_torque_matches_mujoco_at_rest(model):
    import mujoco

    from arm_sorter.control import GravityCompensatedPID

    controller = GravityCompensatedPID(model)
    data = start_at(model, READY_POSE)  # at rest, so the bias force is gravity alone
    assert controller.gravity_torque(data) == pytest.approx(data.qfrc_bias[:4], abs=1e-12)


def test_gravity_compensation_removes_the_sag(model):
    from arm_sorter.control import GravityCompensatedPID

    q1 = solve_closed_form([0.30, 0.12, 0.05]).q
    traj = joint_move(READY_POSE, q1, model.opt.timestep)
    from arm_sorter.control import pd_gains

    pd = pd_gains(25, 1.0)  # no integral term, so only gravity compensation removes the sag
    plain = track(model, start_at(model, READY_POSE), traj, JointPID(model, pd))
    comp = track(model, start_at(model, READY_POSE), traj, GravityCompensatedPID(model, pd))
    assert comp.final_error().max() < 1e-3
    assert comp.rms_error().max() < plain.rms_error().max() / 3
