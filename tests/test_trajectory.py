import numpy as np
import pytest

from arm_sorter.ik import READY_POSE, solve_closed_form
from arm_sorter.trajectory import joint_move, quintic_scaling

DT = 0.002


def test_quintic_end_conditions():
    s, ds, dds = quintic_scaling(np.array([0.0, 1.0]))
    assert s == pytest.approx([0, 1])
    assert ds == pytest.approx([0, 0])
    assert dds == pytest.approx([0, 0])
    assert quintic_scaling(0.5)[1] == pytest.approx(1.875)


def test_joint_move_ends_and_speed_limit():
    q1 = solve_closed_form([0.25, -0.1, 0.06]).q
    traj = joint_move(READY_POSE, q1, DT, v_max=1.5)
    assert traj.q[0] == pytest.approx(READY_POSE)
    assert traj.q[-1] == pytest.approx(q1)
    assert np.abs(traj.qd[[0, -1]]).max() == pytest.approx(0, abs=1e-12)
    assert np.abs(traj.qd).max() <= 1.5 + 1e-3
    assert np.abs(traj.qd).max() >= 1.5 * 0.99  # the slowest-allowed joint sets the duration


def test_velocity_is_the_derivative_of_position():
    traj = joint_move(READY_POSE, READY_POSE + 0.5, DT)
    numeric = np.gradient(traj.q, traj.t, axis=0)
    assert traj.qd[1:-1] == pytest.approx(numeric[1:-1], abs=1e-3)


def test_short_moves_take_the_minimum_duration():
    traj = joint_move(READY_POSE, READY_POSE + 1e-4, DT, min_duration=0.3)
    assert traj.duration == pytest.approx(0.3, abs=DT)
