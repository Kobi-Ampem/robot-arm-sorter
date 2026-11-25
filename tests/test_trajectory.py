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


def test_cartesian_line_stays_on_the_line():
    from arm_sorter.kinematics import tcp_pose
    from arm_sorter.trajectory import cartesian_line

    p0, p1 = np.array([0.25, 0.10, 0.12]), np.array([0.25, 0.10, 0.04])
    traj = cartesian_line(p0, p1, DT, speed=0.1)
    direction = (p1 - p0) / np.linalg.norm(p1 - p0)
    for q in traj.q:
        p = tcp_pose(q)[:3, 3]
        off_line = (p - p0) - np.dot(p - p0, direction) * direction
        assert np.linalg.norm(off_line) < 1e-9
    assert tcp_pose(traj.q[-1])[:3, 3] == pytest.approx(p1, abs=1e-9)
    tool_speed = np.linalg.norm(np.diff([tcp_pose(q)[:3, 3] for q in traj.q], axis=0), axis=1) / DT
    assert tool_speed.max() <= 0.1 * 1.01


def test_a_joint_move_between_the_same_points_leaves_the_line():
    """Why the approach uses a Cartesian line: a joint-space move bows away from it."""
    from arm_sorter.kinematics import tcp_pose

    p0, p1 = np.array([0.20, -0.15, 0.12]), np.array([0.30, 0.10, 0.04])
    traj = joint_move(solve_closed_form(p0).q, solve_closed_form(p1).q, DT)
    direction = (p1 - p0) / np.linalg.norm(p1 - p0)
    worst = max(np.linalg.norm((tcp_pose(q)[:3, 3] - p0)
                               - np.dot(tcp_pose(q)[:3, 3] - p0, direction) * direction)
                for q in traj.q)
    assert worst > 0.005


def test_unreachable_line_is_rejected():
    from arm_sorter.trajectory import cartesian_line

    with pytest.raises(ValueError):
        cartesian_line([0.25, 0, 0.05], [0.9, 0, 0.05], DT)


def test_concatenate_checks_the_join():
    from arm_sorter.trajectory import concatenate

    a = joint_move(READY_POSE, READY_POSE + 0.2, DT)
    b = joint_move(READY_POSE + 0.2, READY_POSE, DT)
    both = concatenate([a, b])
    assert both.duration == pytest.approx(a.duration + b.duration)
    assert np.all(np.diff(both.t) > 0)
    with pytest.raises(ValueError):
        concatenate([a, a])
