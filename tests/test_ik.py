import numpy as np
import pytest

from arm_sorter.ik import TOP_DOWN, solve_closed_form, within_limits
from arm_sorter.kinematics import JOINT_LIMITS, L2, L3, tcp_pose


def top_down_configs(n, seed=0):
    """Random elbow-up configurations within limits whose tool points straight down above the
    table and in front of the base. Configurations with the tool behind the base (shoulder tipped
    past vertical) have a second, equivalent solution with the base turned by 180 degrees, which
    is the one the solver returns."""
    rng = np.random.default_rng(seed)
    configs = []
    while len(configs) < n:
        q = rng.uniform(JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])
        q[3] = TOP_DOWN - q[1] - q[2]
        radial = L2 * np.cos(q[1]) + L3 * np.cos(q[1] + q[2])  # tool reach in the arm's plane
        if within_limits(q) and tcp_pose(q)[2, 3] > 0.0 and q[2] < 0 and radial > 0:
            configs.append(q)
    return np.array(configs)


def pitch_of(pose):
    x_axis = pose[:3, 0]
    return np.arctan2(x_axis[2], np.hypot(x_axis[0], x_axis[1]))


def test_elbow_down_hits_the_wrist_limit_low_and_close():
    result = solve_closed_form([0.20, 0.0, 0.20], elbow_up=False)
    assert not result.ok and result.reason == "joint limits"


def test_closed_form_reaches_targets():
    for q_true in top_down_configs(300):
        target = tcp_pose(q_true)[:3, 3]
        result = solve_closed_form(target)
        assert result.ok, result.reason
        pose = tcp_pose(result.q)
        assert pose[:3, 3] == pytest.approx(target, abs=1e-9)
        assert pitch_of(pose) == pytest.approx(TOP_DOWN, abs=1e-9)


def test_closed_form_recovers_the_elbow_up_solution():
    for q_true in top_down_configs(100, seed=1):
        result = solve_closed_form(tcp_pose(q_true)[:3, 3])
        assert result.q == pytest.approx(q_true, abs=1e-9)


def test_target_behind_base_is_reached_facing_it():
    result = solve_closed_form([-0.25, 0.0, 0.05])
    assert result.ok
    assert abs(result.q[0]) == pytest.approx(np.pi)
    assert tcp_pose(result.q)[:3, 3] == pytest.approx([-0.25, 0.0, 0.05], abs=1e-9)


def test_elbow_down_also_reaches():
    # Elbow down needs a sharp wrist bend, so it only fits far, high targets within the wrist limit.
    target = np.array([0.35, 0.05, 0.12])
    up, down = solve_closed_form(target), solve_closed_form(target, elbow_up=False)
    assert up.ok and down.ok
    assert up.q[2] < 0 < down.q[2]
    assert tcp_pose(down.q)[:3, 3] == pytest.approx(target, abs=1e-9)


def test_out_of_reach():
    result = solve_closed_form([1.0, 0.0, 0.05])
    assert not result.ok and result.reason == "out of reach"
