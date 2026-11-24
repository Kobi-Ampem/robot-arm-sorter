import numpy as np
import pytest

from arm_sorter.ik import READY_POSE, TOP_DOWN, solve_closed_form, solve_dls
from arm_sorter.kinematics import tcp_pose
from arm_sorter.workspace import sample_top_down_configs as top_down_configs


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


def test_ready_pose_points_down():
    assert pitch_of(tcp_pose(READY_POSE)) == pytest.approx(TOP_DOWN, abs=1e-12)


def in_pick_area(target):
    return 0.15 <= np.hypot(target[0], target[1]) <= 0.35 and target[2] <= 0.15


def test_dls_reaches_targets_in_the_pick_area():
    targets = [tcp_pose(q)[:3, 3] for q in top_down_configs(400, seed=4)]
    targets = [t for t in targets if in_pick_area(t)]
    assert len(targets) > 50
    for target in targets:
        result = solve_dls(target)
        assert result.ok, (target, result)
        pose = tcp_pose(result.q)
        assert pose[:3, 3] == pytest.approx(target, abs=1e-6)
        assert pitch_of(pose) == pytest.approx(TOP_DOWN, abs=1e-5)


def test_dls_reports_unreachable_target():
    result = solve_dls([1.0, 0.0, 0.05])
    assert not result.ok and result.reason == "did not converge"
