import mujoco
import numpy as np
import pytest

from arm_sorter import scene as sc
from arm_sorter.perception import CLEAR_POSE, Camera, detect
from arm_sorter.task import Arm, Phase, pick_and_place


def setup(scene):
    model = sc.build(scene)
    data = mujoco.MjData(model)
    data.qpos[:4] = CLEAR_POSE
    mujoco.mj_forward(model, data)
    return model, data, Arm(model, data)


@pytest.mark.parametrize("ripeness,rgb", [("ripe", (0.8, 0.1, 0.08)), ("turning", (0.85, 0.5, 0.1)),
                                          ("green", (0.3, 0.7, 0.2))])
def test_one_tomato_ends_in_its_bin(ripeness, rgb):
    scene = sc.Scene([sc.Tomato("tomato0", ripeness, 0.03, (0.26, -0.04), rgb)])
    model, data, arm = setup(scene)
    camera = Camera(model)
    (det,) = detect(camera, data)
    camera.close()
    out = pick_and_place(arm, det)
    assert out.success, out
    assert out.reached is Phase.DONE
    arm.wait(1.0)  # let it settle in the bin
    assert sc.in_bin(data.body("tomato0").xpos, ripeness)


def test_empty_grasp_is_noticed():
    scene = sc.Scene([sc.Tomato("tomato0", "ripe", 0.03, (0.26, -0.04), (0.8, 0.1, 0.08))])
    model, data, arm = setup(scene)
    camera = Camera(model)
    (det,) = detect(camera, data)
    camera.close()
    data.qpos[model.joint("tomato0").qposadr[0] + 1] += 0.15  # tomato moved after it was seen
    mujoco.mj_forward(model, data)
    out = pick_and_place(arm, det)
    assert not out.success and out.failure == "nothing between the fingers"


def test_every_lift_and_carry_is_reachable_and_clears_the_tray():
    from arm_sorter.ik import solve_closed_form
    from arm_sorter.kinematics import tcp_position
    from arm_sorter.task import CARRY_HEIGHT, carry_path
    from arm_sorter.trajectory import joint_move

    tallest_top = 2 * 0.035
    for x in np.linspace(*sc.PICK_X, 6):
        for y in np.linspace(*sc.PICK_Y, 6):
            lifted = solve_closed_form([x, y, CARRY_HEIGHT])
            assert lifted.ok, (x, y)
            for ripeness in sc.CLASSES:
                for radius in (0.025, 0.035):
                    q = lifted.q
                    for waypoint in carry_path(ripeness, radius, q):
                        low = min(tcp_position(p)[2] for p in joint_move(q, waypoint, 0.002).q[::5])
                        assert low - radius > tallest_top + 0.02
                        q = waypoint
