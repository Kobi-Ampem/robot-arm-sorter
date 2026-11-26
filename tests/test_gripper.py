import mujoco
import numpy as np
import pytest

from arm_sorter.control import GravityCompensatedPID
from arm_sorter.gripper import CLOSED, OPEN, Gripper
from arm_sorter.ik import READY_POSE, solve_closed_form
from arm_sorter.scene import Scene, Tomato, build
from arm_sorter.sim import track
from arm_sorter.trajectory import cartesian_line, joint_move


def pick_and_lift(radius, xy=(0.25, 0.05), release=False):
    model = build(Scene([Tomato("tomato0", "ripe", radius, xy, (0.8, 0.1, 0.1))]))
    data = mujoco.MjData(model)
    data.qpos[:4] = READY_POSE
    mujoco.mj_forward(model, data)
    controller, gripper, dt = GravityCompensatedPID(model), Gripper(model), model.opt.timestep
    gripper.command(data, OPEN)
    p = np.array([*xy, radius])
    above = p + [0, 0, 0.08]
    track(model, data, joint_move(READY_POSE, solve_closed_form(above).q, dt), controller, settle=0.3)
    track(model, data, cartesian_line(above, p, dt, speed=0.08), controller, settle=0.2)
    gripper.command(data, CLOSED)
    track(model, data, cartesian_line(p, p, dt, min_duration=0.01), controller, settle=0.4)
    track(model, data, cartesian_line(p, p + [0, 0, 0.12], dt, speed=0.08), controller, settle=0.3)
    if release:
        gripper.command(data, OPEN)
        track(model, data, cartesian_line(p + [0, 0, 0.12], p + [0, 0, 0.12], dt, min_duration=0.01),
              controller, settle=0.8)
    return model, data


@pytest.mark.parametrize("radius", [0.025, 0.030, 0.035])
def test_tomato_is_held_while_lifted(radius):
    model, data = pick_and_lift(radius)
    assert np.linalg.norm(data.body("tomato0").xpos - data.site("tcp").xpos) < 5e-3
    # The tomato's weight is not in the gravity model; integral action removes the sag.
    assert data.site("tcp").xpos[2] == pytest.approx(radius + 0.12, abs=1e-3)


def test_opening_releases_the_tomato():
    model, data = pick_and_lift(0.03, release=True)
    assert data.body("tomato0").xpos[2] == pytest.approx(0.03, abs=2e-3)  # back on the table
