"""Contact settings for holding a tomato: pick one tomato, lift it 0.12 m, measure the slip.

Each variant starts from the current arm model and removes some of the chosen settings, so the
first row is MuJoCo's defaults and the last row is what assets/arm.xml uses.

    python experiments/grasp_contact_sweep.py
"""

import mujoco
import numpy as np

from arm_sorter import scene as sc
from arm_sorter.control import GravityCompensatedPID
from arm_sorter.gripper import CLOSED, OPEN, Gripper
from arm_sorter.ik import READY_POSE, solve_closed_form
from arm_sorter.sim import track
from arm_sorter.trajectory import cartesian_line, joint_move

CONE = ' cone="elliptic" impratio="10"'
EQUALITY = ' polycoef="0 1 0 0 0" solref="0.004 1"'
FINGER_CONTACT = ' solref="0.004 1" solimp="0.95 0.99 0.001"'


def without(*parts):
    def edit(xml):
        for part in parts:
            if part not in xml:
                raise ValueError(f"setting not found in the model: {part}")
            xml = xml.replace(part, ' polycoef="0 1 0 0 0"' if part == EQUALITY else "")
        return xml
    return edit


VARIANTS = {
    "MuJoCo defaults": without(CONE, EQUALITY, FINGER_CONTACT),
    "elliptic cone, impratio 10": without(EQUALITY, FINGER_CONTACT),
    "+ stiff finger coupling": without(FINGER_CONTACT),
    "+ stiff finger contact (chosen)": without(),
}


def trial(xml_edit, radius, xy=(0.25, 0.05)):
    """Returns (tomato-to-tool distance after the lift, finger mismatch when closed), in m."""
    scene = sc.Scene([sc.Tomato("tomato0", "ripe", radius, xy, (0.8, 0.1, 0.1))])
    model = mujoco.MjModel.from_xml_string(xml_edit(sc.scene_xml(scene)))
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
    mismatch = abs(data.joint("finger_left").qpos[0] - data.joint("finger_right").qpos[0])
    track(model, data, cartesian_line(p, p + [0, 0, 0.12], dt, speed=0.08), controller, settle=0.5)
    slip = np.linalg.norm(data.body("tomato0").xpos - data.site("tcp").xpos)
    return slip, mismatch


def main():
    print("| setting | radius (mm) | held? tomato-to-tool distance after lift (mm) "
          "| finger mismatch when closed (mm) |\n|---|---|---|---|")
    for name, edit in VARIANTS.items():
        for radius in (0.025, 0.030, 0.035):
            slip, mismatch = trial(edit, radius)
            held = "yes" if slip < 0.01 else "NO"
            print(f"| {name} | {radius * 1000:.0f} | {held}: {slip * 1000:.1f} | {mismatch * 1000:.1f} |")


if __name__ == "__main__":
    main()
