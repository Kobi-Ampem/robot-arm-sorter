"""Pick-and-place: move one detected tomato into the bin for its ripeness."""

from dataclasses import dataclass, field
from enum import Enum

import mujoco
import numpy as np

from .control import GravityCompensatedPID
from .gripper import CLOSED, OPEN, Gripper
from .ik import solve_closed_form
from .perception import CLEAR_POSE, Detection
from .scene import BIN_WALL_HEIGHT, BINS
from .sim import track
from .trajectory import cartesian_line, joint_move

APPROACH_HEIGHT = 0.08  # start the straight descent this far above the grasp point (m)
CARRY_HEIGHT = 0.14     # tool height while carrying; the highest reachable over the whole pick area
                        # with margin (0.17 failed at its far corners, see docs/LOG.md)
DROP_CLEARANCE = 0.06   # gap between the bottom of a held tomato and the bin rim at release
EMPTY_GRIP = 0.008      # fingers closing past this opening (m) means nothing was grasped
LINE_SPEED = 0.08       # m/s for straight moves near the tomato


class Phase(Enum):
    APPROACH = "approach"
    DESCEND = "descend"
    GRASP = "grasp"
    LIFT = "lift"
    CARRY = "carry"
    RELEASE = "release"
    RETREAT = "retreat"
    DONE = "done"


@dataclass
class PickOutcome:
    detection: Detection
    target_bin: str | None
    reached: Phase                  # last phase entered
    success: bool                   # as far as the robot can tell: grasped and released
    failure: str = ""
    sim_time: float = 0.0
    events: list = field(default_factory=list)


class Arm:
    """The arm in a running simulation: joint and straight-line moves, gripper, waiting."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, on_step=None):
        self.model, self.data, self.on_step = model, data, on_step
        self.controller = GravityCompensatedPID(model)
        self.gripper = Gripper(model)
        self.dt = model.opt.timestep

    @property
    def q(self) -> np.ndarray:
        return self.data.qpos[self.controller.qpos_idx].copy()

    def tool(self) -> np.ndarray:
        return self.data.site("tcp").xpos.copy()

    def move_joints(self, q_goal, settle=0.15):
        track(self.model, self.data, joint_move(self.q, q_goal, self.dt), self.controller,
              settle=settle, on_step=self.on_step)

    def move_to(self, *waypoints, settle=0.15):
        """Joint moves through each waypoint: a tool point (3 values) or joint angles (4)."""
        for w in waypoints:
            w = np.asarray(w, float)
            if w.shape == (3,):
                goal = solve_closed_form(w)
                if not goal.ok:
                    raise ValueError(f"cannot reach {np.round(w, 3)}: {goal.reason}")
                w = goal.q
            self.move_joints(w, settle)

    def line(self, p0, p1, settle=0.15):
        track(self.model, self.data, cartesian_line(p0, p1, self.dt, speed=LINE_SPEED),
              self.controller, settle=settle, on_step=self.on_step)

    def wait(self, seconds):
        q = self.q
        track(self.model, self.data, joint_move(q, q, self.dt, min_duration=self.dt),
              self.controller, settle=seconds, on_step=self.on_step)

    def grip(self, opening, seconds=0.4):
        self.gripper.command(self.data, opening)
        self.wait(seconds)


def drop_point(ripeness: str, radius: float) -> np.ndarray:
    bx, by = BINS[ripeness]
    return np.array([bx, by, BIN_WALL_HEIGHT + DROP_CLEARANCE + 2 * radius])


def carry_path(ripeness: str, radius: float, q_lifted) -> list[np.ndarray]:
    """Joint waypoints from the lifted pose to the drop pose above the bin.

    First only the base turns, which keeps the tool at the carry height; then the arm reaches out
    in its own plane. A single joint move from lifted to drop dipped the tool to 0.100 m, low
    enough for a held tomato to hit a tall one on the tray (docs/LOG.md).
    """
    goal = solve_closed_form(drop_point(ripeness, radius))
    if not goal.ok:
        raise ValueError(f"cannot reach the {ripeness} bin: {goal.reason}")
    turned = np.array(q_lifted, dtype=float)
    turned[0] = goal.q[0]
    return [turned, goal.q]


def pick_and_place(arm: Arm, det: Detection) -> PickOutcome:
    """Run one pick: approach above, descend, grasp, lift, carry to the bin, release, retreat."""
    start = arm.data.time
    out = PickOutcome(det, det.ripeness, Phase.APPROACH, False)

    def enter(phase):
        out.reached = phase
        out.events.append((round(arm.data.time - start, 3), phase.value))

    if det.ripeness is None:
        out.failure = "unknown ripeness"
        return out
    grasp = np.array([det.xyz[0], det.xyz[1], det.radius])
    above = grasp + [0, 0, APPROACH_HEIGHT]
    try:
        arm.gripper.command(arm.data, OPEN)
        arm.move_to(above)
        enter(Phase.DESCEND)
        arm.line(above, grasp)
        enter(Phase.GRASP)
        arm.grip(CLOSED)
        if arm.gripper.opening(arm.data) < EMPTY_GRIP:
            out.failure = "nothing between the fingers"
            arm.grip(OPEN, 0.2)
            arm.line(grasp, above)
            return out
        enter(Phase.LIFT)
        lifted = np.array([grasp[0], grasp[1], CARRY_HEIGHT])
        arm.line(grasp, lifted)
        enter(Phase.CARRY)
        arm.move_to(*carry_path(det.ripeness, det.radius, arm.q))
        if arm.gripper.opening(arm.data) < EMPTY_GRIP:
            out.failure = "dropped while carrying"
            return out
        enter(Phase.RELEASE)
        arm.grip(OPEN, 0.5)
        enter(Phase.RETREAT)
        arm.move_joints(CLEAR_POSE)
        enter(Phase.DONE)
        out.success = True
    except ValueError as e:
        out.failure = str(e)
    finally:
        out.sim_time = arm.data.time - start
    return out
