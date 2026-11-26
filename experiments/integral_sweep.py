"""Integral time sweep: does integral action remove the sag under a tomato, and at what cost?

The gravity model covers the arm but not what it carries, so a held tomato pulls the tool down.
For each integral time Ti (ki = kp / Ti), with omega 25 rad/s and zeta 1.0:
  - payload: pick and lift one tomato of each size and measure how far the tool sags below its
    target after a 0.5 s hold;
  - no payload: the 20 moves of pid_sweep.py, worst tracking error and worst overshoot past the
    goal, to see whether the integral term winds up during moves.

    python experiments/integral_sweep.py
"""

import mujoco
import numpy as np

from arm_sorter import scene as sc
from arm_sorter.control import GravityCompensatedPID, pd_gains
from arm_sorter.gripper import CLOSED, OPEN, Gripper
from arm_sorter.ik import READY_POSE, solve_closed_form
from arm_sorter.kinematics import tcp_position
from arm_sorter.model import load_model
from arm_sorter.sim import start_at, track
from arm_sorter.trajectory import cartesian_line, joint_move

INTEGRAL_TIMES = [None, 2.0, 1.0, 0.5, 0.25, 0.1]


def payload_sag(gains, radius, xy=(0.25, 0.05)):
    model = sc.build(sc.Scene([sc.Tomato("tomato0", "ripe", radius, xy, (0.8, 0.1, 0.1))]))
    data = mujoco.MjData(model)
    data.qpos[:4] = READY_POSE
    mujoco.mj_forward(model, data)
    controller, gripper, dt = GravityCompensatedPID(model, gains), Gripper(model), model.opt.timestep
    gripper.command(data, OPEN)
    p = np.array([*xy, radius])
    above, lifted = p + [0, 0, 0.08], p + [0, 0, 0.12]
    track(model, data, joint_move(READY_POSE, solve_closed_form(above).q, dt), controller, settle=0.3)
    track(model, data, cartesian_line(above, p, dt, speed=0.08), controller, settle=0.2)
    gripper.command(data, CLOSED)
    track(model, data, cartesian_line(p, p, dt, min_duration=0.01), controller, settle=0.4)
    track(model, data, cartesian_line(p, lifted, dt, speed=0.08), controller, settle=0.5)
    return lifted[2] - data.site("tcp").xpos[2]


def free_moves(model, gains, n=20, seed=0):
    rng = np.random.default_rng(seed)
    worst_track = worst_overshoot = 0.0
    done = 0
    while done < n:
        r, a, z = rng.uniform(0.18, 0.32), rng.uniform(-0.6, 0.6), rng.uniform(0.03, 0.12)
        goal = solve_closed_form([r * np.cos(a), r * np.sin(a), z])
        if not goal.ok:
            continue
        done += 1
        log = track(model, start_at(model, READY_POSE),
                    joint_move(READY_POSE, goal.q, model.opt.timestep), GravityCompensatedPID(model, gains))
        ref, act = log.q_ref[::10], log.q[::10]
        err = [np.linalg.norm(tcp_position(a) - tcp_position(b)) for a, b in zip(ref, act)]
        worst_track = max(worst_track, max(err))
        # Overshoot: how far any joint goes past its goal, beyond where the reference ever went.
        delta = goal.q - READY_POSE
        past = (log.q - goal.q) * np.sign(delta)
        worst_overshoot = max(worst_overshoot, past.max())
    return worst_track, worst_overshoot


def main():
    model = load_model()
    print("| integral time Ti (s) | tool sag, 25 / 30 / 35 mm tomato (mm) "
          "| worst tracking error, no payload (mm) | worst joint overshoot (mrad) |\n|---|---|---|---|")
    for ti in INTEGRAL_TIMES:
        gains = pd_gains(25, 1.0, integral_time=ti)
        sags = " / ".join(f"{1e3 * payload_sag(gains, r):.1f}" for r in (0.025, 0.030, 0.035))
        track_err, overshoot = free_moves(model, gains)
        label = "none" if ti is None else f"{ti}"
        print(f"| {label} | {sags} | {1e3 * track_err:.2f} | {1e3 * overshoot:.2f} |")


if __name__ == "__main__":
    main()
