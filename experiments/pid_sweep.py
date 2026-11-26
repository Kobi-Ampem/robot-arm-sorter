"""PID gain sweep, with and without gravity compensation.

Gains come from a natural frequency omega and damping ratio zeta per joint (control.pd_gains).
Each setting runs the same 20 moves from the ready pose to random pick-area targets, then holds
for 0.5 s. Reported: worst tool-point error during the moves and at the end, the share of control
steps where any motor saturated, and the share of moves that ended within 1 mm.

    python experiments/pid_sweep.py [--moves 20] [--seed 0]
"""

import argparse

import numpy as np

from arm_sorter.control import GravityCompensatedPID, JointPID, pd_gains
from arm_sorter.ik import READY_POSE, solve_closed_form
from arm_sorter.kinematics import tcp_position
from arm_sorter.model import load_model
from arm_sorter.sim import start_at, track
from arm_sorter.trajectory import joint_move


def pick_targets(n, seed):
    rng = np.random.default_rng(seed)
    out = []
    while len(out) < n:
        r, a, z = rng.uniform(0.18, 0.32), rng.uniform(-0.6, 0.6), rng.uniform(0.03, 0.12)
        result = solve_closed_form([r * np.cos(a), r * np.sin(a), z])
        if result.ok:
            out.append(result.q)
    return out


def run(model, controller_cls, gains, goals):
    worst_track, worst_final, saturated, steps, within = 0.0, 0.0, 0, 0, 0
    limits = model.actuator_ctrlrange[:4]
    for q1 in goals:
        log = track(model, start_at(model, READY_POSE),
                    joint_move(READY_POSE, q1, model.opt.timestep), controller_cls(model, gains))
        if not np.all(np.isfinite(log.q)):
            return None
        err = np.array([np.linalg.norm(tcp_position(a) - tcp_position(b))
                        for a, b in zip(log.q_ref[::10], log.q[::10])])
        final = np.linalg.norm(tcp_position(log.q_ref[-1]) - tcp_position(log.q[-1]))
        worst_track = max(worst_track, err.max())
        worst_final = max(worst_final, final)
        within += final < 1e-3
        at_limit = (np.isclose(log.tau, limits[:, 0]) | np.isclose(log.tau, limits[:, 1])).any(axis=1)
        saturated += at_limit.sum()
        steps += len(log.tau)
    return worst_track, worst_final, saturated / steps, within / len(goals)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--moves", type=int, default=20)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()
    model = load_model()
    goals = pick_targets(args.moves, args.seed)
    print("| controller | omega (rad/s) | zeta | worst tracking error (mm) | worst final error (mm) "
          "| steps saturated | moves ending within 1 mm |\n|---|---|---|---|---|---|---|")
    for name, cls in [("PD", JointPID), ("PD + gravity", GravityCompensatedPID)]:
        for omega in [10, 15, 25, 35, 50, 80]:
            for zeta in [0.5, 1.0, 1.5]:
                out = run(model, cls, pd_gains(omega, zeta), goals)
                if out is None:
                    print(f"| {name} | {omega} | {zeta} | unstable | | | |")
                    continue
                track_err, final_err, sat, within = out
                print(f"| {name} | {omega} | {zeta} | {1e3 * track_err:.2f} | {1e3 * final_err:.3f} "
                      f"| {100 * sat:.1f}% | {100 * within:.0f}% |")


if __name__ == "__main__":
    main()
