"""IK benchmark: closed form against damped least squares, and a sweep of the DLS damping.

Targets are the tool points of random top-down poses (so every target is reachable), split into
the whole workspace and the pick area (radius 0.15 to 0.35 m, height up to 0.15 m).

    python experiments/ik_benchmark.py [--n 1000] [--seed 4]
"""

import argparse
import time

import numpy as np

from arm_sorter.ik import TOP_DOWN, solve_closed_form, solve_dls
from arm_sorter.kinematics import tcp_pose
from arm_sorter.workspace import sample_top_down_configs


def pitch_error(pose):
    x = pose[:3, 0]
    return abs(np.arctan2(x[2], np.hypot(x[0], x[1])) - TOP_DOWN)


def evaluate(solver, targets):
    ok, pos_err, ang_err, iters, times = 0, [], [], [], []
    for t in targets:
        start = time.perf_counter()
        result = solver(t)
        times.append(time.perf_counter() - start)
        pose = tcp_pose(result.q)
        ok += result.ok
        if result.ok:
            pos_err.append(np.linalg.norm(pose[:3, 3] - t))
            ang_err.append(pitch_error(pose))
            iters.append(result.iterations)
    n = len(targets)
    return {
        "success": f"{ok}/{n} ({100 * ok / n:.1f}%)",
        "max_pos_err_m": f"{max(pos_err):.1e}" if pos_err else "-",
        "max_pitch_err_rad": f"{max(ang_err):.1e}" if ang_err else "-",
        "median_iters": f"{np.median(iters):.0f}" if iters else "-",
        "median_time_us": f"{1e6 * np.median(times):.0f}",
    }


def row(name, stats):
    return f"| {name} | " + " | ".join(stats.values()) + " |"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--n", type=int, default=1000)
    parser.add_argument("--seed", type=int, default=4)
    args = parser.parse_args()

    targets = np.array([tcp_pose(q)[:3, 3] for q in sample_top_down_configs(args.n, args.seed)])
    radius = np.hypot(targets[:, 0], targets[:, 1])
    pick = targets[(radius >= 0.15) & (radius <= 0.35) & (targets[:, 2] <= 0.15)]

    header = ("| solver | success | max position error (m) | max pitch error (rad) "
              "| median iterations | median time (us) |\n|---|---|---|---|---|---|")
    for label, subset in [(f"whole workspace, {len(targets)} targets", targets),
                          (f"pick area, {len(pick)} targets", pick)]:
        print(f"\n{label}\n\n{header}")
        print(row("closed form", evaluate(solve_closed_form, subset)))
        for damping in [0.001, 0.01, 0.02, 0.05, 0.1, 0.2]:
            stats = evaluate(lambda t: solve_dls(t, damping=damping), subset)
            print(row(f"DLS, damping {damping}", stats))

    # Out of reach (radius 0.50 to 0.60 m, past the 0.48 m reach): no solution exists, so the
    # question is how close the solver gets and how much the joints jump between iterations.
    rng = np.random.default_rng(args.seed)
    angle = rng.uniform(-np.pi, np.pi, 200)
    r = rng.uniform(0.50, 0.60, 200)
    far = np.column_stack([r * np.cos(angle), r * np.sin(angle), rng.uniform(0.02, 0.15, 200)])
    print("\nout of reach, 200 targets\n\n| solver | median final distance (m) "
          "| median last joint step (rad) |\n|---|---|---|")
    for damping in [0.001, 0.01, 0.02, 0.05, 0.1, 0.2]:
        dist, last_step = [], []
        for t in far:
            a = solve_dls(t, damping=damping, max_iter=199)
            b = solve_dls(t, damping=damping, max_iter=200)
            dist.append(np.linalg.norm(tcp_pose(b.q)[:3, 3] - t))
            last_step.append(np.linalg.norm(b.q - a.q))
        print(f"| DLS, damping {damping} | {np.median(dist):.4f} | {np.median(last_step):.1e} |")


if __name__ == "__main__":
    main()
