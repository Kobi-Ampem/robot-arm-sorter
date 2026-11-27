"""Run many randomised sorting episodes and summarise them.

    python experiments/trials.py [--episodes 200] [--seed0 5000] [--light 0.6 1.1]
"""

import argparse
import time
from collections import Counter

import numpy as np

from arm_sorter.trials import run_episode


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--episodes", type=int, default=200)
    parser.add_argument("--seed0", type=int, default=5000)
    parser.add_argument("--light", type=float, nargs=2, default=(0.6, 1.1))
    args = parser.parse_args()

    start = time.time()
    episodes = [run_episode(s, tuple(args.light)) for s in range(args.seed0, args.seed0 + args.episodes)]
    wall = time.time() - start

    tomatoes = sum(e.tomatoes for e in episodes)
    totals = {k: sum(getattr(e, k) for e in episodes) for k in ("correct", "wrong_bin", "on_tray", "elsewhere", "picks")}
    failures = sum((e.failures for e in episodes), Counter())
    sim = sum(e.sim_time for e in episodes)
    per_tomato = np.array([e.sim_time / e.tomatoes for e in episodes])
    print(f"{args.episodes} episodes (seeds {args.seed0} to {args.seed0 + args.episodes - 1}), "
          f"light {args.light[0]} to {args.light[1]}, {wall:.0f} s wall clock\n")
    print("| measure | value |\n|---|---|")
    print(f"| episodes fully sorted | {sum(e.success for e in episodes)}/{len(episodes)} |")
    print(f"| tomatoes in the correct bin | {totals['correct']}/{tomatoes} ({100 * totals['correct'] / tomatoes:.1f}%) |")
    print(f"| in a wrong bin | {totals['wrong_bin']} |")
    print(f"| left on the tray | {totals['on_tray']} |")
    print(f"| elsewhere (fell off) | {totals['elsewhere']} |")
    print(f"| pick attempts | {totals['picks']} ({totals['picks'] - tomatoes} more than tomatoes) |")
    print(f"| failed attempts by reason | {dict(failures) or 'none'} |")
    print(f"| simulated time per tomato, median (s) | {np.median(per_tomato):.1f} |")
    print(f"| throughput (tomatoes per simulated minute) | {60 * tomatoes / sim:.1f} |")
    bad = [e for e in episodes if not e.success]
    if bad:
        print("\nEpisodes not fully sorted:\n")
        for e in bad:
            print(f"- seed {e.seed}: {e.correct}/{e.tomatoes} correct, wrong bin {e.wrong_bin}, "
                  f"on tray {e.on_tray}, elsewhere {e.elsewhere}, failures {dict(e.failures)}")


if __name__ == "__main__":
    main()
