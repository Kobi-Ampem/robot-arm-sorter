"""Why did tomatoes end up off the table? Drop height, rolling friction and bin wall height.

Runs the 12 episodes that lost a tomato in the first 200-episode run (seeds below) plus 40 that
did not (seeds 5000 to 5039), for each setting. Settings are patched in at run time.

    python experiments/bin_drop_sweep.py
"""

import itertools
from collections import Counter

import arm_sorter.scene as sc
import arm_sorter.task as task
from arm_sorter.trials import run_episode

FAILING = [5041, 5052, 5081, 5115, 5118, 5124, 5135, 5158, 5166, 5168, 5176, 5199]
OTHERS = list(range(5000, 5040))
TOMATO_XML = sc._tomato_xml


def with_rolling_friction(t):
    # condim 6 adds rolling friction (third friction value) so a tomato cannot roll forever.
    return TOMATO_XML(t).replace('condim="4" friction="1 0.01 0.001"', 'condim="6" friction="1 0.01 0.002"')


def run(label):
    failing = [run_episode(s) for s in FAILING]
    others = [run_episode(s) for s in OTHERS]
    lost = sum(e.tomatoes - e.correct for e in failing + others)
    failures = sum((e.failures for e in failing + others), Counter())
    print(f"| {label} | {sum(e.success for e in failing)}/12 | {sum(e.success for e in others)}/40 "
          f"| {lost} | {dict(failures) or 'none'} |")


def main():
    header = ("| setting | previously failing episodes now sorted | other episodes sorted "
              "| tomatoes lost | failed picks |\n|---|---|---|---|---|")
    walls = sc.BIN_WALL_HEIGHT
    print("Bin walls 0.04 m: drop clearance and rolling friction\n\n" + header)
    sc.BIN_WALL_HEIGHT = task.BIN_WALL_HEIGHT = 0.04
    for clearance, roll in itertools.product([0.06, 0.03, 0.01], [False, True]):
        task.DROP_CLEARANCE = clearance
        sc._tomato_xml = with_rolling_friction if roll else TOMATO_XML
        run(f"clearance {clearance} m, rolling friction {'on' if roll else 'off'}")
    sc._tomato_xml = TOMATO_XML
    print("\nDrop clearance 0.01 m: bin wall height\n\n" + header)
    task.DROP_CLEARANCE = 0.01
    for wall in (0.06, 0.07, 0.08):
        sc.BIN_WALL_HEIGHT = task.BIN_WALL_HEIGHT = wall
        run(f"walls {wall} m")
    sc.BIN_WALL_HEIGHT = task.BIN_WALL_HEIGHT = walls


if __name__ == "__main__":
    main()
