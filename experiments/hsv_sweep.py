"""HSV threshold sweep for tomato detection and ripeness grading.

Renders random scenes once (arm parked at CLEAR_POSE), then scores each threshold setting on the
saved images. A tomato counts as found if a detection lies within 15 mm of it; it is graded
correctly if that detection has its class. False positives are detections matching no tomato.

Caveat: the ripeness classes are defined by the hue ranges the scene generator uses, so this
measures robustness to lighting, shading, size and position, not real-world ripeness grading.

    python experiments/hsv_sweep.py [--scenes 500] [--light 0.6 1.1]
"""

import argparse
import itertools
from dataclasses import replace

import mujoco
import numpy as np

from arm_sorter import scene as sc
from arm_sorter.perception import CLEAR_POSE, Camera, Thresholds, detect


def render_all(n, light, seed0=1000):
    out = []
    for seed in range(seed0, seed0 + n):
        scene = sc.random_scene(seed, light_range=light)
        model = sc.build(scene)
        data = mujoco.MjData(model)
        data.qpos[:4] = CLEAR_POSE
        mujoco.mj_forward(model, data)
        camera = Camera(model)
        rgb = camera.render(data)
        # Free the GL context now: each holds a large offscreen buffer, and 500 open ones ran
        # the GPU out of memory (docs/LOG.md). Rays and detection do not need the renderer.
        camera.close()
        out.append((scene, camera, data, rgb))
    return out


def score(renders, th):
    tomatoes = found = correct = false_pos = 0
    pos_err, confusion = [], {}
    for scene, camera, data, rgb in renders:
        dets = detect(camera, data, th, rgb=rgb)
        used = set()
        for t in scene.tomatoes:
            tomatoes += 1
            dist = [np.linalg.norm(d.xyz[:2] - np.array(t.xy)) for d in dets]
            if not dist or min(dist) > 0.015:
                confusion[(t.ripeness, "missed")] = confusion.get((t.ripeness, "missed"), 0) + 1
                continue
            k = int(np.argmin(dist))
            used.add(k)
            found += 1
            pos_err.append(dist[k])
            correct += dets[k].ripeness == t.ripeness
            key = (t.ripeness, dets[k].ripeness or "unknown")
            confusion[key] = confusion.get(key, 0) + 1
        false_pos += len(dets) - len(used)
    return tomatoes, found, correct, false_pos, np.array(pos_err), confusion


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenes", type=int, default=500)
    parser.add_argument("--light", type=float, nargs=2, default=(0.6, 1.1))
    args = parser.parse_args()
    renders = render_all(args.scenes, tuple(args.light))
    base = Thresholds()
    print(f"{args.scenes} scenes, light {args.light[0]} to {args.light[1]}\n")
    print("| s_min | v_min | turning_max | found | graded correctly | false positives "
          "| median / max position error (mm) |\n|---|---|---|---|---|---|---|")
    for s_min, v_min, turning_max in itertools.product([60, 100, 140, 180], [20, 40, 80], [20, 25, 30]):
        th = replace(base, s_min=s_min, v_min=v_min, turning_max=turning_max)
        n, found, correct, fp, err, _ = score(renders, th)
        errs = f"{1e3 * np.median(err):.1f} / {1e3 * err.max():.1f}" if len(err) else "-"
        print(f"| {s_min} | {v_min} | {turning_max} | {found}/{n} | {correct}/{n} ({100 * correct / n:.1f}%) "
              f"| {fp} | {errs} |")
    n, found, correct, fp, err, confusion = score(renders, base)
    print(f"\nConfusion for the defaults {base}:\n")
    print("| true \\ graded | " + " | ".join(sc.CLASSES) + " | unknown | missed |\n|---|---|---|---|---|---|")
    for true in sc.CLASSES:
        cells = [str(confusion.get((true, g), 0)) for g in (*sc.CLASSES, "unknown", "missed")]
        print(f"| {true} | " + " | ".join(cells) + " |")


if __name__ == "__main__":
    main()
