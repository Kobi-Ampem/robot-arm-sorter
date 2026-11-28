"""Render one sorting episode from the angled camera, plus the camera view with detections.

    python experiments/render_demo.py [--seed 6001] [--out docs]

Writes <out>/demo.gif (small, for the README), experiments/out/demo.mp4 (full size, not
committed) and <out>/perception.png.
"""

import argparse
from pathlib import Path

import cv2
import mujoco

from arm_sorter import scene as sc
from arm_sorter.perception import CLEAR_POSE, Camera, detect
from arm_sorter.trials import run_episode
from arm_sorter.viz import FrameRecorder, draw_detections, write_video


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=6001)
    parser.add_argument("--out", default="docs")
    args = parser.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    # What the overhead camera sees, with what perception found, before anything moves.
    model = sc.build(sc.random_scene(args.seed))
    data = mujoco.MjData(model)
    data.qpos[:4] = CLEAR_POSE
    mujoco.mj_forward(model, data)
    camera = Camera(model)
    rgb = camera.render(data)
    found = detect(camera, data, rgb=rgb)
    camera.close()
    big = cv2.resize(draw_detections(rgb, found), (640, 480), interpolation=cv2.INTER_NEAREST)
    cv2.imwrite(str(out / "perception.png"), big[:, :, ::-1])

    # The episode, one frame every 33 physics steps: 15 frames per simulated second.
    recorders = []

    def on_model(m):
        recorders.append(FrameRecorder(m, every=33))
        return recorders[-1]

    episode = run_episode(args.seed, on_model=on_model)
    recorder = recorders[0]
    recorder.close()
    print(episode)
    # Full video at 2x speed. README GIF at 4x speed and 256 x 192, every third frame, to keep
    # it near 2 MB (every other frame at 320 x 240 came to 5.4 MB).
    write_video(recorder.frames, Path("experiments/out/demo.mp4"), fps=30)
    small = [cv2.resize(f, (256, 192), interpolation=cv2.INTER_AREA) for f in recorder.frames[::3]]
    write_video(small, out / "demo.gif", fps=20)
    print(f"{len(recorder.frames)} frames; GIF {(out / 'demo.gif').stat().st_size / 1e6:.1f} MB")


if __name__ == "__main__":
    main()
