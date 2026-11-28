"""Render a sorting episode to video, and draw detections on a camera image."""

from pathlib import Path

import cv2
import mujoco
import numpy as np

from .perception import Detection

CLASS_COLOURS = {"ripe": (230, 40, 40), "turning": (240, 150, 30), "green": (60, 200, 60), None: (255, 255, 255)}


class FrameRecorder:
    """Called after every physics step; keeps one frame every `every` steps."""

    def __init__(self, model: mujoco.MjModel, camera="demo", width=640, height=480, every=33):
        self.renderer = mujoco.Renderer(model, height, width)
        self.camera, self.every = camera, every
        self.frames: list[np.ndarray] = []
        self.steps = 0

    def __call__(self, data: mujoco.MjData):
        if self.steps % self.every == 0:
            self.renderer.update_scene(data, camera=self.camera)
            self.frames.append(self.renderer.render().copy())
        self.steps += 1

    def close(self):
        self.renderer.close()


def write_video(frames, path: Path, fps: int):
    import imageio.v2 as imageio

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.suffix == ".gif":
        imageio.mimsave(path, frames, duration=1000 / fps, loop=0)
    else:
        imageio.mimsave(path, frames, fps=fps, macro_block_size=8)


def draw_detections(rgb: np.ndarray, detections: list[Detection]) -> np.ndarray:
    out = rgb.copy()
    for d in detections:
        colour = CLASS_COLOURS[d.ripeness]
        u, v = int(round(d.pixel[0])), int(round(d.pixel[1]))
        r = int(round(np.sqrt(d.area / np.pi)))
        cv2.circle(out, (u, v), r + 3, colour, 1, cv2.LINE_AA)
        cv2.putText(out, d.ripeness or "?", (u - r, v - r - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.35, colour, 1,
                    cv2.LINE_AA)
    return out
