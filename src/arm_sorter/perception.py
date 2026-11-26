"""The overhead camera: render images and turn pixels into rays in the world."""

from dataclasses import dataclass

import mujoco
import numpy as np

# Arm pose that keeps the arm out of the camera's view of the pick area.
CLEAR_POSE = np.array([-1.842, 1.501, -1.216, -1.855])

WIDTH, HEIGHT = 320, 240


class Camera:
    def __init__(self, model: mujoco.MjModel, name: str = "overhead",
                 width: int = WIDTH, height: int = HEIGHT):
        self.model = model
        self.name = name
        self.cam_id = model.camera(name).id
        self.width, self.height = width, height
        self.renderer = mujoco.Renderer(model, height, width)
        # Focal length in pixels from the vertical field of view.
        self.f = (height / 2) / np.tan(np.radians(model.cam_fovy[self.cam_id]) / 2)

    def render(self, data: mujoco.MjData) -> np.ndarray:
        self.renderer.update_scene(data, camera=self.name)
        return self.renderer.render()

    def ray(self, data: mujoco.MjData, u: float, v: float) -> tuple[np.ndarray, np.ndarray]:
        """Origin and direction in world coordinates of the ray through pixel (u, v).

        MuJoCo cameras look along their -z axis with +y up; image rows grow downwards.
        """
        d_cam = np.array([(u - self.width / 2) / self.f, -(v - self.height / 2) / self.f, -1.0])
        rot = data.cam_xmat[self.cam_id].reshape(3, 3)
        return data.cam_xpos[self.cam_id].copy(), rot @ d_cam

    def close(self):
        self.renderer.close()
