"""Find tomatoes in the overhead camera image, grade their ripeness and locate them on the table.

Pipeline: render RGB, convert to HSV, keep strongly coloured pixels (the tray, arm and bin
edges are grey or dull blue), split them into blobs, grade each blob by its median hue, and
cast a ray from the camera through the blob centre onto the plane at the tomato's centre height.
"""

from dataclasses import dataclass

import cv2
import mujoco
import numpy as np

# Arm pose that keeps the arm out of the camera's view of the pick area.
CLEAR_POSE = np.array([-1.842, 1.501, -1.216, -1.855])

WIDTH, HEIGHT = 320, 240


@dataclass
class Thresholds:
    """HSV limits, with OpenCV's scales: hue 0 to 179 (degrees / 2), saturation and value 0 to 255.

    Ripe hues wrap around red: below ripe_max or at or above ripe_wrap.
    Chosen by the threshold sweep in experiments/RESULTS.md.
    """
    s_min: int = 100
    v_min: int = 20
    ripe_max: int = 8
    ripe_wrap: int = 170
    turning_max: int = 25
    green_min: int = 30
    green_max: int = 75
    min_area: int = 40  # pixels; smaller blobs are noise or a sliver of a tomato at the edge


@dataclass
class Detection:
    ripeness: str | None  # None if the hue fits no class
    xyz: np.ndarray        # estimated centre on the table (m)
    radius: float          # estimated radius (m)
    pixel: tuple[float, float]
    hue: float
    area: int


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

        (u, v) are pixel indices as OpenCV reports them: pixel (0, 0) covers the image corner
        and its centre is at (0.5, 0.5) in image-plane units, hence the + 0.5.
        MuJoCo cameras look along their -z axis with +y up; image rows grow downwards.
        """
        x = (u + 0.5 - self.width / 2) / self.f
        y = -(v + 0.5 - self.height / 2) / self.f
        d_cam = np.array([x, y, -1.0])
        rot = data.cam_xmat[self.cam_id].reshape(3, 3)
        return data.cam_xpos[self.cam_id].copy(), rot @ d_cam

    def close(self):
        self.renderer.close()


def classify_hue(hue: float, th: Thresholds) -> str | None:
    if hue < th.ripe_max or hue >= th.ripe_wrap:
        return "ripe"
    if hue < th.turning_max:
        return "turning"
    if th.green_min <= hue < th.green_max:
        return "green"
    return None


def mask(rgb: np.ndarray, th: Thresholds) -> np.ndarray:
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    keep = (hsv[..., 1] >= th.s_min) & (hsv[..., 2] >= th.v_min)
    # Drop the blue of the bins (hue about 110 on OpenCV's scale) whatever its saturation.
    keep &= (hsv[..., 0] < 90) | (hsv[..., 0] >= th.ripe_wrap)
    return keep.astype(np.uint8)


def detect(camera: Camera, data: mujoco.MjData, th: Thresholds | None = None,
           rgb: np.ndarray | None = None) -> list[Detection]:
    th = th or Thresholds()
    rgb = camera.render(data) if rgb is None else rgb
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask(rgb, th), connectivity=8)
    cam_z = data.cam_xpos[camera.cam_id][2]
    found = []
    for i in range(1, count):
        area = int(stats[i, cv2.CC_STAT_AREA])
        if area < th.min_area:
            continue
        hues = hsv[..., 0][labels == i].astype(float)
        # Median of hues rotated so red sits in the middle, not split across 0 and 179.
        hue = float((np.median((hues + 90) % 180) - 90) % 180)
        u, v = centroids[i]
        # Silhouette radius in pixels; a sphere of radius r whose centre is cam_z - r below the
        # camera appears with radius r_px = f r / (cam_z - r), so r = r_px cam_z / (f + r_px).
        r_px = np.sqrt(area / np.pi)
        radius = r_px * cam_z / (camera.f + r_px)
        origin, direction = camera.ray(data, u, v)
        t = (radius - origin[2]) / direction[2]
        found.append(Detection(classify_hue(hue, th), origin + t * direction, float(radius),
                               (float(u), float(v)), hue, area))
    return found
