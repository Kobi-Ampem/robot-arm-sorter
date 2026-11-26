import mujoco
import numpy as np
import pytest

from arm_sorter import scene as sc
from arm_sorter.perception import CLEAR_POSE, Camera


def view(scene):
    model = sc.build(scene)
    data = mujoco.MjData(model)
    data.qpos[:4] = CLEAR_POSE
    mujoco.mj_forward(model, data)
    return model, data


def test_camera_ray_hits_a_known_point():
    model, data = view(sc.Scene([]))
    camera = Camera(model)
    origin, direction = camera.ray(data, camera.width / 2, camera.height / 2)
    t = -origin[2] / direction[2]
    assert (origin + t * direction)[:2] == pytest.approx([0.25, 0.0], abs=1e-9)
    camera.close()


def test_upper_image_is_further_from_the_arm():
    model, data = view(sc.Scene([]))
    camera = Camera(model)
    _, up = camera.ray(data, camera.width / 2, 0)
    _, right = camera.ray(data, camera.width, camera.height / 2)
    assert up[0] > 0 and right[1] < 0
    camera.close()


def test_render_size_and_parked_arm_clears_the_pick_area():
    model, data = view(sc.Scene([]))
    camera = Camera(model)
    image = camera.render(data)
    assert image.shape == (camera.height, camera.width, 3)
    # Pixels over the pick area show only the grey tray: no strong colour, no dark arm.
    centre = image[60:150, 110:210].astype(int)
    assert np.ptp(centre, axis=2).max() < 30 and centre.min() > 80
    camera.close()
