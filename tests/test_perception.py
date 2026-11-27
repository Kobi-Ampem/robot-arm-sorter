import mujoco
import numpy as np
import pytest

from arm_sorter import scene as sc
from arm_sorter.perception import CLEAR_POSE, Camera, Thresholds, classify_hue, detect


def view(scene):
    model = sc.build(scene)
    data = mujoco.MjData(model)
    data.qpos[:4] = CLEAR_POSE
    mujoco.mj_forward(model, data)
    return model, data


def match(detections, tomato):
    return min(detections, key=lambda d: np.linalg.norm(d.xyz[:2] - np.array(tomato.xy)))


def test_classify_hue():
    th = Thresholds()
    assert classify_hue(2, th) == "ripe" and classify_hue(175, th) == "ripe"
    assert classify_hue(15, th) == "turning"
    assert classify_hue(45, th) == "green"
    assert classify_hue(120, th) is None


def test_camera_ray_hits_a_known_point():
    model, data = view(sc.Scene([]))
    camera = Camera(model)
    # The optical axis passes through the corner shared by the four middle pixels.
    origin, direction = camera.ray(data, camera.width / 2 - 0.5, camera.height / 2 - 0.5)
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


@pytest.mark.parametrize("seed", range(5))
def test_finds_every_tomato_with_its_class_and_position(seed):
    scene = sc.random_scene(seed)
    model, data = view(scene)
    camera = Camera(model)
    found = detect(camera, data)
    camera.close()
    assert len(found) == len(scene.tomatoes)
    for tomato in scene.tomatoes:
        d = match(found, tomato)
        assert d.ripeness == tomato.ripeness
        assert np.linalg.norm(d.xyz[:2] - np.array(tomato.xy)) < 2e-3
        assert d.radius == pytest.approx(tomato.radius, abs=4e-3)


def test_render_size_and_parked_arm_clears_the_pick_area():
    model, data = view(sc.Scene([]))
    camera = Camera(model)
    image = camera.render(data)
    assert image.shape == (camera.height, camera.width, 3)
    # Pixels over the pick area show only the grey tray: no strong colour, no dark arm.
    centre = image[60:150, 110:210].astype(int)
    assert np.ptp(centre, axis=2).max() < 30 and centre.min() > 80
    camera.close()
