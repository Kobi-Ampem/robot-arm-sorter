import mujoco
import numpy as np

from arm_sorter.ik import READY_POSE, solve_closed_form
from arm_sorter.scene import BINS, BIN_WALL_HEIGHT, CLASSES, MIN_SPACING, build, random_scene


def test_random_scene_is_reproducible_and_spaced():
    a, b = random_scene(3), random_scene(3)
    assert a == b
    xy = np.array([t.xy for t in a.tomatoes])
    for i in range(len(xy)):
        for j in range(i + 1, len(xy)):
            assert np.linalg.norm(xy[i] - xy[j]) >= MIN_SPACING


def test_every_class_appears_across_scenes():
    seen = {t.ripeness for s in range(20) for t in random_scene(s).tomatoes}
    assert seen == set(CLASSES)


def test_tomatoes_settle_on_the_table():
    scene = random_scene(1)
    model = build(scene)
    data = mujoco.MjData(model)
    data.qpos[:4] = READY_POSE
    for _ in range(500):
        data.qfrc_applied[:4] = data.qfrc_bias[:4]  # hold the arm still while things settle
        mujoco.mj_step(model, data)
    for t in scene.tomatoes:
        z = data.body(t.name).xpos[2]
        assert abs(z - t.radius) < 2e-3
        assert np.linalg.norm(data.body(t.name).xpos[:2] - t.xy) < 2e-3


def test_bins_are_reachable_for_a_drop():
    for x, y in BINS.values():
        assert solve_closed_form([x, y, BIN_WALL_HEIGHT + 0.10]).ok


def test_dense_scenes_always_finish():
    for seed in range(50):
        assert len(random_scene(seed, n=6).tomatoes) == 6
