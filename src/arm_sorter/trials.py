"""Randomised sorting episodes, scored against where each tomato really ended up."""

from collections import Counter
from dataclasses import dataclass, field

import mujoco

from . import scene as sc
from .perception import CLEAR_POSE, Camera
from .task import Arm, sort_all


@dataclass
class Episode:
    seed: int
    tomatoes: int
    correct: int          # in the bin for its true ripeness
    wrong_bin: int
    on_tray: int          # still in or near the pick area
    elsewhere: int        # fell off, or anywhere else
    picks: int
    failures: Counter = field(default_factory=Counter)
    sim_time: float = 0.0

    @property
    def success(self) -> bool:
        return self.correct == self.tomatoes


def where(xyz, true_class: str) -> str:
    if sc.in_bin(xyz, true_class):
        return "correct"
    if any(sc.in_bin(xyz, c) for c in sc.CLASSES):
        return "wrong_bin"
    x, y = xyz[:2]
    if (sc.PICK_X[0] - 0.05 <= x <= sc.PICK_X[1] + 0.05
            and sc.PICK_Y[0] - 0.05 <= y <= sc.PICK_Y[1] + 0.05 and xyz[2] > 0):
        return "on_tray"
    return "elsewhere"


def run_episode(seed: int, light_range=(0.6, 1.1), on_step=None) -> Episode:
    scene = sc.random_scene(seed, light_range=light_range)
    model = sc.build(scene)
    data = mujoco.MjData(model)
    data.qpos[:4] = CLEAR_POSE
    mujoco.mj_forward(model, data)
    arm = Arm(model, data, on_step=on_step)
    camera = Camera(model)
    try:
        result = sort_all(arm, camera)
    finally:
        camera.close()
    arm.wait(1.0)  # let everything come to rest before scoring
    places = Counter(where(data.body(t.name).xpos, t.ripeness) for t in scene.tomatoes)
    return Episode(seed, len(scene.tomatoes), places["correct"], places["wrong_bin"],
                   places["on_tray"], places["elsewhere"], len(result.outcomes),
                   Counter(o.failure for o in result.outcomes if not o.success), result.sim_time)
