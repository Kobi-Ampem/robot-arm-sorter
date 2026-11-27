"""Sorting scenes: the arm plus randomised tomatoes in the pick area and three bins."""

import colorsys
from dataclasses import dataclass, field

import mujoco
import numpy as np

from .model import ARM_XML

CLASSES = ("ripe", "turning", "green")

# Hue ranges in degrees for each class when generating tomatoes (ripe wraps through 0).
HUE_RANGES = {"ripe": (-8.0, 8.0), "turning": (20.0, 35.0), "green": (75.0, 110.0)}

# Bin centres on the table (x, y), chosen to be reachable with a top-down drop.
BINS = {"ripe": (0.02, 0.30), "turning": (0.02, -0.30), "green": (-0.24, 0.06)}
BIN_INNER = 0.12  # inner width of a bin (m)
BIN_WALL_HEIGHT = 0.04

# Pick area on the table, in front of the arm.
PICK_X = (0.17, 0.32)
PICK_Y = (-0.14, 0.14)
MIN_SPACING = 0.09  # centre-to-centre, so the open fingers fit between neighbours


@dataclass
class Tomato:
    name: str
    ripeness: str
    radius: float
    xy: tuple[float, float]
    rgb: tuple[float, float, float]


@dataclass
class Scene:
    tomatoes: list[Tomato]
    light: float = 0.9  # diffuse intensity of the top light
    seed: int | None = None
    extra: dict = field(default_factory=dict)


def random_scene(seed: int, n: int | None = None, light_range=(0.6, 1.1)) -> Scene:
    """Random tomatoes (3 to 5 unless n is given) with random class, size, colour and position.

    Positions are drawn at random and rejected if too close to an earlier one. If a draw gets
    boxed in (no room left after 200 tries), the layout starts again, so the loop always ends
    for counts that fit: the 0.15 x 0.28 m pick area holds about 8 at 0.09 m spacing.
    """
    rng = np.random.default_rng(seed)
    n = int(rng.integers(3, 6)) if n is None else n
    positions: list[np.ndarray] = []
    tries = 0
    while len(positions) < n:
        xy = np.array([rng.uniform(*PICK_X), rng.uniform(*PICK_Y)])
        tries += 1
        if all(np.linalg.norm(xy - p) >= MIN_SPACING for p in positions):
            positions.append(xy)
            tries = 0
        elif tries > 200:
            positions, tries = [], 0
    tomatoes = []
    for i, xy in enumerate(positions):
        ripeness = CLASSES[int(rng.integers(3))]
        hue = rng.uniform(*HUE_RANGES[ripeness]) % 360.0
        rgb = colorsys.hsv_to_rgb(hue / 360.0, rng.uniform(0.6, 0.95), rng.uniform(0.5, 0.9))
        tomatoes.append(Tomato(f"tomato{i}", ripeness, float(rng.uniform(0.025, 0.035)),
                               (float(xy[0]), float(xy[1])), tuple(float(c) for c in rgb)))
    return Scene(tomatoes, light=float(rng.uniform(*light_range)), seed=seed)


def _bin_xml(name: str, x: float, y: float) -> str:
    h, t = BIN_WALL_HEIGHT / 2, 0.005
    w = BIN_INNER / 2 + t
    walls = [(0, w, w, t), (0, -w, w, t), (w, 0, t, w), (-w, 0, t, w)]
    geoms = "".join(
        f'<geom type="box" size="{sx} {sy} {h}" pos="{dx} {dy} {h}" rgba="0.25 0.3 0.45 1"/>'
        for dx, dy, sx, sy in walls)
    return f'<body name="bin_{name}" pos="{x} {y} 0">{geoms}</body>'


def _tomato_xml(t: Tomato) -> str:
    r, g, b = t.rgb
    # condim 4 adds torsional friction so a held sphere does not spin between the fingers.
    return (f'<body name="{t.name}" pos="{t.xy[0]} {t.xy[1]} {t.radius}">'
            f'<freejoint name="{t.name}"/>'
            f'<geom name="{t.name}" type="sphere" size="{t.radius}" rgba="{r} {g} {b} 1" '
            f'density="1000" condim="4" friction="1 0.01 0.001"/></body>')


def scene_xml(scene: Scene) -> str:
    xml = ARM_XML.read_text()
    xml = xml.replace('diffuse="0.9 0.9 0.9"', f'diffuse="{scene.light} {scene.light} {scene.light}"')
    bodies = "".join(_bin_xml(n, *BINS[n]) for n in CLASSES)
    bodies += "".join(_tomato_xml(t) for t in scene.tomatoes)
    return xml.replace("</worldbody>", bodies + "</worldbody>")


def build(scene: Scene) -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_string(scene_xml(scene))


def tomato_position(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> np.ndarray:
    return data.body(name).xpos.copy()


def in_bin(xyz, bin_name: str) -> bool:
    """True if a tomato centre lies inside the given bin's walls.

    The height limit allows for a tomato resting on another one in the bin (up to one largest
    tomato diameter above the rim), but not for one still in the air above it.
    """
    bx, by = BINS[bin_name]
    half = BIN_INNER / 2
    return (abs(xyz[0] - bx) < half and abs(xyz[1] - by) < half
            and xyz[2] < BIN_WALL_HEIGHT + 0.07)
