"""Sampling poses the arm can actually reach, for tests and experiments."""

import numpy as np

from .ik import TOP_DOWN, within_limits
from .kinematics import JOINT_LIMITS, L2, L3, tcp_pose


def sample_top_down_configs(n: int, seed: int = 0) -> np.ndarray:
    """Random elbow-up configurations within limits whose tool points straight down above the
    table and in front of the base.

    Configurations with the tool behind the base (shoulder tipped past vertical) have a second,
    equivalent solution with the base turned by pi, which is the one the solvers return, so they
    are left out (see docs/LOG.md, 24 Nov 2026).
    """
    rng = np.random.default_rng(seed)
    configs = []
    while len(configs) < n:
        q = rng.uniform(JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])
        q[3] = TOP_DOWN - q[1] - q[2]
        radial = L2 * np.cos(q[1]) + L3 * np.cos(q[1] + q[2])  # tool reach in the arm's plane
        if within_limits(q) and tcp_pose(q)[2, 3] > 0.0 and q[2] < 0 and radial > 0:
            configs.append(q)
    return np.array(configs)
