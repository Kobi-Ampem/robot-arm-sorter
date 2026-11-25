"""Time-parameterised paths for the arm."""

from dataclasses import dataclass

import numpy as np

# The quintic profile's peak speed is 1.875 times its average speed.
QUINTIC_PEAK = 1.875


@dataclass
class Trajectory:
    t: np.ndarray    # (N,) seconds from the start
    q: np.ndarray    # (N, 4) joint positions
    qd: np.ndarray   # (N, 4) joint velocities
    qdd: np.ndarray  # (N, 4) joint accelerations

    @property
    def duration(self) -> float:
        return float(self.t[-1])


def quintic_scaling(tau):
    """Progress s from 0 to 1 for tau in [0, 1], with zero velocity and acceleration at both ends.

    s = 10 tau^3 - 15 tau^4 + 6 tau^5 is the lowest-order polynomial meeting those six end
    conditions, so the arm starts and stops without a jerk in acceleration.
    """
    tau = np.clip(tau, 0.0, 1.0)
    s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
    ds = 30 * tau**2 - 60 * tau**3 + 30 * tau**4
    dds = 60 * tau - 180 * tau**2 + 120 * tau**3
    return s, ds, dds


def _times(duration: float, dt: float) -> np.ndarray:
    steps = max(1, int(np.ceil(duration / dt)))
    return np.linspace(0.0, steps * dt, steps + 1)


def joint_move(q0, q1, dt: float, v_max: float = 1.5, min_duration: float = 0.3) -> Trajectory:
    """Quintic move in joint space; the duration keeps every joint at or under v_max rad/s."""
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    delta = q1 - q0
    duration = max(min_duration, QUINTIC_PEAK * np.max(np.abs(delta)) / v_max)
    t = _times(duration, dt)
    T = t[-1]
    s, ds, dds = quintic_scaling(t / T)
    return Trajectory(t, q0 + np.outer(s, delta), np.outer(ds / T, delta), np.outer(dds / T**2, delta))
