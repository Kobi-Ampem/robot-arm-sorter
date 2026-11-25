"""Time-parameterised paths for the arm: joint-space moves and straight Cartesian lines."""

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


def cartesian_line(p0, p1, dt: float, speed: float = 0.10, min_duration: float = 0.3,
                   pitch: float | None = None) -> Trajectory:
    """Tool point moves along the straight line p0 to p1, with a quintic profile along it.

    Each sample is solved with the closed-form IK, so the tool stays on the line and keeps its
    pitch. Joint velocities and accelerations come from finite differences of those samples.
    `speed` is the peak tool speed in m/s. Raises ValueError if any point on the line cannot be
    reached.
    """
    from .ik import TOP_DOWN, solve_closed_form

    pitch = TOP_DOWN if pitch is None else pitch
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    duration = max(min_duration, QUINTIC_PEAK * np.linalg.norm(p1 - p0) / speed)
    t = _times(duration, dt)
    s, _, _ = quintic_scaling(t / t[-1])
    q = np.empty((len(t), 4))
    for i, si in enumerate(s):
        result = solve_closed_form(p0 + si * (p1 - p0), pitch)
        if not result.ok:
            raise ValueError(f"point {i} of the line is not reachable: {result.reason}")
        q[i] = result.q
    q[:, 0] = np.unwrap(q[:, 0])  # no jump if the line crosses yaw = pi
    qd = np.gradient(q, t, axis=0)
    qdd = np.gradient(qd, t, axis=0)
    qd[[0, -1]] = 0.0
    qdd[[0, -1]] = 0.0
    return Trajectory(t, q, qd, qdd)


def concatenate(parts: list[Trajectory]) -> Trajectory:
    """Join trajectories end to end; each part must start where the previous one ended."""
    out = parts[0]
    for part in parts[1:]:
        if not np.allclose(part.q[0], out.q[-1], atol=1e-9):
            raise ValueError("trajectories do not join")
        offset = out.t[-1]
        out = Trajectory(np.concatenate([out.t, part.t[1:] + offset]),
                         np.vstack([out.q, part.q[1:]]),
                         np.vstack([out.qd, part.qd[1:]]),
                         np.vstack([out.qdd, part.qdd[1:]]))
    return out
