"""Run the arm along a trajectory in simulation and record how well it follows."""

from dataclasses import dataclass

import mujoco
import numpy as np

from .control import JointPID
from .trajectory import Trajectory


@dataclass
class TrackingLog:
    t: np.ndarray
    q_ref: np.ndarray
    q: np.ndarray
    tau: np.ndarray

    @property
    def error(self) -> np.ndarray:
        return self.q_ref - self.q

    def rms_error(self) -> np.ndarray:
        return np.sqrt(np.mean(self.error**2, axis=0))

    def final_error(self) -> np.ndarray:
        return np.abs(self.error[-1])


def track(model: mujoco.MjModel, data: mujoco.MjData, traj: Trajectory, controller: JointPID,
          settle: float = 0.5, on_step=None) -> TrackingLog:
    """Step the simulation once per trajectory sample, then hold the last sample for `settle` s.

    The trajectory must be sampled at the model's timestep, so control runs at the physics rate.
    `on_step(data)` is called after every step (used for the gripper and for video frames).
    """
    dt = model.opt.timestep
    if len(traj.t) > 1 and not np.isclose(traj.t[1] - traj.t[0], dt):
        raise ValueError("trajectory must be sampled at the model timestep")
    hold = int(round(settle / dt))
    n = len(traj.t) + hold
    q_ref = np.vstack([traj.q, np.repeat(traj.q[-1:], hold, axis=0)])
    qd_ref = np.vstack([traj.qd, np.zeros((hold, 4))])
    log_q, log_tau = np.empty((n, 4)), np.empty((n, 4))
    for i in range(n):
        controller.apply(data, q_ref[i], qd_ref[i])
        mujoco.mj_step(model, data)
        log_q[i] = data.qpos[controller.qpos_idx]
        log_tau[i] = data.ctrl[controller.act_idx]
        if on_step is not None:
            on_step(data)
    t = np.arange(1, n + 1) * dt
    return TrackingLog(t, q_ref, log_q, log_tau)


def start_at(model: mujoco.MjModel, q) -> mujoco.MjData:
    """Fresh simulation state with the arm at rest at joint positions q."""
    data = mujoco.MjData(model)
    data.qpos[:4] = q
    mujoco.mj_forward(model, data)
    return data
