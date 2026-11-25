"""Joint-space PID control of the arm, sending torques to the four motors."""

from dataclasses import dataclass, field

import mujoco
import numpy as np

from .model import ARM_JOINTS, arm_dof_indices, arm_qpos_indices


@dataclass
class PIDGains:
    """Per-joint gains, in joint order (base yaw, shoulder, elbow, wrist).

    Defaults aim for a natural frequency near 25 rad/s with critical damping, from the joint
    inertias at the ready pose (about 0.04, 0.04, 0.019 and 0.010 kg m^2):
    kp = I w^2 and kd = 2 I w. The gain sweep in experiments/RESULTS.md checks them.
    """
    kp: np.ndarray = field(default_factory=lambda: np.array([25.0, 25.0, 12.0, 6.4]))
    ki: np.ndarray = field(default_factory=lambda: np.zeros(4))
    kd: np.ndarray = field(default_factory=lambda: np.array([2.0, 2.0, 0.95, 0.5]))


class JointPID:
    """tau = kp e + ki integral(e) + kd de/dt, clipped to each motor's torque limit.

    The integral only accumulates while the output is not saturated (anti-windup), so a long
    stall against a limit does not leave a large stored torque behind.
    """

    def __init__(self, model: mujoco.MjModel, gains: PIDGains | None = None):
        self.model = model
        self.gains = gains or PIDGains()
        self.qpos_idx = arm_qpos_indices(model)
        self.dof_idx = arm_dof_indices(model)
        self.act_idx = [model.actuator(name).id for name in ARM_JOINTS]
        self.limits = model.actuator_ctrlrange[self.act_idx]
        self.dt = model.opt.timestep
        self.integral = np.zeros(4)

    def reset(self):
        self.integral[:] = 0.0

    def feedforward(self, data: mujoco.MjData) -> np.ndarray:
        """Torque added to the PID output; none in plain PID."""
        return np.zeros(4)

    def torque(self, data: mujoco.MjData, q_ref, qd_ref) -> np.ndarray:
        g = self.gains
        e = np.asarray(q_ref) - data.qpos[self.qpos_idx]
        ed = np.asarray(qd_ref) - data.qvel[self.dof_idx]
        candidate = self.integral + e * self.dt
        raw = g.kp * e + g.ki * candidate + g.kd * ed + self.feedforward(data)
        tau = np.clip(raw, self.limits[:, 0], self.limits[:, 1])
        unsaturated = tau == raw
        self.integral = np.where(unsaturated, candidate, self.integral)
        return tau

    def apply(self, data: mujoco.MjData, q_ref, qd_ref):
        data.ctrl[self.act_idx] = self.torque(data, q_ref, qd_ref)
