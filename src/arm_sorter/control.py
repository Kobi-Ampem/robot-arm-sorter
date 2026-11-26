"""Joint-space PID control of the arm, sending torques to the four motors."""

from dataclasses import dataclass, field

import mujoco
import numpy as np

from .model import ARM_JOINTS, arm_dof_indices, arm_qpos_indices


# Joint inertias at the ready pose (kg m^2), from mj_fullM, armature included.
READY_INERTIA = np.array([0.040, 0.040, 0.019, 0.010])


def pd_gains(omega: float, zeta: float, inertia=READY_INERTIA,
             integral_time: float | None = None) -> "PIDGains":
    """Gains giving each joint a natural frequency omega (rad/s) and damping ratio zeta,
    treating it as a lone inertia: kp = I omega^2, kd = 2 zeta I omega. With an integral time
    Ti (s), ki = kp / Ti; None means no integral action."""
    inertia = np.asarray(inertia, float)
    kp = inertia * omega**2
    ki = np.zeros(4) if integral_time is None else kp / integral_time
    return PIDGains(kp=kp, ki=ki, kd=2 * zeta * inertia * omega)


@dataclass
class PIDGains:
    """Per-joint gains, in joint order (base yaw, shoulder, elbow, wrist).

    Defaults aim for a natural frequency near 25 rad/s with critical damping, from the joint
    inertias at the ready pose (about 0.04, 0.04, 0.019 and 0.010 kg m^2):
    kp = I w^2 and kd = 2 I w. The gain sweep in experiments/RESULTS.md checks them.
    ki = kp / 1.0 s (integral time 1 s) removes the sag under a held tomato, which the gravity
    model does not include; chosen by the integral sweep in experiments/RESULTS.md.
    """
    kp: np.ndarray = field(default_factory=lambda: np.array([25.0, 25.0, 12.0, 6.4]))
    ki: np.ndarray = field(default_factory=lambda: np.array([25.0, 25.0, 12.0, 6.4]))
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


class GravityCompensatedPID(JointPID):
    """PID plus the torque that exactly holds the arm still against gravity at its current pose.

    The gravity torque g(q) comes from MuJoCo's inverse dynamics (mj_rne) on a scratch copy of
    the state with zero velocity, so Coriolis and centrifugal terms are left out. This uses the
    simulator's own model of the arm; a real arm would use masses identified from the hardware.
    """

    def __init__(self, model: mujoco.MjModel, gains: PIDGains | None = None):
        super().__init__(model, gains)
        self.scratch = mujoco.MjData(model)
        self.bias = np.zeros(model.nv)

    def gravity_torque(self, data: mujoco.MjData) -> np.ndarray:
        s = self.scratch
        s.qpos[:] = data.qpos
        s.qvel[:] = 0.0
        mujoco.mj_kinematics(self.model, s)
        mujoco.mj_comPos(self.model, s)
        mujoco.mj_rne(self.model, s, 0, self.bias)
        return self.bias[self.dof_idx]

    def feedforward(self, data: mujoco.MjData) -> np.ndarray:
        return self.gravity_torque(data)
