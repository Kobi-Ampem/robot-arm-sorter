"""Inverse kinematics: where should the joints be for the tool to reach a point?"""

from dataclasses import dataclass

import numpy as np

from .kinematics import D1, JOINT_LIMITS, L2, L3, L4

# Tool pitch for picking from above: the gripper points straight down.
TOP_DOWN = -np.pi / 2


@dataclass
class IKResult:
    q: np.ndarray
    ok: bool
    reason: str = ""
    iterations: int = 0


def within_limits(q) -> bool:
    return bool(np.all(q >= JOINT_LIMITS[:, 0] - 1e-9) and np.all(q <= JOINT_LIMITS[:, 1] + 1e-9))


def wrap_angle(a: float) -> float:
    return (a + np.pi) % (2 * np.pi) - np.pi


def solve_closed_form(target, pitch: float = TOP_DOWN, elbow_up: bool = True) -> IKResult:
    """Exact IK for a tool point and a tool pitch (angle below horizontal is negative).

    The base yaw points the arm's vertical plane at the target. In that plane, stepping back
    from the target by L4 along the tool direction gives the wrist point, which the
    shoulder-elbow pair reaches as a two-link planar arm (law of cosines). The wrist takes up
    whatever angle is left to make the tool pitch: q2 + q3 + q4 = pitch.

    The base always turns to face the target, so this returns the front-facing solution. A pose
    with the shoulder tipped back over the base can reach the same point from the other side;
    the solver does not return that family.
    """
    x, y, z = target
    r = np.hypot(x, y)
    q1 = np.arctan2(y, x) if r > 1e-9 else 0.0

    rw = r - L4 * np.cos(pitch)
    zw = z - D1 - L4 * np.sin(pitch)
    c3 = (rw**2 + zw**2 - L2**2 - L3**2) / (2 * L2 * L3)
    if abs(c3) > 1.0:
        return IKResult(np.zeros(4), False, "out of reach")
    q3 = -np.arccos(c3) if elbow_up else np.arccos(c3)
    q2 = np.arctan2(zw, rw) - np.arctan2(L3 * np.sin(q3), L2 + L3 * np.cos(q3))
    q4 = wrap_angle(pitch - q2 - q3)

    q = np.array([q1, q2, q3, q4])
    if not within_limits(q):
        return IKResult(q, False, "joint limits")
    return IKResult(q, True)


# A comfortable pose above the pick area; the numerical solver starts here by default.
READY_POSE = np.array([0.0, 1.2, -1.8, TOP_DOWN - 1.2 + 1.8])


def tool_direction(target, pitch: float) -> np.ndarray:
    """Unit vector the gripper should point along: pitch below horizontal, facing the target."""
    yaw = np.arctan2(target[1], target[0])
    return np.array([np.cos(pitch) * np.cos(yaw), np.cos(pitch) * np.sin(yaw), np.sin(pitch)])


def solve_dls(target, pitch: float = TOP_DOWN, q0=None, damping: float = 0.05,
              max_iter: int = 200, tol: float = 1e-6, max_step: float = 0.2) -> IKResult:
    """Numerical IK by damped least squares (Levenberg-Marquardt style).

    Each iteration solves J dq = e for a 6-vector error e: the position error, and the
    rotation that would turn the gripper's x axis onto the wanted direction (x cross d). The
    damping term keeps steps bounded near singularities, at the cost of slower convergence:
        dq = J^T (J J^T + damping^2 I)^-1 e
    Steps are capped at max_step radians and joints are clipped to their limits.

    With no q0, it starts from READY_POSE with the base already turned to face the target.
    Starting with the base facing straight ahead, it converged on only 129 of 1,000 targets
    spread around the base (see docs/LOG.md, 24 Nov 2026).
    """
    # Imported here so the closed-form solver stays usable without the Jacobian code.
    from .kinematics import jacobian, tcp_pose

    target = np.asarray(target, dtype=float)
    wanted = tool_direction(target, pitch)
    if q0 is None:
        q0 = READY_POSE.copy()
        q0[0] = np.arctan2(target[1], target[0])
    q = np.array(q0, dtype=float)
    eye = np.eye(6)
    for it in range(1, max_iter + 1):
        pose = tcp_pose(q)
        e = np.concatenate([target - pose[:3, 3], np.cross(pose[:3, 0], wanted)])
        if np.linalg.norm(e[:3]) < tol and np.linalg.norm(e[3:]) < tol:
            return IKResult(q, True, iterations=it - 1)
        J = jacobian(q)
        dq = J.T @ np.linalg.solve(J @ J.T + damping**2 * eye, e)
        norm = np.linalg.norm(dq)
        if norm > max_step:
            dq *= max_step / norm
        q = np.clip(q + dq, JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])
    return IKResult(q, False, "did not converge", iterations=max_iter)
