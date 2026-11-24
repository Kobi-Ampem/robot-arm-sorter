"""Forward kinematics of the arm from its Denavit-Hartenberg (DH) parameters.

Standard DH convention: frame i is reached from frame i-1 by
    Rz(theta_i) * Tz(d_i) * Tx(a_i) * Rx(alpha_i).
Joint angles are used directly as theta, with no offsets, because the MJCF joint axes were
chosen to match (see docs/decisions/0001).
"""

import numpy as np

# Link dimensions in metres; they must match assets/arm.xml (checked in the tests).
D1 = 0.10  # table to shoulder axis
L2 = 0.20  # shoulder to elbow
L3 = 0.18  # elbow to wrist
L4 = 0.10  # wrist to tool centre point

# One row per joint: (a, alpha, d).
DH_PARAMS = (
    (0.0, np.pi / 2, D1),
    (L2, 0.0, 0.0),
    (L3, 0.0, 0.0),
    (L4, 0.0, 0.0),
)

# Joint limits in radians, as in assets/arm.xml.
JOINT_LIMITS = np.array([
    [-np.pi, np.pi],
    [-0.5, 3.0],
    [-2.7, 2.7],
    [-2.7, 2.7],
])


def dh_transform(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
    """Homogeneous transform from frame i-1 to frame i."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0.0, sa, ca, d],
        [0.0, 0.0, 0.0, 1.0],
    ])


def joint_frames(q) -> list[np.ndarray]:
    """Frames 0 to 4 in the base frame; frame i sits at joint i+1 and frame 4 at the tool point."""
    frames = [np.eye(4)]
    for (a, alpha, d), theta in zip(DH_PARAMS, q):
        frames.append(frames[-1] @ dh_transform(a, alpha, d, theta))
    return frames


def tcp_pose(q) -> np.ndarray:
    """Pose of the tool centre point. Its x axis is the direction the gripper points."""
    return joint_frames(q)[-1]


def tcp_position(q) -> np.ndarray:
    return tcp_pose(q)[:3, 3]
