"""Load the arm model and look up its joints."""

from pathlib import Path

import mujoco

ASSETS = Path(__file__).resolve().parents[2] / "assets"
ARM_XML = ASSETS / "arm.xml"

ARM_JOINTS = ("base_yaw", "shoulder", "elbow", "wrist")


def load_model(path: Path = ARM_XML) -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_path(str(path))


def arm_qpos_indices(model: mujoco.MjModel) -> list[int]:
    """Positions of the four arm joints in qpos, in kinematic order."""
    return [int(model.jnt_qposadr[model.joint(name).id]) for name in ARM_JOINTS]


def arm_dof_indices(model: mujoco.MjModel) -> list[int]:
    """Positions of the four arm joints in qvel, qfrc and Jacobian columns."""
    return [int(model.jnt_dofadr[model.joint(name).id]) for name in ARM_JOINTS]
