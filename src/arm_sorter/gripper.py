"""Open and close the two-finger gripper."""

import mujoco

OPEN = 0.04    # opening per finger (m): an 80 mm gap for tomatoes up to 70 mm across
CLOSED = 0.0   # commanding fully closed squeezes whatever is between the fingers


class Gripper:
    def __init__(self, model: mujoco.MjModel):
        self.act = model.actuator("gripper").id
        self.joint = model.joint("finger_left").id
        self.qpos_adr = model.jnt_qposadr[self.joint]

    def command(self, data: mujoco.MjData, opening: float):
        data.ctrl[self.act] = opening

    def opening(self, data: mujoco.MjData) -> float:
        return float(data.qpos[self.qpos_adr])
