# 0001: MuJoCo, and an arm model written by hand

Date: 24 Nov 2026

## Context

The project needs a physics simulator with contacts (for grasping), a camera (for colour
sorting) and fast headless runs on a laptop with about 2 GB of free RAM and no GPU. It also needs
an arm whose kinematics can be derived and checked by hand, because the point is to show forward
kinematics, inverse kinematics and control from first principles.

## Options

1. **MuJoCo** with a hand-written MJCF arm.
2. **MuJoCo** with a ready-made arm from MuJoCo Menagerie (for example a Franka Panda).
3. **PyBullet** with a URDF arm.
4. **Gazebo with ROS 2.**

## Choice

Option 1.

## Why

- MuJoCo installs with `pip`, runs on CPU, renders headless and has stable contact physics. It
  also exposes the quantities we want to check our own code against (body positions, `mj_jacSite`,
  bias forces).
- A hand-written four-joint arm (one yaw joint carrying three parallel pitch joints) keeps the
  kinematics small enough to derive on paper and gives a closed-form inverse kinematics solution
  for a top-down grasp. A seven-joint Panda is redundant and would hide that derivation behind a
  numerical solver.
- The joint axes are set so MuJoCo's joint angles equal the DH angles with no offsets, which keeps
  the comparison between our FK and MuJoCo's simple.
- Gazebo and ROS 2 are heavier on this laptop and would overlap with other robotics work
  (mobile navigation in ROS 2) that is planned elsewhere.

## Trade-off accepted

A four-joint arm cannot reach every orientation: with three parallel pitch joints, the tool can
only point within the vertical plane through the base. That is enough for picking round produce
from above, but it is not a general-purpose manipulator. A hand-written model is also less
realistic than a manufacturer's model: masses, inertias and motor limits are estimates sized to
hobby servos, not measured values.
