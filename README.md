# robot-arm-sorter

A simulated robot arm that sorts tomatoes into ripe, turning and green, built in MuJoCo.

Smallholder farmers and aggregators in the Ashanti Region of Ghana grade tomatoes by hand, and the
grade sets the price. This project asks what a low-cost sorting cell would need: kinematics,
inverse kinematics, trajectory planning, joint control and colour vision, each written from first
principles and checked against the simulator.

Status: in progress.

## Setup

```bash
python -m venv .venv && . .venv/bin/activate
pip install -e ".[dev]"
pytest
```
