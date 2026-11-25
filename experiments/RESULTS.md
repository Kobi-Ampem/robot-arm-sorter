# Experiment results

Every run with its settings and results, failures included, newest last.

## 2026-11-24: IK benchmark and DLS damping sweep

- **Script.** `python experiments/ik_benchmark.py --n 1000 --seed 4` (PYTHONPATH=src).
- **Settings.** Targets are tool points of 1,000 random elbow-up, top-down poses in front of the
  base (`workspace.sample_top_down_configs`). The pick area is radius 0.15 to 0.35 m and height
  up to 0.15 m. DLS: tolerance 1e-6 m and 1e-6 rad, 200 iterations, steps capped at 0.2 rad,
  seed pose with the base facing the target. Times are wall clock on the laptop (one core, Python).
**Whole workspace, 1,000 targets.**

| solver | success | max position error (m) | max pitch error (rad) | median iterations | median time (us) |
|---|---|---|---|---|---|
| closed form | 1000/1000 (100.0%) | 1.7e-16 | 4.4e-16 | 0 | 10 |
| DLS, damping 0.001 | 1000/1000 (100.0%) | 1.0e-06 | 8.3e-08 | 8 | 1045 |
| DLS, damping 0.01 | 997/1000 (99.7%) | 1.0e-06 | 1.7e-07 | 9 | 1057 |
| DLS, damping 0.02 | 985/1000 (98.5%) | 1.0e-06 | 1.5e-07 | 10 | 1170 |
| DLS, damping 0.05 | 956/1000 (95.6%) | 1.0e-06 | 1.6e-07 | 13 | 1619 |
| DLS, damping 0.1 | 931/1000 (93.1%) | 1.0e-06 | 1.8e-07 | 23 | 2871 |
| DLS, damping 0.2 | 816/1000 (81.6%) | 1.0e-06 | 1.7e-07 | 57 | 8014 |

**Pick area, 228 targets.**

| solver | success | max position error (m) | max pitch error (rad) | median iterations | median time (us) |
|---|---|---|---|---|---|
| closed form | 228/228 (100.0%) | 1.2e-16 | 4.4e-16 | 0 | 11 |
| DLS, damping 0.001 | 228/228 (100.0%) | 9.9e-07 | 1.3e-08 | 5 | 611 |
| DLS, damping 0.01 | 228/228 (100.0%) | 9.8e-07 | 1.7e-07 | 5 | 620 |
| DLS, damping 0.02 | 228/228 (100.0%) | 9.9e-07 | 1.5e-07 | 6 | 725 |
| DLS, damping 0.05 | 228/228 (100.0%) | 9.9e-07 | 1.6e-07 | 8 | 1013 |
| DLS, damping 0.1 | 227/228 (99.6%) | 1.0e-06 | 1.8e-07 | 16 | 1874 |
| DLS, damping 0.2 | 224/228 (98.2%) | 1.0e-06 | 1.7e-07 | 43 | 5146 |

| solver | median final distance (m) | median last joint step (rad) |
|---|---|---|
| DLS, damping 0.001 | 0.1862 | 2.0e-01 |
| DLS, damping 0.01 | 0.1849 | 2.0e-01 |
| DLS, damping 0.02 | 0.1845 | 2.0e-01 |
| DLS, damping 0.05 | 0.1841 | 2.0e-01 |
| DLS, damping 0.1 | 0.1837 | 8.3e-16 |
| DLS, damping 0.2 | 0.1837 | 7.9e-17 |

**Reading.**
- Closed form is exact (errors at rounding level) and about 100 times faster than DLS
  (10 us against 600 to 1,000 us).
- For reachable targets, less damping converges more often and in fewer iterations: 100% at
  0.001 against 95.6% at 0.05 over the whole workspace.
- For out-of-reach targets, damping of 0.05 or less never settles: the joints jump the full
  0.2 rad step on every iteration (chattering). Damping of 0.1 or more settles to a fixed point.
  The final distance is about the same either way (0.184 to 0.186 m).

**Choice.** Default damping 0.01: 99.7% over the whole workspace and 100% in the pick area,
median 9 and 5 iterations. Accepted trade-off: it chatters on unreachable targets. That is safe
here because IK runs before any motion and an unconverged result is rejected (`ok` is false), so
the chattering joints are never sent to the arm. If DLS were ever used for live tracking, adaptive
damping (raise it when the error stops falling) would be the fix.
