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

## 2026-11-24: PID gain sweep, with and without gravity compensation

- **Script.** `python experiments/pid_sweep.py --moves 20 --seed 0` (PYTHONPATH=src), 10 s on the
  laptop.
- **Settings.** 20 moves from the ready pose to random pick-area targets (radius 0.18 to 0.32 m,
  angle -0.6 to 0.6 rad, height 0.03 to 0.12 m), quintic joint moves with v_max 1.5 rad/s, then a
  0.5 s hold. Gains from `pd_gains(omega, zeta)` with the ready-pose inertias; no integral term.
  Control at the 500 Hz physics rate. Errors are tool-point distances (FK of reference against
  FK of actual), tracking error sampled every 10 steps.

| controller | omega (rad/s) | zeta | worst tracking error (mm) | worst final error (mm) | steps saturated | moves ending within 1 mm |
|---|---|---|---|---|---|---|
| PD | 10 | 0.5 | 100.23 | 91.784 | 0.0% | 0% |
| PD | 10 | 1.0 | 90.98 | 89.623 | 0.0% | 0% |
| PD | 10 | 1.5 | 90.16 | 89.947 | 0.0% | 0% |
| PD | 15 | 0.5 | 51.04 | 50.861 | 0.0% | 0% |
| PD | 15 | 1.0 | 50.86 | 50.859 | 0.0% | 0% |
| PD | 15 | 1.5 | 50.76 | 50.765 | 0.0% | 0% |
| PD | 25 | 0.5 | 19.45 | 18.875 | 0.0% | 0% |
| PD | 25 | 1.0 | 19.09 | 18.874 | 0.0% | 0% |
| PD | 25 | 1.5 | 18.87 | 18.873 | 0.0% | 0% |
| PD | 35 | 0.5 | 10.03 | 9.698 | 0.0% | 0% |
| PD | 35 | 1.0 | 9.92 | 9.698 | 0.0% | 0% |
| PD | 35 | 1.5 | 9.79 | 9.698 | 0.0% | 0% |
| PD | 50 | 0.5 | 4.96 | 4.769 | 0.0% | 0% |
| PD | 50 | 1.0 | 4.92 | 4.769 | 0.0% | 0% |
| PD | 50 | 1.5 | 4.87 | 4.769 | 0.0% | 0% |
| PD | 80 | 0.5 | 2.00 | 1.867 | 0.0% | 10% |
| PD | 80 | 1.0 | 1.98 | 1.867 | 0.0% | 10% |
| PD | 80 | 1.5 | 1.96 | 1.867 | 0.0% | 10% |
| PD + gravity | 10 | 0.5 | 15.29 | 0.876 | 0.0% | 100% |
| PD + gravity | 10 | 1.0 | 10.03 | 0.367 | 0.0% | 100% |
| PD + gravity | 10 | 1.5 | 7.38 | 0.577 | 0.0% | 100% |
| PD + gravity | 15 | 0.5 | 8.46 | 0.095 | 0.0% | 100% |
| PD + gravity | 15 | 1.0 | 5.56 | 0.054 | 0.0% | 100% |
| PD + gravity | 15 | 1.5 | 4.03 | 0.151 | 0.0% | 100% |
| PD + gravity | 25 | 0.5 | 3.67 | 0.004 | 0.0% | 100% |
| PD + gravity | 25 | 1.0 | 2.36 | 0.001 | 0.0% | 100% |
| PD + gravity | 25 | 1.5 | 1.68 | 0.014 | 0.0% | 100% |
| PD + gravity | 35 | 0.5 | 2.03 | 0.000 | 0.0% | 100% |
| PD + gravity | 35 | 1.0 | 1.28 | 0.000 | 0.0% | 100% |
| PD + gravity | 35 | 1.5 | 0.88 | 0.002 | 0.0% | 100% |
| PD + gravity | 50 | 0.5 | 1.14 | 0.000 | 0.0% | 100% |
| PD + gravity | 50 | 1.0 | 0.83 | 0.000 | 0.0% | 100% |
| PD + gravity | 50 | 1.5 | 0.63 | 0.000 | 0.0% | 100% |
| PD + gravity | 80 | 0.5 | 0.80 | 0.000 | 0.0% | 100% |
| PD + gravity | 80 | 1.0 | 0.73 | 0.000 | 0.0% | 100% |
| PD + gravity | 80 | 1.5 | 0.71 | 0.000 | 0.0% | 100% |

**Reading.**
- Without gravity compensation the arm sags: at omega 25 every move ends 18.9 mm off, and even
  at omega 80 only 2 of 20 moves end within 1 mm. The sag only shrinks as 1/kp, it never goes.
- With gravity compensation every setting from omega 10 up ends within 1 mm, and from omega 25 up
  within 0.02 mm. Higher omega cuts the error during the move (2.36 mm at 25, 0.73 mm at 80).
- No setting saturated a motor: the 1.5 rad/s speed limit keeps torques small.
- The damping ratio matters little here; 0.5 overshoots slightly more during the move.

**Choice.** Gravity compensation on, omega 25 rad/s and zeta 1.0 (the existing defaults): the
lowest stiffness that ends within 0.01 mm and tracks within 5 mm (the gripper clears a 70 mm
tomato by 10 mm). Accepted trade-off: the simulator has no sensor noise, encoder quantisation or
delay, so stiffer gains look free here. On a real servo they would amplify noise, so the sweep
is not a reason to pick omega 80.
