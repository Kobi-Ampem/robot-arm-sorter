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

## 2026-11-24: contact settings for holding a tomato

- **Script.** `python experiments/grasp_contact_sweep.py` (PYTHONPATH=src).
- **Settings.** One tomato at (0.25, 0.05) m on the table. Ready pose, joint move to 0.08 m above
  it, Cartesian descent at 0.08 m/s until the tool point is at the tomato's centre, close for
  0.4 s, lift 0.12 m at 0.08 m/s, hold 0.5 s. Gravity-compensated PID with the default gains.
  "Held" means the tomato centre ends within 10 mm of the tool point. Each row removes some of
  the chosen settings from the arm model.

| setting | radius (mm) | held? tomato-to-tool distance after lift (mm) | finger mismatch when closed (mm) |
|---|---|---|---|
| MuJoCo defaults | 25 | NO: 135.6 | 7.4 |
| MuJoCo defaults | 30 | NO: 134.2 | 9.0 |
| MuJoCo defaults | 35 | NO: 143.3 | 10.6 |
| elliptic cone, impratio 10 | 25 | yes: 5.1 | 7.3 |
| elliptic cone, impratio 10 | 30 | yes: 6.5 | 8.3 |
| elliptic cone, impratio 10 | 35 | yes: 8.3 | 9.5 |
| + stiff finger coupling | 25 | yes: 3.0 | 0.6 |
| + stiff finger coupling | 30 | yes: 4.3 | 0.6 |
| + stiff finger coupling | 35 | yes: 6.0 | 0.7 |
| + stiff finger contact (chosen) | 25 | yes: 1.6 | 0.6 |
| + stiff finger contact (chosen) | 30 | yes: 2.2 | 0.6 |
| + stiff finger contact (chosen) | 35 | yes: 2.8 | 0.7 |

**Reading.**
- With MuJoCo's defaults (pyramidal friction cone, impratio 1) the tomato slides out of the
  fingers during the lift and stays on the table, at every size.
- The elliptic cone with impratio 10 (MuJoCo's documented advice for grasping) holds every size.
- The soft finger coupling let the two fingers close 7 to 11 mm apart; a stiffer equality
  constraint (solref 0.004, twice the timestep, the stiffest MuJoCo allows) cuts that to under
  1 mm and halves the slip.
- Stiffer finger contacts cut the remaining offset to 1.6 to 2.8 mm (larger tomatoes are
  heavier and slip more).

**Choice.** All three, now in `assets/arm.xml`. Trade-off: stiffer constraints need the small
2 ms timestep to stay stable, so the timestep cannot be raised later to speed up the trials
without re-running this.

## 2026-11-24: integral time sweep (payload sag against overshoot)

- **Script.** `python experiments/integral_sweep.py` (PYTHONPATH=src).
- **Settings.** Gravity-compensated PID, omega 25 rad/s, zeta 1.0, ki = kp / Ti. Payload: the
  pick-and-lift of the grasp sweep, sag = target height minus tool height after a 0.5 s hold.
  No payload: the 20 moves of the PID sweep (seed 0).

| integral time Ti (s) | tool sag, 25 / 30 / 35 mm tomato (mm) | worst tracking error, no payload (mm) | worst joint overshoot (mrad) |
|---|---|---|---|
| none | 3.3 / 5.7 / 9.0 | 2.36 | 7.13 |
| 2.0 | 0.8 / 1.4 / 2.3 | 2.33 | 8.39 |
| 1.0 | 0.2 / 0.3 / 0.6 | 2.39 | 9.59 |
| 0.5 | 0.0 / 0.0 / 0.1 | 2.70 | 11.78 |
| 0.25 | 0.0 / 0.0 / 0.0 | 3.17 | 15.40 |
| 0.1 | -0.0 / 0.0 / 0.0 | 3.91 | 21.62 |

**Reading.** Without integral action a held tomato pulls the tool 3.3 to 9.0 mm low. Ti = 1 s
cuts that to 0.6 mm or less; shorter Ti removes it fully but raises tracking error and
overshoot (21.6 mrad at Ti = 0.1 s against 7.1 without).

**Choice.** Ti = 1.0 s: sag at most 0.6 mm, overshoot 9.6 mrad (about 3 mm at the tool), tracking
error unchanged at 2.4 mm. Trade-off: a slower integral leaves a little sag in exchange for
calmer moves.

## 2026-11-24: HSV threshold sweep on 500 random scenes

- **Script.** `python experiments/hsv_sweep.py --scenes 500` and the same with `--light 0.3 1.3`
  (PYTHONPATH=src, MUJOCO_GL=egl), about 1 minute each.
- **Settings.** Scenes with seeds 1000 to 1499 (1,987 tomatoes), arm parked at CLEAR_POSE,
  320 x 240 render. Found = a detection within 15 mm of the tomato. Run after the half-pixel fix
  (docs/LOG.md); before it, every row had a median position error of 2.1 mm.
- **Caveat.** The classes are the hue ranges the scene generator draws from (ripe within 8
  degrees of red, turning 20 to 35, green 75 to 110), so this measures robustness to lighting,
  shading, size and position, not grading of real tomatoes, whose colours overlap.

**Normal lighting (0.6 to 1.1), turning_max 25** (turning_max 20 and 30 gave identical rows):

| s_min | v_min | turning_max | found | graded correctly | false positives | median / max position error (mm) |
|---|---|---|---|---|---|---|
| 60 | 20 | 25 | 1987/1987 | 1987/1987 (100.0%) | 0 | 0.3 / 0.7 |
| 60 | 40 | 25 | 1987/1987 | 1987/1987 (100.0%) | 0 | 0.3 / 0.7 |
| 60 | 80 | 25 | 1987/1987 | 1987/1987 (100.0%) | 0 | 0.7 / 3.3 |
| 100 | 20 | 25 | 1987/1987 | 1987/1987 (100.0%) | 0 | 0.3 / 0.9 |
| 100 | 40 | 25 | 1987/1987 | 1987/1987 (100.0%) | 0 | 0.3 / 0.9 |
| 100 | 80 | 25 | 1987/1987 | 1987/1987 (100.0%) | 0 | 0.6 / 3.0 |
| 140 | 20 | 25 | 1987/1987 | 1987/1987 (100.0%) | 0 | 0.3 / 0.9 |
| 140 | 40 | 25 | 1987/1987 | 1987/1987 (100.0%) | 0 | 0.3 / 0.9 |
| 140 | 80 | 25 | 1987/1987 | 1987/1987 (100.0%) | 0 | 0.6 / 3.2 |
| 180 | 20 | 25 | 1411/1987 | 1411/1987 (71.0%) | 5 | 0.4 / 13.2 |
| 180 | 40 | 25 | 1411/1987 | 1411/1987 (71.0%) | 5 | 0.4 / 13.2 |
| 180 | 80 | 25 | 1411/1987 | 1411/1987 (71.0%) | 4 | 0.6 / 13.2 |

**Harsh lighting (0.3 to 1.3), turning_max 25:**

| s_min | v_min | turning_max | found | graded correctly | false positives | median / max position error (mm) |
|---|---|---|---|---|---|---|
| 60 | 20 | 25 | 1987/1987 | 1987/1987 (100.0%) | 0 | 0.3 / 0.8 |
| 60 | 40 | 25 | 1987/1987 | 1987/1987 (100.0%) | 0 | 0.3 / 1.5 |
| 60 | 80 | 25 | 1987/1987 | 1987/1987 (100.0%) | 0 | 0.8 / 4.8 |
| 100 | 20 | 25 | 1987/1987 | 1987/1987 (100.0%) | 0 | 0.3 / 0.9 |
| 100 | 40 | 25 | 1987/1987 | 1987/1987 (100.0%) | 0 | 0.3 / 1.3 |
| 100 | 80 | 25 | 1987/1987 | 1987/1987 (100.0%) | 0 | 0.7 / 4.8 |
| 140 | 20 | 25 | 1987/1987 | 1987/1987 (100.0%) | 0 | 0.3 / 1.1 |
| 140 | 40 | 25 | 1987/1987 | 1987/1987 (100.0%) | 0 | 0.3 / 1.3 |
| 140 | 80 | 25 | 1987/1987 | 1987/1987 (100.0%) | 0 | 0.7 / 4.7 |
| 180 | 20 | 25 | 1408/1987 | 1408/1987 (70.9%) | 4 | 0.4 / 14.6 |
| 180 | 40 | 25 | 1408/1987 | 1408/1987 (70.9%) | 4 | 0.4 / 14.6 |
| 180 | 80 | 25 | 1408/1987 | 1408/1987 (70.9%) | 3 | 0.7 / 14.0 |

Confusion matrix with the defaults, harsh lighting:

| true \ graded | ripe | turning | green | unknown | missed |
|---|---|---|---|---|---|
| ripe | 689 | 0 | 0 | 0 | 0 |
| turning | 0 | 645 | 0 | 0 | 0 |
| green | 0 | 0 | 653 | 0 | 0 |

**Reading.**
- Saturation thresholds from 60 to 140 find and grade every tomato with no false positives in
  both lighting ranges. At 180, dull tomatoes (generated saturation down to 0.6, about 153 on
  OpenCV's scale) drop out: 71% found.
- A brightness floor of 80 cuts the dark edge of each silhouette and worsens the position (worst
  4.8 mm under harsh light); 20 keeps it (worst 0.9 mm).
- The turning/green boundary anywhere from 20 to 30 gives the same grading: the generated hues
  leave a wide gap.

**Choice.** s_min 100 (middle of the 60 to 140 plateau, margin on both sides) and v_min 20.

## 2026-11-24: 200 randomised episodes (first run)

- **Script.** `python experiments/trials.py --episodes 200` (PYTHONPATH=src, MUJOCO_GL=egl),
  243 s wall clock.
- **Settings.** Seeds 5000 to 5199, 3 to 5 tomatoes each, light 0.6 to 1.1. Full pipeline: look
  from CLEAR_POSE, pick the nearest graded tomato, carry, drop, look again. Bin walls 0.04 m,
  drop clearance 0.06 m. Scored by where each tomato is after a 1 s settle, not by what the
  robot believes.

| measure | value |
|---|---|
| episodes fully sorted | 188/200 |
| tomatoes in the correct bin | 779/793 (98.2%) |
| in a wrong bin | 0 |
| left on the tray | 0 |
| elsewhere (fell off) | 14 |
| pick attempts | 793 (0 more than tomatoes) |
| failed attempts by reason | none |
| simulated time per tomato, median (s) | 15.1 |
| throughput (tomatoes per simulated minute) | 3.9 |

Episodes not fully sorted:

- seed 5041: 4/5 correct, wrong bin 0, on tray 0, elsewhere 1, failures {}
- seed 5052: 4/5 correct, wrong bin 0, on tray 0, elsewhere 1, failures {}
- seed 5081: 4/5 correct, wrong bin 0, on tray 0, elsewhere 1, failures {}
- seed 5115: 4/5 correct, wrong bin 0, on tray 0, elsewhere 1, failures {}
- seed 5118: 3/4 correct, wrong bin 0, on tray 0, elsewhere 1, failures {}
- seed 5124: 4/5 correct, wrong bin 0, on tray 0, elsewhere 1, failures {}
- seed 5135: 4/5 correct, wrong bin 0, on tray 0, elsewhere 1, failures {}
- seed 5158: 3/4 correct, wrong bin 0, on tray 0, elsewhere 1, failures {}
- seed 5166: 3/5 correct, wrong bin 0, on tray 0, elsewhere 2, failures {}
- seed 5168: 2/3 correct, wrong bin 0, on tray 0, elsewhere 1, failures {}
- seed 5176: 3/5 correct, wrong bin 0, on tray 0, elsewhere 2, failures {}
- seed 5199: 4/5 correct, wrong bin 0, on tray 0, elsewhere 1, failures {}

**Reading.** No tomato went into a wrong bin and every pick looked successful to the robot, yet
14 tomatoes (1.8%) ended up off the table. Tracing seed 5041: the tomato was released above the
ripe bin, landed on a tomato already in it, bounced over the 0.04 m wall at about 0.4 m/s, and
rolled across the tray and off the edge. A silent failure: the robot has no sensor that sees it.
Per tomato this meets the 95% target (98.2%); per episode it does not (94%).

## 2026-11-24: drop height, rolling friction and bin walls

- **Script.** `python experiments/bin_drop_sweep.py`, about 10 minutes.
- **Settings.** The 12 episodes that lost a tomato above, plus 40 that did not (seeds 5000 to
  5039).

| setting | previously failing episodes now sorted | other episodes sorted | tomatoes lost |
|---|---|---|---|
| walls 0.04 m, clearance 0.06 m, rolling friction off | 0/12 | 40/40 | 14 |
| walls 0.04 m, clearance 0.06 m, rolling friction on | 3/12 | 12/40 | 54 |
| walls 0.04 m, clearance 0.03 m, rolling friction off | 4/12 | 37/40 | 12 |
| walls 0.04 m, clearance 0.03 m, rolling friction on | 3/12 | 13/40 | 49 |
| walls 0.04 m, clearance 0.01 m, rolling friction off | 4/12 | 39/40 | 9 |
| walls 0.04 m, clearance 0.01 m, rolling friction on | 2/12 | 11/40 | 53 |
| walls 0.06 m, clearance 0.01 m | 7/12 | 39/40 | 6 |
| walls 0.07 m, clearance 0.01 m | 12/12 | 40/40 | 0 |
| walls 0.08 m, clearance 0.01 m | 12/12 | 40/40 | 0 |

**Reading.**
- Rolling friction (condim 6), meant to stop tomatoes rolling forever, made things far worse:
  49 to 54 lost. Not investigated further; recorded as a failed idea in docs/LOG.md.
- A lower drop helps a little (14 lost down to 9); the wall height is what matters. At 0.04 m a
  tomato landing on another one rolls over the wall.
- 0.07 m walls with a 0.01 m drop lose nothing on these 52 episodes; 0.06 m still loses 6.

**Choice.** Walls 0.07 m (a shallow produce crate) and drop clearance 0.01 m. 0.08 m works too
but raises the drop point towards the edge of the arm's reach over the bins.
