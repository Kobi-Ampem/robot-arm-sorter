# Engineering log

Real bugs (symptom, root cause, fix, lesson) and failed ideas (with numbers), newest last.

## 24 Nov 2026: closed-form IK "failed" on poses with the tool behind the base

- **Symptom.** The first version of `test_closed_form_recovers_the_elbow_up_solution` failed: for
  random top-down poses, the solver returned joint angles that differed from the ones that made
  the target, with the base yaw off by exactly pi. The companion test (does the solver's answer
  reach the target?) passed on the same poses.
- **Root cause.** Not a solver bug. The test sampled shoulder angles up to 3.0 rad, past vertical.
  With the shoulder tipped back (for example q2 = 2.5, q3 = -0.5) the tool reach in the arm's
  plane is negative (-0.235 m): the tool is behind the base. The same point can be reached facing
  it, with the base turned by pi, and that is the solution the solver returns.
- **Fix.** The test now samples only poses with the tool in front of the base, the solver's
  docstring names the second solution family, and a new test checks a target behind the base.
- **Lesson.** Check reachability by position first, and treat "the same joint angles" as a
  stronger claim that only holds within one solution family.

## 24 Nov 2026: elbow-down IK hits the wrist limit for low, close targets

- **Symptom.** `test_elbow_down_also_reaches` failed for the target (0.25, 0.05, 0.08): the solver
  returned "joint limits".
- **Root cause.** With the elbow down, the wrist must bend sharply to point the gripper straight
  down: q4 = -2.72 rad there, past the -2.7 limit. For (0.20, 0, 0.20) it needs -3.14.
- **Fix.** The test uses a far, high target (0.35, 0.05, 0.12), where q4 = -2.14, and a separate
  test checks that the low, close case is reported as a joint-limit failure.
- **Lesson.** Elbow-up is the only practical branch for picking from the table; elbow-down stays
  as an option but should not be the default.

## 24 Nov 2026: damped least squares IK from a fixed start pose (failed idea)

- **Idea.** Start the numerical solver from one fixed ready pose (base facing +x, gripper
  pointing down) for every target.
- **Numbers.** On 1,000 random top-down targets spread all around the base (seed 4, damping 0.05,
  200 iterations, tolerance 1e-6 m and 1e-6 rad): 129 converged, median iterations 200 (the cap).
  For targets behind the arm, the solver bent the shoulder back over the base instead of turning
  the base, and stalled with joints at their limits.
- **What replaced it.** Turn the base to face the target in the start pose (one atan2): 956 of
  1,000 converged, median 14 iterations. None of the 44 left were at a joint limit; they converge
  slowly (median error 48 micrometres after 200 iterations), and all lie outside the pick area
  (radius 0.15 to 0.38 m, shoulder at 0.03 to 1.21 rad). All 228 targets inside the pick area
  (radius 0.15 to 0.35 m, height up to 0.15 m) converged.
- **Lesson.** For a local solver the seed matters as much as the update rule; use what is known in
  closed form (here, the base yaw) to start it in the right basin.

## 24 Nov 2026: the base joint would not turn

- **Symptom.** The first PID tracking test moved the arm from the ready pose to a target 0.38 rad
  round the base. The base finished 0.36 rad short with its motor saturated at the 2 N m limit,
  although no gravity acts on a vertical axis.
- **Root cause.** `data.contact` showed four contacts between the `base` and `turret` bodies at
  10 mm depth. The base body has no joint, so MuJoCo welds it to the world, and the rule that
  ignores contacts between a body and its parent does not apply to a parent welded to the world.
  The turret cylinder (z 0.01 to 0.10) overlapped the base plate (top at z 0.02), so friction
  fought every base rotation. The closed fingers also touched each other (four more contacts).
- **Fix.** The turret geometry now starts at z 0.02 (no overlap), and the model excludes
  base-turret and finger-finger contacts explicitly. New tests: no contacts at rest in free space,
  and the base turns more than 0.5 rad in 0.5 s under 0.2 N m with gravity off. Under that test,
  the base turned 0.0097 rad before the fix and 0.789 rad after. (The first version of the test
  left gravity on; with no controller the arm fell onto the table and table friction slowed the
  base, so the test measured the wrong thing.) The PID test move then tracked the base to
  0.0014 rad RMS error instead of finishing 0.36 rad short.
- **Lesson.** After building a model, check `ncon` at rest before tuning anything; a controller
  will happily "work" against a hidden contact, and the symptom looks like a tuning problem.

## 24 Nov 2026: scene generation hung

- **Symptom.** The first run of the scene tests never finished (stopped after 120 s).
- **Root cause.** Tomato positions were drawn by rejection sampling with no limit. Six tomatoes
  at 0.09 m spacing in the first pick area (0.14 x 0.24 m) only fit in near-perfect grid
  layouts, so once a few random positions boxed the rest out, no later draw could succeed and
  the loop ran forever.
- **Fix.** Scenes now hold 3 to 5 tomatoes, the pick area grew to 0.15 x 0.28 m (all of it
  still within 0.35 m of the base, where IK was benchmarked), and a layout that fails 200 draws
  in a row starts again. A new test builds 50 six-tomato scenes.
- **Lesson.** Any rejection-sampling loop needs a bound or a restart; "it usually finishes" is
  not a property a test suite can rely on.

## 24 Nov 2026: the gripper could not lift a tomato

- **Symptom.** In the first grasp trial the fingers closed on the tomato, but during the lift
  the tomato slid out of them and stayed on the table, 134 to 143 mm below the tool point, for
  every size from 25 to 35 mm radius.
- **Root cause.** Two things, found by printing finger positions, contacts and the tomato's
  position in the hand frame at each phase. The approach was accurate (tomato centre within
  0.3 mm of the tool point). But (1) with MuJoCo's default pyramidal friction cone and impratio 1,
  friction is soft and a constant load makes the contact creep, so the tomato slid down out of
  the grip as the arm lifted; and (2) the soft equality constraint coupling the fingers gave way
  under the actuator force, so the driven finger closed 7 to 11 mm further than the other.
- **Fix.** Elliptic friction cones with impratio 10, a stiffer finger coupling and stiffer finger
  contacts (sweep and numbers in experiments/RESULTS.md). The tomato now stays within 1.6 to
  2.8 mm of the tool point through the lift. New tests lift each size and release one.
- **Lesson.** In MuJoCo, grasping needs the contact model set up for it; the defaults are tuned
  for locomotion-style contacts, not for holding an object against gravity.

## 24 Nov 2026: tomato positions were 1.5 mm off in x and y

- **Symptom.** In the first threshold sweep (500 scenes) the median detection position error was
  2.1 mm for every threshold setting, too uniform to be noise.
- **Root cause.** Over 100 scenes the mean error was +1.54 mm in x and +1.50 mm in y and did not
  grow with distance from the camera axis (slopes of 0.003 and 0.002). A constant offset of about
  half a pixel (one pixel covers about 3 mm of table): OpenCV reports a pixel's centre at its
  integer index, while the ray code measured from the image corner, so every ray was shifted by
  half a pixel.
- **Fix.** `Camera.ray` adds 0.5 to both pixel indices. Median error over the same 100 scenes
  fell from 2.1 mm to 0.31 mm, worst case from 2.8 mm to 0.75 mm. The test bound tightened from
  4 mm to 2 mm, and the optical-axis test now uses the corner shared by the four middle pixels.
- **Lesson.** A constant bias that does not depend on position points at a convention
  (pixel centres, frames, units), not at the model.
