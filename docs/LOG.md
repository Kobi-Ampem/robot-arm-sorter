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
