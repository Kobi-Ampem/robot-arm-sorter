# 0003: score against ground truth, and report held-out seeds

Date: 24 Nov 2026

## Context

The robot judges its own success from what it can sense: whether the fingers closed on
something, and whether the tray looks empty. The simulator knows where every tomato really is.

## Options

1. Report the robot's own view: a pick counts if it believed the pick worked.
2. Score every episode by where each tomato really is after everything settles.
3. As 2, and also keep a set of random seeds that no design choice ever looks at.

## Choice

Option 3.

## Why

- The first 200-episode run showed why: every pick looked successful to the robot, yet 14
  tomatoes had bounced out of the bins and off the table. Option 1 would have reported 100%.
- The bin fix was chosen by looking at failing seeds from that same run, so a rerun on those
  seeds measures the fix on the data it was tuned on. Seeds 6000 to 6199 were used only once,
  for the final number.

## Trade-off accepted

Ground truth only exists in simulation. On hardware the same gap would need extra sensing (for
example looking at the bins), which this project names but does not build.
