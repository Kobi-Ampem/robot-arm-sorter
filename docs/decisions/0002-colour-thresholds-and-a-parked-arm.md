# 0002: colour thresholds, a fixed overhead camera and a parked arm

Date: 24 Nov 2026

## Context

The arm needs each tomato's ripeness and position on the tray. The laptop has no GPU and about
2 GB of free RAM, and the tomatoes sit on a plain grey tray under one overhead light.

## Options

1. HSV colour thresholds and connected components on a fixed overhead camera image.
2. A trained detector (for example a small YOLO model) on rendered images.
3. A camera on the wrist, looking at each tomato from close up.

## Choice

Option 1, with the arm parked out of view (`CLEAR_POSE`) for every look, and a fresh look after
every pick.

## Why

- On a plain tray with saturated fruit, colour separates tomato from background without any
  training data, runs in milliseconds on CPU, and every threshold can be justified by a sweep.
- A fixed camera above the tray gives one known geometry: a pixel becomes a ray, and the ray
  meets the plane at the tomato's centre height, which comes from the blob's size.
- Parking the arm and looking again after each pick keeps the view clear and catches tomatoes a
  failed grasp knocked aside, without tracking anything between looks.

## Trade-off accepted

- Colour thresholds would not survive a cluttered or changing background, and the ripeness
  classes are the generator's own hue ranges, so the grading result says nothing about real
  tomatoes. A trained model on labelled photos would be the step towards hardware.
- Parking and looking after every pick costs time: it is a large part of the 15 s per tomato.
- The fixed camera does not see the green bin, so it cannot confirm every drop.
