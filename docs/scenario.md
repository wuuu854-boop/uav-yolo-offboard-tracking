# Scenario and budget

Why these numbers and not others. A pure-CV candidate can train a detector; the
part that requires drone knowledge is knowing whether the detector can see the
target at the chosen altitude, and what happens when it cannot.

## Camera footprint at 5 m

```
HFOV          = 60 deg
fx            = (W/2) / tan(HFOV/2) = 320 / tan(30 deg) = 554.26 px
footprint W   = 2 * h * tan(HFOV/2) = 2 * 5 * 0.5774 = 5.77 m
ground sample = h / fx              = 5 / 554.26     = 9.0 mm/px
```

| Altitude | Footprint (W × H) | GSD | 0.5 m target |
|---|---|---|---|
| 3 m | 3.46 × 2.60 m | 5.4 mm/px | 92 px |
| **5 m** | **5.77 × 4.33 m** | **9.0 mm/px** | **55 px** |
| 8 m | 9.24 × 6.93 m | 14.4 mm/px | 35 px |
| 10 m | 11.55 × 8.66 m | 18.0 mm/px | 28 px |

At 5 m a 0.5 m cube is about 55 px wide — comfortably inside the range where a
small detector works, and the whole footprint still fits on screen so the target
is visible for the entire approach.

## Why the mission is limited to a 6 × 8 m area

The three targets sit within a 6 m × 7 m box. At 5 m altitude the camera sees
5.77 × 4.33 m, so **at most one target is in view at a time**. That is a design
choice, not a limitation:

- the mission is "find the one target in view and fly to it", not "track many";
- it makes the ground-truth association unambiguous (nearest target = the one
  that was in view), which is what the landing-error metric relies on;
- multi-target association is a v0.2.0 problem and needs a tracker, not a filter.

## Failure budget

| Source | Estimated contribution | How it is measured |
|---|---|---|
| Intrinsics error | ±0.1 m at 5 m if `fx` is off by 2 % | static test, W2 |
| Roll/pitch not compensated | up to `h * tan(roll)`; 3° at 5 m ≈ 0.26 m | `results/projection_error.csv` |
| Detection box centre noise | ~±0.1 m for a 55 px target | repeated static frames |
| Control overshoot at 1 m/s | ~0.3 m | `landing_error.csv` |

The 1.5 m success threshold is set above the sum of these (~0.7 m) with margin,
so a pass means the pipeline works, not that the threshold was tuned to the result.

## What would change at real scale

- 30 m altitude: GSD 54 mm/px, a 0.5 m target is 9 px — a small-object problem,
  which is why the VisDrone/SAHI line of work exists.
- Wind and attitude transients make the nadir assumption invalid; you need the
  full rotation matrix, not just yaw.
- Both are explicitly out of scope for v0.1.0 and are listed in the README's
  Limitations section.
