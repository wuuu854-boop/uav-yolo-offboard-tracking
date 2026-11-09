# Scenario and budget

Why these numbers and not others. A pure-CV candidate can train a detector; the
part that requires drone knowledge is knowing whether that detector can see the
target at the chosen altitude — and what happens when it cannot.

## Camera, measured from the model (not guessed)

Source: `models/mono_cam/model.sdf` in
[PX4-gazebo-models](https://github.com/PX4/PX4-gazebo-models), used by
`x500_mono_cam_down`:

```
horizontal_fov = 1.74 rad (99.7 deg)
image          = 1280 x 960
fx = (W/2) / tan(HFOV/2) = 640 / tan(0.87) = 538.4 px
fy = fx                                  (square pixels)
vertical FOV = 2 * atan(480 / 538.4) = 83.5 deg
```

An earlier version of this project assumed 640x480 at 60 deg — a 2x error in
`cx,cy` and the wrong FOV. That is why the source of every intrinsic is written
down here and repeated in `config/camera.yaml`.

## Footprint

| Altitude | Visible patch (W x H) | GSD | 0.5 m target |
|---|---|---|---|
| 3 m | 7.1 x 5.4 m | 5.6 mm/px | 90 px |
| **5 m** | **11.9 x 8.9 m** | **9.3 mm/px** | **54 px** |
| 8 m | 19.0 x 14.3 m | 14.9 mm/px | 34 px |
| 10 m | 23.8 x 17.8 m | 18.6 mm/px | 27 px |

At the mission altitude of 5 m the vehicle sees a 11.9 x 8.9 m patch and a 0.5 m
cube covers about 54 px — comfortably inside the range where both the colour
baseline and a small detector work.

## Why the targets are 20 m apart

A 11.9 m wide patch means three targets placed a few metres apart would all be
in view at once, and "which one is the mission about?" becomes ambiguous — which
silently ruins the landing-error metric.

So the targets sit at `(0,0)`, `(20,0)` and `(0,20)`, and each trial **spawns
the vehicle 3.5 m west of the target under test**:

| Trial target | Spawn pose (`PX4_GZ_MODEL_POSE`) |
|---|---|
| `target_0` | `-3.5,0,0.05` |
| `target_1` | `16.5,0,0.05` |
| `target_2` | `0,16.5,0.05` |

At 5 m altitude, a target 3.5 m ahead lands at `v ≈ 103 px` (the frame is 960 px
tall), so it is inside the image from shortly after takeoff and stays there
during the approach. The approach distance is 3.5 m — short, because this is a
measurement harness, not a long-range search demo (search is v0.2.0).

## Failure budget

| Source | Contribution at 5 m | How it is measured |
|---|---|---|
| Intrinsics error | 0.1 m if `fx` is off by 2 % | static test, W2 |
| Roll/pitch not compensated | `h * tan(roll)`; 3 deg = 0.26 m | `results/projection_error.csv` |
| Camera mount offset ignored (if you disable it) | 0.10 m vertical | unit test `test_camera_offset_*` |
| Detection centroid noise | ~0.1 m for a 54 px target | repeated static frames |
| Control overshoot at 1 m/s | ~0.3 m | `landing_error.csv` |

The 1.5 m success threshold sits above the sum of these (~0.8 m) with margin, so
passing means the pipeline works rather than the threshold being tuned to the
result.

## What changes at real scale

- 30 m altitude: GSD 56 mm/px, a 0.5 m target is 9 px — a small-object problem,
  which is why the VisDrone / SAHI line of work exists.
- Wind and attitude transients invalidate the nadir assumption; you need the
  full rotation matrix, not just yaw.
- Both are out of scope for v0.1.0 and listed in the README's Limitations.
