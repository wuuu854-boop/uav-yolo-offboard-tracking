# Skill 04 — Derive camera intrinsics from the model file

| | |
|---|---|
| **When** | You need `fx, fy, cx, cy` and nobody wrote them down |
| **Time** | 10 minutes |
| **Needs** | The camera model's `model.sdf` |
| **From** | `docs/scenario.md` — the 640×480 assumption that was wrong |

## Symptom

The projection is "basically right" near the centre and wrong at the edges, or
the error grows linearly with altitude. Nobody changed the code.

## Why

Gazebo cameras specify a **horizontal field of view and a resolution**, not a
camera matrix. Guessing the resolution (or copying intrinsics from a tutorial)
gives a matrix that is off by a scale factor, and the error scales with height:
a 2× error in `cx` is 2× the offset at every altitude.

## Procedure

1. Read the sensor block:

   ```bash
   sed -n '/<sensor name="camera"/,/<\/sensor>/p' \
     "$PX4_DIR/Tools/simulation/gz/models/mono_cam/model.sdf"
   ```

   For PX4's `mono_cam`: `horizontal_fov = 1.74` rad, image `1280 × 960`.

2. Convert. With square pixels:

   ```
   fx = (W / 2) / tan(HFOV / 2)
   fy = fx
   cx = W / 2      cy = H / 2
   ```

   For the numbers above: `fx = 640 / tan(0.87) = 538.4`.

3. Write them down **with the source**, in `config/camera.yaml`, and recompute any
   range budget that used the old values.

## Verify

Pick one target at a known offset and predict its pixel before running anything:

```
u = cx + (east offset)  * fx / altitude
v = cy - (forward offset) * fy / altitude
```

At 5 m with a target 2 m to the right: `u = 640 + 2 × 538.4 / 5 = 855`. Hover,
detect, and compare. A few pixels of error is noise; tens of pixels means the
intrinsics are still wrong.

## The trap

Intrinsics are only valid for the resolution and FOV they were derived from.
Changing the model, the image size, or the FOV silently invalidates every number
in the results table — which is why the values live in a config file with the
derivation next to them, not inline in the code.
