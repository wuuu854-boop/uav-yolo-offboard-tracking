# Offline depth-map quality gate

The first occupancy-map attempts failed before any planner question could be
asked: the projected points did not match the known world. This gate makes that
check explicit and reproducible **without flying**.

## What it does

`make map-gate` takes:

- a recorded depth frame (`results/search_depth_frame.npy`),
- the pose row where it was captured (`results/search_run.csv`, row 215),
- three known boxes from the search world (wall + two pillars).

It projects sampled pixels into local ENU, counts how many points are within 1 m
of a known box, and requires:

- at least 100 points,
- at least 70% within 1 m,
- median distance at most 1 m.

## Result

```text
frame: results/search_depth_frame.npy
pose row: 215 at 21.50 s
points: 1080
near fraction (<= 1 m): 0.834
median distance: 0.339 m
thresholds: points >= 100, near >= 0.7, median <= 1 m
PASS
```

Raw output: [`results/map_gate.md`](../results/map_gate.md) /
[`results/map_gate.json`](../results/map_gate.json).

## The convention that was wrong

The measured projection uses image row increasing **upward** and:

```text
y_cam = (cy - v) / f * z
down  = z * sin(tilt) + y_cam * cos(tilt)
fwd   = mount_forward + z * cos(tilt) - y_cam * sin(tilt)
```

The opposite signs (used in the failed occupancy-map attempts) produce **56%**
within 1 m instead of **83.4%**. That is why those maps were contaminated and
clipped walls; the planner was not the first problem.

## What this verifies, and what it does not

| Claim | Status | Evidence |
|---|---|---|
| The projection convention matches the recorded frame | ✅ verified | 1080 points, 0.834 near, 0.339 m median |
| The gate rejects a wrong sign | ✅ checked during bring-up | wrong sign gave 0.556 near |
| The gate is reproducible | ✅ `make map-gate` | fixed frame + explicit row |
| A full occupancy map | ❌ **not implemented** | this is point projection only |
| SITL map planning | ❌ **not verified** | no flight in this gate |
| Automatic frame/pose pairing | ❌ **not implemented** | the row is explicit; the recorder overwrites frames |

## Reproducing it

```bash
make map-gate
```

This is a pre-flight gate, not a new avoidance claim. The next step, if pursued,
is to build the occupancy grid from these points and pass the same known-box
check before any SITL map run.
