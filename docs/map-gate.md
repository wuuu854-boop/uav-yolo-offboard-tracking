# Offline depth-map quality gate

The first occupancy-map attempts failed before any planner question could be
asked: the projected points did not match the known world. This gate makes that
check explicit and reproducible **without flying**. It also builds a small
occupancy grid from the same points and rejects it if the cells are noisy.

## Result: point projection passes, occupancy grid fails

```text
frame: results/search_depth_frame.npy
pose row: 215 at 21.50 s
points: 1080
near fraction (<= 1 m): 0.834
median distance: 0.339 m
occupied cells: 174
cell precision: 0.678
boxes covered: 3/3
start free: True
thresholds: points >= 100, near >= 0.7, median <= 1 m, grid precision >= 0.80
FAIL
```

The **point projection** is good (83.4% within 1 m, median 0.339 m), but the
**occupancy cells** are not: only 67.8% of the 174 cells are within 1 m of a known
box. 32% of the cells are sparse outliers that the 3-cell component filter does
not remove. A planner fed this map would detour around phantom obstacles or clip a
real one; therefore **no occupancy-map SITL run is attempted**.

Raw output: [`results/map_gate.md`](../results/map_gate.md) /
[`results/map_gate.json`](../results/map_gate.json),
[`results/map_cells.json`](../results/map_cells.json).

## The camera convention

The measured projection uses image row increasing **upward**:

```text
y_cam = (cy - v) / f * z
down  = z * sin(tilt) + y_cam * cos(tilt)
fwd   = mount_forward + z * cos(tilt) - y_cam * sin(tilt)
```

The opposite signs (used in the failed attempts) give **56%** near 1 m instead of
**83.4%**. That was the projection bug; the cell-noise result above is a second,
separate problem.

## What this verifies, and what it does not

| Claim | Status | Evidence |
|---|---|---|
| Projection convention matches the recorded frame | ✅ verified | 1080 points, 83.4% near, 0.339 m median |
| Occupancy cells are clean enough to plan on | ❌ **FAIL** | 174 cells, precision 0.678 < 0.80 |
| Known boxes covered | ✅ | 3/3 |
| Start free | ✅ | pose cell not occupied |
| A full occupancy map | ❌ **not delivered** | this is a one-frame check |
| SITL map planning | ❌ **not attempted** | the grid fails |

## Reproducing it

```bash
make map-gate
```

It intentionally exits non-zero while the grid precision is below 0.80. The next
step is to reduce the sparse outliers (multi-frame accumulation, height/region
filtering, stronger component/cluster filtering) and make this gate pass before
touching the SITL map planner.
