# Offline depth-map quality gate

The first occupancy-map attempts failed before any planner question could be
asked. This gate checks both the projected points and the occupancy grid built
from them, **without flying**.

## Result: PASS

```text
frame: results/search_depth_frame.npy
pose row: 215 at 21.50 s
points: 972
near fraction (<= 1 m): 0.902
median distance: 0.260 m
occupied cells: 143
cell precision: 0.825
boxes covered: 3/3
start free: True
thresholds: points >= 100, near >= 0.7, median <= 1 m, grid precision >= 0.80
PASS
```

Raw output: [`results/map_gate.md`](../results/map_gate.md) /
[`results/map_gate.json`](../results/map_gate.json),
[`results/map_cells.json`](../results/map_cells.json).

## The two fixes that mattered

1. **Camera sign.** The measured projection uses image row increasing upward:
   `y_cam = (cy - v)/f*z`, `down = z sin + y_cam cos`,
   `fwd = mount_forward + z cos - y_cam sin`. The opposite signs gave 55.6% near
   1 m instead of 90.2%.
2. **Height floor at 2.5 m.** The low 2.0–2.5 m band contained sparse outliers
   that formed false cells east of the wall. Raising the floor to 2.5 m (the
   wall/pillars extend above it) took the grid precision from 0.678 to **0.825**
   without changing the 1.0 m tolerance. The SOR/cluster experiments did not fix
   it; the height band did.

## What this verifies, and what it does not

| Claim | Status | Evidence |
|---|---|---|
| Projection convention matches the recorded frame | ✅ verified | 972 points, 90.2% within 1 m, 0.260 m median |
| Occupancy cells clean enough to plan on | ✅ **PASS** | 143 cells, precision 0.825, 3/3 boxes, start free |
| Low-height outliers were the grid problem | ✅ verified | 2.0 m floor 0.678 → 2.5 m floor 0.825 |
| A multi-frame map | ❌ **not implemented** | one recorded frame/pose pair |
| Map planning / SITL | ❌ **not attempted** | this gate is the pre-flight condition, not a flight |
| Real sensor noise | ❌ **not verified** | ideal Gazebo depth |

## Reproducing it

```bash
make map-gate
```

The next phase is the occupancy-map planner and its 20-run SITL acceptance. The
grid gate passing does not make the map planner verified; it only allows that
work to start.
