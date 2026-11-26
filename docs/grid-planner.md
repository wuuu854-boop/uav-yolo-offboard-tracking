# Offline grid A* on the depth map

Phase B starts with a pure planner over the occupancy grid that passed Phase A.
No ROS, no SITL: this proves the map cells can produce a collision-free path
before wiring the planner into the offboard node.

## Result

```text
frame: meta_avoid4 (sidecar pose)
start: (4.204, -0.549), goal: (9.5, 0.0)
points: 756, occupied cells: 62
path: 5 points, 5.86 m
plan time: 1.72 ms
min grid clearance: 1.414 m
PASS
```

The wall is at local x = 6.7; the path goes around its edge and never enters an
inflated occupied cell. A 2-cell **dilation/closing** bridges the one-cell sparse
gaps in the wall before inflation; without it the path was 4.42 m and only
0.901 m clear.

Raw output: [`results/grid_plan.md`](../results/grid_plan.md) /
[`results/grid_plan.json`](../results/grid_plan.json) /
[`results/grid_plan.png`](../results/grid_plan.png).

## Algorithm

- build `OccupancyGrid` from projected depth points (central ROI);
- keep cells with ≥2 points and a ≥3-cell connected component;
- dilate/close occupied cells by 2 cells to bridge one-cell sparse gaps;
- inflate occupied cells by 3 cells (0.75 m at 0.25 m resolution);
- 8-connected A*, no corner cutting;
- line-of-sight smoothing on the inflated grid;
- return points after the start, ending exactly at the goal.

## What this verifies, and what it does not

| Claim | Status | Evidence |
|---|---|---|
| Grid A* avoids the recorded wall | ✅ PASS | path length 5.86 m, clearance 1.414 m |
| Planner is fast enough offline | ✅ 1.72 ms | `path_length`/`plan_ms` |
| Start/goal blocking is rejected | ✅ unit tests | 4 planner tests |
| Replanning while flying | ❌ not implemented | no ROS node yet |
| Map ready/stale gate | ❌ not implemented | next Phase B task |
| SITL acceptance | ❌ not started | after integration |

## Reproducing it

```bash
make grid-plan
```

The next step is to connect this planner to the offboard node with a map-ready
check and a bounded replan rate, then run the 20-run SITL acceptance.
