# Receding-horizon frontier selection

A single forward frame cannot observe a route around a wall. The correct response
is not to plan through unknown space; it is to move to the edge of what has been
observed, look again, and replan.

## What is implemented

`planning/frontier.py`:

- `frontier_cells()`: observed-free cells with an unknown 8-neighbour;
- `choose_frontier()`: plans a **known-free** path to candidate frontiers and
  picks the one minimizing path length + straight distance to the final goal.

Only the selection is implemented; the offboard integration is not.

## Offline result

```text
frame: meta_avoid4
start: (4.20, -0.55), goal: (9.5, 0.0)
points: 756
frontier: (4.875, -0.375)
path: 0.69 m
total cost: 5.33 m
corridor coverage to final goal: 0.232
PASS (frontier reachable)
```

The frontier is only 0.69 m ahead; that is expected. The vehicle must move,
accumulate another frame, and select the next frontier. The final goal is not yet
observable (23% corridor coverage), so the planner refuses a direct path.

## What this verifies, and what it does not

| Claim | Status | Evidence |
|---|---|---|
| Frontier cells found from observed free space | ✅ | 0.69 m frontier |
| Known-free path to a frontier | ✅ | path 0.69 m |
| Route coverage increases by moving | ❌ not tested | no scan/motion loop yet |
| Offboard follows frontier paths | ❌ not integrated | next Phase B step |
| 20-run SITL | ❌ not started | — |

## Reproducing it

```bash
make frontier-plan
```

The next step is a receding-horizon node: publish the frontier path, move a short
segment, rebuild the map with new frames, and repeat until the final goal is
observed. Only then can a map SITL be attempted.
