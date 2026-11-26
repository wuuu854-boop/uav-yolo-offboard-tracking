# Receding-horizon map integration (experimental)

This is the current Phase B attempt. It is **not verified**: the map planner code
is opt-in (`global_map_planning: true`), the default missions are unchanged, and
no 20-run acceptance has passed.

## What is wired

- `map_planner_node.py`: accumulates observed-free space from live depth frames;
  when the final goal is observed it publishes a known-free goal path, otherwise a
  known-free frontier path;
- offboard: holds until `/map/ready` + `/map/path`, then follows the latest path;
- runner: `MAP_PLANNING=1` starts the node; otherwise the old behaviour is
  byte-for-byte unchanged.

## SITL trials (2026-11-26)

| Trial | Goal | Result |
|---|---|---|
| h1 | 5.7 m, wall 4.2 m | no reachable frontier; held; target error 4.04 m; 0 contacts |
| h2 | same, inflation 2, 50 frontier candidates | moved ~2.5 m; 0 contacts; clearance 1.235 m; then no frontier; target error 2.26 m |

h2 is the first map trial with **0 contacts and >1 m clearance** while moving, but
it did not reach the goal. The frontier loop dead-ends when the observed-free
corridor narrows; coverage collapses to 0 as the vehicle moves into unobserved
space.

## What this means

- The receding-horizon idea is in the right direction but the free-space
  connectivity/coverage is still insufficient.
- A single forward depth camera at 5 m with a 60% ROI does not observe enough
  around the wall to keep a connected frontier chain.
- The next requirement is an **active scan** (yaw sweep) or a wider/multi-frame
  observation policy, not more path smoothing.

## Status

- map SITL acceptance: **not passed**;
- default flight missions: unchanged and still 20/20;
- no map claim in README verified table.

## Reproducing the attempt

```bash
POSITION=-5,0,0.05,0,0,1.5708 \
MISSION=config/mission_map.yaml PREFIX=map_h2 MODE=global TARGET_ID=0 \
OBSTACLE=config/obstacle.yaml MAP_PLANNING=1 MAP_GOAL=5.7,0 \
bash scripts/run_avoidance.sh
```

The next design should add a scan phase before the first frontier plan.
