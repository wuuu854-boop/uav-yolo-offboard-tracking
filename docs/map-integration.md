# SITL map-planner integration: attempts and why it was reverted

The offline grid A* passed (v0.40), so the next step was to wire it into the
offboard node. That integration is **not verified** and was reverted. This page
records the attempts so the next design does not repeat them.

## What was tried

- `map_planner_node.py`: depth frame -> pose projection -> `OccupancyGrid` ->
  `plan_grid_path` -> `/map/path` + `/map/ready`;
- offboard: hold until ready, then follow the path;
- mission: spawn at local x=0, wall at local x=6.7 or 4.2 depending on spawn,
  goal east of the wall, target_0 within 1.5 m of the goal.

## Trials

| # | Setup | Result | Root cause |
|---|---|---|---|
| 1 | spawn -7.5, goal 8.2, frame maxdepth 4 | hovered 80 s; target error 7.15 m; 0 contacts | wall at 6.7 m beyond the 4 m depth limit -> grid empty -> no path |
| 2 | maxdepth 8 | flew north; 73 contacts; error 6.9 m | path/control mismatch, path crossed the wall |
| 3 | map path logged | reached 8.2 area but 31 contacts | map had a gap; final segment crossed the wall |
| 4 | waypoint tolerance 0.5 | 23 contacts; target error 2.12 m | same incomplete wall |
| 5 | maxdepth 8, inflation 3 | **landed 0.173 m, 0 contacts**, but clearance 0.227 m < 0.4 | path clipped the wall envelope |
| 6 | inflation 4 | landed 0.725 m; 17 contacts | landing/path still inside envelope |
| 7 | sample 6 | 29 contacts; error 2.58 m | denser points did not fill the wall |
| 8 | spawn -5, wall 4.2, goal 5.7 | 257 contacts; error 1.85 m | map sparse/incomplete, direct path through wall |
| 9 | accumulate 10 frames | never ready; hover; error 5.68 m | goal inside inflated cells from accumulated outliers |

## Why it was reverted

The map was **not complete enough to plan on** at the time of first use:

- at long range the wall is only a few pixels/cells; A* finds a gap that is not there;
- accumulated frames add outliers; the goal can become "inside an inflated obstacle";
- there was no map-completeness gate, only point count;
- the path can graze the wall even when the landing row is successful.

The offline map gate (v0.39) passes on two chosen frames, but that is not the same
as a map that is complete from every spawn pose.

## Status

- no map-planner code is merged in this release;
- no map SITL claim;
- v0.40 offline grid A* remains the last verified map result;
- map integration stays a Phase B task.

## Next design requirements

1. A **completeness gate** (wall cells along the expected corridor, not just point
   count) before `/map/ready`.
2. Wall/plane extraction (RANSAC or column occupancy) rather than raw sparse cells.
3. Multi-frame accumulation **while moving** or a deliberate scan, not a single
   hover frame.
4. Receding-horizon replan with stale-map handling.
5. Re-run the 20-run SITL acceptance only after the above; keep failures.
