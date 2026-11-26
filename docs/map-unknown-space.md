# Unknown space is not free: a correctness finding

The v0.40 offline grid A* passed, but it treated every cell that was not a depth
hit as **free**. That is optimistic: an unseen gap may contain a wall. This page
records the stricter check and why Phase B SITL failed.

## Known-free mode

`OccupancyGrid.add_points_with_rays(points, sensor_xy)` now marks the cells along
each sensor ray as observed free, and `plan_grid_path(..., require_known_free=True)`
blocks every cell that was never observed. A path may only use cells the sensor
has seen through; the goal must be observed free.

## Results on the Phase A avoidance frame

| Start | Goal | Known-free result |
|---|---|---|
| (4.204, -0.549) | (9.5, 0.0) | **FAIL: goal is not observed free** |
| (4.204, -0.549) | (5.0, 0.0) | **FAIL: goal is not observed free** |

The v0.40 path to 9.5 m existed only because unknown space was assumed free. In
SITL this is exactly how A* found a "gap" through a wall that the sparse depth map
had not observed. The strict gate rejects it.

The second failure shows a coverage problem, not just a goal behind the wall: the
ray fan is sparse at 0.25 m cells, so even a nearby goal can be unobserved. A real
map planner needs a **free-space coverage** gate (or a deliberate scan), not only
point count and wall cells.

## What this changes

- v0.40 remains an offline path under the unknown-free assumption; it is **not** a
  flight-ready planner.
- The SITL map integration failures are explained by this assumption plus sparse
  maps; the integration remains reverted.
- Phase B now requires:
  1. free-space coverage/scan so the route is observed;
  2. a map-completeness gate before `/map/ready`;
  3. wall extraction/denoising;
  4. then the 20-run SITL acceptance.

## Tests

`tests/test_grid_planner.py` covers:
- a known-free path through observed cells;
- an unknown goal rejected when `require_known_free=True`.

This is a correctness gate, not a new flight claim.
