# Receding-horizon map SITL trials

Experimental; no acceptance. These runs used the opt-in `MAP_PLANNING=1` path.

| Trial | Setup | Outcome |
|---|---|---|
| h1 | wall 4.2 m, goal 5.7 m | held, target error 4.04 m, 0 contacts |
| h2 | inflation 2 cells, 50 frontier candidates | moved ~2.5 m, 0 contacts, clearance 1.235 m, then no frontier; target error 2.26 m |
| h3 | 12 s yaw scan ±80° | coverage 0.72, no reachable frontier; error 5.35 m; 0 contacts |
| h4 | h3 + inflation 1 | frontier 0.1 m; moved to (0.03, -1.27); error 5.13 m; 0 contacts |
| h5 | continuous yaw scan | coverage 0.80, frontier west; moved ~0.5 m, stalled; error 5.97 m; 0 contacts |

Details and next requirements: [`docs/map-receding.md`](../docs/map-receding.md).
