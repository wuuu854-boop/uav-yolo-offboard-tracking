# Receding-horizon map SITL trials

Experimental; no acceptance. These runs used the opt-in `MAP_PLANNING=1` path.

| Trial | Setup | Outcome |
|---|---|---|
| h1 | wall 4.2 m, goal 5.7 m | held, target error 4.04 m, 0 contacts |
| h2 | inflation 2 cells, 50 frontier candidates | moved ~2.5 m, 0 contacts, clearance 1.235 m, then no frontier; target error 2.26 m |

Details and next requirements: [`docs/map-receding.md`](../docs/map-receding.md).
