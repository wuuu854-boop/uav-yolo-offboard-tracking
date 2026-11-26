# SITL map-planner integration attempts

These are failure evidence; no map-planner code is merged and no SITL map claim is
made. The offline grid A* (v0.40) remains the last verified map result.

| Trial | Setup | Outcome |
|---:|---|---|
| 1 | map depth limit 4 m, spawn -7.5 | hovered; target error 7.15 m; 0 contacts |
| 2 | depth limit 8 m | 73 contacts; error 6.9 m |
| 3 | path logged | 31 contacts; error 1.97 m |
| 4 | waypoint tolerance 0.5 | 23 contacts; error 2.12 m |
| 5 | inflation 3 cells | landing 0.173 m, 0 contacts, clearance 0.227 m < 0.4 |
| 6 | inflation 4 cells | landing 0.725 m; 17 contacts |
| 7 | sample every 6 px | 29 contacts; error 2.58 m |
| 8 | spawn -5, goal 5.7 | 257 contacts; error 1.85 m |
| 9 | accumulate 10 frames | never ready; hover; error 5.68 m |

Root causes and the next design requirements are in
[`docs/map-integration.md`](../docs/map-integration.md).
