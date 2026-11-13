# results/

Raw experiment data. These files are the evidence behind every number in the
README and in the resume, so they are written by scripts, never by hand.

| File | Written by | Content |
|---|---|---|
| `landing_error.csv` | `src/uav_tracking/control/recorder.py` during `make e2e` | one row per trial: nearest target id, landing error, latency percentiles, success |
| `latency_raw.csv` | same recorder, same run | every latency sample, so the run's p95 is computed by pooling rather than by averaging per-trial percentiles |
| `state_trace.csv` | `src/uav_tracking/control/trace.py` during `make e2e` | one row per state change: when the mission armed, hovered, searched, landed, and why |
| `v02_validation.csv` | three runs with the v0.2.0 safety constraints | 0.94 / 1.01 / 1.49 m, all within tolerance; the nine-row table above stays the v0.1.0 measurement |
| `v03_yolo_trials.csv` | four runs with the YOLO backend and hard negatives | one within tolerance (0.52 m), three at 2.2–2.4 m; kept as the evidence for `docs/model.md` |
| `backend_comparison.txt` | both backends on the same 30 recorded frames and poses | mean difference (+0.49, +0.35) m — the per-frame projection is close; the trial error is closed-loop |
| `yolo_close_range.txt` | the same comparison on a YOLO flight, 30 frames | (+0.52, +0.39) m — the bias is stable across both backends' flights |
| `yolo_failure_trial2.txt`, `yolo_failure_trial3.txt` | replay of the two failing YOLO trials | both backends' projections swing ~1 m near the target while their difference stays ~0.6 m: the swing is the flat-nadir projection under pitch, not the detector |
| `v04_attitude.csv` | three trials per backend after the projection learned roll/pitch | baseline 0.86/1.00/0.66 m, YOLO 0.64/1.57/1.49 m — the 2.2 m mode is gone |
| `yolo_pixel_boxes.txt` | raw boxes from both models on the recorded 1280×960 frames | YOLO predicts 436–490 px boxes for a 54 px cube in every configuration: the model detects the region, not the object |
| `v05_yolo_real.csv` | six YOLO trials after fine-tuning on auto-labelled real frames | 1.265 / 0.822 / 0.386 / 1.066 / 1.321 / 1.235 m — 6/6 within tolerance, median 1.15 m |
| `v06_patrol.csv` | one two-waypoint patrol run | visited both waypoints, landed 0.438 m from the second target |
| `v07_moving_target.txt` | tracker replay over a moving-target flight | cube moved 3 m at 0.5 m/s; 88 detections, one id, with and without the motion model |
| `avoidance_run.csv` | `scripts/record_avoidance.py` during `make avoid` | 547 rows at 10 Hz: pose, nearest depth in the ROI, stop flag, contact count |
| `avoidance_landing.csv` | `offboard_node` recorder on the same run | one row: stopped 3.70 m from `target_0`, landed at the 40 s timeout — expected, not a tracking success |
| `avoidance_trace.csv` | `src/uav_tracking/control/trace.py` | `GOTO,avoidance stop` held 25.1 s, then `LAND,mission timeout` |
| `avoidance_summary.json` | `scripts/avoidance_report.py` | PASS: min clearance 2.204 m, settled drift 0.124 m, 0 contacts, 822 depth frames |
| `avoidance_run.png` | `scripts/avoidance_report.py` | flight path with the wall, and the depth gate against its thresholds |
| `avoidance_depth_frame.png` / `.npy` | `scripts/record_avoidance.py` | the frame that stopped the vehicle, nearest 2.45 m; PNG for humans, float32 array for re-analysis |
| `contact_sensor_check.txt` | manual positive test of the wall contact sensor | a 0.5 kg box on the wall produced 4632 messages |
| `detour_run.csv` | `scripts/record_avoidance.py` during `make detour` | 527 rows: approach, L-shaped detour, final approach |
| `detour_landing.csv` | `offboard_node` recorder on the same run | one row: 0.63 m from `target_0`, `success=True`, `patrol complete` |
| `detour_trace.csv` | `src/uav_tracking/control/trace.py` | lateral and forward leg timestamps, gate release, hover, land |
| `detour_summary.json` | `scripts/avoidance_report.py --mode detour` | PASS: 3.394 m lateral, gate released, 0.527 m target error, 0.999 m clearance, 0 contacts |
| `detour_run.png` | the same report | L-shaped path around the wall and the depth gate |
| `detour_depth_frame.png` / `.npy` | `scripts/record_avoidance.py` | the wall frame that started the detour |
| `autod_run.csv` | `scripts/record_avoidance.py` during `make auto-detour` | 518 rows: pose, nearest depth, stop, the chosen clear side, contacts |
| `autod_landing.csv` | `offboard_node` recorder on the same run | one row: 0.591 m from `target_0`, `success=True`, `patrol complete` |
| `autod_trace.csv` | `src/uav_tracking/control/trace.py` | detour to the **left**, lateral/forward legs, gate release, hover, land |
| `autod_summary.json` | `scripts/avoidance_report.py --mode detour --expect-side left` | PASS: side left, 4.181 m lateral, 0.714 m target error, 0.688 m clearance, 0 contacts |
| `autod_run.png` | the same report | the wall, the side block, and the left-hand L |
| `blocked_run.csv` | `scripts/record_avoidance.py` during `make blocked` | 547 rows: pose, nearest depth, stop, `blocked` side label, 0 contacts |
| `blocked_landing.csv` | `offboard_node` recorder on the same run | one row: 4.032 m from `target_0`, `success=False`, `mission timeout` (expected) |
| `blocked_trace.csv` | `src/uav_tracking/control/trace.py` | `avoidance stop` → `blocked: holding` → `LAND` |
| `blocked_summary.json` | `scripts/avoidance_report.py --mode hold` | PASS: 35.0 s hold, 0.129 m settled drift, 2.68 m clearance, 0 contacts |
| `blocked_run.png` | the same report | the path stops at the barrier and stays there |
| `backoff_run.csv` | `scripts/record_avoidance.py` during `make backoff` | 587 rows: stop, back-off, left side, depth, 0 contacts |
| `backoff_landing.csv` | `offboard_node` recorder on the same run | one row: 0.684 m from `target_0`, `success=True`, `patrol complete` |
| `backoff_trace.csv` | `src/uav_tracking/control/trace.py` | `blocked: backing off` → `backoff done: lateral leg` → forward → land |
| `backoff_summary.json` | `scripts/avoidance_report.py --mode detour` | PASS: 1.63 m back-off, 3.254 m lateral, 0.831 m target error, 0.595 m clearance, 0 contacts |
| `backoff_run.png` | the same report | the barrier and the back-off-then-L path |
| `sweep_run.csv` | `scripts/record_avoidance.py` during `make backoff-sweep` | 807 rows: four back-off steps, left at the last, 0 contacts |
| `sweep_landing.csv` | `offboard_node` recorder on the same run | one row: 0.808 m from `target_0`, `success=True`, `patrol complete` |
| `sweep_trace.csv` | `src/uav_tracking/control/trace.py` | `backing off` → `(2/4)`…`(4/4)` → lateral → forward → land |
| `sweep_summary.json` | `scripts/avoidance_report.py --mode detour` | PASS: 0.62 m back-off travel, 3.772 m lateral, 0.852 m target error, 0.639 m clearance, 0 contacts |
| `sweep_run.png` | the same report | the four-step back-off cluster and the L |
| `adaptive_run.csv` | `scripts/record_avoidance.py` during `make adaptive` | 667 rows: `left_m`/`right_m`, computed step, left after it, 0 contacts |
| `adaptive_landing.csv` | `offboard_node` recorder on the same run | one row: 0.572 m from `target_0`, `success=True`, `patrol complete` |
| `adaptive_trace.csv` | `src/uav_tracking/control/trace.py` | `backing off 2.00 m` → lateral → forward → land |
| `adaptive_summary.json` | `scripts/avoidance_report.py --mode detour` | PASS: 2.02 m back-off, 3.566 m lateral, 0.618 m target error, 0.47 m clearance, 0 contacts |
| `adaptive_run.png` | the same report | the measured 2 m back-off and the L |
| `search_run.csv` | `scripts/record_avoidance.py` during `make search` | 785 rows: every stopped row blocked (L2.75/R1.91), side-search left, 0 contacts |
| `search_landing.csv` | `offboard_node` recorder on the same run | one row: 0.663 m from `target_0`, `success=True`, `patrol complete` |
| `search_trace.csv` | `src/uav_tracking/control/trace.py` | `side-search left` → `search found a side` → lateral → forward → land |
| `search_summary.json` | `scripts/avoidance_report.py --mode detour` | PASS: 1.93 m search, 3.36 m lateral, 0.669 m target error, 0.869 m clearance, 0 contacts |
| `search_run.png` | the same report | the side-step-then-L path |
| `search_depth_frame.png` / `.npy` | the recorder's blocked frame with the L/R strip overlay | L 3.96 / R 3.97, both inside the floor |
| `global_run.csv` | `scripts/record_avoidance.py` during `make global` | 426 rows: planned path around the wall, 0 contacts |
| `global_landing.csv` | `offboard_node` recorder on the same run | one row: 0.216 m from the target, `success=True`, `patrol complete` |
| `global_trace.csv` | `src/uav_tracking/control/trace.py` | takeoff, GOTO, hover, land |
| `global_summary.json` | `scripts/avoidance_report.py --mode global` | PASS: 0.24 m target error, 0.554 m clearance, 0 contacts |
| `global_run.png` | the same report | the path around the wall's north end |
| `global_multi_run.csv` | `scripts/record_avoidance.py` during `make global-multi` | 463 rows: three-box path, 0 contacts |
| `global_multi_landing.csv` | `offboard_node` recorder on the same run | one row: 1.001 m from the target, `success=True`, `patrol complete` |
| `global_multi_trace.csv` | `src/uav_tracking/control/trace.py` | takeoff, GOTO, hover, land |
| `global_multi_summary.json` | `scripts/avoidance_report.py --mode global` | PASS: 0.616 m target error, 0.959 m clearance, 0 contacts |
| `global_multi_run.png` | the same report | the path around the wall and both pillars |
| `global_perception_run.csv` | `scripts/record_avoidance.py` during `make global-perception` | 465 rows: empty initial plan, stop, one measured box, replan |
| `global_perception_landing.csv` | `offboard_node` recorder on the same run | one row: 1.13 m from the target, `success=True`, `patrol complete` |
| `global_perception_trace.csv` | `src/uav_tracking/control/trace.py` | takeoff, GOTO, hover, land |
| `global_perception_summary.json` | `scripts/avoidance_report.py --mode global` | PASS: 0.777 m target error, 0.732 m clearance, 0 contacts |
| `global_perception_run.png` | the same report | the straight plan, stop, and replanned path |
| `global_rotated_run.csv` | `scripts/record_avoidance.py` during `make global-rotated` | 427 rows: two-waypoint path around the 30° wall, 0 contacts |
| `global_rotated_landing.csv` | `offboard_node` recorder on the same run | one row: 0.391 m from the target, `success=True`, `patrol complete` |
| `global_rotated_trace.csv` | `src/uav_tracking/control/trace.py` | takeoff, GOTO, hover, land |
| `global_rotated_summary.json` | `scripts/avoidance_report.py --mode global` | PASS: 0.218 m target error, 0.509 m exact rotated clearance, 0 contacts |
| `global_rotated_run.png` | the same report | the path around the rotated wall |
| `global_rotated_multi_run.csv` | `scripts/record_avoidance.py` during `make global-rotated-multi` | 437 rows: three-waypoint path around two rotated obstacles, 0 contacts |
| `global_rotated_multi_landing.csv` | `offboard_node` recorder on the same run | one row: 0.439 m from the target, `success=True`, `patrol complete` |
| `global_rotated_multi_trace.csv` | `src/uav_tracking/control/trace.py` | takeoff, GOTO, hover, land |
| `global_rotated_multi_summary.json` | `scripts/avoidance_report.py --mode global` | PASS: 0.586 m target error, 0.698 m exact wall clearance, 0 contacts |
| `global_rotated_multi_run.png` | the same report | the path around the wall and pillar |
| `gate_run.csv` | `scripts/record_avoidance.py` during `make gate` | 425 rows: one-waypoint doorway traversal, landing on target_0 |
| `gate_landing.csv` | `offboard_node` recorder on the same run | one row: 0.275 m from the target, `success=True`, `patrol complete` |
| `gate_trace.csv` | `src/uav_tracking/control/trace.py` | takeoff, GOTO, hover, land |
| `gate_summary.json` | `scripts/avoidance_report.py --mode global` | PASS: 0.49 m target error, 1.17 m clearance, 0 contacts |
| `gate_run.png` | the same report | the path through the 2.4 m gap |
| `gate_target1_run.csv` | `scripts/record_avoidance.py` during `make gate-target1` | 625 rows: offset gap, turn, target_1 |
| `gate_target1_landing.csv` | `offboard_node` recorder on the same run | one row: 0.425 m from target_1, target 1, `success=True` |
| `gate_target1_trace.csv` | `src/uav_tracking/control/trace.py` | takeoff, GOTO, hover, land |
| `gate_target1_summary.json` | `scripts/avoidance_report.py --mode global --target-id 1` | PASS: 0.277 m target error, 0.42 m clearance, 0 contacts |
| `gate_target1_run.png` | the same report | the three-waypoint offset-gap route |
| `gate_double_run.csv` | `scripts/record_avoidance.py` during `make gate-double` | 746 rows: north gate, south gate, target_1 |
| `gate_double_landing.csv` | `offboard_node` recorder on the same run | one row: 0.404 m from target_1, target 1, `success=True` |
| `gate_double_trace.csv` | `src/uav_tracking/control/trace.py` | takeoff, GOTO, hover, land |
| `gate_double_summary.json` | `scripts/avoidance_report.py --mode global --target-id 1` | PASS: 0.082 m target error, 0.45 m clearance, 0 contacts |
| `gate_double_run.png` | the same report | the five-waypoint zigzag through two gates |
| `repeat_report.md` / `.json` | `scripts/repeat_report.py` over `results/repeat/` | gate-safe **20/20**, double-safe **20/20**, gate old 17/20, gate-double old 15/20, target1 20/20 landings / 19/20 final |
| `repeat/<scenario>_<n>_summary.json` | per-run `avoidance_report.py` output | 133 repeat summaries (20 + 20 + 20 + 20 + 3 + 10 + 20 + 20) |
| `repeat/<scenario>_<n>_landing.csv` | per-run `offboard_node` landing row | landing error and success per repeat |
| `map_gate.md` / `.json` | `scripts/check_depth_map.py` on `search_depth_frame.npy` + row 215 | PASS: 1080 points, 83.4% within 1 m, median 0.339 m |
| `detections.geojson` | `scripts/export_geojson.py` on the moving-target capture | 88 Point features with track id, timestamp and ENU offsets — the GIS-ready deliverable an inspection operator asks for |
| `qa_moving.json` | `scripts/qa_report.py` on the moving-target capture | the asset was 3 m from its declared position, and the report says so (association 7.1 m, FAIL) |
| `qa_north.json` | the same report with the asset position passed in | median association drops from 4.95 m to 0.62 m when the asset is where it actually was |
| `latency.csv` | `scripts/benchmark.py` (via `make bench`) | aggregate error/latency statistics for the README table |
| `projection_error.csv` | W2 static test | pixel-to-ground error against the three known target coordinates |
| `detection_bench.csv` | `scripts/verify_model.py` + manual runs | detection rate and latency per backend |

An empty file with only a header row is the correct state before the MVP is
finished. Do not paste placeholder numbers into these files.

Why `latency_raw.csv` exists: a percentile of percentiles is not the percentile.
With raw samples the whole run has a real p50 and p95; without them
`benchmark.py` says so in the table instead of producing a number that looks
precise and is not.
