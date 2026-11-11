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
