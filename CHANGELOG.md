# Changelog

Notable changes to this project. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), versions follow
[Semantic Versioning](https://semver.org/). Every measurement quoted here is in
`results/` and was produced by a script in this repository.

## [Unreleased]

## [0.20.0] - 2026-11-11

### Added

- **Rotated-box planning.** `Box` gains `angle_rad`; `contains()` rotates the
  query point into the box frame, so grid A*, inflation and line-of-sight work
  for a wall rotated about z. The report gains `point_to_box_distance()` and
  measures the exact rotated shape instead of an axis-aligned bound.
- `sim/worlds/target_tracking_rotated.sdf` (30° wall),
  `config/mission_global_rotated.yaml`, `config/rotated_obstacle.yaml`, and
  `make global-rotated`.
- One run: **2 waypoints around 1 rotated obstacle**, final target error
  **0.218 m**, landing **0.391 m** `success=True`, exact clearance **0.509 m**,
  **0** contacts, 580 depth frames, 42.6 s.

### Why

The global planner could only represent axis-aligned boxes, so a wall at an
angle had to be approximated. Rotating the point into the box frame is a small,
testable change that makes the planner exact for this case while keeping the rest
of the stack unchanged.

## [0.19.0] - 2026-11-11

### Added

- **One-shot perception-to-planning.** `global_perception` starts the map empty;
  when the depth gate stops in cruise, the offboard turns `nearest_m` plus the
  current pose and yaw into one conservative axis-aligned box
  (`global_perception_width_m`, `global_perception_thickness_m`), replans, and
  ignores the reactive stop for the rest of the mission.
- `config/mission_global_perception.yaml` (`make global-perception`).
- One run: initial plan 1 waypoint / 0 obstacles; stop at 20.2 s at 2.46 m; box at
  `(6.57, 0.17)`, 0.40 × 4.00 m; replan 3 waypoints around 1 obstacle; target
  **0.777 m**, landing 1.13 m `success=True`, minimum clearance **0.732 m**,
  **0** contacts, 698 depth frames, 46.4 s.

### Why

The known-map demos had the boxes by hand. This is the smallest honest
perception-to-planning step: one measured range, one box, one replan. It proves
the handoff without claiming an occupancy map or replanning after the handoff;
the next step is accumulating depth points instead of one range.

## [0.18.0] - 2026-11-11

### Added

- **Multi-obstacle known-map planning.** `config/mission_global_multi.yaml`
  (`make global-multi`) gives the A* the wall and both search pillars as three
  boxes; 0.9 m inflation closes the gap between the pillars, so the route goes
  around the outside.
- One run: **3 waypoints around 3 obstacles**, final target error **0.616 m**,
  landing 1.001 m `success=True`, minimum wall clearance **0.959 m**, **0**
  contacts, 686 depth frames, 46.2 s (463 rows).

### Why

The single-wall demo could be solved with one detour. Three boxes force the
planner to route around a union, which is the next thing the local stack cannot
do. The map is still hand-entered and the reactive stop is still disabled; this
is a planner test, not perception.

## [0.17.0] - 2026-11-11

### Added

- **Known-map global planning.** `src/uav_tracking/planning/global_planner.py`
  adds a 2-D grid A* around inflated axis-aligned boxes with line-of-sight
  simplification. The offboard plans once per goal from `global_obstacles`,
  follows the waypoints, and takes the final target.
- `config/mission_global.yaml` (`make global`) uses the wall map at local
  `(6.7, 0)`, 0.5 m inflation, a pre-goal at `(8.2, 0)` and
  `avoidance_stop_enabled: false`.
- One run: **3 planned waypoints**, final target error **0.24 m**, landing
  0.216 m `success=True`, minimum clearance **0.554 m**, **0** contacts, 640
  depth frames, 42.5 s.

### Why

The local stack had no path before the obstacle. A known map is the smallest
honest way to add one: it proves the plan-and-follow interface without claiming
perception. The stand-off and the disabled reactive stop are deliberate; target_0
is too close to the wall for a clean landing, and this run tests the planner, not
the local avoidance.

## [0.16.2] - 2026-11-11

### Fixed

- The two search pillars are now at `(-1.5, -0.8)` and `(-1.5, +1.8)`, placing
  both inside the side strips at the stop. Every stopped row is `blocked`
  (left 2.75 → 2.70 m, right 1.91 → 1.88 m); the earlier attempts had one strip
  open at the stop.
- One run: stop at 17.9 s, `side-search left` → `search found a side`, 1.93 m
  search step, 3.36 m lateral leg, final target error **0.669 m**, landing
  0.663 m `success=True`, minimum clearance **0.869 m**, **0** contacts, 1177
  depth frames, 78.4 s.
- The v0.16.1 caveat is resolved: the both-strips-blocked state is present on
  the stopped rows, not only during the search.

## [0.16.1] - 2026-11-11

### Fixed

- The right search pillar moved from `(-1.5, -1.5)` to `(-1.5, -1.0)`, putting it
  inside the left side strip at the stop. The blocked frame now reads
  **L 2.61 m / R 1.98 m**, both inside the 4.0 m floor (the earlier frame read
  L 9.47 / R 2.05).
- The recorder saves a frame whenever the depth label becomes `blocked` and
  draws the two side strips on it, so the search trigger is visible.

### Evidence

One run: stop at 17.6 s, `side-search left` → `search found a side`, 1.95 m
search step, 3.31 m lateral leg, final target error **0.381 m**, landing 0.506 m
`success=True`, minimum clearance **0.836 m**, **0** contacts, 1181 depth frames,
78.6 s.

### Caveat

The both-blocked frame is captured **during** the search; the first stopped rows
still read left 7.66 / right 2.47, so the search starts from the latched
`blocked` label rather than a stable both-blocked stop. A stop that is blocked on
both sides for several frames remains unverified.

## [0.16.0] - 2026-11-11

### Added

- **A blocked stop can side-step and re-read.** A `SEARCH` phase shifts
  `search_step_m` (2 m) toward the freer side, re-reads, and starts the L from
  the searched position if a side is free; bounded by `search_max_attempts`.
- `target_tracking_search.sdf` adds two 6 m side pillars with contact sensors;
  `config/mission_search.yaml` (`make search`) uses a 4.0 m floor and no back-off.
- One run: stop at 17.9 s, `side-search right` → `search found a side`, 2.1 m
  search step, 3.0 m lateral leg, final target error **0.537 m**, landing
  0.652 m `success=True`, minimum clearance **1.221 m**, **0** contacts, 1072
  depth frames, 74.4 s.
- The recorder can save an indexed frame per stop with the side-strip overlay;
  the retained `search_depth_frame.png` documents the caveat.

### Why

The back-off moves along the approach only. A lateral shift is the other local
degree of freedom, and it is enough when the obstacle ends just beyond the side
strip. The retained run proves the SEARCH transition and recovery; it does **not**
prove a stop where both strips are simultaneously below the floor. At the stop
frame the left strip reads 9.47 m and the right 2.05 m; the `blocked` label that
started the search arrived 0.7 s after the stop. That caveat is in the README and
`docs/search.md` rather than hidden.

## [0.15.0] - 2026-11-11

### Added

- **The back-off distance is measured, not configured.** The depth node publishes
  `/avoidance/side_clearance` (`std_msgs/Float32MultiArray`, `[left_m, right_m]`,
  NaN for no return). The controller computes
  `clamp(backoff_target_m - max(left, right), min_step, max_step)` and passes it
  to `DetourPlanner.set_backoff_step()`; the trace records the chosen step.
- `config/mission_adaptive.yaml` (`make adaptive`) uses the phase-4 barrier world,
  a 6.0 m floor and a 6.0 m target, with `detour_backoff_m: 0.0`: without the
  measured step it would hold.
- The recorder logs `left_m`, `right_m` per row.
- One run: stop at 18.9 s, left 3.20 m / right 2.32 m, computed step **2.00 m**,
  2.02 m of travel, left after it, 3.566 m lateral leg, 7.5 m forward, final
  target error **0.618 m**, landing 0.572 m `success=True`, minimum clearance
  **0.47 m**, **0** contacts, 1001 depth frames, 66.6 s.

### Why

A fixed step is still a guess about how far the obstacle reaches. The depth node
already measures both sides; using the better one to size the retreat is the
smallest closed-loop improvement. It is open-loop between steps (the 1:1 growth
assumption), and this flight needed only one step; the loop remains the fallback.

## [0.14.0] - 2026-11-11

### Added

- **The back-off can repeat until a side clears.** `backoff_max_attempts` keeps
  stepping `backoff_m` straight back, re-reading the strips at each new position;
  the trace says `still blocked: backing off again (n/max)`. If the budget runs
  out, the planner holds. `HOLDING` after a back-off no longer resumes just
  because the gate reads clear: backing away makes the gate clear while the
  obstacle is still in front.
- `config/mission_backoff_sweep.yaml` (`make backoff-sweep`) uses the phase-4
  barrier world, a 6.0 m side floor and 0.2 m steps with a 0.05 m reach tolerance.
- One run: stop at 17.9 s, four back-off attempts (`2/4`…`4/4`), 0.62 m of
  back-off travel, **left** at the last step, 3.772 m lateral leg, 6 m forward,
  final target error **0.852 m**, landing 0.808 m `success=True`, minimum
  clearance **0.639 m**, **0** contacts, 1212 depth frames, 80.6 s.

### Why

A single fixed back-off distance is still a guess. Repeating the same small step
and making a fresh decision at each position is the smallest loop that can find a
clear side at an unknown distance — and the budget stops it from reversing
forever. It is still a straight-back sweep, not a search in two dimensions.

## [0.13.0] - 2026-11-11

### Added

- **Both sides blocked can now recover by backing off.** `DetourConfig.backoff_m`
  adds a `BACKOFF` phase: the planner drives straight away from the approach by
  2 m, re-reads the left/right strips and starts the L from there if a side is
  free. If neither clears it enters `HOLDING` and does not repeat.
- `config/mission_backoff.yaml` (`make backoff`) uses the phase-4 barrier world
  with a 4.0 m side floor (`config/depth_backoff.yaml`). The forward leg is 7 m:
  5 m from the back-off point clipped the barrier's north edge on the return.
- One run: stop at 19.7 s, back-off 1.63 m measured (2.0 m waypoint, 0.5 m
  tolerance), **left** side at the back-off point, 3.254 m lateral leg, 7 m
  forward, final target error **0.831 m**, landing 0.684 m `success=True`,
  minimum clearance **0.595 m**, **0** contacts, 882 depth frames, 58.6 s.

### Why

Phase 4 held, which is safe but leaves the mission stuck. If the obstacle is
narrow enough that its edges leave the side strips once the vehicle is a little
further back, a single controlled back-off is enough to find a side. It is still
one fixed distance and one retry, not a search; if it fails it holds.

## [0.12.0] - 2026-11-11

### Added

- **Both sides blocked is now a refusal, not a guess.** `choose_side()` takes a
  `min_clearance_m` floor; when both strips are closer than it the depth node
  publishes `blocked`. The controller holds instead of starting a detour and
  latches `blocked` for the rest of the stop, so a one-frame `left`/`right`
  between two `blocked` frames cannot start an escape.
- `detour_start_delay_s` (used by the blocked mission, 1.0 s) holds through the
  settle transient before committing to a side.
- `sim/worlds/target_tracking_blocked.sdf` adds a 4 m barrier across the path
  with its own contact sensor; `config/depth_blocked.yaml` raises the side floor
  to 3.0 m; `make blocked` flies it with the hold report.
- One run: stop at 19.6 s, `blocked` latched 0.6 s later, **held 35.0 s**, settled
  drift **0.129 m**, minimum wall clearance **2.68 m**, **0** contacts on the
  wall and the barrier, 821 depth frames. The mission times out and lands
  4.032 m from target_0 (`success=False`) — the refusal is the expected result.

### Why

Phase 3 chose the freer side, but if both sides are unsafe it still guessed the
less-bad one. A hold is the honest answer until there is a search behaviour. The
1 s delay and the latch exist because SITL showed the side label changing from
`left` to `blocked` 0.6 s after the stop as the vehicle pitched; committing on
the first frame chose the wrong answer.

## [0.11.0] - 2026-11-11

### Added

- **The detour side is measured, not fixed.** `side_clearance()` compares the
  nearest return in a left 20% and a right 20% strip outside the centre ROI;
  `choose_side()` applies a 0.5 m margin and the depth node publishes
  `/avoidance/clear_side`. The planner uses the value at the first stopped tick
  when `detour_auto_side: true`; `detour_side` stays the fallback. The whole-run
  majority is not the decision — after the detour the free side often flips.
- `sim/worlds/target_tracking_detour.sdf` adds a 6 m side pillar in the right
  strip (clear of the wall, so static bodies do not overlap), with its own
  contact sensor. `make auto-detour` flies it; the report checks the side label
  and that the lateral direction matches it, in addition to the phase-2 checks.
- One run: stop at 18.5 s, side at the stop **left**, 4.181 m lateral leg, gate
  released at 22.7 s, final target error **0.714 m**, landing 0.591 m
  `success=True`, minimum clearance **0.688 m**, **0** contacts on the wall and
  the pillar, 790 depth frames.

### Why

A fixed side is a guess. The smallest honest improvement is to look at the two
places a forward depth camera can see beside the nose and pick the freer one. It
still does not search when both are blocked, and it still has no map; the docs
say so.

## [0.10.0] - 2026-11-11

### Added

- **An L-shaped detour around the wall.** Phase 2 of avoidance: when the depth
  gate stops the approach, the vehicle flies 3.5 m to a fixed side, 5 m forward
  past the wall's face, and resumes the waypoint once the gate has been clear for
  a second. The forward leg is not decoration: the target is 0.8 m behind the
  wall, and a diagonal back from the lateral point cuts the corner. The pure
  planner is `src/uav_tracking/control/detour.py`, with 20 unit tests; it extends
  the forward leg once and holds if the gate never clears.
- `config/mission_detour.yaml`, `make detour`, and `avoidance_report.py --mode
  detour`: the same orchestration produces either the phase-1 hold or the phase-2
  detour, with separate `detour_*` evidence files. The report checks eight
  things: stop, detour started, lateral distance, gate release, target reached,
  clearance, zero contacts and depth frames.
- The accepted run reaches target_0 with a **0.527 m** final error and lands
  `success=True` at 0.63 m, with **0.999 m** minimum wall clearance and **0**
  contacts in 791 depth frames. The first attempt used a 3 m forward leg, grazed
  the wall's corner and registered contacts; the forward leg is 5 m.

### Why

Stopping is safe but not useful: the mission has not reached its target. The
smallest step that is still honest is a fixed-side L around one known obstacle,
not a planner. The report refuses a run that touches the wall, misses the target
or never releases the gate, so the number in the README cannot be a lucky one.

## [0.9.0] - 2026-11-10

### Added

- **A forward depth camera and a reactive stop.** `sim/models/x500_mono_cam_down`
  now carries a 640×480 `R_FLOAT32` depth camera, mounted 0.20 m forward and
  pitched 30° down, next to the nadir tracking camera. `scripts/gz_depth_bridge.py`
  republishes `/depth_camera` as ROS `32FC1`; `src/uav_tracking/perception/depth_node.py`
  turns it into `/avoidance/stop`. The thresholds, ROI and fail-safe are in
  `config/depth.yaml`; the statistics, hysteresis and watchdog are pure Python in
  `src/uav_tracking/perception/depth.py`, with unit and ROS tests.
- **The controller holds instead of advancing.** `offboard_node` latches the
  vehicle position on the first stopped tick and holds the horizontal setpoint,
  but only in `GOTO` — takeoff, hover and landing keep their commands. A 1.0 s
  watchdog treats a `/avoidance/stop` stream that stops arriving as a stop; before
  the first message it is inactive, so missions without the depth node are
  unchanged. `MissionConfig.heading_yaw_deg` makes the camera face the approach
  (90° east here); the previous fixed 0° was only harmless because the tracking
  camera is nadir.
- **A scenario and a harness.** `sim/worlds/target_tracking_obstacle.sdf` adds a
  6 m wall 0.8 m in front of `target_0`, with a contact sensor;
  `config/mission_avoidance.yaml` is a one-waypoint patrol at 0.6 m/s. `make avoid`
  runs the headless flight and `scripts/record_avoidance.py` writes the pose, depth
  distance, stop flag and contacts. `scripts/avoidance_report.py` checks the run
  and writes `results/avoidance_summary.json`; it fails on no stop, insufficient
  clearance, drift or a contact. One run: stop at 19.8 s, held 25.1 s, settled
  drift 0.124 m, min clearance 2.204 m, 0 contacts, 822 depth frames.
- `docs/avoidance.md`, four troubleshooting entries, and a positive test of the
  wall contact sensor (a dropped box produced 4632 messages).

### Why

Stopping is the safety floor before path planning, and it is the last item on the
market checklist that can be verified in simulation without new hardware. The
design is deliberately the smallest one that can be measured: one camera, one
boolean, one hold. Going around the wall is phase 2; claiming it here would make
the verified table lie.

## [0.8.0] - 2026-11-10

## [0.7.0] - 2026-11-10

### Added

- **One pipeline over a capture.** `scripts/pipeline.py` (and `make pipeline`)
  runs the QA report, the shot sheet and the GeoJSON export in one pass and
  returns a single verdict: on the north-cube flight the shot standard passes,
  the data quality fails, and the summary says both. Output in
  `results/pipeline.json` and `results/pipeline.geojson`.
- **A shot sheet and a checker.** `config/shot_sheet.yaml` states what a usable
  capture of this structure type requires — mounting, ground sample distance,
  framing margin, minimum frames — and `scripts/shot_sheet_check.py` enforces it
  on a recording, naming the reason every rejected frame was rejected. On the
  north-cube flight: `nadir 26/3` frames, with 50 frames rejected for clipping.
  `control/shot_sheet.py` carries the arithmetic and ten tests.
- `control/qa.py:association_confidence()` and the report's confidence columns:
  a per-detection score that multiplies a Gaussian on the distance to the asset
  by how much support the track has. On the north-cube run with the true asset
  position the median is 0.71, and 34 of 76 detections sit below 0.5 — mostly the
  second, spurious track. It is a heuristic, and `docs/enterprise-needs.md` says
  so instead of implying a calibrated probability.

## [0.6.1] - 2026-11-10

### Added

- `scripts/qa_report.py` takes the asset position as an input (`--asset`) and
  reports detection coverage per flight phase; a moved target no longer reads as
  a broken projection, and takeoff frames no longer drag the average down. The
  metrics run end to end on a generated capture in `tests/test_qa_capture.py`,
  so a change in the detector or the projection fails a test.
- `make qa CAPTURE=<dir>` wraps the report.

## [0.6.0] - 2026-11-10

### Added

- `scripts/export_geojson.py`: a recorded flight becomes a GeoJSON feature
  layer — every detection projected with the recorded pose, with track id,
  timestamp, confidence and the local ENU offsets kept in the properties. Five
  tests pin the ENU-to-WGS84 conversion.
- `docs/enterprise-needs.md`: a drone service provider's five data-quality
  failures and four utilities describing their next requirement, read against
  this repository.
- `docs/model-card.md`: the governance document — intended use, data provenance,
  metrics, failure modes, and what it does not certify.
- `scripts/qa_report.py` and `control/qa.py`: a data-quality report over a
  recorded flight — detection coverage, longest gap, track continuity, and the
  distance from every detection to the asset it is associated with. Six tests on
  the arithmetic; the script exits non-zero when a run fails its thresholds.
  Sample reports are in `results/qa_static.json` and `results/qa_moving.json`.

## [0.5.0] - 2026-11-10

### Added

- **Motion model in the tracker.** `Track` carries a smoothed velocity and a
  capped constant-velocity prediction; association scores the last box and the
  predicted one, then falls back to nearest predicted centre within a gate, which
  is what lets a target bootstrap its velocity. Seven tests cover a fast target
  keeping its id, a missed frame, two crossing targets and the cap.
- **Moving-target recording.** `scripts/move_target.py` drives a target with
  `gz service set_pose`; a flight with the cube moving 3 m at 0.5 m/s recorded
  120 airborne frames and the tracker held one id across 88 detections
  (`results/v07_moving_target.txt`). At that speed plain IoU suffices; the
  motion model earns its place when the per-frame shift exceeds the overlap
  window, which the unit tests set up.
- **Multi-target patrol.** `patrol_waypoints` in `config/mission.yaml` turns the
  mission into an ordered route: fly each local ENU waypoint, hover, land after
  the last. One SITL run with two waypoints 20 m apart visited both and landed
  0.44 m from the second target (`results/v06_patrol.csv`).
- `control/patrol.py`: the queue as a pure module with six tests;
  `MissionConfig.patrol_waypoints` and `Observation.goal_pending` wire it in.
- `scripts/run_e2e.sh` accepts `MISSION_CONFIG`, so a patrol route can be a
  generated file instead of an edit to the shipped config.

### Fixed

- A patrol leg used to be overridden by the detection of the target it had just
  left, and the last waypoint sent the vehicle back toward the first target. The
  legs are now waypoint-driven and the mission ends with an explicit
  `request_land`.
- A waypoint goal has no image stamp; the latency measurement dereferences the
  detection message only when there is one.

## [0.4.0] - 2026-11-10

### Added

- **A YOLO backend that flies.** `scripts/label_real_frames.py` produces
  real-domain labels from geometry: the colour baseline finds the cube, its pixel
  is projected to the ground with the recorded pose, and the cube's known 0.3 m
  corners are projected back into the image with `ground_point_to_pixel` (the
  inverse of the projection, round-trip tested). Fine-tuning the synthetic
  checkpoint on 59 such frames took the predicted boxes from 436–490 px for a
  54 px cube to 32–45 px, with centres within 3 px of the baseline's.
- Six closed-loop trials with that model: 1.265, 0.822, 0.386, 1.066, 1.321 and
  1.235 m — **6/6 within the 1.5 m tolerance**, median 1.15 m
  (`results/v05_yolo_real.csv`). The README table has the YOLO row it had been
  carrying an empty slot for, with the narrow domain written next to it.

### Changed

- README results split into the baseline column and the YOLO column, both with
  their trial counts.

## [0.3.0] - 2026-11-10

### Added

- **Attitude-aware projection.** `projection.py` takes the vehicle's PX4 roll and
  pitch, builds the body-to-ENU rotation and intersects the pixel ray with the
  ground instead of assuming a level camera. Ten degrees of pitch at 5 m is
  0.88 m of ground error, which is what the failing YOLO trials were losing to.
  With both angles at zero the result is unchanged and every existing projection
  test passes untouched; eight new tests pin the attitude cases.
- `scripts/capture_pose_frames.py` records frames, odometry and attitude
  together, and `scripts/compare_backends.py` runs two detectors on the same
  recorded frames and projects both with the same pose. `run_trials.sh` starts
  the recorder when `CAPTURE_DIR` is set, one directory per trial.
- `scripts/make_synthetic_dataset.py` and `scripts/harvest_negatives.py`: a
  seeded nadir dataset and background frames harvested from the demo GIF.
- `docs/model.md`: what was trained, the decoder bug it exposed, the controlled
  comparison, and the replay that named the pitch problem.

### Changed

- Three trials per backend after the fix (`results/v04_attitude.csv`): baseline
  0.86/1.00/0.66 m, YOLO 0.64/1.57/1.49 m. The 2.2 m YOLO failure mode is gone.
  The detector's remaining 0.6 m bias kept it at the edge of tolerance at that
  point; 0.4.0 removes it.

### CI

- The image build retries `apt-get update`; a mirror mid-sync failed a release
  job after twenty minutes of fetching.

## [0.2.1] - 2026-11-10

### Fixed

- `perception/yolo.py` decoded a channel-first export `(1, 4 + C, N)` as if the
  five channels were five boxes: every detection carried coordinates from the
  wrong axis and a "confidence" above 1. The YOLO backend could never have flown
  before this fix. Two tests cover channel-first tensors, including a single-box
  export.
- `scripts/detect_images.py` no longer imports the ROS node module (it needed
  `rclpy` to evaluate images), accepts `--imgsz`, and allows `--out` outside the
  repository.
- `scripts/run_e2e.sh` preflight had a broken `else` that made every run report
  "MISS rclpy" and exit before starting the nodes.

## [0.2.0] - 2026-11-10

### Added

- `control/geofence.py`: every horizontal setpoint is clamped to
  `geofence_radius_m` (30 m by default) around the spawn point, so a bad estimate
  becomes a boundary command instead of a flight away.
- `max_mission_s`: a mission that never converges lands instead of hovering until
  the battery decides (disabled by default).
- `confirm_frames`: arrival, reacquisition and the end of a loss hover need
  several consecutive frames with a target (3 in the shipped config). A single
  bad projection during a search produced a 5.6 m miss without it.
- `scripts/run_trials.sh` / `make trials N=3`: the trial procedure as a script,
  restarting the simulation between runs.

### Changed

- Verified with three runs on the same targets: 0.94 m, 1.01 m and 1.49 m, all
  inside the 1.5 m tolerance (`results/v02_validation.csv`). The nine-row table
  in the README remains the v0.1.0 measurement; this release changes safety
  behaviour only.

## [0.1.0] - 2026-11-09

### Added

- The closed loop: a detector node (colour baseline, optional ONNX backend), an
  offboard node that projects detections to the ground, filters, tracks and flies
  to them, and the shared `uav_interfaces` messages between them.
- The pieces that made it measurable: mission recorder, state trace, landing
  error and latency CSVs, and the scripts that turn them into the README table.
- The reproducibility layer: `patches/px4-v1.15-gz-magnetometer.patch` with
  `make patch-px4`, `requirements.lock.txt`, `SECURITY.md`, Dependabot, a
  `docker` CI job that builds and tests the image, and checkers for links, README
  claims and dependency drift.
- `main` protection: `test` and `docker` must pass, history is linear, force
  pushes and deletions are disabled.

### Measured

- Nine SITL trials, three per target, all successful: **0.73 m median landing
  error**, p95 1.14 m; end-to-end latency p50 143 ms / p95 272 ms over 18,222
  samples (`results/landing_error.csv`, `results/latency_raw.csv`).
