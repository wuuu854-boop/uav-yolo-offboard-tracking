# Changelog

Notable changes to this project. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), versions follow
[Semantic Versioning](https://semver.org/). Every measurement quoted here is in
`results/` and was produced by a script in this repository.

## [Unreleased]

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
