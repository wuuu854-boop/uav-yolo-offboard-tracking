# Changelog

Notable changes to this project. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), versions follow
[Semantic Versioning](https://semver.org/). Every measurement quoted here is in
`results/` and was produced by a script in this repository.

## [Unreleased]

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
