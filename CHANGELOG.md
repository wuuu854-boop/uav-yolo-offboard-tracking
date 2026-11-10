# Changelog

Notable changes to this project. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), versions follow
[Semantic Versioning](https://semver.org/). Every measurement quoted here is in
`results/` and was produced by a script in this repository.

## [Unreleased]

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
  0.86/1.00/0.66 m, YOLO 0.64/1.57/1.49 m. The 2.2 m YOLO failure mode is gone;
  the detector's remaining 0.6 m bias keeps it at the edge of tolerance, so the
  measured loop stays on the colour baseline.

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

- Verified with three runs on the same targets: 0.92 m, 1.01 m and 1.49 m, all
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
