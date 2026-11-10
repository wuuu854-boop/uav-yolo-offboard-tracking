# Changelog

Notable changes to this project. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), versions follow
[Semantic Versioning](https://semver.org/).

## [0.2.0] - 2026-11-10

### Added

- `control/geofence.py`: every horizontal setpoint is clamped to
  `geofence_radius_m` (30 m by default) around the spawn point, so a bad
  estimate becomes a boundary command instead of a flight away.
- `max_mission_s`: a mission that never converges lands instead of hovering until
  the battery decides (disabled by default).
- `confirm_frames`: arrival, reacquisition and the end of a loss hover need
  several consecutive frames with a target (3 in the shipped config). A single
  bad projection during a search produced a 5.6 m miss without it.
- `scripts/run_trials.sh` / `make trials N=3`: the nine-trial procedure as a
  script — restart the simulation between runs, one row per trial.

## [Unreleased]

### Added

- `patches/px4-v1.15-gz-magnetometer.patch` and `make patch-px4`: the PX4 change
  the SITL trials need (a magnetometer callback for the gz bridge, plus the
  airframe that spawns the nadir camera) is now a reviewed patch with a version
  guard and an already-applied check, instead of edits that only existed on one
  machine.
- `requirements.lock.txt`: the exact versions the nine trials ran on.
- `SECURITY.md`: scope and a private reporting path.
- The tests badge in the README is checked by `check_doc_claims.py`, so it cannot
  go stale the way the prose count used to.
- `scripts/check_dependencies.py` (in CI and `make check`): `pyproject.toml` and
  `requirements.txt` are two lists of the same thing, and they drifted — the
  packaging list put the GUI OpenCV back into the container and broke it on
  libGL. They now have to agree.
- The `docker` CI job builds the image and runs the unit tests inside it, so the
  container is verified on every push instead of advertised.
- `.github/dependabot.yml` watches pip and the actions, with ignores for the
  numpy and OpenCV ceilings that exist for the cv_bridge ABI.

### Changed

- `main` is protected: `test` and `docker` must pass, the branch must be up to
  date, history is linear (squash merges), and force pushes and deletions are
  disabled. See `CONTRIBUTING.md` for how Dependabot PRs are triaged.
- Runtime dependency is `opencv-python-headless`: the GUI wheel links `libGL`,
  which CI containers and headless SITL machines do not ship.
- The Docker image installs only `python3-pip` (it had been pulling git, cmake
  and a compiler for a pure-Python test run) and asserts at build time that it
  can collect the tests.

### Fixed

- The two nodes and the camera bridge now exit cleanly on SIGINT/SIGTERM instead
  of printing an `rclpy` traceback every time a trial ends.
- `tests/ros/*` skip when `uav_interfaces` has not been generated, instead of
  aborting collection and failing every job that runs pytest without colcon.
- `run_e2e.sh --dry-run` (also `make preflight`): checks configuration files,
  `rclpy`, the package import and writable directories without a simulator, and
  the ROS workflow runs it, so the trial script itself is exercised in CI.
- `control/trace.py`: one row per mission state change in
  `results/state_trace.csv`, so a failed trial can be explained rather than
  guessed at (state, reason, position, altitude).
- `control/selection.py`: a `tracked` strategy that follows the `track_id` the
  mission locked onto, falling back to `nearest_centre` when that object is gone.
  The lock is released when the target is lost, so a new sighting can be picked.
- `perception/tracking.py`: greedy IoU tracker with a stable `track_id` on every
  detection, so a momentary miss does not look like a new target and distinct
  objects can be counted.
- `control/search.py`: a lawnmower pattern and a walker for it. When the target
  stays lost for `search_after_s` the mission sweeps the area around the last
  sighting instead of hovering forever, and resumes the approach on reacquisition.
- `scripts/doctor.py`: six environment checks with a fix hint per failure, and a
  `--static` mode that runs in CI without ROS.
- `scripts/benchmark.py`: the only code allowed to turn
  `results/landing_error.csv` into a claim.
- `scripts/offline_demo.py` / `src/uav_tracking/control/offline_sim.py`: runs the real
  projection and state machine without ROS or Gazebo, on any OS.
- `scripts/detect_images.py`: batch evaluation of a folder of images with no ROS
  and, for the baseline backend, no model.
- `scripts/check_links.py`, `scripts/check_doc_claims.py`: fail CI when a
  relative link moves or the test count quoted in the README drifts.
- `src/uav_tracking/control/selection.py`: `nearest_centre` (default), `nearest_previous` and
  `highest_confidence` target selection, configurable per mission.
- `scripts/publish.ps1`: creates the repository and pushes in one command.
- `ROS build` workflow: builds `px4_msgs` + `uav_interfaces` in `ros:humble`,
  drives the detector with synthetic images and the controller with fake PX4
  traffic, and installs the package with `pip -e` to check the metadata.
- `skills/`: seven debugging procedures distilled from this build.

### Changed

- The Python code moved from a package literally named `src` to `uav_tracking`
  under `src/`, so `pip install -e .` works and imports read like a library
  rather than a folder.
- The world declares Physics, UserCommands, SceneBroadcaster, Sensors and Imu
  itself: PX4 v1.15's launcher passes no config, and without the Sensors system
  the camera never publishes. Physics stays at 0.004 s / 250 Hz.
- Targets are 20 m apart with a documented spawn pose per trial, because the
  real camera footprint would otherwise show all three at once.

### Fixed

- The camera model is `x500_mono_cam_down`, not the forward-looking
  `x500_mono_cam`; intrinsics are 1280×960 at HFOV 1.74 rad (`fx = 538.4`).
- PX4 heading (clockwise from north) is converted to ROS yaw before projection;
  the missing conversion was a constant 90° error.
- `/fmu/out/*` subscriptions use `SensorDataQoS`; a reliable subscription never
  matched PX4's best-effort publishers.
- `limited_step()` enforces a true speed limit; per-axis clipping allowed a
  diagonal setpoint to move at √2 times the configured speed.
- An in-flight disarm (failsafe, battery, manual kill) now returns the mission to
  IDLE instead of continuing to command takeoff or goto to a falling vehicle.
- An unknown `target_selection` is rejected when the node is constructed, not on
  the first frame with two detections.
- YOLO output orientation is decided from the class count instead of
  `rows < columns`, which broke single-box exports.

## [0.1.0] — unreleased

First release is blocked on one thing: the nine SITL trials that fill
`results/landing_error.csv`. Everything else in the README is either tested or
explicitly marked as not yet run.
