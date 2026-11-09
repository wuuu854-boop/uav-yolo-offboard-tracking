# Changelog

Notable changes to this project. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), versions follow
[Semantic Versioning](https://semver.org/).

## [Unreleased]

### Added

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
- The world file follows PX4 conventions: no `<plugin>` elements (PX4 loads the
  systems from `server.config`), physics at 0.004 s / 250 Hz.
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
