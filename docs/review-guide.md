# Reviewing this code

A map for someone reading the source rather than the README: what each module
guarantees, and the test that would fail if the guarantee broke. Every claim here
is enforced in `make check`; nothing is aspirational.

## Where to start

1. `docs/architecture.md` — data flow and the four frame conventions. Most bugs
   in this stack are a frame convention, so read that before the code.
2. `src/uav_tracking/perception/projection.py` — 130 lines, pure functions, the
   derivation is in the function that uses it.
3. `src/uav_tracking/control/state_machine.py` — the mission logic, no ROS
   imports, so it can be read in one sitting.

## Guarantees and their proofs

| Module | Guarantee | Proof |
|---|---|---|
| `perception/projection.py` | pixel → ground ENU under a nadir camera, with or without attitude; the mount offset is applied | `tests/test_projection.py` (18) + `tests/test_projection_attitude.py` (7) |
| `perception/yolo.py` | letterbox → blob → decode round-trips; class filter precedes NMS | `tests/test_yolo.py`, `tests/test_detector.py` (19) |
| `perception/tracking.py` | the same object keeps its id across frames; a gap shorter than `max_age_s` does not create a new one | `tests/test_tracking.py` (12) |
| `control/frames.py` | NED → ENU including the 90° heading convention, and roll/pitch out of a quaternion | `tests/test_frames.py` (9) + `tests/test_frames_attitude.py` (6) |
| `control/state_machine.py` | a lost target hovers then searches, never lands; an in-flight disarm stops the mission | `tests/test_state_machine.py` (12) |
| `control/search.py` | lanes stay inside the area, never exceed the spacing, and the walker advances only on arrival | `tests/test_search.py` (9) |
| `control/limiter.py` | the speed bound holds in every direction, including diagonal | `tests/test_limiter.py` (7) |
| `control/selection.py` | the default ignores a more confident edge detection; `tracked` holds one id and falls back when it disappears | `tests/test_selection.py` (12) |
| `control/arming.py` | bounded retries, then one actionable warning | `tests/test_arming.py` (6) |
| `control/recorder.py` | append-only; raw latency samples are kept for pooling | `tests/test_recorder.py` (6) |
| `control/trace.py` | one row per state change, and a second run appends | `tests/test_trace.py` (6) |
| `control/geofence.py` | a target outside the circle is pulled onto it, direction preserved | `tests/test_geofence.py` (7) |
| `control/state_machine.py` (safety) | a mission that never converges lands; a target is believed only after `confirm_frames` frames | `tests/test_state_machine.py` (18) |
| `control/offline_sim.py` | the whole loop terminates and converges with a fixed seed | `tests/test_offline_sim.py` (8) |

## What is not unit-tested, and where it is covered instead

| Item | Why | Covered by |
|---|---|---|
| `perception/detector_node.py` | needs `rclpy` and a middleware to import | the `ROS build` workflow: publishes a synthetic `Image`, asserts the `TargetArray` a normal subscriber receives |
| `control/offboard_node.py` | needs `rclpy` and PX4 message types | the `ROS build` workflow: feeds `VehicleOdometry`/`VehicleStatus` at 50 Hz and asserts the mode request, the arming request, the setpoint stream, the NED sign, and the projected target for two known pixels |
| `scripts/run_e2e.sh` | needs a running simulator | manual, documented in `docs/experiment-protocol.md`; `--dry-run` runs in CI |
| Gazebo world | needs Gazebo to load it | `scripts/check_world_targets.py` (structure, physics, lights, ground truth) |
| `patches/px4-v1.15-gz-magnetometer.patch` | it is a change to another project | `scripts/apply_px4_patch.sh` guards the tag and detects an applied tree; the nine trials are the functional proof |
| `Dockerfile` | needs Docker | the `docker` job in `ci.yml` builds it and runs the unit tests inside |

## Reproducing the trials

1. `bash scripts/sitl_setup.sh` — installs and verifies the stack, checks before
   it installs anything.
2. `make patch-px4` — applies the one PX4 change the trials need, with a version
   guard.
3. `make install-world` — puts the world **and** the nadir camera model where
   PX4 v1.15's launcher actually reads them.
4. `make sim`, then `make e2e` per trial; `make bench` regenerates the table.

The exact dependency set is `requirements.lock.txt`; the environment the numbers
came from is `docs/setup-log.md`, including what broke on the way.

## Questions a reviewer should ask

- **Why two nodes?** Projection needs the pose; the pose lives with the
  controller. Keeping them together means the detector can be replayed from a
  bag with no PX4 running (`docs/architecture.md`).
- **Why is yaw commanded to zero?** A fixed heading keeps the camera's image-up
  direction constant across trials, which is what makes `image_up_is_forward`
  comparable. The alternative (`NaN`, hold current heading) is noted in
  `_publish_setpoint`.
- **Why is latency pooled from raw samples?** A percentile of percentiles is not
  a percentile (`scripts/benchmark.py`, `docs/experiment-protocol.md`).
- **Why are the targets 20 m apart?** So exactly one is inside the 11.9 × 8.9 m
  nadir patch per trial and the ground-truth association is unambiguous
  (`docs/scenario.md`).

## Comment policy

Comments explain **why** and **what breaks if changed**. Anything that restates
the line below it is noise and does not belong. Numbers with a physical meaning
carry their units and their source (model file, datasheet, or measurement).
