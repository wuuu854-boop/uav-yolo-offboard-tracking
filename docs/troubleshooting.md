# Troubleshooting

Every entry below is a failure that real users hit in this stack, with the
upstream issue it was observed in. `scripts/doctor.py` checks for the ones that
can be detected mechanically.

Each entry has a matching step-by-step procedure, with a verification step, in
[`skills/`](../skills/README.md) — this file is the "what and why", that folder is
the "how, and how to know it worked".

Run first:

```bash
python3 scripts/doctor.py          # static + ROS checks
python3 scripts/doctor.py --static # no ROS needed
```

---

## 1. `colcon build` fails: `px4_msgs/msg/sensor_gps.hpp: No such file`

**Symptom**

```
fatal error: px4_msgs/msg/sensor_gps.hpp: No such file or directory
Failed <<< px4_ros_com
```

**Cause** `px4_ros_com` is not maintained against every `px4_msgs` release. The
message definitions were renamed (`sensor_gps` → `sensor_gps` moved packages),
so a `main`-branch clone of one against a tagged clone of the other breaks.

**Fix** Pin both to the same PX4 minor version, do not track `main`:

```bash
git clone -b release/1.15 https://github.com/PX4/px4_msgs.git
# and build only our interfaces, not the upstream examples:
colcon build --packages-select uav_interfaces
```

This repository never builds `px4_ros_com`; it depends on `px4_msgs` only.

**Detected by** `doctor.py` → `px4_msgs version` (compares the checked-out branch
against the PX4 tag).

**Source** [monemati#42](https://github.com/monemati/PX4-ROS2-Gazebo-YOLOv8/issues/42)

---

## 2. `E: Unable to locate package ros-humble-ros-gzgarden`

**Cause** Gazebo Garden reached end of life and the `ros_gzgarden` binary is no
longer in the default Humble repositories. Mixing `ros_gz` (Harmonic) with
`ros_gzgarden` (Garden) produces two bridges that cannot see each other.

**Fix** Use one bridge for one Gazebo generation:

```bash
sudo apt install ros-humble-ros-gz   # Harmonic, matches PX4 >= v1.15
```

Do not install both. If you must stay on Garden, add the OSRF repository first
and pin the whole toolchain, as in the upstream issue.

**Detected by** `doctor.py` → `ros_gz bridge` (reports which one is installed).

**Source** [monemati#24](https://github.com/monemati/PX4-ROS2-Gazebo-YOLOv8/issues/24)

---

## 3. `ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'`

**Cause** A Python 3.11+ interpreter importing a `rclpy` built for 3.10. The
`.so` filename in the error message gives it away:
`_rclpy_pybind11.cpython-311-...so`.

**Fix** Use the system Python 3.10 and a venv that can still see ROS:

```bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -c "import sys; print(sys.version)"   # must be 3.10.x
```

Never use conda for a ROS 2 workspace: it ships its own Python and the ABI will
not match.

**Detected by** `doctor.py` → `python` and `rclpy` checks.

**Source** [monemati#22](https://github.com/monemati/PX4-ROS2-Gazebo-YOLOv8/issues/22)

---

## 4. `gz_bridge: Service call timed out` and the drone never spawns

**Symptom**

```
ERROR [gz_bridge] Service call timed out. Check GZ_SIM_RESOURCE_PATH is set correctly.
ERROR [gz_bridge] Task start failed (-1)
```

**Cause** Gazebo cannot find the model/world resources, or the spawn service is
called before the world finished loading. It often succeeds on the second run,
which makes it look random.

**Fix**

```bash
export GZ_SIM_RESOURCE_PATH="$HOME/PX4-Autopilot/Tools/simulation/gz/models:$GZ_SIM_RESOURCE_PATH"
# start PX4 only after Gazebo's GUI is up, or pass a longer spawn timeout
```

Also confirm the world file is where PX4 looks for it:

```bash
ls "$PX4_DIR/Tools/simulation/gz/worlds/target_tracking.sdf"
```

**Detected by** `doctor.py` → `world file` and `GZ_SIM_RESOURCE_PATH`.

**Source** [monemati#21](https://github.com/monemati/PX4-ROS2-Gazebo-YOLOv8/issues/21)

---

## 5. Arming denied: `Preflight Fail: Compass Sensor 0 missing`

**Cause** SITL still runs the arming checks. Without a compass model and a
ground station heartbeat, PX4 refuses to arm — even though the simulation looks
healthy.

**Fix** Either add the missing sensors to the airframe, or relax the checks in
SITL only:

```bash
param set COM_ARM_WO_GPS 1
param set COM_ARM_MAG_STR 0
```

Do this **only in simulation**. These parameters bypass safety logic; never ship
them to a real vehicle.

**Detected by** `doctor.py` cannot see PX4 parameters — it reports the check as
manual and links here.

**Source** [monemati#19](https://github.com/monemati/PX4-ROS2-Gazebo-YOLOv8/issues/19)

---

## 6. `COMMAND_DENIED` when arming from a script

**Cause** Arming is requested before the controller owns the flight mode, or
before the Offboard setpoint stream has been running for a few frames. PX4
rejects the command instead of queueing it.

**Fix** In this repository the order is enforced in `offboard_node.py`:

1. publish `OffboardControlMode` + `TrajectorySetpoint` continuously;
2. request `VEHICLE_CMD_DO_SET_MODE` (Offboard);
3. only then request arming.

If you write your own node, keep that order and stream setpoints at a stable
rate before switching modes.

**Detected by** logs: if `requested OFFBOARD mode` never appears, the mode switch
itself was rejected.

**Source** [monemati#20](https://github.com/monemati/PX4-ROS2-Gazebo-YOLOv8/issues/20)

---

## 7. "It works in Docker but not on my machine"

**Cause** The Docker image pins every version; the host does not. Manual installs
drift (issue #18, #41 in the upstream repo are both this).

**Fix** Treat `versions.yaml` as the contract and let `doctor.py` compare your
machine against it. If a check fails, fix the machine — do not edit the code.

**Source** [monemati#18](https://github.com/monemati/PX4-ROS2-Gazebo-YOLOv8/issues/18),
[#41](https://github.com/monemati/PX4-ROS2-Gazebo-YOLOv8/pull/41)

---

## 8. The drone flies to a mirrored position (this project)

**Cause** One of three frame assumptions is wrong, in this order of likelihood:

1. `image_up_is_forward` in `config/camera.yaml` (camera rolled 180°);
2. PX4 heading (clockwise from north) used as if it were ROS yaw (counter-
   clockwise from east) — a constant 90° error;
3. NED/ENU axis swap.

**Fix** This repository converts at a single boundary (`src/uav_tracking/control/frames.py`)
and has tests for each conversion. If you fork it, keep that boundary.

**Source** found during development; see `docs/architecture.md`.

---

## 9. Double physics / everything drifts when you add a custom world

**Symptom** Your own world behaves differently from `default.sdf`: the vehicle
jitters, contacts are wrong, or Gazebo logs a system being loaded more than once.

**Cause** Two loaders for the same system. Either the world declares the system
plugins *and* Gazebo is started with a config that loads them, or the same
`<plugin>` line appears twice.

**Fix** This repository's `sim/worlds/target_tracking.sdf` declares Physics,
UserCommands, SceneBroadcaster, Sensors and Imu, because PX4 v1.15's launcher
passes no config. If your launcher *does* pass one (`gz sim --config ...`),
delete the block instead of adding to it — one loader, not two.

**Detected by** `scripts/check_world_targets.py`: it fails on a missing Sensors
system (no camera images) and on duplicate plugin filenames. The topic is the
real check: `gz topic -l | grep sensor/camera`.

---

## 10. The camera shows the horizon, not the ground

**Symptom** Detection works, but the projected target is hundreds of metres away
and the vehicle flies off.

**Cause** `x500_mono_cam` is a **forward-looking** camera (pose `.12 .03 .242`,
no rotation). The nadir model is `x500_mono_cam_down`, which mounts the same
sensor with a +90° pitch and a 0.10 m height offset.

**Fix** Use the `_down` model and the matching intrinsics:

```bash
make sim PX4_MODEL=gz_x500_mono_cam_down
```

`config/camera.yaml` already carries the `_down` intrinsics (1280x960, HFOV
1.74 rad, fx 538.4) and the 0.10 m mount offset.

**Detected by** `doctor.py` → `gz assets` checks that the `_down` model exists.

---

## 11. `gz topic -l` shows no `/depth_camera`

**Symptom** The avoidance run exits with "the vehicle depth camera did not
appear"; `gz topic -l | grep depth` is empty, while the nadir camera topic is
present.

**Cause** The installed vehicle model is the *old* one. `make install-world` used
to copy the model only when the directory was missing, so a store populated by
v0.8.0 silently kept a camera-less model after the repository gained the depth
sensor. The depth camera is also published on the sensor topic directly
(`/depth_camera`), unlike the nadir camera, whose topic is scoped under
`/world/<world>/model/.../link/camera_link/sensor/camera/image`.

**Fix**

```bash
make install-world          # now always overwrites the installed model
gz topic -l | grep depth    # /depth_camera and /depth_camera/points
```

**Detected by** `scripts/run_avoidance.sh`, which checks for `/depth_camera`
before starting the nodes and prints the Gazebo topics it did see.

**Source** found during v0.9.0 bring-up.

---

## 12. The depth image is all zeros, or `nearest` reads 0.25 m during takeoff

**Symptom** The depth frame is black/empty at the start, then the node logs
`STOP: nearest 0.25 m` while the vehicle is still climbing, and clears a second
later.

**Cause** At ground level the depth camera is inside its 0.2 m near clip and sees
only the ground; 0.25 m is the `min_valid_m` filter, not an obstacle. It is the
ground below the vehicle. The same happens if the airframe faces away from the
approach: a forward camera looking at empty ground or sky returns no wall.

**Fix** Do not lower the threshold. Confirm the vehicle is in `GOTO` before
acting on the stop (the controller ignores it in `TAKEOFF`/`HOVER`/`LAND`), and
confirm `heading_yaw_deg` points at the obstacle. `stop_events` in
`results/avoidance_run.csv` makes the takeoff event visible instead of hiding it.

**Detected by** `log/depth_node.log` and the `stop_events` column.

**Source** found during v0.9.0 bring-up.

---

## 13. The stop never releases, or the vehicle stops immediately after takeoff

**Symptom** `/avoidance/stop` stays true with a wall metres away, or the vehicle
stops on the first control tick and never resumes.

**Cause** Three separate things look the same:

1. **Hysteresis.** The gate releases only above `clear_distance_m` (3.2 m), not
   at the 2.5 m stop threshold. A vehicle that stopped at 2.4 m reads 2.4 m
   forever and correctly stays stopped.
2. **Stale stream.** Once `/avoidance/stop` has been seen, a gap longer than
   `avoidance_timeout_s` is treated as stop. A crashed depth node therefore looks
   like a permanently blocked path, by design.
3. **Wrong ROI.** If the ROI is on the ground or the sky rather than the wall,
   the nearest value can sit below the threshold without an obstacle in front.

**Fix** Read the three numbers together: `nearest_m`, `stop`, and the vehicle
position in `results/avoidance_run.csv`. If `nearest_m` is small while the wall
is far, the ROI or the camera heading is wrong; if `nearest_m` is genuine and
below 3.2 m, the stop is correct.

**Detected by** `scripts/avoidance_report.py` fails a run with no stop, no hold,
or insufficient clearance; it does not just print the numbers.

**Source** found during v0.9.0 bring-up.

---

## 14. `make avoid` reports “depth-camera model not installed” after pulling

**Cause** The model store and both worlds are installed copies; a `git pull`
does not update them.

**Fix**

```bash
make install-world
```

Then rerun `make avoid`. The target calls `install-world` itself, so this only
bites when the simulator is started by hand with `make sim`.

---

## 15. The L-shaped detour touches the wall's corner

**Symptom** The detour reaches the target but the wall's contact sensor fires, or
the vehicle loses altitude/yaw near the end of the detour.

**Cause** The forward leg is too short. The target is only 0.8 m behind the wall,
so the final approach must come from **east** of the wall's east face. If the
forward leg ends at the face (or short of it), the diagonal back to the target
cuts the corner. The first phase-2 attempt used a 3 m forward leg and did this:
the wall's contact sensor fired and the vehicle lost altitude. The accepted
configuration uses 3.5 m lateral and 5 m forward.

**Fix** Check `detour_forward_m` and the trace: the forward waypoint must be east
of the wall's east face (world x ≈ -0.7) by more than the airframe. The accepted
run's forward point is local x ≈ 8.45 (world x ≈ 0.95), with the target at world
x = 0.

**Detected by** `scripts/avoidance_report.py --mode detour` fails on a contact or
clearance below 0.4 m; the trace shows where the forward leg ended.

**Source** found during v0.10.0 bring-up.

---

## 16. The automatic detour picks the "wrong" side, or the report disagrees with the log

**Symptom** The offboard log says "to the left", but the report's side check
fails; or the side topic flips between `left` and `right` during the flight.

**Cause** Three different things:

1. **Whole-run mode vs the decision.** The side that looks freer often flips
   after the vehicle is around the obstacle. The planner uses the value at the
   first stopped tick; a report that counts the whole run can disagree.
2. **Margin.** A difference below `side_margin_m` (0.5 m) falls back to
   `detour_side` instead of flipping on noise. If the two strips are nearly
   equal, the fallback is the expected answer.
3. **Both sides blocked / no return.** The node publishes `unknown`; the
   controller uses the fallback. There is no search for another exit.

**Fix** Read `clear_side` on the first `stop=1` row, not the mode over the whole
CSV. `scripts/avoidance_report.py --expect-side` does exactly that and also
checks that the lateral direction matches the chosen side.

**Detected by** the `side_selected` and `lateral_direction_matches_side` checks
in `results/autod_summary.json`.

**Source** found during v0.11.0 bring-up.

