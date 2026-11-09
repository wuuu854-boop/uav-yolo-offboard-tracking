# Troubleshooting

Every entry below is a failure that real users hit in this stack, with the
upstream issue it was observed in. `scripts/doctor.py` checks for the ones that
can be detected mechanically.

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

**Fix** This repository converts at a single boundary (`src/control/frames.py`)
and has tests for each conversion. If you fork it, keep that boundary.

**Source** found during development; see `docs/architecture.md`.

---

## 9. Double physics / everything drifts when you add a custom world

**Symptom** Your own world behaves differently from `default.sdf`: the vehicle
jitters, contacts are wrong, or Gazebo logs a system being loaded more than once.

**Cause** PX4 ships a `server.config` that loads Physics, UserCommands,
SceneBroadcaster, Sensors, Imu, NavSat and Magnetometer into **every** world. A
world file that also declares `<plugin filename="gz-sim-physics-system">`
loads a second physics engine.

**Fix** Do not put `<plugin>` tags in a PX4 world. PX4's own worlds
(`PX4-gazebo-models/worlds/*.sdf`) contain none — copy that style.
`sim/worlds/target_tracking.sdf` in this repository follows it.

**Detected by** reading the file: `grep -c "<plugin" sim/worlds/target_tracking.sdf`
must print `0`.

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

