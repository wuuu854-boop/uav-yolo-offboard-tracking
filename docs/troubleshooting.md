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

---

## 17. The both-sides-blocked scenario still detours

**Symptom** The blocked world has a barrier in both strips, but the vehicle flies
the left L anyway; the hold report fails and sometimes the barrier contact fires.

**Cause** The side label is not stable at the instant the gate stops. In SITL the
first stopped frame said `left`; only 0.6 s later did both strips read below the
floor and the node say `blocked`. The controller committed on the first frame.
The label can then flicker between `blocked` and `left` as the vehicle pitches to
a halt.

**Fix** Two things, both in v0.12.0:

1. `detour_start_delay_s: 1.0` holds through the settle transient before
   committing to a side.
2. The controller latches `blocked` for the rest of the stop once it has seen it,
   so a one-frame `left` between two `blocked` frames cannot start the detour.

The barrier itself is 4 m wide for the same reason: two side pillars moved in and
out of the strips as the vehicle drifted laterally; a barrier is in both strips
regardless.

**Detected by** the `blocked_run.csv` `clear_side` sequence and the absent
`blocked: holding` trace note in the failing runs.

**Source** found during v0.12.0 bring-up.

---

## 18. The back-off recovery clips the barrier

**Symptom** The trace shows the back-off and lateral legs, but the run has
contacts and the final target error is large; the return path crosses the
barrier's north edge.

**Cause** The forward leg is relative to the **back-off point**. Reusing the
phase-2/3 5 m forward leg from a point 2 m further back ends at the barrier's x;
the diagonal back to target_0 then crosses the barrier's edge. The back-off is
correct; the leg after it is too short.

**Fix** `config/mission_backoff.yaml` uses `detour_forward_m: 7.0`. That puts the
forward point east of the barrier before turning back. The retained run has
0.595 m clearance and 0 contacts.

**Detected by** `contacts_total` and `clearance_at_least_min` in the first
phase-5 report.

**Source** found during v0.13.0 bring-up.

---

## 19. The repeated back-off never clears, or holds forever

**Symptom** The trace shows `still blocked: backing off again (n/max)` until the
budget is spent, then a hold; or the vehicle resumes as soon as the gate reads
clear and drives straight back into the obstacle.

**Cause** Three things:

1. **The floor is above the ground return.** At 5 m altitude the lower part of a
   side strip sees the floor at roughly 7 m. An 8 m side floor counts the floor as
   a permanent obstacle, so no amount of backing off clears it.
2. **The step is larger than the distance to the first clear point.** A 1 m step
   can pass the first clear position; the next decision is made past it. The
   loop only helps if the step is small enough to sample the gap.
3. **Gate-clear is not path-clear.** After the budget is spent, `HOLDING` must not
   reset on a clear gate: backing away makes the gate clear while the obstacle is
   still in front. `_backoff_hold` keeps the hold until a side clears.

**Fix** The retained phase-6 scenario uses a 6.0 m floor, 0.2 m steps, a 0.05 m
reach tolerance and `backoff_max_attempts: 4`. The trace shows `(2/4)`…`(4/4)`
before the lateral leg.

**Detected by** the sweep trace reaching `HOLDING` without a lateral leg, or the
masked loop in `offboard_avoidance.log`.

**Source** found during v0.14.0 bring-up.

---

## 20. The measured back-off holds instead of moving, or overshoots

**Symptom** With `detour_backoff_m: 0.0` and adaptive enabled, the trace says
`both sides blocked: holding`; or the computed step is much larger than expected.

**Cause**

1. **No side-clearance data.** The controller needs
   `/avoidance/side_clearance`; if it has never received it, both values stay
   `NaN`, `adaptive_backoff_step()` returns `None`, and a zero configured step
   means hold. Check `ros2 topic echo /avoidance/side_clearance`.
2. **The target and the depth floor disagree.** The controller's
   `backoff_target_m` and the node's `min_clear_side_m` are separate config keys.
   If the target is above the floor, the first step can leave the label blocked
   and a second step is needed; if the target is below the floor, the step may be
   zero while the label is still blocked.
3. **The 1:1 growth assumption is wrong for the geometry.** The step is open-loop
   between measurements; the loop (`backoff_max_attempts`) is the correction.
4. **The floor is above the ground return** (~7.3 m at 5 m altitude), so the
   floor is a permanent obstacle.

**Fix** The retained phase-7 mission uses a 6.0 m floor and a 6.0 m target, a
0.2–2.0 m clamp and `backoff_max_attempts: 4`. It logs `left_m`/`right_m` and the
chosen step, so a mismatch is visible in the CSV and trace.

**Detected by** the trace holding with no `backing off` note, or a step outside
the clamp in `results/adaptive_trace.csv`.

**Source** found during v0.15.0 bring-up.

---

## 21. The side search starts although the stop frame shows a free side

**Symptom** `search_run.csv` and `search_depth_frame.png` show one strip open
(left 9.47 m, right 2.05 m) yet the trace says `both sides blocked: side-search
right`.

**Cause** The controller latches `blocked` during the settle and uses the
**latest** side-clearance topic values at the search start, not the stop frame.
The label arrives about 0.7 s after the stop; the values also change as the
vehicle pitches. The search is therefore a recovery from a transient/latched
block, not proof that both strips were simultaneously below the floor.

**Decision** Keep the run: it verifies the SEARCH transition, the 2.1 m step, the
re-read and the recovery to target_0 with 0 contacts. Document the caveat in the
README and [`search.md`](search.md); do **not** claim a simultaneous both-strip
block.

**Fix for the missing case** Needs a geometry where both strips stay inside the
floor at the stop for several frames. The retained two-pillar world does not: the
left pillar is outside the left strip's view at the stop.

**Detected by** the strip overlay on the stop frame and the `left_m`/`right_m`
columns.

**Source** found during v0.16.0 bring-up.

---

## 22. The blocked frame is captured during the search, not at the stop

**Symptom** `search_depth_frame.png` shows both strips blocked (`L 2.61`,
`R 1.98`) but the first stopped CSV rows are `L 7.66 / R 2.47`; the trace's
`side-search` starts before the both-blocked frame.

**Cause** The recorder saves a frame whenever the depth label becomes `blocked`,
and the both-blocked state in this geometry happens **during** the side-step, not
on the first stopped frame. The first stop has one strip open; the search starts
from the latched `blocked` label during the settle.

**What changed from v0.16.0** The right pillar was moved from `(-1.5, -1.5)` to
`(-1.5, -1.0)`, putting it inside the left strip. The earlier frame read
`L 9.47 / R 2.05`; now a blocked frame reads `L 2.61 / R 1.98`. That verifies
both strips can be inside the floor, but only during the search.

**Still open** A stop where both strips are inside the floor for several frames
**at** the stop. The search trigger is still the settle latch. Do not describe
the frame as a "both-blocked stop" in prose; say "both-blocked frame captured
during the search".

**Detected by** comparing the frame's L/R values with the stop rows' timestamps.

**Source** found during v0.16.1 bring-up.

