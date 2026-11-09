# PX4 notes

Working notes kept while building this project. They exist so that the next
person (or future me) does not rediscover the same things.

## Version pinning

The PX4 / Gazebo / ROS 2 combination is the single biggest time sink. Pin it:

```
PX4-Autopilot  v1.15.x   (Gazebo Harmonic / gz-sim 8)
ROS 2          Humble    (Ubuntu 22.04, Python 3.10)
Gazebo         Harmonic  (gz sim 8)
```

Do not mix Gazebo Classic 11 with a PX4 release built for `gz`. If your PX4
target is `gz_x500`, you are on Harmonic; if it is `gazebo-classic`, you are not.

## uORB message chain

What actually happens when the vehicle flies to a detected target:

```
camera sensor (gz)  ->  /camera/image_raw (ROS 2 via ros_gz_bridge)
                    ->  detector_node  ->  /detections
                    ->  offboard_node  ->  /fmu/in/trajectory_setpoint
                    ->  uORB: trajectory_setpoint
                    ->  mc_pos_control -> vehicle_attitude_setpoint
                    ->  mc_att_control  ->  actuator_motors
```

Reading `trajectory_setpoint` and `vehicle_local_position` in Flight Review is
the fastest way to tell whether a problem is in your node or in the controller.

## Coordinates

- PX4 internal and odometry topics are **NED** (x north, y east, z down).
- Gazebo worlds are **ENU**.
- ROS REP-103 is **ENU** with yaw 0 = east.
- The conversion happens in exactly one place: `offboard_node.on_odometry`.

## Timestamps

- Use the message header stamp for latency, not the receipt time: the receipt
  time hides the transport delay you are trying to measure.
- Set `use_sim_time:=true` on every node when running against Gazebo, otherwise
  `now()` and the image stamps are on different clocks and the latency is garbage.

## Camera

`gz_x500_mono_cam` exposes the camera under a topic that contains the world name,
the spawned model name (`..._0`), the link and the sensor name. Find it with:

```bash
gz topic -l | grep -i image
```

## ULog

Every SITL run writes a `.ulg` under the PX4 build directory. Analyse it with
Flight Review (upload) or `pyulog`:

```bash
python3 -m pyulog info log.ulg
python3 -m pyulog ulog2csv log.ulg
```

Flight logs contain GPS coordinates — treat them as personal data and keep them
out of the repository.
