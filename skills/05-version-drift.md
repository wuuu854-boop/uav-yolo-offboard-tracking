# Skill 05 — Pin PX4 message packages

| | |
|---|---|
| **When** | `colcon build` fails with a missing `px4_msgs` header |
| **Time** | 10 minutes |
| **Needs** | git, network access |
| **From** | [PR #43](https://github.com/monemati/PX4-ROS2-Gazebo-YOLOv8/pull/43), fixing upstream issue #42 |

## Symptom

```
fatal error: px4_msgs/msg/sensor_gps.hpp: No such file or directory
Failed <<< px4_ros_com
```

Usually right after following an install guide that says `git clone` with no
branch.

## Why

`px4_msgs` and `px4_ros_com` are separate repositories with their own release
branches. `main` of one is not guaranteed to match `main` of the other, and
message definitions get renamed or moved between releases. Cloning both from the
default branch is a coin flip that fails a few months after the guide was written.

## Procedure

1. See which releases exist, instead of guessing:

   ```bash
   git ls-remote --heads https://github.com/PX4/px4_msgs.git | grep release/
   git ls-remote --heads https://github.com/PX4/px4_ros_com.git | grep release/
   ```

2. Pick a branch that exists in **both** and matches your PX4 version, then pin:

   ```bash
   git clone -b release/1.15 https://github.com/PX4/px4_msgs.git
   git clone -b release/1.15 https://github.com/PX4/px4_ros_com.git
   ```

3. Build only what you need. Most projects do not need the upstream examples:

   ```bash
   colcon build --packages-select px4_msgs uav_interfaces
   ```

## Verify

```bash
source install/setup.bash
ros2 interface show px4_msgs/msg/VehicleStatus | head
```

If the message resolves, the interface package is correct. A full `colcon build`
with no failed package is the stronger check.

## Prevention

Write the branch into a version file and check it on the machine, which is what
`versions.yaml` + `python3 scripts/doctor.py` do here: a mismatch between
`px4_msgs` and the PX4 tag is reported as a failure with the fix in the hint.
