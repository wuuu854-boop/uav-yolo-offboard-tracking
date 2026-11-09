# Verification log

Every non-obvious claim in this repository, what it was checked against, and
what the check changed. Kept because "it works on my machine" is not evidence,
and because the *corrections* are the useful part of a project like this.

| # | Claim | Checked against | Outcome |
|---|---|---|---|
| 1 | PX4 v1.15 pairs with Gazebo Harmonic | `PX4-Autopilot` v1.15 source, `px4_msgs` `release/1.15` | pinned in `versions.yaml`, checked by `doctor.py` |
| 2 | The nadir camera is `x500_mono_cam_down` | `PX4-gazebo-models/models/x500_mono_cam_down/model.sdf`: +90° pitch, 0.10 m offset | default model changed; the forward `x500_mono_cam` documented as a trap (#10) |
| 3 | Camera intrinsics | `models/mono_cam/model.sdf`: HFOV 1.74 rad, 1280×960 | `fx = 538.4`, `cx = 640`, `cy = 480`; all range numbers in `scenario.md` recomputed |
| 4 | PX4 worlds must not declare systems | `PX4-gazebo-models/server.config` and `worlds/default.sdf` | **claim was wrong** (2026-11-09): v1.15's launcher passes no config, so a plugin-free world publishes no camera. World now declares Physics/Sensors/SceneBroadcaster; see row 12 |
| 5 | `/fmu/out/*` is best-effort | PX4 ROS 2 examples vs `px4_msgs` | subscriptions moved to `qos_profile_sensor_data`; a reliable subscription silently received nothing |
| 6 | PX4 heading ≠ ROS yaw | `px4_msgs/VehicleOdometry.msg` ("FRD body frame to reference frame") | `frames.py` conversion plus 8 tests; the missing 90° was a real defect |
| 7 | Message field names used by the nodes exist | every `.msg` in `px4_msgs` `release/1.15` | confirmed; no change needed |
| 8 | The niche is not "detection" | GitHub API on 2026-11-09: the 406★ repo has no controller; `px4 offboard target tracking` returns 0 repositories | positioning rewritten; `docs/prior-art.md` |
| 9 | Setup, not algorithms, is the blocker | 6 of 11 open issues in the leading repo | `doctor.py` and `docs/troubleshooting.md` |
| 10 | The upstream install instructions are unpinned | `PX4/px4_msgs` and `PX4/px4_ros_com` branch lists | [PR #43](https://github.com/monemati/PX4-ROS2-Gazebo-YOLOv8/pull/43) opened upstream |

## How a check is run

1. Read the upstream source at the pinned version — not the documentation, the
   file the simulator actually loads.
2. Write the finding into `docs/` with the path that proves it.
3. If the finding contradicts the code, change the code and add a test that
   fails without the change.
4. If it cannot be tested in CI, make `doctor.py` check it on the machine that
   can.

## What is deliberately not in this table

- Gazebo SITL results. They are not a claim yet; `results/landing_error.csv` is
  the place they will appear.
- Anything from a blog post or a tutorial that could not be traced to a file in
  the pinned upstream version.
