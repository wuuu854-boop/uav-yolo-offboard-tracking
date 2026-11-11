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
| 12 | The closed loop lands on target in SITL | 9 trials, 3 per target, `results/landing_error.csv` | 9/9 within 1.5 m, median 0.73 m, p50 latency 143 ms; two measurement defects (local/world frame, stale hover setpoint) found and fixed on the way |
| 13 | The depth gate stops before a wall in SITL | `results/avoidance_run.csv`, `avoidance_summary.json`, `avoidance_trace.csv`, `contact_sensor_check.txt` | PASS: stop at 19.8 s, 25.1 s hold, settled drift 0.124 m, min clearance 2.204 m, 0 contacts, 822 depth frames. Three defects found on the way: the hold followed the vehicle's own drift (1.34 m), a frozen setpoint let it coast 1.17 m, and the first 1.2 m wall was stopped on its corner; the final design latches the vehicle position at 0.6 m/s and uses a 3 m wall |
| 14 | The L-shaped detour gets around the wall and back to target_0 | `results/detour_run.csv`, `detour_summary.json`, `detour_trace.csv`, `detour_landing.csv` | PASS: 3.394 m lateral leg, gate released at 29.8 s, 0.527 m final target error, landing 0.63 m `success=True`, 0.999 m minimum clearance, 0 contacts, 791 depth frames. The first attempt used a 3 m forward leg, grazed the wall's corner and the contact sensor fired; 3.5 m lateral + 5 m forward fixed it |
| 15 | The detour side can be chosen from the depth strips | `results/autod_run.csv`, `autod_summary.json`, `autod_trace.csv`, `autod_landing.csv` | PASS: `clear_side_at_stop=left`, 4.181 m lateral leg, gate released at 22.7 s, 0.714 m final target error, landing 0.591 m `success=True`, 0.688 m minimum clearance, 0 contacts on the wall and pillar, 790 depth frames. The whole-run mode was `right`, so the report reads the value at the first stopped row, not the mode |
| 16 | Both side strips blocked: hold instead of guessing | `results/blocked_run.csv`, `blocked_summary.json`, `blocked_trace.csv`, `blocked_landing.csv` | PASS (hold): stop at 19.6 s, `blocked` latched 0.6 s later, 35.0 s hold, 0.129 m settled drift (0.5 m limit), 2.68 m minimum wall clearance, 0 contacts on the wall and barrier, 821 depth frames. The mission times out and lands 4.032 m from target_0, `success=False`; that is the expected result of refusing to move |
| 17 | Both side strips blocked: back off and recover | `results/backoff_run.csv`, `backoff_summary.json`, `backoff_trace.csv`, `backoff_landing.csv` | PASS: stop at 19.7 s, `backing off` then `backoff done: lateral leg` in the trace, 1.63 m measured back-off (2.0 m waypoint), left side at the back-off point, 3.254 m lateral leg, 7 m forward, 0.831 m final target error, landing 0.684 m `success=True`, 0.595 m minimum clearance, 0 contacts, 882 depth frames |
| 18 | Both side strips blocked: repeat the back-off | `results/sweep_run.csv`, `sweep_summary.json`, `sweep_trace.csv`, `sweep_landing.csv` | PASS: stop at 17.9 s, four attempts (`2/4`…`4/4`), 0.62 m back-off travel, left at the last step, 3.772 m lateral leg, 6 m forward, 0.852 m final target error, landing 0.808 m `success=True`, 0.639 m minimum clearance, 0 contacts, 1212 depth frames |
| 19 | Both side strips blocked: size the back-off from depth | `results/adaptive_run.csv`, `adaptive_summary.json`, `adaptive_trace.csv`, `adaptive_landing.csv` | PASS: stop at 18.9 s, left 3.20 m / right 2.32 m, computed 2.00 m step, 2.02 m travel, left after it, 3.566 m lateral leg, 7.5 m forward, 0.618 m final target error, landing 0.572 m `success=True`, 0.47 m minimum clearance, 0 contacts, 1001 depth frames |
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

- A claim without a file in `results/`. The CSV, JSON or log is the claim's
  evidence; this table only says what was checked and what the check changed.
- Anything from a blog post or a tutorial that could not be traced to a file in
  the pinned upstream version.
