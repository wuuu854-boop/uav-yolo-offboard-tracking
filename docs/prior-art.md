# Prior art and positioning

Evidence collected on **2026-11-09** via the GitHub REST API. Star counts and
issue numbers were read on that date; they will drift.

The question this file answers: *why does this repository exist when there are
already popular PX4 + Gazebo + YOLO projects?*

## The landscape

| Repository | Stars | What it actually does | What it does not do |
|---|---|---|---|
| [JacopoPan/aerial-autonomy-stack](https://github.com/JacopoPan/aerial-autonomy-stack) | 656 | Broad harness: PX4/ArduPilot/Betaflight, ROS 2, YOLO, Jetson, swarms, HITL | Very wide scope; you adopt a framework, not a result |
| [monemati/PX4-ROS2-Gazebo-YOLOv8](https://github.com/monemati/PX4-ROS2-Gazebo-YOLOv8) | 406 | Camera + YOLOv8 detection in Gazebo, keyboard flight via MAVSDK, Docker | **No closed loop, no metrics.** Main branch is `uav_camera_det.py` + `keyboard-mavsdk-test.py` |
| [formiat/px4-ros2-drone-nav](https://github.com/formiat/px4-ros2-drone-nav) | 169 | GNSS-denied navigation: MSCKF VIO, LiDAR-inertial, D* Lite, MPPI | Research-grade stack; no vision-detection-to-approach loop |
| [Zhefan-Xu/CERLAB-UAV-Autonomy](https://github.com/Zhefan-Xu/CERLAB-UAV-Autonomy) | 886 | C++/ROS autonomy framework: exploration, inspection, planning | Framework, not a small reproducible experiment |

## What the issues say

The most popular repository in this niche has 11 open issues, and they are
almost entirely **setup** problems, not algorithm problems:

| Issue | Title | Root cause |
|---|---|---|
| [#42](https://github.com/monemati/PX4-ROS2-Gazebo-YOLOv8/issues/42) | Manual installation fatal error | `px4_ros_com` not maintained against current `px4_msgs`; asks for pinned branches |
| [#24](https://github.com/monemati/PX4-ROS2-Gazebo-YOLOv8/issues/24) | `ros-humble-ros-gzgarden` not found | Gazebo Garden EOL; mixed `ros_gz` / `ros_gzgarden` |
| [#22](https://github.com/monemati/PX4-ROS2-Gazebo-YOLOv8/issues/22) | `rclpy._rclpy_pybind11` missing | Python 3.11 interpreter against a 3.10 ROS build |
| [#21](https://github.com/monemati/PX4-ROS2-Gazebo-YOLOv8/issues/21) | gz_bridge timeout on first run | `GZ_SIM_RESOURCE_PATH`, spawn race |
| [#19](https://github.com/monemati/PX4-ROS2-Gazebo-YOLOv8/issues/19) | Preflight check failed | Compass / GCS link missing in SITL |
| [#20](https://github.com/monemati/PX4-ROS2-Gazebo-YOLOv8/issues/20) | `COMMAND_DENIED` when arming | Arming before the controller owns the mode |

There is also direct demand evidence: a contributor opened
[PR #40](https://github.com/monemati/PX4-ROS2-Gazebo-YOLOv8/pull/40) adding
"autonomous car tracking using YOLOv11 + PX4 offboard mode" with a state machine
and CSV logging — i.e. exactly the closed loop plus measurement. **It was closed
without being merged**, so `main` still has no closed loop. Someone wanted this;
nobody shipped it there.

One small thing was contributed back: [#43](https://github.com/monemati/PX4-ROS2-Gazebo-YOLOv8/pull/43)
pins the two PX4 message packages to the same release branch, which is the fix
asked for in #42. The repository you are reading exists because that class of
setup failure was worth solving once instead of per user.

Read that table again: **the bottleneck in this stack is not the model, it is
getting to a state where you can trust a number.**

## The gap this project targets

Two searches on 2026-11-09:

- `px4 offboard target tracking` → **0 repositories**
- `uav benchmark reproducible simulation` → 1 repository, 0 stars

So the niche is not "detect objects from a drone" (saturated). It is
**"close the loop and measure whether it worked, reproducibly"**:

1. detection → ground coordinate → Offboard approach, in one runnable chain;
2. every claim backed by a CSV row written by code, not by hand;
3. a `doctor` command that diagnoses the six setup failures above before you
   waste a day on them.

## How this repo differs

| | Typical detection demo | This repository |
|---|---|---|
| Output | bounding boxes on screen | landing error (m), latency p50/p95 (ms), success rate |
| Control | keyboard / none | autonomous Offboard approach, speed-limited state machine |
| Evidence | a GIF | `results/*.csv` + a protocol in `docs/experiment-protocol.md` |
| Setup | README instructions | `make doctor` checks versions, ABI, bridge, world, topics |
| Time to first verifiable result | a day of installation | `make demo` in ten seconds, `make e2e` for the real thing |
| Failure modes | undocumented | `docs/troubleshooting.md`, sourced from real issues |
| Scope | "a platform" | one loop, finished, measured |

## What this is not

- Not a framework. If you want a research platform, use `aerial-autonomy-stack`.
- Not a detector contribution. It uses stock YOLO and a colour baseline.
- Not real flight. SITL only; the README says so in the first paragraph.

Being explicit about the three "nots" is what makes the one "is" credible.
