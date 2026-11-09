# Skills

Debugging procedures distilled from building this repository. Each one is a
problem that actually cost time here, written so the next person — or the next
agent — can skip the guessing.

They are deliberately short: a symptom, why it happens, the commands, and how to
know it is fixed. Every one names the file or run where it was found, so nothing
here is folklore.

| # | Skill | Symptom in one line | Time |
|---|---|---|---|
| [01](01-silent-topic.md) | Find a silent ROS 2 topic | Node runs, receives nothing, logs nothing | 5 min |
| [02](02-camera-mount.md) | Tell a nadir camera from a forward one | Projected target lands hundreds of metres away | 10 min |
| [03](03-world-plugins.md) | Keep a PX4 Gazebo world valid | Jitter, double physics, model will not spawn | 10 min |
| [04](04-intrinsics-from-sdf.md) | Derive intrinsics from the model | Error grows linearly with altitude | 10 min |
| [05](05-version-drift.md) | Pin PX4 message packages | `sensor_gps.hpp: No such file or directory` | 10 min |
| [06](06-numpy-abi.md) | Survive the numpy 2 / cv_bridge clash | `_ARRAY_API not found`, then a segfault | 5 min |
| [07](07-claim-to-csv.md) | Turn a run into evidence | Numbers in the README drift from reality | 30 min |

## How to use them

1. Match the symptom, not the technology. Several of these apply to any ROS 2 +
   simulator stack, not only to PX4.
2. Run the **Verify** step before believing the fix. Every skill has one.
3. If a skill is wrong for your version, fix the skill in the same commit as the
   code — that is the only way this folder stays useful.

## Where they come from

| Skill | Found in |
|---|---|
| 01 | `CHANGELOG.md` → *Fixed*: subscriptions moved to `SensorDataQoS` |
| 02 | `docs/verification-log.md` row 2 |
| 03 | `docs/verification-log.md` row 4 |
| 04 | `docs/scenario.md` — the 640×480 assumption that was wrong |
| 05 | [PR #43](https://github.com/monemati/PX4-ROS2-Gazebo-YOLOv8/pull/43) upstream |
| 06 | `ROS build` run #4, fixed in the numpy pin |
| 07 | `docs/experiment-protocol.md` |
