# Market requirements → evidence in this repository

The job postings this project was written against, and the file that answers
each of them. The right-hand column is what a reviewer can open, not a claim.

Sample: 10 public postings collected on 2026-11-09 (Chengdu and national),
listed in `docs/prior-art.md`'s companion notes and summarised here.

| What the postings ask for | Where this repository shows it |
|---|---|
| PX4 architecture: uORB, modules, control loop, EKF2 | `src/uav_tracking/control/offboard_node.py`, `docs/px4-notes.md` |
| ROS 2 nodes, custom messages, launch, QoS | `src/uav_interfaces/`, `launch/`, the `SensorDataQoS` fix in `CHANGELOG.md` |
| Gazebo / SITL, algorithm-in-the-loop | `sim/`, `scripts/doctor.py`, `check_world_targets.py` |
| YOLO training/export, ONNX Runtime, CPU inference | `src/uav_tracking/perception/yolo.py`, `scripts/export_onnx.py`, `scripts/verify_model.py` |
| Small-object / low-light robustness | v0.2.0 scope, stated in the README's limitations — not claimed today |
| TensorRT / RKNN / INT8 quantization | **not implemented**; `config/detector.yaml` and the roadmap say so |
| Frame conventions, camera model, calibration | `src/uav_tracking/control/frames.py`, `src/uav_tracking/perception/projection.py`, `config/camera.yaml` |
| Experiment method: baselines, repeats, failure analysis | `docs/experiment-protocol.md`, `src/uav_tracking/control/recorder.py`, `scripts/benchmark.py` |
| Engineering habits: CI, tests, releases, changelog | `.github/workflows/`, `tests/`, `CHANGELOG.md`, the draft `v0.1.0` |
| AI-assisted development, used honestly | `docs/ai-usage.md` (what was delegated, what was verified, where it failed) |
| Debugging a real stack from source | `docs/verification-log.md` — ten claims, four of which were defects |

## What this table is not

It is not a skills list. A row only survives if the linked file exists and does
what the row says; the two rows marked **not implemented** are there to make the
gap obvious rather than to be quietly omitted.

## The one row that is still open

| Requirement | Status |
|---|---|
| Closed loop in simulation, with measured landing error | `results/landing_error.csv` is empty until the nine SITL trials run |

That is the difference between a repository that describes a pipeline and one
that has demonstrated it. The README says which one this is.
