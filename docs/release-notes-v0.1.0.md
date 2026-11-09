# v0.1.0 — PX4/Gazebo SITL closed loop: detection → projection → Offboard

> Draft. Fill the `—` cells after the nine trials, then paste the "How to verify"
> block into the GitHub Release body.

First reproducible MVP: a UAV in PX4 SITL detects a ground target in Gazebo,
projects the pixel to a ground ENU coordinate, and flies to it via ROS 2
Offboard control. One command reproduces a trial.

## Added

- PX4 v1.15 SITL + Gazebo Harmonic world with 3 targets at known coordinates
- `detector_node`: pluggable backend — HSV colour baseline and YOLO11n ONNX
  (ONNX Runtime, CPU)
- `projection`: pinhole + nadir + yaw compensation, unit tested
- `offboard_node`: IDLE → TAKEOFF → GOTO → HOVER → LAND, 1 m/s speed limit,
  target-loss hover, jump filter
- `MissionRecorder`: writes one CSV row per trial (error, latency p50/p95, success)
- One-command reproduction: `make e2e`

## Results

| metric | baseline (HSV) | YOLO11n ONNX (CPU) |
|---|---|---|
| detection rate | — | — |
| latency p50 / p95 (ms) | — | — |
| landing error median (m) | — | — |
| success (of 9) | — | — |

Hardware: — (CPU model, RAM, no GPU).

## Known limitations

- SITL only, no real flight.
- x86 CPU inference; no Jetson / RK3588 measurement.
- Static targets, flat ground, nadir camera; roll/pitch not compensated.
- The COCO pre-trained YOLO does not detect the synthetic cubes zero-shot, so the
  closed-loop demo runs the colour baseline; YOLO is measured for latency and is
  the subject of the v0.2.0 fine-tuning work.

## How to verify

```bash
git clone https://github.com/wuuu854-boop/uav-yolo-offboard-tracking.git
cd uav-yolo-offboard-tracking
git checkout v0.1.0
bash scripts/setup.sh
make sim          # terminal A
make e2e          # terminal B
cat results/landing_error.csv
```
