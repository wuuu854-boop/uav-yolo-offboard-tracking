# Deployment performance

The job postings for this field ask for "model deployment on NVIDIA Jetson with
TensorRT" and "real-time perception". This page is the part of that claim this
project can answer without the hardware: how fast the detector actually runs, at
which input size, on how many threads — measured on the same decode path the
flight loop uses.

## Method

- 60 recorded frames, 1280×960, from a real SITL flight (`capture_pose_frames.py`).
- ONNX Runtime CPU, the model from [`docs/model.md`](model.md), the same
  `YoloOnnxDetector` the node uses.
- `scripts/bench_detector.py`; p50/p95 over the run, throughput from the **mean**
  latency so one slow frame is not hidden by the median.
- Machine: Intel i7-14650HX (24 threads), WSL2 Ubuntu 22.04, Python 3.10.

## Results

640 px export, 640 input:

| Threads | p50 | p95 | FPS |
|---|---|---|---|
| 1 | 72.9 ms | 75.2 ms | 13.7 |
| 2 | 41.6 ms | 42.9 ms | 23.9 |
| 4 | **26.1 ms** | 31.6 ms | **37.3** |
| 8 | 27.7 ms | 30.4 ms | 35.9 |

320 px export, 320 input:

| Threads | p50 | p95 | FPS |
|---|---|---|---|
| 1 | 17.7 ms | 18.3 ms | 56.1 |
| 2 | 11.3 ms | 12.8 ms | 86.7 |
| 4 | **8.1 ms** | 10.3 ms | **119.0** |
| 8 | 8.4 ms | 9.9 ms | 121.0 |

## What the numbers say

- **320 at 4 threads clears a 30 Hz camera twice over** (8.1 ms, 119 FPS). The
  detector is not the bottleneck at this size.
- **640 at 4 threads is 37 FPS** — enough for a 30 Hz camera, but with little
  headroom once the rest of the loop is added.
- **Four threads is the knee.** Eight threads add nothing measurable on this
  machine; the extra threads cost memory bandwidth and scheduling. On a
  companion computer with fewer cores the knee will be lower, which is why the
  benchmark takes the thread count as an argument instead of hardcoding it.
- The closed loop's 143 ms end-to-end latency is **not** detection time. It is
  dominated by image transport and the control loop; detection is 8–26 ms of it.

## What this does not answer

- **No Jetson measurement.** No board, so no TensorRT, no INT8, no GPU memory
  budget. The numbers above are a desktop CPU and should not be quoted as an
  edge result.
- **No quantisation.** INT8 calibration and its accuracy cost are unmeasured
  here; that is the next step that needs the target hardware to be meaningful.
- **No batching or pipeline overlap.** The node processes one frame at a time,
  which is right for a control loop but is not the maximum throughput.

## Reproduce

```bash
python3 scripts/bench_detector.py --images ~/capture --model models/yolo11n.onnx \
    --imgsz 640 --threads 1 2 4 8
```
