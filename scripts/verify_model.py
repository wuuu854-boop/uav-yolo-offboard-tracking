#!/usr/bin/env python3
"""Check an exported ONNX model without ROS, Gazebo or ultralytics.

This is the "day 10" gate: if this prints a sensible shape and a stable latency,
the perception half of the pipeline is ready to be wired into the simulator.
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np


def percentile(values: list[float], q: float) -> float:
    return float(np.percentile(np.asarray(values), q))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", default="models/yolo11n.onnx")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--runs", type=int, default=50)
    args = parser.parse_args()

    model_path = Path(args.model)
    if not model_path.exists():
        print(f"model not found: {model_path}")
        return 1

    import onnxruntime as ort

    session = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
    input_name = session.get_inputs()[0].name
    dummy = np.random.rand(1, 3, args.imgsz, args.imgsz).astype(np.float32)

    for _ in range(10):  # warm-up, excluded from the measurement
        session.run(None, {input_name: dummy})

    latencies: list[float] = []
    for _ in range(args.runs):
        started = time.perf_counter()
        output = session.run(None, {input_name: dummy})[0]
        latencies.append((time.perf_counter() - started) * 1000.0)

    print(f"model      : {model_path} ({model_path.stat().st_size / 1e6:.1f} MB)")
    print(f"input      : {session.get_inputs()[0].shape}")
    print(f"output     : {output.shape}")
    print(f"latency ms : p50={percentile(latencies, 50):.1f}  p95={percentile(latencies, 95):.1f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
