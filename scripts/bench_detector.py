#!/usr/bin/env python3
"""Measure detector latency on real frames, per input size and thread count.

The number that matters for a companion computer is not "does it detect" but
"how many milliseconds, on how many threads, at what input size". This runs the
same recorded frames through the same decode path the node uses.

    python3 scripts/bench_detector.py --images ~/capture --model models/yolo11n.onnx \
        --imgsz 640 320 --threads 1 2 4 8
"""

from __future__ import annotations

import argparse
import pathlib
import sys
import time

import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from uav_tracking.perception.yolo import YoloOnnxDetector  # noqa: E402


def summarise(latencies: list[float]) -> tuple[float, float, float]:
    """Return (p50, p95, fps) for a list of millisecond timings."""
    if not latencies:
        raise ValueError("no samples")
    array = np.asarray(latencies, dtype=float)
    p50 = float(np.percentile(array, 50))
    p95 = float(np.percentile(array, 95))
    # Throughput is the mean over the run, not 1000/p50: a single slow frame is
    # exactly what a real-time budget cares about.
    fps = 1000.0 / float(array.mean())
    return p50, p95, fps


def bench(frames: list[np.ndarray], detector: YoloOnnxDetector, warmup: int = 3) -> list[float]:
    for frame in frames[:warmup]:
        detector.infer(frame)
    latencies = []
    for frame in frames:
        started = time.perf_counter()
        detector.infer(frame)
        latencies.append((time.perf_counter() - started) * 1000.0)
    return latencies


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--images", required=True, help="folder of recorded frames")
    parser.add_argument("--model", default="models/yolo11n.onnx")
    parser.add_argument("--imgsz", type=int, nargs="+", default=[640])
    parser.add_argument("--threads", type=int, nargs="+", default=[0])
    parser.add_argument("--limit", type=int, default=120, help="max frames to use")
    args = parser.parse_args()

    import cv2  # noqa: PLC0415 - bench-only dependency

    folder = pathlib.Path(args.images)
    paths = sorted(folder.rglob("*.jpg")) + sorted(folder.rglob("*.png"))
    paths = paths[: args.limit]
    if not paths:
        print(f"no frames under {folder}")
        return 2
    frames = [cv2.imread(str(path)) for path in paths]
    frames = [frame for frame in frames if frame is not None]
    print(f"{len(frames)} frames, {frames[0].shape[1]}x{frames[0].shape[0]}")

    print(f"{'imgsz':>6} {'threads':>8} {'p50 ms':>8} {'p95 ms':>8} {'fps':>7}")
    for imgsz in args.imgsz:
        for threads in args.threads:
            detector = YoloOnnxDetector(
                pathlib.Path(args.model),
                input_size=imgsz,
                classes=(0,),
                intra_op_threads=threads,
            )
            p50, p95, fps = summarise(bench(frames, detector))
            label = "auto" if threads == 0 else str(threads)
            print(f"{imgsz:6d} {label:>8} {p50:8.1f} {p95:8.1f} {fps:7.1f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
