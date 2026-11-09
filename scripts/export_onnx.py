#!/usr/bin/env python3
"""Export YOLO11n to ONNX and verify the exported graph.

Training dependencies are not installed by default (they are AGPL-3.0), so this
script fails with a clear message instead of being imported by the test suite.

    pip install -r requirements-train.txt
    python3 scripts/export_onnx.py --weights yolo11n.pt --imgsz 640
"""

from __future__ import annotations

import argparse
import shutil
import time
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--weights", default="yolo11n.pt")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--opset", type=int, default=12)
    parser.add_argument("--out", default="models/yolo11n.onnx")
    args = parser.parse_args()

    try:
        from ultralytics import YOLO
    except ImportError:
        print("ultralytics is not installed. Run:")
        print("  pip install -r requirements-train.txt")
        return 1

    model = YOLO(args.weights)
    exported = Path(model.export(format="onnx", imgsz=args.imgsz, opset=args.opset, simplify=True))

    target = REPO_ROOT / args.out
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.move(str(exported), target)
    print(f"exported -> {target} ({target.stat().st_size / 1e6:.1f} MB)")

    # Verify the exported graph with ONNX Runtime only (no ultralytics needed later).
    import onnxruntime as ort

    session = ort.InferenceSession(str(target), providers=["CPUExecutionProvider"])
    dummy = np.random.rand(1, 3, args.imgsz, args.imgsz).astype(np.float32)
    session.run(None, {session.get_inputs()[0].name: dummy})  # warm-up

    runs = 20
    started = time.perf_counter()
    for _ in range(runs):
        output = session.run(None, {session.get_inputs()[0].name: dummy})[0]
    mean_ms = (time.perf_counter() - started) / runs * 1000.0

    print(f"output shape: {output.shape}   mean latency: {mean_ms:.1f} ms (CPU)")
    print("next: python3 scripts/verify_model.py")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
