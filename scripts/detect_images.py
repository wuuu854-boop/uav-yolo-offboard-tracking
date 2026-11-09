#!/usr/bin/env python3
"""Run the detector over a folder of images and write a CSV.

This is the part of the pipeline that is useful without a simulator: point it at
frames you already have (a flight, a dataset, a screen recording) and get per
image detections and latency, with the same decoding code the node uses.

    python3 scripts/detect_images.py --images ~/frames --backend baseline
    python3 scripts/detect_images.py --images ~/frames --backend yolo --model models/yolo11n.onnx

The baseline backend needs no model, which is why it is the default: it is the
fastest way to check that images load, decode and write before involving ONNX.
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from collections.abc import Callable, Sequence
from pathlib import Path

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from uav_tracking.perception.color_baseline import detect_color_centroid  # noqa: E402

IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp"}


def list_images(folder: Path) -> list[Path]:
    return sorted(path for path in folder.rglob("*") if path.suffix.lower() in IMAGE_SUFFIXES)


def evaluate(
    images: Sequence[Path],
    detect: Callable[[np.ndarray], list[dict]],
) -> list[dict]:
    """Run ``detect`` over each image, returning one row per image."""
    rows: list[dict] = []
    for path in images:
        frame = cv2.imread(str(path))
        if frame is None:
            rows.append(
                {"image": path.name, "detections": -1, "latency_ms": "", "cx": "", "cy": ""}
            )
            continue

        started = time.perf_counter()
        detections = detect(frame)
        latency_ms = (time.perf_counter() - started) * 1000.0

        best = detections[0] if detections else None
        rows.append(
            {
                "image": path.name,
                "detections": len(detections),
                "latency_ms": round(latency_ms, 2),
                "cx": round(best["bbox_cx"], 1) if best else "",
                "cy": round(best["bbox_cy"], 1) if best else "",
            }
        )
    return rows


def write_csv(path: Path, rows: Sequence[dict]) -> None:
    if not rows:
        raise ValueError("nothing to write")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def summarise(rows: Sequence[dict]) -> str:
    ok = [row for row in rows if row["detections"] >= 0]
    with_detection = [row for row in ok if row["detections"] > 0]
    latencies = [row["latency_ms"] for row in ok if row["latency_ms"] != ""]
    if not ok:
        return "no readable images"
    mean = sum(latencies) / len(latencies) if latencies else float("nan")
    return (
        f"{len(ok)} images, {len(with_detection)} with a detection "
        f"({len(with_detection) / len(ok):.0%}), mean {mean:.1f} ms"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--images", required=True, help="folder to scan recursively")
    parser.add_argument("--backend", choices=("baseline", "yolo"), default="baseline")
    parser.add_argument("--model", default="models/yolo11n.onnx")
    parser.add_argument("--out", default="results/detect_images.csv")
    parser.add_argument("--limit", type=int, default=0, help="0 means every image")
    args = parser.parse_args()

    folder = Path(args.images).expanduser()
    if not folder.is_dir():
        print(f"not a folder: {folder}")
        return 2

    images = list_images(folder)
    if args.limit:
        images = images[: args.limit]
    if not images:
        print(f"no images under {folder}")
        return 2

    if args.backend == "baseline":
        detect = _baseline_detector()
    else:
        detect = _yolo_detector(Path(args.model))

    rows = evaluate(images, detect)
    out = REPO_ROOT / args.out
    write_csv(out, rows)

    print(summarise(rows))
    print(f"written: {out.relative_to(REPO_ROOT)}")
    return 0


def _baseline_detector() -> Callable[[np.ndarray], list[dict]]:
    def detect(frame: np.ndarray) -> list[dict]:
        found = detect_color_centroid(frame, (35, 80, 80), (85, 255, 255), 150)
        if found is None:
            return []
        u, v, _area, width, height = found
        return [{"confidence": 1.0, "bbox_cx": u, "bbox_cy": v, "bbox_w": width, "bbox_h": height}]

    return detect


def _yolo_detector(model_path: Path) -> Callable[[np.ndarray], list[dict]]:
    from uav_tracking.perception.detector_node import YoloOnnxDetector  # noqa: PLC0415 - optional dependency

    detector = YoloOnnxDetector(model_path if model_path.is_absolute() else REPO_ROOT / model_path)
    return detector.infer


if __name__ == "__main__":
    raise SystemExit(main())
