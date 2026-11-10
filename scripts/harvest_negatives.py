#!/usr/bin/env python3
"""Harvest hard negatives from the demo GIF: real Gazebo frames with no target.

The YOLO model fires on the vehicle's own shadow because the synthetic dataset
only ever showed shadows attached to a square. These frames are the deployment
domain and contain no target (the colour baseline finds nothing in them), so
they are honest background examples.

    python3 scripts/harvest_negatives.py --gif assets/01-demo.gif --dataset /root/synthetic_dataset
"""

from __future__ import annotations

import argparse
import pathlib

import cv2
import numpy as np

from uav_tracking.perception.color_baseline import detect_color_centroid

RANGES = [
    ((0, 120, 70), (10, 255, 255)),
    ((170, 120, 70), (180, 255, 255)),
    ((40, 120, 60), (85, 255, 255)),
    ((100, 120, 60), (140, 255, 255)),
]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gif", default="assets/01-demo.gif")
    parser.add_argument("--dataset", default="/root/synthetic_dataset")
    parser.add_argument("--prefix", default="neg_gif")
    args = parser.parse_args()

    from PIL import Image

    image = Image.open(args.gif)
    total = getattr(image, "n_frames", 1)

    dataset = pathlib.Path(args.dataset)
    images_dir = dataset / "images" / "train"
    labels_dir = dataset / "labels" / "train"
    images_dir.mkdir(parents=True, exist_ok=True)
    labels_dir.mkdir(parents=True, exist_ok=True)

    lower = [item[0] for item in RANGES]
    upper = [item[1] for item in RANGES]

    kept = 0
    skipped = 0
    for index in range(total):
        image.seek(index)
        frame = cv2.cvtColor(np.asarray(image.convert("RGB")), cv2.COLOR_RGB2BGR)
        if detect_color_centroid(frame, lower, upper, 150) is not None:
            skipped += 1  # the target is in this frame: not a negative
            continue

        name = f"{args.prefix}_{index:04d}"
        cv2.imwrite(str(images_dir / f"{name}.jpg"), frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
        (labels_dir / f"{name}.txt").write_text("", encoding="utf-8")
        kept += 1

    print(f"{total} frames, {kept} negatives added, {skipped} skipped (target visible)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
