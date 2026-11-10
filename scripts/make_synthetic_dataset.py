#!/usr/bin/env python3
"""Generate a synthetic nadir-view dataset for fine-tuning the detector.

The Gazebo targets are flat coloured squares seen from directly above, so the
training distribution is that: a grey ground with noise, one or two coloured
squares at a random position, size and small rotation, plus a soft shadow. This
is not a general detector; it is a detector for this scenario, and the point is
the pipeline plus an honest measurement of it.

    python3 scripts/make_synthetic_dataset.py --out data/synthetic --train 500 --val 100
"""

from __future__ import annotations

import argparse
import math
import pathlib
import random

import numpy as np
from PIL import Image, ImageDraw

TARGET_RGB = [
    (204, 13, 13),  # target_0, red
    (13, 179, 26),  # target_1, green
    (26, 51, 204),  # target_2, blue
]


def _jitter(rng: random.Random, colour: tuple[int, int, int]) -> tuple[int, int, int]:
    return tuple(max(0, min(255, channel + rng.randint(-12, 12))) for channel in colour)


def _square(
    centre: tuple[int, int],
    side: int,
    angle_deg: float,
) -> list[tuple[float, float]]:
    half = side / 2.0
    angle = math.radians(angle_deg)
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    corners = []
    for dx, dy in ((-half, -half), (half, -half), (half, half), (-half, half)):
        corners.append(
            (
                centre[0] + dx * cos_a - dy * sin_a,
                centre[1] + dx * sin_a + dy * cos_a,
            )
        )
    return corners


def make_sample(
    rng: random.Random, size: int
) -> tuple[Image.Image, list[tuple[float, float, float, float]]]:
    background = rng.randint(190, 214)
    pixels = np.full((size, size, 3), background, dtype=np.float32)
    pixels += np.random.normal(0.0, 4.0, pixels.shape)
    image = Image.fromarray(np.clip(pixels, 0, 255).astype(np.uint8))
    draw = ImageDraw.Draw(image)

    boxes: list[tuple[float, float, float, float]] = []
    for _ in range(rng.randint(1, 2)):
        side = rng.randint(28, 130)
        margin = side
        cx = rng.randint(margin, size - margin)
        cy = rng.randint(margin, size - margin)
        angle = rng.uniform(-20.0, 20.0)
        colour = _jitter(rng, rng.choice(TARGET_RGB))

        # soft shadow first, offset down-right like a sun overhead-left
        shadow = _square((cx + int(side * 0.14), cy + int(side * 0.16)), side, angle)
        draw.polygon(shadow, fill=(max(0, background - 28),) * 3)

        corners = _square((cx, cy), side, angle)
        draw.polygon(corners, fill=colour)

        # a brighter top face, because the cube is not perfectly flat
        if rng.random() < 0.5:
            inner = _square((cx, cy - 2), int(side * 0.82), angle)
            draw.polygon(inner, fill=_jitter(rng, colour[:3]))

        xs = [corner[0] for corner in corners]
        ys = [corner[1] for corner in corners]
        x_min, x_max = max(0.0, min(xs)), min(float(size), max(xs))
        y_min, y_max = max(0.0, min(ys)), min(float(size), max(ys))
        boxes.append((x_min / size, y_min / size, x_max / size, y_max / size))

    return image, boxes


def write_split(
    root: pathlib.Path,
    split: str,
    count: int,
    size: int,
    seed: int,
) -> None:
    images_dir = root / "images" / split
    labels_dir = root / "labels" / split
    images_dir.mkdir(parents=True, exist_ok=True)
    labels_dir.mkdir(parents=True, exist_ok=True)

    rng = random.Random(seed)
    for index in range(count):
        image, boxes = make_sample(rng, size)
        name = f"{split}_{index:04d}"
        image.save(images_dir / f"{name}.jpg", quality=92)
        lines = ["0 " + " ".join(f"{value:.6f}" for value in box) for box in boxes]
        (labels_dir / f"{name}.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default="data/synthetic")
    parser.add_argument("--train", type=int, default=500)
    parser.add_argument("--val", type=int, default=100)
    parser.add_argument("--size", type=int, default=640)
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args()

    root = pathlib.Path(args.out)
    write_split(root, "train", args.train, args.size, args.seed)
    write_split(root, "val", args.val, args.size, args.seed + 1)

    (root / "dataset.yaml").write_text(
        f"path: {root.resolve()}\ntrain: images/train\nval: images/val\n\nnames:\n  0: target\n",
        encoding="utf-8",
    )
    print(f"wrote {args.train} train / {args.val} val images to {root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
