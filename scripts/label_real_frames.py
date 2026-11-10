#!/usr/bin/env python3
"""Label real simulator frames using the baseline detector and known geometry.

The YOLO model trained on synthetic squares predicts boxes half the frame wide on
live 1280x960 frames (docs/model.md). The fix is real-domain supervision, and the
labels can be produced without hand annotation because the geometry is known:

1. the colour baseline finds the cube in the frame,
2. its pixel projects to a ground point with the recorded pose,
3. the cube's eight known corners at that point project back to a tight box.

The box is only as good as step 1, so a frame is skipped when the projection is
implausible (box too small, too large, or outside the image).

    python3 scripts/label_real_frames.py --capture /root/capture_att/trial_1 \
        --target 3.5,0 --out /root/real_dataset
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys

import cv2
import yaml

REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from uav_tracking.perception.color_baseline import detect_color_centroid  # noqa: E402
from uav_tracking.perception.projection import (  # noqa: E402
    estimate_target_enu,
    ground_point_to_pixel,
)

CUBE_SIZE_M = 0.3


def label_capture(
    capture: pathlib.Path,
    target: tuple[float, float],
    camera: dict,
    ranges: list[list[list[int]]],
    min_area: int,
    out_images: pathlib.Path,
    out_labels: pathlib.Path,
    prefix: str,
) -> tuple[int, int]:
    poses = [
        json.loads(line)
        for line in (capture / "poses.jsonl").read_text(encoding="utf-8").splitlines()
        if line
    ]
    K = ((camera["fx"], 0.0, camera["cx"]), (0.0, camera["fy"], camera["cy"]), (0.0, 0.0, 1.0))
    lower = [tuple(item[0]) for item in ranges]
    upper = [tuple(item[1]) for item in ranges]
    offset = tuple(camera.get("mount_offset_xyz", (0.0, 0.0, 0.0)))
    up_is_forward = bool(camera.get("image_up_is_forward", True))

    written = skipped = 0
    for pose in poses:
        frame = cv2.imread(str(capture / pose["file"]))
        if frame is None:
            continue
        found = detect_color_centroid(frame, lower, upper, min_area)
        if found is None:
            skipped += 1
            continue

        east, north, _ = estimate_target_enu(
            (pose["east"], pose["north"]),
            found[0],
            found[1],
            pose["altitude"],
            K,
            yaw=pose["yaw"],
            image_up_is_forward=up_is_forward,
            offset_body=offset,
            roll=pose.get("roll", 0.0),
            pitch=pose.get("pitch", 0.0),
        )

        half = CUBE_SIZE_M / 2.0
        corners = []
        for dx in (-half, half):
            for dy in (-half, half):
                for z in (0.0, CUBE_SIZE_M):
                    try:
                        u, v = ground_point_to_pixel(
                            (east + dx, north + dy, z),
                            (pose["east"], pose["north"]),
                            pose["altitude"],
                            K,
                            yaw=pose["yaw"],
                            roll=pose.get("roll", 0.0),
                            pitch=pose.get("pitch", 0.0),
                            image_up_is_forward=up_is_forward,
                            offset_body=offset,
                        )
                    except ValueError:
                        corners = []
                        break
                    corners.append((u, v))
                if not corners:
                    break
            if not corners:
                break

        if not corners:
            skipped += 1
            continue

        xs = [c[0] for c in corners]
        ys = [c[1] for c in corners]
        height, width = frame.shape[:2]
        x_min, x_max = max(0.0, min(xs)), min(float(width), max(xs))
        y_min, y_max = max(0.0, min(ys)), min(float(height), max(ys))
        box_w, box_h = x_max - x_min, y_max - y_min
        # Reject the labels that cannot be trusted: a 0.3 m cube at 5 m is about
        # 32 px, so anything wildly off means the detection or the pose was bad.
        if box_w < 8 or box_h < 8 or box_w > 200 or box_h > 200:
            skipped += 1
            continue

        name = f"{prefix}_{pose['file'].replace('.jpg', '')}"
        cv2.imwrite(str(out_images / f"{name}.jpg"), frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
        (out_labels / f"{name}.txt").write_text(
            "0 "
            f"{(x_min + x_max) / 2 / width:.6f} {(y_min + y_max) / 2 / height:.6f} "
            f"{box_w / width:.6f} {box_h / height:.6f}\n",
            encoding="utf-8",
        )
        written += 1

    return written, skipped


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--capture", action="append", required=True, help="capture directory, repeatable"
    )
    parser.add_argument(
        "--target", default="3.5,0", help="cube position in the local frame (east,north)"
    )
    parser.add_argument("--out", default="/root/real_dataset")
    parser.add_argument("--split", default="train", choices=("train", "val"))
    parser.add_argument("--config", default="config")
    args = parser.parse_args()

    config_dir = pathlib.Path(args.config)
    camera = yaml.safe_load((config_dir / "camera.yaml").read_text(encoding="utf-8"))
    detector = yaml.safe_load((config_dir / "detector.yaml").read_text(encoding="utf-8"))
    ranges = detector.get("baseline_hsv_ranges") or [
        [detector["baseline_hsv_lower"], detector["baseline_hsv_upper"]]
    ]
    min_area = int(detector.get("baseline_min_area_px", 150))

    out = pathlib.Path(args.out)
    images = out / "images" / args.split
    labels = out / "labels" / args.split
    images.mkdir(parents=True, exist_ok=True)
    labels.mkdir(parents=True, exist_ok=True)

    target = tuple(float(value) for value in args.target.split(","))
    total_written = total_skipped = 0
    for index, capture in enumerate(args.capture):
        written, skipped = label_capture(
            pathlib.Path(capture),
            target,
            camera,
            ranges,
            min_area,
            images,
            labels,
            f"{args.split}{index}",
        )
        print(f"{capture}: {written} labelled, {skipped} skipped")
        total_written += written
        total_skipped += skipped

    (out / "dataset.yaml").write_text(
        f"path: {out.resolve()}\ntrain: images/train\nval: images/val\n\nnames:\n  0: target\n",
        encoding="utf-8",
    )
    print(f"total: {total_written} labelled, {total_skipped} skipped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
