#!/usr/bin/env python3
"""Compare the two detectors on identical frames with identical poses.

For every captured frame both backends run, each detection is projected to the
ground with the recorded pose, and the distance to the known target is printed.
A constant delta between the two projections is a systematic offset; a varying
one is noise.

    python3 scripts/compare_backends.py --capture /root/capture --target 3.5,0
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import sys

import cv2
import numpy as np
import yaml

REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from uav_tracking.perception.color_baseline import detect_color_centroid  # noqa: E402
from uav_tracking.perception.projection import estimate_target_enu  # noqa: E402
from uav_tracking.perception.yolo import YoloOnnxDetector  # noqa: E402


def _ranges(config: dict) -> tuple[list, list]:
    ranges = config.get("baseline_hsv_ranges")
    if ranges:
        return [tuple(item[0]) for item in ranges], [tuple(item[1]) for item in ranges]
    return [tuple(config["baseline_hsv_lower"])], [tuple(config["baseline_hsv_upper"])]


def _project(box: dict, pose: dict, camera: dict) -> tuple[float, float]:
    east, north, _ = estimate_target_enu(
        (pose["east"], pose["north"]),
        box["bbox_cx"],
        box["bbox_cy"],
        pose["altitude"],
        ((camera["fx"], 0.0, camera["cx"]), (0.0, camera["fy"], camera["cy"]), (0.0, 0.0, 1.0)),
        yaw=pose["yaw"],
        image_up_is_forward=bool(camera.get("image_up_is_forward", True)),
        offset_body=camera.get("mount_offset_xyz", (0.0, 0.0, 0.0)),
    )
    return east, north


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture", default="/root/capture")
    parser.add_argument(
        "--target", default="3.5,0", help="target position in the local frame (east,north)"
    )
    parser.add_argument("--model", default="models/yolo11n.onnx")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--config", default="config")
    args = parser.parse_args()

    capture = pathlib.Path(args.capture)
    poses = [
        json.loads(line)
        for line in (capture / "poses.jsonl").read_text(encoding="utf-8").splitlines()
        if line
    ]
    camera = yaml.safe_load((pathlib.Path(args.config) / "camera.yaml").read_text(encoding="utf-8"))
    detector_cfg = yaml.safe_load(
        (pathlib.Path(args.config) / "detector.yaml").read_text(encoding="utf-8")
    )
    lower, upper = _ranges(detector_cfg)
    min_area = int(detector_cfg.get("baseline_min_area_px", 150))

    yolo = YoloOnnxDetector(pathlib.Path(args.model), input_size=args.imgsz, classes=(0,))
    target = tuple(float(value) for value in args.target.split(","))

    print(f"{'frame':>10} {'baseline err':>13} {'yolo err':>9} {'delta e':>8} {'delta n':>8}")
    deltas = []
    for pose in poses:
        frame = cv2.imread(str(capture / pose["file"]))
        if frame is None:
            continue

        found = detect_color_centroid(frame, lower, upper, min_area)
        yolo_boxes = yolo.infer(frame)
        if found is None or not yolo_boxes:
            continue

        baseline_box = {"bbox_cx": found[0], "bbox_cy": found[1]}
        base_east, base_north = _project(baseline_box, pose, camera)
        yolo_east, yolo_north = _project(yolo_boxes[0], pose, camera)

        base_err = math.hypot(base_east - target[0], base_north - target[1])
        yolo_err = math.hypot(yolo_east - target[0], yolo_north - target[1])
        delta_east, delta_north = yolo_east - base_east, yolo_north - base_north
        deltas.append((delta_east, delta_north))
        print(
            f"{pose['file']:>10} {base_err:13.2f} {yolo_err:9.2f} {delta_east:8.2f} {delta_north:8.2f}"
        )

    if deltas:
        array = np.asarray(deltas)
        print(
            f"\n{len(deltas)} comparable frames; mean delta "
            f"({array[:, 0].mean():+.2f}, {array[:, 1].mean():+.2f}) m, "
            f"std ({array[:, 0].std():.2f}, {array[:, 1].std():.2f}) m"
        )
    else:
        print("no frame had detections from both backends")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
