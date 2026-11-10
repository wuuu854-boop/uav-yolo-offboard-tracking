#!/usr/bin/env python3
"""Run the detector and the tracker over a recorded frame sequence.

This is the offline half of a moving-target test: the flight records frames and
poses, and this replays the detector plus ``IouTracker`` over them. What it
answers is whether the id survives a moving box and whether the estimated
velocity points where the target actually went.

    python3 scripts/track_sequence.py --capture /root/capture_moving --stride 1
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
from uav_tracking.perception.tracking import IouTracker  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture", required=True)
    parser.add_argument("--config", default="config")
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--fps", type=float, default=3.0, help="capture rate, for the time axis")
    args = parser.parse_args()

    capture = pathlib.Path(args.capture)
    poses = [
        json.loads(line)
        for line in (capture / "poses.jsonl").read_text(encoding="utf-8").splitlines()
        if line
    ]
    detector = yaml.safe_load(
        (pathlib.Path(args.config) / "detector.yaml").read_text(encoding="utf-8")
    )
    ranges = detector.get("baseline_hsv_ranges") or [
        [detector["baseline_hsv_lower"], detector["baseline_hsv_upper"]]
    ]
    lower = [tuple(item[0]) for item in ranges]
    upper = [tuple(item[1]) for item in ranges]
    min_area = int(detector.get("baseline_min_area_px", 150))

    tracker = IouTracker()
    ids: list[int] = []
    print(f"{'frame':>10} {'id':>4} {'cx':>7} {'cy':>7} {'vx px/s':>9} {'vy px/s':>9}")
    for index, pose in enumerate(poses[:: args.stride]):
        frame = cv2.imread(str(capture / pose["file"]))
        if frame is None:
            continue
        found = detect_color_centroid(frame, lower, upper, min_area)
        detections = []
        if found is not None:
            detections.append(
                {"bbox_cx": found[0], "bbox_cy": found[1], "bbox_w": found[3], "bbox_h": found[4]}
            )
        annotated = tracker.update(detections, index / args.fps)
        for detection in annotated:
            ids.append(detection["track_id"])
            print(
                f"{pose['file']:>10} {detection['track_id']:4d} {detection['bbox_cx']:7.1f}"
                f" {detection['bbox_cy']:7.1f}"
            )
        if annotated:
            track = tracker.active[0]
            print(f"{'':>10} {'':>4} {'':>7} {'':>7} {track.vx:9.1f} {track.vy:9.1f}")

    if ids:
        print(f"\n{len(ids)} detections, distinct ids: {sorted(set(ids))}")
    else:
        print("\nno detections in the sequence")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
