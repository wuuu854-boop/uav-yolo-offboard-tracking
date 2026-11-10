#!/usr/bin/env python3
"""A quality report over a recorded flight, not over the code.

Utility inspection teams spend days per flight deciding whether the data is
usable, and the failures they name are measurable: detection coverage, gaps,
track continuity, and how far each detection lands from the asset it claims.
This runs those checks over a capture directory and exits non-zero when the run
fails its thresholds, so it can gate a pipeline.

    python3 scripts/qa_report.py --capture /root/capture/trial_1 --out results/qa.json
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

from uav_tracking.control.qa import (  # noqa: E402
    DEFAULT_THRESHOLDS,
    longest_gap,
    nearest_target,
    percentiles,
    verdict,
)
from uav_tracking.perception.color_baseline import detect_color_centroid  # noqa: E402
from uav_tracking.perception.projection import estimate_target_enu  # noqa: E402
from uav_tracking.perception.tracking import IouTracker  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture", required=True)
    parser.add_argument("--config", default="config")
    parser.add_argument("--out", default="")
    args = parser.parse_args()

    capture = pathlib.Path(args.capture)
    poses = [
        json.loads(line)
        for line in (capture / "poses.jsonl").read_text(encoding="utf-8").splitlines()
        if line
    ]
    if not poses:
        print("empty capture")
        return 2

    config_dir = pathlib.Path(args.config)
    camera = yaml.safe_load((config_dir / "camera.yaml").read_text(encoding="utf-8"))
    detector = yaml.safe_load((config_dir / "detector.yaml").read_text(encoding="utf-8"))
    mission = yaml.safe_load((config_dir / "mission.yaml").read_text(encoding="utf-8"))
    targets_raw = yaml.safe_load((config_dir / "targets.yaml").read_text(encoding="utf-8"))[
        "targets"
    ]
    spawn = mission.get("spawn_offset_enu", (0.0, 0.0))
    targets = [
        (float(t["x"]) - float(spawn[0]), float(t["y"]) - float(spawn[1])) for t in targets_raw
    ]

    ranges = detector.get("baseline_hsv_ranges") or [
        [detector["baseline_hsv_lower"], detector["baseline_hsv_upper"]]
    ]
    lower = [tuple(item[0]) for item in ranges]
    upper = [tuple(item[1]) for item in ranges]
    min_area = int(detector.get("baseline_min_area_px", 150))
    K = ((camera["fx"], 0.0, camera["cx"]), (0.0, camera["fy"], camera["cy"]), (0.0, 0.0, 1.0))
    offset = tuple(camera.get("mount_offset_xyz", (0.0, 0.0, 0.0)))
    up_is_forward = bool(camera.get("image_up_is_forward", True))

    tracker = IouTracker()
    started = poses[0]["wall_time"]
    detected_times: list[float] = []
    association_errors: list[float] = []
    associations: list[int] = []
    ids: list[int] = []

    for pose in poses:
        frame = cv2.imread(str(capture / pose["file"]))
        if frame is None:
            continue
        found = detect_color_centroid(frame, lower, upper, min_area)
        detections = []
        if found is not None:
            detections.append(
                {"bbox_cx": found[0], "bbox_cy": found[1], "bbox_w": found[3], "bbox_h": found[4]}
            )
        annotated = tracker.update(detections, pose["wall_time"] - started)
        if annotated:
            detected_times.append(pose["wall_time"] - started)
        for detection in annotated:
            ids.append(detection["track_id"])
            east, north, _ = estimate_target_enu(
                (pose["east"], pose["north"]),
                detection["bbox_cx"],
                detection["bbox_cy"],
                pose["altitude"],
                K,
                yaw=pose["yaw"],
                image_up_is_forward=up_is_forward,
                offset_body=offset,
                roll=pose.get("roll", 0.0),
                pitch=pose.get("pitch", 0.0),
            )
            index, distance = nearest_target(east, north, targets)
            associations.append(index)
            association_errors.append(distance)

    total_s = poses[-1]["wall_time"] - started
    distinct_ids = sorted(set(ids))
    metrics = {
        "frames": len(poses),
        "detections": len(ids),
        "detection_rate": len(detected_times) / len(poses),
        "longest_gap_s": round(longest_gap(detected_times, total_s), 3),
        "track_ids": len(distinct_ids),
        "longest_track_run": max((ids.count(track_id) for track_id in distinct_ids), default=0),
        "association_m": percentiles(association_errors),
        "assets_seen": sorted(set(associations)),
    }
    passed, reasons = verdict(metrics, DEFAULT_THRESHOLDS)
    report = {
        "capture": str(capture),
        "thresholds": DEFAULT_THRESHOLDS,
        "metrics": metrics,
        "passed": passed,
        "failures": reasons,
    }

    if args.out:
        pathlib.Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        pathlib.Path(args.out).write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    print(f"frames            {metrics['frames']}")
    print(f"detections        {metrics['detections']} ({metrics['detection_rate']:.0%})")
    print(f"longest gap       {metrics['longest_gap_s']:.2f} s of {total_s:.1f} s")
    print(f"track ids         {metrics['track_ids']} (longest run {metrics['longest_track_run']})")
    print(
        f"association       median {metrics['association_m']['median']:.2f} m, "
        f"p95 {metrics['association_m']['p95']:.2f} m"
    )
    print(f"verdict           {'PASS' if passed else 'FAIL: ' + '; '.join(reasons)}")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
