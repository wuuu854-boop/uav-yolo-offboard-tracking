#!/usr/bin/env python3
"""A quality report over a recorded flight, not over the code.

Utility inspection teams spend days per flight deciding whether the data is
usable, and the failures they name are measurable: detection coverage, gaps,
track continuity, and how far each detection lands from the asset it claims.
This runs those checks over a capture directory and exits non-zero when the run
fails its thresholds, so it can gate a pipeline.

Coverage is reported per flight phase, because takeoff and landing frames drag a
whole-flight average down without telling anyone anything. The asset position is
an input, not an assumption: a moved target must not look like a broken
projection.

    python3 scripts/qa_report.py --capture /root/capture/trial_1 \
        --asset 3.5,0 --out results/qa.json
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
    association_confidence,
    longest_gap,
    nearest_target,
    percentiles,
    summarise_phases,
    verdict,
)
from uav_tracking.perception.color_baseline import detect_color_centroid  # noqa: E402
from uav_tracking.perception.projection import estimate_target_enu  # noqa: E402
from uav_tracking.perception.tracking import IouTracker  # noqa: E402


def analyse_capture(
    capture: pathlib.Path,
    camera: dict,
    detector: dict,
    targets: list[tuple[float, float]],
) -> dict:
    """Run the four checks over a capture and return the report body.

    Kept free of argparse so the CI regression test can call it on a generated
    capture: the metrics are worth protecting, not just the pure helpers.
    """
    poses = [
        json.loads(line)
        for line in (capture / "poses.jsonl").read_text(encoding="utf-8").splitlines()
        if line
    ]
    if not poses:
        raise ValueError("empty capture")
    if not targets:
        raise ValueError("at least one asset position is required")

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
    samples: list[tuple[float, float, bool]] = []
    association_errors: list[float] = []
    confidences: list[float] = []
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
        now_s = pose["wall_time"] - started
        annotated = tracker.update(detections, now_s)
        samples.append((now_s, pose["altitude"], bool(annotated)))
        hits = {track.track_id: track.hits for track in tracker.active}
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
            confidences.append(
                association_confidence(
                    distance,
                    hits.get(detection["track_id"], 1),
                    sigma_m=DEFAULT_THRESHOLDS["confidence_sigma_m"],
                )
            )

    total_s = poses[-1]["wall_time"] - started
    phases = summarise_phases(samples)
    level = phases.get(
        "level", {"frames": 0, "detections": 0, "detection_rate": 0.0, "detection_times": []}
    )
    distinct_ids = sorted(set(ids))
    association_summary = percentiles(association_errors)
    confidence_summary = percentiles(confidences)

    # The verdict is about the survey phase: that is when the camera should be
    # seeing the asset. Ground and climb frames are reported, not judged.
    checked = {
        "detection_rate": level["detection_rate"],
        "longest_gap_s": round(longest_gap(level["detection_times"], total_s), 3),
        "association_m": association_summary,
    }
    passed, reasons = verdict(checked, DEFAULT_THRESHOLDS)
    metrics = {
        "frames": len(poses),
        "detections": len(ids),
        "detection_rate": len([s for s in samples if s[2]]) / len(samples) if samples else 0.0,
        "level_detection_rate": level["detection_rate"],
        "longest_gap_s": checked["longest_gap_s"],
        "track_ids": len(distinct_ids),
        "longest_track_run": max((ids.count(track_id) for track_id in distinct_ids), default=0),
        "association_m": association_summary,
        "association_confidence": confidence_summary,
        "weak_associations": sum(1 for value in confidences if value < 0.5),
        "assets_seen": sorted(set(associations)),
        "phases": {
            phase: {"frames": entry["frames"], "detection_rate": entry["detection_rate"]}
            for phase, entry in phases.items()
        },
    }
    return {
        "capture": str(capture),
        "assets": targets,
        "thresholds": DEFAULT_THRESHOLDS,
        "metrics": metrics,
        "passed": passed,
        "failures": reasons,
    }


def _assets_from_args(assets: list[str], config_dir: pathlib.Path) -> list[tuple[float, float]]:
    if assets:
        return [tuple(float(value) for value in asset.split(",")) for asset in assets]
    mission = yaml.safe_load((config_dir / "mission.yaml").read_text(encoding="utf-8"))
    raw = yaml.safe_load((config_dir / "targets.yaml").read_text(encoding="utf-8"))["targets"]
    spawn = mission.get("spawn_offset_enu", (0.0, 0.0))
    return [(float(t["x"]) - float(spawn[0]), float(t["y"]) - float(spawn[1])) for t in raw]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture", required=True)
    parser.add_argument("--config", default="config")
    parser.add_argument(
        "--asset",
        action="append",
        default=[],
        metavar="EAST,NORTH",
        help="asset position in the local frame, repeatable; defaults to targets.yaml",
    )
    parser.add_argument("--out", default="")
    args = parser.parse_args()

    config_dir = pathlib.Path(args.config)
    camera = yaml.safe_load((config_dir / "camera.yaml").read_text(encoding="utf-8"))
    detector = yaml.safe_load((config_dir / "detector.yaml").read_text(encoding="utf-8"))
    targets = _assets_from_args(args.asset, config_dir)
    if any(len(target) != 2 for target in targets):
        print("each --asset needs two values: east,north")
        return 2

    try:
        report = analyse_capture(pathlib.Path(args.capture), camera, detector, targets)
    except ValueError as error:
        print(error)
        return 2

    if args.out:
        pathlib.Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        pathlib.Path(args.out).write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    metrics = report["metrics"]
    print(f"frames            {metrics['frames']} ({metrics['detections']} detections)")
    for phase, entry in sorted(metrics["phases"].items()):
        print(
            f"  {phase:<8}        {entry['frames']:>4} frames, {entry['detection_rate']:.0%} detected"
        )
    print(f"level gap         {metrics['longest_gap_s']:.2f} s")
    print(f"track ids         {metrics['track_ids']} (longest run {metrics['longest_track_run']})")
    print(
        f"association       median {metrics['association_m']['median']:.2f} m, "
        f"p95 {metrics['association_m']['p95']:.2f} m"
    )
    print(
        f"confidence        median {metrics['association_confidence']['median']:.2f}"
        f", {metrics['weak_associations']} detections below 0.5"
    )
    print(
        "verdict           "
        + ("PASS" if report["passed"] else "FAIL: " + "; ".join(report["failures"]))
    )
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
