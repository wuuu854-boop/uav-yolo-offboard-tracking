#!/usr/bin/env python3
"""Check a recorded capture against a shot sheet.

For every frame in which an asset is visible, decide whether it satisfies each
required view: resolution from the recorded altitude and the focal length,
framing from the detection box, and attribution to the nearest declared asset.
A view passes with enough good frames; an asset passes with all its views.

    python3 scripts/shot_sheet_check.py --capture /root/capture_north/trial_1 \
        --asset 0,3.5
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

from uav_tracking.control.qa import nearest_target  # noqa: E402
from uav_tracking.control.shot_sheet import (  # noqa: E402
    coverage,
    ground_sample_distance_cm,
    view_rejections,
)
from uav_tracking.perception.color_baseline import detect_color_centroid  # noqa: E402
from uav_tracking.perception.projection import estimate_target_enu  # noqa: E402


def analyse_shots(
    capture: pathlib.Path,
    sheet: dict,
    camera: dict,
    detector: dict,
    targets: list[tuple[float, float]],
) -> dict:
    """Apply the shot sheet to a capture and return counts, rejections, verdict."""
    poses = [
        json.loads(line)
        for line in (capture / "poses.jsonl").read_text(encoding="utf-8").splitlines()
        if line
    ]
    if not poses:
        raise ValueError("empty capture")
    views = sheet.get("views", [])
    if not views:
        raise ValueError("shot sheet has no views")
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

    observations = []
    for pose in poses:
        frame = cv2.imread(str(capture / pose["file"]))
        if frame is None:
            continue
        found = detect_color_centroid(frame, lower, upper, min_area)
        if found is None:
            continue
        height, width = frame.shape[:2]
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
        asset, _distance = nearest_target(east, north, targets)
        gsd = ground_sample_distance_cm(pose["altitude"], float(camera["fx"]))
        bbox = (found[0], found[1], found[3], found[4])
        for view in views:
            if view.get("camera", "down") != "down":
                continue  # this platform has no gimbal
            reasons = view_rejections(
                view, pose["altitude"], float(camera["fx"]), bbox, width, height
            )
            observations.append((asset, view["name"], gsd, reasons))

    if not observations:
        raise ValueError("no detections in the capture")

    result = coverage(observations, views)
    summary = {}
    for asset, per_view in sorted(result["counts"].items()):
        for view in views:
            summary[f"asset{asset}.{view['name']}"] = per_view.get(view["name"], 0)
    return {
        "counts": {str(asset): per_view for asset, per_view in result["counts"].items()},
        "rejected": {
            str(asset): {reason: len(names) for reason, names in reasons.items()}
            for asset, reasons in result["rejected"].items()
        },
        "passed": all(result["passed"].values()) if result["passed"] else False,
        "summary": summary,
    }


def assets_from_args(assets: list[str], config_dir: pathlib.Path) -> list[tuple[float, float]]:
    if assets:
        return [tuple(float(value) for value in asset.split(",")) for asset in assets]
    mission = yaml.safe_load((config_dir / "mission.yaml").read_text(encoding="utf-8"))
    raw = yaml.safe_load((config_dir / "targets.yaml").read_text(encoding="utf-8"))["targets"]
    spawn = mission.get("spawn_offset_enu", (0.0, 0.0))
    return [(float(t["x"]) - float(spawn[0]), float(t["y"]) - float(spawn[1])) for t in raw]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture", required=True)
    parser.add_argument("--sheet", default="config/shot_sheet.yaml")
    parser.add_argument("--asset", action="append", default=[], metavar="EAST,NORTH")
    args = parser.parse_args()

    config_dir = pathlib.Path("config")
    camera = yaml.safe_load((config_dir / "camera.yaml").read_text(encoding="utf-8"))
    detector = yaml.safe_load((config_dir / "detector.yaml").read_text(encoding="utf-8"))
    sheet = yaml.safe_load(pathlib.Path(args.sheet).read_text(encoding="utf-8"))
    targets = assets_from_args(args.asset, config_dir)

    try:
        result = analyse_shots(pathlib.Path(args.capture), sheet, camera, detector, targets)
    except ValueError as error:
        print(error)
        return 2

    for key, count in sorted(result["summary"].items()):
        print(f"{key}: {count} frames")
    for asset, reasons in sorted(result["rejected"].items()):
        for reason, count in sorted(reasons.items()):
            print(f"  asset {asset} rejected: {reason} ({count} frames)")
    print(f"verdict           {'PASS' if result['passed'] else 'FAIL'}")
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
