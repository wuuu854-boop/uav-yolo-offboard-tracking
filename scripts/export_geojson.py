#!/usr/bin/env python3
"""Export detections as GeoJSON: the structured deliverable an operator asks for.

Utility inspection teams reject folders of imagery, not imagery: they want a
feature layer with an asset association, a timestamp and a confidence, ready for
GIS. This turns a recorded flight into that layer — every detection is projected
to the ground with the recorded pose, given a track id, and written as a Point
Feature.

Coordinates are WGS84 and need an origin: the simulator has no real location, so
the origin is whatever you pass to ``--origin-lat/--origin-lon``. The local ENU
offset is kept in the properties so nothing is lost in the conversion.

    python3 scripts/export_geojson.py --capture /root/capture --origin-lat 52.52 \
        --origin-lon 13.40 --out results/detections.geojson
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import sys

import cv2
import yaml

REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from uav_tracking.control.qa import association_confidence, nearest_target  # noqa: E402
from uav_tracking.perception.color_baseline import detect_color_centroid  # noqa: E402
from uav_tracking.perception.projection import estimate_target_enu  # noqa: E402
from uav_tracking.perception.tracking import IouTracker  # noqa: E402

METRES_PER_DEGREE_LAT = 111_320.0


def enu_to_lonlat(
    east: float, north: float, origin_lat: float, origin_lon: float
) -> tuple[float, float]:
    """Local ENU metres -> (lon, lat), equirectangular around the origin.

    Good to centimetres over a few kilometres, which is the range a nadir flight
    covers. A geodesy library would be overkill for a local, planar projection.
    """
    if not -90.0 <= origin_lat <= 90.0:
        raise ValueError("origin_lat must be in [-90, 90]")
    lat = origin_lat + north / METRES_PER_DEGREE_LAT
    lon = origin_lon + east / (METRES_PER_DEGREE_LAT * math.cos(math.radians(origin_lat)))
    return lon, lat


def build_features(
    capture: pathlib.Path,
    camera: dict,
    detector: dict,
    targets: list[tuple[float, float]],
    origin_lat: float,
    origin_lon: float,
) -> list[dict]:
    """GeoJSON features for every detection in a capture."""
    poses = [
        json.loads(line)
        for line in (capture / "poses.jsonl").read_text(encoding="utf-8").splitlines()
        if line
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
    features = []
    started = poses[0]["wall_time"]
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
        hits = {track.track_id: track.hits for track in tracker.active}
        for detection in annotated:
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
            lon, lat = enu_to_lonlat(east, north, origin_lat, origin_lon)
            properties = {
                "frame": pose["file"],
                "t_s": round(pose["wall_time"] - started, 3),
                "track_id": detection["track_id"],
                "confidence": 1.0,
                "detector": "colour_baseline",
                "east_m": round(east, 3),
                "north_m": round(north, 3),
                "drone_alt_m": round(pose["altitude"], 3),
            }
            if targets:
                asset, distance = nearest_target(east, north, targets)
                properties["asset"] = asset
                properties["association_m"] = round(distance, 3)
                properties["association_confidence"] = round(
                    association_confidence(distance, hits.get(detection["track_id"], 1)), 3
                )
            features.append(
                {
                    "type": "Feature",
                    "geometry": {"type": "Point", "coordinates": [round(lon, 8), round(lat, 8)]},
                    "properties": properties,
                }
            )
    return features


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture", required=True)
    parser.add_argument("--config", default="config")
    parser.add_argument("--origin-lat", type=float, default=52.52)
    parser.add_argument("--origin-lon", type=float, default=13.40)
    parser.add_argument("--asset", action="append", default=[], metavar="EAST,NORTH")
    parser.add_argument("--out", default="results/detections.geojson")
    args = parser.parse_args()

    config_dir = pathlib.Path(args.config)
    camera = yaml.safe_load((config_dir / "camera.yaml").read_text(encoding="utf-8"))
    detector = yaml.safe_load((config_dir / "detector.yaml").read_text(encoding="utf-8"))
    if args.asset:
        targets = [tuple(float(value) for value in asset.split(",")) for asset in args.asset]
    else:
        targets = []

    features = build_features(
        pathlib.Path(args.capture), camera, detector, targets, args.origin_lat, args.origin_lon
    )
    output = pathlib.Path(args.out)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps({"type": "FeatureCollection", "features": features}, indent=1) + "\n",
        encoding="utf-8",
    )
    print(f"{len(features)} detections -> {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
