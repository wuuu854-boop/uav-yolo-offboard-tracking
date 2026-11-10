#!/usr/bin/env python3
"""Run the whole capture review in one command.

The tools exist separately because they answer different questions; a customer
buys them as one pass over a flight: is the data usable (QA), does it meet the
written shot standard, and is there a structured deliverable (GeoJSON)?

    python3 scripts/pipeline.py --capture /root/capture_north/trial_1 --asset 0,3.5 \
        --out results/pipeline.json
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys

import yaml

REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))
sys.path.insert(0, str(REPO_ROOT))

from scripts.export_geojson import build_features  # noqa: E402
from scripts.qa_report import analyse_capture, assets_from_args  # noqa: E402
from scripts.shot_sheet_check import analyse_shots  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture", required=True)
    parser.add_argument("--config", default="config")
    parser.add_argument("--sheet", default="config/shot_sheet.yaml")
    parser.add_argument("--asset", action="append", default=[], metavar="EAST,NORTH")
    parser.add_argument("--origin-lat", type=float, default=52.52)
    parser.add_argument("--origin-lon", type=float, default=13.40)
    parser.add_argument("--out", default="")
    parser.add_argument("--geojson-out", default="")
    args = parser.parse_args()

    config_dir = pathlib.Path(args.config)
    capture = pathlib.Path(args.capture)
    camera = yaml.safe_load((config_dir / "camera.yaml").read_text(encoding="utf-8"))
    detector = yaml.safe_load((config_dir / "detector.yaml").read_text(encoding="utf-8"))
    targets = assets_from_args(args.asset, config_dir)

    qa = analyse_capture(capture, camera, detector, targets)
    sheet = yaml.safe_load(pathlib.Path(args.sheet).read_text(encoding="utf-8"))
    shots = analyse_shots(capture, sheet, camera, detector, targets)
    features = build_features(capture, camera, detector, targets, args.origin_lat, args.origin_lon)

    passed = bool(qa["passed"] and shots["passed"])
    report = {
        "capture": str(capture),
        "assets": targets,
        "qa": qa,
        "shot_sheet": shots,
        "features": len(features),
        "passed": passed,
    }

    if args.out:
        pathlib.Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        pathlib.Path(args.out).write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    if args.geojson_out:
        pathlib.Path(args.geojson_out).parent.mkdir(parents=True, exist_ok=True)
        pathlib.Path(args.geojson_out).write_text(
            json.dumps({"type": "FeatureCollection", "features": features}, indent=1) + "\n",
            encoding="utf-8",
        )

    metrics = qa["metrics"]
    print(f"frames            {metrics['frames']} ({metrics['detections']} detections)")
    print(
        f"qa                level coverage {metrics['level_detection_rate']:.0%}, "
        f"association {metrics['association_m']['median']:.2f} m, "
        f"{'PASS' if qa['passed'] else 'FAIL: ' + '; '.join(qa['failures'])}"
    )
    print(
        f"shot sheet        {', '.join(f'{name} {count} frames' for name, count in shots['summary'].items())}"
        f" - {'PASS' if shots['passed'] else 'FAIL'}"
    )
    print(f"features          {len(features)}")
    print(f"verdict           {'PASS' if passed else 'FAIL'}")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
