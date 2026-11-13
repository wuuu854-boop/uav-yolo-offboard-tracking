#!/usr/bin/env python3
"""Offline depth-map quality gate (no SITL).

    python3 scripts/check_depth_map.py \
        --depth results/search_depth_frame.npy \
        --run-csv results/search_run.csv --row 215 \
        --box 6.7,0,0.2,3.0 --box 6.0,-0.8,0.8,0.8 --box 6.0,1.8,0.8,0.8
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from uav_tracking.perception.map_gate import project_depth, summarise  # noqa: E402


def _box(text: str) -> tuple[float, float, float, float]:
    values = tuple(float(v) for v in text.split(","))
    if len(values) != 4:
        raise argparse.ArgumentTypeError("box must be cx,cy,sx,sy")
    return values


def _pose_from_run(path: Path, row: int | None) -> dict:
    rows = np.genfromtxt(path, delimiter=",", names=True)
    if rows.size == 0:
        raise ValueError(f"{path} has no rows")
    if row is not None:
        index = int(row)
        if index < 0 or index >= len(rows):
            raise ValueError(f"row {row} outside 0..{len(rows) - 1}")
    else:
        candidates = [
            i
            for i in range(len(rows))
            if int(rows["stop"][i]) == 1 and float(rows["altitude_m"][i]) > 4.0
        ]
        index = candidates[-1] if candidates else len(rows) - 1
    return {
        "row": int(index),
        "t_s": float(rows["t_s"][index]),
        "east_m": float(rows["east_m"][index]),
        "north_m": float(rows["north_m"][index]),
        "altitude_m": float(rows["altitude_m"][index]),
        "yaw_deg": float(rows["yaw_deg"][index]),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--depth", default="results/search_depth_frame.npy")
    parser.add_argument("--run-csv", default="results/search_run.csv")
    parser.add_argument("--row", type=int, default=None)
    parser.add_argument("--box", type=_box, action="append", required=True)
    parser.add_argument("--sample-px", type=int, default=12)
    parser.add_argument("--min-depth", type=float, default=0.3)
    parser.add_argument("--min-height", type=float, default=2.0)
    parser.add_argument("--max-height", type=float, default=5.5)
    parser.add_argument("--tolerance", type=float, default=1.0)
    parser.add_argument("--min-points", type=int, default=100)
    parser.add_argument("--min-near-fraction", type=float, default=0.70)
    parser.add_argument("--max-median", type=float, default=1.0)
    parser.add_argument("--out", default="results/map_gate")
    args = parser.parse_args()

    depth = np.load(args.depth)
    pose = _pose_from_run(Path(args.run_csv), args.row)
    points = project_depth(
        depth,
        (pose["east_m"], pose["north_m"], pose["altitude_m"]),
        math.radians(pose["yaw_deg"]),
        sample_px=args.sample_px,
        min_depth_m=args.min_depth,
        min_height_m=args.min_height,
        max_height_m=args.max_height,
    )
    summary = summarise(points, args.box, tolerance_m=args.tolerance)
    ok = (
        summary["points"] >= args.min_points
        and summary["near_fraction"] >= args.min_near_fraction
        and summary["median_distance_m"] <= args.max_median
    )
    report = {
        "depth": str(args.depth),
        "run_csv": str(args.run_csv),
        "pose": pose,
        "boxes": args.box,
        "summary": summary,
        "thresholds": {
            "min_points": args.min_points,
            "min_near_fraction": args.min_near_fraction,
            "max_median_distance_m": args.max_median,
        },
        "verdict": "PASS" if ok else "FAIL",
    }
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    lines = [
        "# Depth-map quality gate",
        "",
        f"- frame: `{args.depth}`",
        f"- pose row: {pose['row']} at {pose['t_s']:.2f} s",
        f"- points: {summary['points']}",
        f"- near fraction (<= {args.tolerance:g} m): {summary['near_fraction']:.3f}",
        f"- median distance: {summary['median_distance_m']:.3f} m",
        f"- thresholds: points >= {args.min_points}, near >= {args.min_near_fraction:g}, median <= {args.max_median:g} m",
        "",
        f"**{report['verdict']}**",
        "",
    ]
    out.with_suffix(".md").write_text("\n".join(lines), encoding="utf-8")
    print("\n".join(lines))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
