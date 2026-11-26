#!/usr/bin/env python3
"""Offline frontier selection on a recorded depth frame (no SITL)."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from uav_tracking.perception.frame_meta import read_frame_meta  # noqa: E402
from uav_tracking.perception.map_gate import OccupancyGrid, project_depth  # noqa: E402
from uav_tracking.perception.map_quality import corridor_coverage  # noqa: E402
from uav_tracking.planning.frontier import choose_frontier  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--depth", default="results/meta_avoid4_depth_frame.npy")
    parser.add_argument("--meta", default="results/meta_avoid4_depth_frame.npy.json")
    parser.add_argument("--goal", type=float, nargs=2, default=(9.5, 0.0))
    parser.add_argument("--out", default="results/frontier_plan")
    args = parser.parse_args()

    meta = read_frame_meta(args.meta)
    depth = np.load(args.depth)
    start = (meta.east_m, meta.north_m)
    points = project_depth(
        depth,
        (meta.east_m, meta.north_m, meta.altitude_m),
        math.radians(meta.yaw_deg),
        sample_px=12,
        max_depth_m=4.0,
        min_height_m=2.5,
        roi_fraction=0.6,
    )
    grid = OccupancyGrid(resolution_m=0.25)
    grid.add_points_with_rays(points, start)
    try:
        result = choose_frontier(
            grid, start, tuple(args.goal), max_candidates=40, inflation_cells=2, close_radius=1
        )
        error = ""
    except ValueError as exc:
        result = {}
        error = str(exc)
    coverage = corridor_coverage(grid, start, tuple(args.goal), half_width_m=0.3)
    report = {
        "frame": args.depth,
        "start": list(start),
        "goal": list(args.goal),
        "points": len(points),
        "frontier": list(result.get("cell", [])) if result else None,
        "frontier_point": list(result.get("point", [])) if result else None,
        "path": [list(p) for p in result.get("path", [])],
        "path_length_m": result.get("path_length_m"),
        "total_cost_m": result.get("total_cost_m"),
        "coverage": coverage,
        "error": error,
        "verdict": "PASS" if result else "FAIL",
    }
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    lines = [
        "# Frontier selection, offline",
        "",
        f"- frame: `{args.depth}`",
        f"- start: {tuple(round(v, 2) for v in start)}, goal: {args.goal}",
        f"- points: {len(points)}",
        f"- frontier point: {report['frontier_point']}",
        f"- path: {len(report['path'])} points, {report['path_length_m']} m",
        f"- total cost: {report['total_cost_m']} m",
        f"- corridor coverage: {coverage['coverage']:.3f}",
        "",
        f"**{report['verdict']}**" + (f" ({error})" if error else ""),
        "",
    ]
    out.with_suffix(".md").write_text("\n".join(lines), encoding="utf-8")
    print("\n".join(lines))
    return 0 if result else 1


if __name__ == "__main__":
    raise SystemExit(main())
