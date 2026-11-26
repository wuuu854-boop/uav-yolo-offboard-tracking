#!/usr/bin/env python3
"""Offline Phase B: build the grid from a recorded frame and run A* on it.

    make grid-plan
    # or: python3 scripts/plan_depth_map.py --depth results/meta_avoid4_depth_frame.npy

No SITL: this checks that the depth-derived grid can produce a collision-free
path around the wall before the planner is wired into the offboard node.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from uav_tracking.perception.frame_meta import read_frame_meta  # noqa: E402
from uav_tracking.perception.map_gate import (  # noqa: E402
    OccupancyGrid,
    project_depth,
)
from uav_tracking.planning.grid_planner import plan_grid_path  # noqa: E402


def _box(text: str) -> tuple[float, float, float, float]:
    values = tuple(float(v) for v in text.split(","))
    if len(values) != 4:
        raise argparse.ArgumentTypeError("box must be cx,cy,sx,sy")
    return values


def _xy(text: str) -> tuple[float, float]:
    values = tuple(float(v) for v in text.split(","))
    if len(values) != 2:
        raise argparse.ArgumentTypeError("point must be x,y")
    return values


def _pose_from_meta_or_row(args) -> tuple[tuple[float, float, float], float, dict]:
    if args.meta:
        meta = read_frame_meta(args.meta)
        return (
            (meta.east_m, meta.north_m, meta.altitude_m),
            math.radians(meta.yaw_deg),
            {
                "source": "meta",
                "t_s": meta.t_s,
            },
        )
    rows = np.genfromtxt(args.run_csv, delimiter=",", names=True)
    if rows.size == 0:
        raise ValueError(f"{args.run_csv} has no rows")
    if args.row is None:
        candidates = [
            i
            for i in range(len(rows))
            if int(rows["stop"][i]) == 1 and float(rows["altitude_m"][i]) > 4.0
        ]
        index = candidates[-1] if candidates else len(rows) - 1
    else:
        index = int(args.row)
    return (
        (
            float(rows["east_m"][index]),
            float(rows["north_m"][index]),
            float(rows["altitude_m"][index]),
        ),
        math.radians(float(rows["yaw_deg"][index])),
        {"source": "csv", "row": index, "t_s": float(rows["t_s"][index])},
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--depth", default="results/meta_avoid4_depth_frame.npy")
    parser.add_argument("--meta", default="results/meta_avoid4_depth_frame.npy.json")
    parser.add_argument("--run-csv", default="results/avoidance_run.csv")
    parser.add_argument("--row", type=int, default=None)
    parser.add_argument("--box", type=_box, action="append", required=True)
    parser.add_argument("--start", type=_xy, default=None)
    parser.add_argument("--goal", type=_xy, default=(9.5, 0.0))
    parser.add_argument("--resolution", type=float, default=0.25)
    parser.add_argument("--inflation-cells", type=int, default=3)
    parser.add_argument("--roi-fraction", type=float, default=0.6)
    parser.add_argument("--max-depth", type=float, default=4.0)
    parser.add_argument("--min-height", type=float, default=2.5)
    parser.add_argument("--out", default="results/grid_plan")
    args = parser.parse_args()

    depth = np.load(args.depth)
    position, yaw, pose_source = _pose_from_meta_or_row(args)
    start = args.start or (position[0], position[1])
    goal = args.goal

    points = project_depth(
        depth,
        position,
        yaw,
        sample_px=12,
        max_depth_m=args.max_depth,
        min_height_m=args.min_height,
        roi_fraction=args.roi_fraction,
    )
    grid = OccupancyGrid(resolution_m=args.resolution)
    grid.add_points(points)

    began = time.perf_counter()
    try:
        path = plan_grid_path(
            grid,
            start,
            goal,
            inflation_cells=args.inflation_cells,
            min_cell_points=2,
            min_component=3,
        )
        error = ""
    except ValueError as exc:
        path = []
        error = str(exc)
    elapsed_ms = (time.perf_counter() - began) * 1000.0

    occupied = grid.filtered_cells(2, 0, 3)
    clearances = (
        [
            min(
                math.hypot(
                    grid.cell_centre(cell)[0] - x,
                    grid.cell_centre(cell)[1] - y,
                )
                for cell in occupied
            )
            for x, y in path
        ]
        if path and occupied
        else []
    )
    path_length = sum(
        math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
        for i in range(1, len(path))
    )
    report = {
        "depth": args.depth,
        "pose": {"position_enu": list(position), "yaw_deg": math.degrees(yaw), **pose_source},
        "start": list(start),
        "goal": list(goal),
        "points": len(points),
        "occupied_cells": len(occupied),
        "path": [list(p) for p in path],
        "path_length_m": path_length,
        "min_grid_clearance_m": min(clearances) if clearances else None,
        "plan_ms": elapsed_ms,
        "error": error,
        "verdict": "PASS" if path and not error else "FAIL",
    }

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    lines = [
        "# Grid planner, offline",
        "",
        f"- frame: `{args.depth}`",
        f"- pose: ({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}) yaw {math.degrees(yaw):.2f} deg",
        f"- start: {start}, goal: {goal}",
        f"- points: {len(points)}, occupied cells: {len(occupied)}",
        f"- path points: {len(path)}, length: {path_length:.2f} m",
        f"- plan time: {elapsed_ms:.2f} ms",
        f"- min grid clearance: {report['min_grid_clearance_m']}",
        "",
        f"**{report['verdict']}**" + (f" ({error})" if error else ""),
        "",
    ]
    out.with_suffix(".md").write_text("\n".join(lines), encoding="utf-8")

    figure, axis = plt.subplots(figsize=(6, 6))
    if occupied:
        xs = [grid.cell_centre(cell)[0] for cell in occupied]
        ys = [grid.cell_centre(cell)[1] for cell in occupied]
        axis.scatter(xs, ys, s=18, c="0.45", label="occupied")
    for box in args.box:
        axis.add_patch(
            plt.Rectangle(
                (box[0] - box[2] / 2.0, box[1] - box[3] / 2.0),
                box[2],
                box[3],
                fill=False,
                edgecolor="tab:red",
                label="known box",
            )
        )
    if path:
        axis.plot([p[0] for p in path], [p[1] for p in path], "o-", color="tab:blue", label="path")
    axis.scatter([start[0]], [start[1]], marker="s", c="tab:green", label="start")
    axis.scatter([goal[0]], [goal[1]], marker="*", c="tab:orange", label="goal")
    axis.set_aspect("equal")
    axis.set_xlabel("east (m)")
    axis.set_ylabel("north (m)")
    axis.legend(loc="upper left", fontsize=8)
    axis.set_title("Offline grid A* on a recorded depth frame")
    figure.tight_layout()
    figure.savefig(out.with_suffix(".png"), dpi=120)
    plt.close(figure)

    print("\n".join(lines))
    return 0 if report["verdict"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
