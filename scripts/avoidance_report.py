#!/usr/bin/env python3
"""Turn an avoidance flight CSV into the numbers the docs quote.

Reads what `scripts/record_avoidance.py` wrote and checks the claim "the vehicle
stopped in front of the wall and did not collide", using the wall geometry from
config/obstacle.yaml and the spawn point from the mission config. Writes a JSON
summary and a two-panel plot; exits non-zero when a check fails, so a bad run
cannot be quoted as a good one.

    python3 scripts/avoidance_report.py \
        --csv results/avoidance_run.csv \
        --summary results/avoidance_summary.json \
        --plot results/avoidance_run.png
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from collections import Counter
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]


def read_rows(path: Path) -> list[dict]:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        rows = []
        for raw in reader:
            row = {
                "t_s": float(raw["t_s"]),
                "east_m": float(raw["east_m"]),
                "north_m": float(raw["north_m"]),
                "altitude_m": float(raw["altitude_m"]),
                "yaw_deg": float(raw["yaw_deg"]),
                "stop": bool(int(raw["stop"])),
                "contacts_total": int(raw["contacts_total"]),
                "depth_frames": int(raw["depth_frames"]),
            }
            nearest = raw.get("nearest_m", "").strip()
            row["nearest_m"] = None if nearest in ("", "nan") else float(nearest)
            row["clear_side"] = raw.get("clear_side", "unknown").strip() or "unknown"
            rows.append(row)
    return rows


def read_trace(path: Path) -> list[str]:
    """Return the `note` column of a StateTrace, or [] when it is absent."""
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as handle:
        return [row.get("note", "") for row in csv.DictReader(handle)]


def point_to_aabb_distance(x: float, y: float, bounds: tuple[float, float, float, float]) -> float:
    """Shortest 2-D distance from a point to an axis-aligned rectangle."""
    x_min, x_max, y_min, y_max = bounds
    dx = max(x_min - x, 0.0, x - x_max)
    dy = max(y_min - y, 0.0, y - y_max)
    return math.hypot(dx, dy)


def point_to_box_distance(
    x: float,
    y: float,
    centre: tuple[float, float],
    size: tuple[float, float],
    angle_deg: float,
) -> float:
    """Shortest 2-D distance from a point to a box rotated about z."""
    dx = x - float(centre[0])
    dy = y - float(centre[1])
    angle = math.radians(float(angle_deg))
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    local_x = cos_a * dx + sin_a * dy
    local_y = -sin_a * dx + cos_a * dy
    half_x = float(size[0]) / 2.0
    half_y = float(size[1]) / 2.0
    gap_x = max(abs(local_x) - half_x, 0.0)
    gap_y = max(abs(local_y) - half_y, 0.0)
    return math.hypot(gap_x, gap_y)


def longest_stop_run(rows: list[dict]) -> tuple[int, int]:
    """Indices of the longest consecutive stopped interval."""
    best = (-1, -1)
    start: int | None = None
    for index, row in enumerate(rows):
        if row["stop"] and start is None:
            start = index
        if not row["stop"] and start is not None:
            if index - start > best[1] - best[0]:
                best = (start, index - 1)
            start = None
    if start is not None and len(rows) - start > best[1] - best[0]:
        best = (start, len(rows) - 1)
    return best


def analyse(
    rows: list[dict],
    obstacle: dict,
    mission: dict,
    target: tuple[float, float] = (0.0, 0.0),
    side_block: dict | None = None,
) -> dict:
    if not rows:
        raise ValueError("the CSV has no rows; did the recorder see odometry?")

    spawn_east, spawn_north = (float(v) for v in mission["spawn_offset_enu"])
    pose = obstacle["pose_enu"]
    size = obstacle["size_m"]
    x_min, x_max = pose[0] - size[0] / 2.0, pose[0] + size[0] / 2.0
    y_min, y_max = pose[1] - size[1] / 2.0, pose[1] + size[1] / 2.0
    bounds = (x_min, x_max, y_min, y_max)

    world = [(spawn_east + row["east_m"], spawn_north + row["north_m"]) for row in rows]
    if obstacle.get("angle_deg") is not None:
        clearances = [
            point_to_box_distance(
                x,
                y,
                (obstacle["pose_enu"][0], obstacle["pose_enu"][1]),
                (obstacle["size_m"][0], obstacle["size_m"][1]),
                obstacle["angle_deg"],
            )
            for x, y in world
        ]
    else:
        clearances = [point_to_aabb_distance(x, y, bounds) for x, y in world]
    min_index = min(range(len(clearances)), key=clearances.__getitem__)

    start, end = longest_stop_run(rows)
    stopped = start >= 0
    stop_start_s = rows[start]["t_s"] if stopped else None
    stop_end_s = rows[end]["t_s"] if stopped else None
    stop_hold_s = (stop_end_s - stop_start_s) if stopped else 0.0
    if stopped:
        anchor = world[start]
        drift = max(math.hypot(x - anchor[0], y - anchor[1]) for x, y in world[start : end + 1])
        # The first seconds of a stop are braking and settling, not hovering:
        # measuring drift from the instant the gate fired would charge the
        # controller for its own stopping distance. Settle for 10 s, then
        # measure what moved. Advance and retreat are reported separately so a
        # coast toward the wall cannot hide inside a total distance.
        settle_index = next(
            (i for i in range(start, end + 1) if rows[i]["t_s"] - stop_start_s >= 10.0),
            None,
        )
        if settle_index is not None:
            settled = world[settle_index]
            settled_drift = max(
                math.hypot(x - settled[0], y - settled[1]) for x, y in world[settle_index : end + 1]
            )
        else:
            settled_drift = None
        # East is the wall direction for this scenario.
        advance = max(x for x, _ in world[start : end + 1]) - anchor[0]
        retreat = anchor[0] - min(x for x, _ in world[start : end + 1])
    else:
        drift = None
        settled_drift = None
        advance = None
        retreat = None

    final_world = world[-1]
    final_error = math.hypot(final_world[0] - target[0], final_world[1] - target[1])
    north_values = [north for _, north in world]
    max_abs_north = max(abs(north) for north in north_values)
    gate_released = bool(stopped and any(not row["stop"] for row in rows[start:]))
    side_votes = [
        side
        for side in (row.get("clear_side", "unknown") for row in rows)
        if side in ("left", "right")
    ]
    clear_side_mode = Counter(side_votes).most_common(1)[0][0] if side_votes else "unknown"
    # The mode over the whole flight is not the choice the planner made: after
    # the detour the free side often flips. The decision is the value at the
    # first stopped row, which is the tick the detour started on.
    clear_side_at_stop = rows[start].get("clear_side", "unknown") if stopped else "unknown"

    return {
        "obstacle": {
            "model": obstacle["model"],
            "centre_enu": [float(v) for v in pose],
            "size_m": [float(v) for v in size],
            "aabb_enu_m": [x_min, x_max, y_min, y_max],
        },
        "spawn_enu": [spawn_east, spawn_north],
        "target_enu": [float(target[0]), float(target[1])],
        "side_block": (
            {
                "model": side_block["model"],
                "centre_enu": [float(v) for v in side_block["pose_enu"]],
                "size_m": [float(v) for v in side_block["size_m"]],
            }
            if side_block
            else None
        ),
        "metrics": {
            "rows": len(rows),
            "duration_s": round(rows[-1]["t_s"] - rows[0]["t_s"], 2),
            "stop_engaged": stopped,
            "stop_start_s": stop_start_s,
            "stop_end_s": stop_end_s,
            "stop_hold_s": round(stop_hold_s, 2),
            "stop_drift_m": None if drift is None else round(drift, 3),
            "stop_advance_m": None if advance is None else round(advance, 3),
            "stop_retreat_m": None if retreat is None else round(retreat, 3),
            "settled_drift_m": None if settled_drift is None else round(settled_drift, 3),
            "gate_released": gate_released,
            "clear_side_mode": clear_side_mode,
            "clear_side_at_stop": clear_side_at_stop,
            "max_north_m": round(max(north_values), 3),
            "min_north_m": round(min(north_values), 3),
            "max_abs_north_m": round(max_abs_north, 3),
            "final_error_m": round(final_error, 3),
            "min_clearance_m": round(clearances[min_index], 3),
            "min_clearance_at_s": rows[min_index]["t_s"],
            "final_east_m": round(rows[-1]["east_m"], 3),
            "max_east_m": round(max(row["east_m"] for row in rows), 3),
            "max_altitude_m": round(max(row["altitude_m"] for row in rows), 3),
            "contacts_total": max(row["contacts_total"] for row in rows),
            "depth_frames": max(row["depth_frames"] for row in rows),
            "min_nearest_m": (
                None
                if all(row["nearest_m"] is None for row in rows)
                else round(min(row["nearest_m"] for row in rows if row["nearest_m"] is not None), 3)
            ),
        },
        "checks": {},
        "world_track": world,
    }


def verdict(
    report: dict,
    obstacle: dict,
    mode: str = "hold",
    mission: dict | None = None,
    trace_notes: list[str] | tuple[str, ...] = (),
    expect_side: str | None = None,
) -> dict:
    """Apply the acceptance checks for one of the two behaviours.

    ``hold`` is phase 1: stop and stay stopped. ``detour`` is phase 2: stop,
    move sideways by at least the mission's minimum, release the gate, and get
    to the target without a contact. They share the clearance, contact and
    depth-frame checks but not the "did it stay put" ones.
    """
    metrics = report["metrics"]
    if mode == "hold":
        drift_limit = float(mission.get("hold_drift_limit_m", 0.3)) if mission else 0.3
        checks = {
            "stop_engaged": {
                "ok": bool(metrics["stop_engaged"]),
                "detail": f"stop_start_s={metrics['stop_start_s']}",
            },
            "stop_held_at_least_3s": {
                "ok": metrics["stop_hold_s"] >= 3.0,
                "detail": f"stop_hold_s={metrics['stop_hold_s']}",
            },
            "settled_drift_under_limit": {
                "ok": metrics["settled_drift_m"] is not None
                and metrics["settled_drift_m"] <= drift_limit,
                "detail": (
                    f"settled_drift_m={metrics['settled_drift_m']} after 10 s <= {drift_limit}"
                ),
            },
            "advance_after_stop_under_1m": {
                "ok": metrics["stop_advance_m"] is not None and metrics["stop_advance_m"] <= 1.0,
                "detail": (
                    f"stop_advance_m={metrics['stop_advance_m']}, "
                    f"retreat_m={metrics['stop_retreat_m']}"
                ),
            },
            "clearance_at_least_min": {
                "ok": metrics["min_clearance_m"] >= float(obstacle["min_clearance_m"]),
                "detail": (
                    f"min_clearance_m={metrics['min_clearance_m']} >= {obstacle['min_clearance_m']}"
                ),
            },
            "no_contact": {
                "ok": metrics["contacts_total"] == 0,
                "detail": f"contacts_total={metrics['contacts_total']}",
            },
            "depth_frames_seen": {
                "ok": metrics["depth_frames"] > 0,
                "detail": f"depth_frames={metrics['depth_frames']}",
            },
        }
    elif mode == "detour":
        if mission is None:
            raise ValueError("detour mode needs the mission config")
        min_lateral = float(mission.get("detour_min_lateral_m", 3.0))
        min_clearance = float(mission.get("detour_min_clearance_m", 0.4))
        trace_started = any("detour" in note.lower() for note in trace_notes)
        checks = {
            "stop_engaged": {
                "ok": bool(metrics["stop_engaged"]),
                "detail": f"stop_start_s={metrics['stop_start_s']}",
            },
            "detour_started": {
                "ok": trace_started or metrics["max_abs_north_m"] >= min_lateral,
                "detail": (
                    f"trace_note={trace_started}, max_abs_north_m={metrics['max_abs_north_m']}"
                ),
            },
            "lateral_leg_at_least_min": {
                "ok": metrics["max_abs_north_m"] >= min_lateral,
                "detail": (f"max_abs_north_m={metrics['max_abs_north_m']} >= {min_lateral}"),
            },
            "gate_released": {
                "ok": bool(metrics["gate_released"]),
                "detail": f"gate_released={metrics['gate_released']}",
            },
            "target_reached": {
                "ok": metrics["final_error_m"] <= 1.5,
                "detail": f"final_error_m={metrics['final_error_m']} <= 1.5",
            },
            "clearance_at_least_min": {
                "ok": metrics["min_clearance_m"] >= min_clearance,
                "detail": (
                    f"min_clearance_m={metrics['min_clearance_m']} >= {min_clearance} "
                    "(the target is closer to the wall than the phase-1 stop point)"
                ),
            },
            "no_contact": {
                "ok": metrics["contacts_total"] == 0,
                "detail": f"contacts_total={metrics['contacts_total']}",
            },
            "depth_frames_seen": {
                "ok": metrics["depth_frames"] > 0,
                "detail": f"depth_frames={metrics['depth_frames']}",
            },
        }
        if expect_side is not None:
            expected = str(expect_side).lower()
            if expected not in ("left", "right"):
                raise ValueError(f"expect_side must be 'left' or 'right', got {expect_side!r}")
            chosen = metrics["clear_side_at_stop"]
            checks["side_selected"] = {
                "ok": chosen == expected,
                "detail": (
                    f"clear_side_at_stop={chosen}, expected={expected} "
                    f"(whole-run mode {metrics['clear_side_mode']})"
                ),
            }
            if chosen == "left":
                lateral_ok = metrics["max_north_m"] >= min_lateral
            elif chosen == "right":
                lateral_ok = metrics["min_north_m"] <= -min_lateral
            else:
                lateral_ok = False
            checks["lateral_direction_matches_side"] = {
                "ok": lateral_ok,
                "detail": (
                    f"chosen={chosen}, max_north_m={metrics['max_north_m']}, "
                    f"min_north_m={metrics['min_north_m']}, min_lateral_m={min_lateral}"
                ),
            }
    elif mode == "global":
        min_clearance = float(mission.get("global_min_clearance_m", 0.4)) if mission else 0.4
        checks = {
            "target_reached": {
                "ok": metrics["final_error_m"] <= 1.5,
                "detail": f"final_error_m={metrics['final_error_m']} <= 1.5",
            },
            "clearance_at_least_min": {
                "ok": metrics["min_clearance_m"] >= min_clearance,
                "detail": (f"min_clearance_m={metrics['min_clearance_m']} >= {min_clearance}"),
            },
            "no_contact": {
                "ok": metrics["contacts_total"] == 0,
                "detail": f"contacts_total={metrics['contacts_total']}",
            },
            "depth_frames_seen": {
                "ok": metrics["depth_frames"] > 0,
                "detail": f"depth_frames={metrics['depth_frames']}",
            },
        }
    else:
        raise ValueError(f"unknown report mode {mode!r}; expected 'hold', 'detour' or 'global'")

    report["mode"] = mode
    report["checks"] = checks
    report["verdict"] = "PASS" if all(item["ok"] for item in checks.values()) else "FAIL"
    return report


def plot(report: dict, rows: list[dict], out: Path, stop_m: float, clear_m: float) -> None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle
    except ImportError:
        print("matplotlib is not installed; skipping the plot")
        return

    world = report["world_track"]
    obstacle = report["obstacle"]
    x_min, x_max, y_min, y_max = obstacle["aabb_enu_m"]

    figure, (top, bottom) = plt.subplots(2, 1, figsize=(7.2, 7.0), sharex=False)
    top.plot([p[0] for p in world], [p[1] for p in world], color="#1f77b4", label="flight path")
    stopped = [i for i, row in enumerate(rows) if row["stop"]]
    if stopped:
        start, end = stopped[0], stopped[-1]
        top.plot(
            [world[i][0] for i in range(start, end + 1)],
            [world[i][1] for i in range(start, end + 1)],
            color="#d62728",
            linewidth=3,
            label="stopped",
        )
    top.add_patch(
        Rectangle(
            (x_min, y_min),
            x_max - x_min,
            y_max - y_min,
            color="#ff7f0e",
            alpha=0.8,
            label="obstacle_wall",
        )
    )
    side_block = report.get("side_block")
    if side_block:
        sx, sy, _ = side_block["centre_enu"]
        sw, sd, _ = side_block["size_m"]
        top.add_patch(
            Rectangle(
                (sx - sw / 2.0, sy - sd / 2.0),
                sw,
                sd,
                color="#9467bd",
                alpha=0.8,
                label=str(side_block.get("model", "side_block")),
            )
        )
    top.set_xlabel("east (world ENU, m)")
    top.set_ylabel("north (world ENU, m)")
    if report.get("mode") == "detour":
        top.set_title("detour flight: around the wall and on to the target")
    elif report.get("mode") == "global":
        top.set_title("global flight: around the known wall and on to the target")
    elif report.get("side_block"):
        top.set_title("hold flight: both sides blocked, the vehicle stays at the stop")
    else:
        top.set_title("avoidance flight: the wall is on the path, the stop is before it")
    top.axis("equal")
    top.grid(True, alpha=0.3)
    top.legend(loc="best")

    times = [row["t_s"] for row in rows]
    nearest = [math.nan if row["nearest_m"] is None else row["nearest_m"] for row in rows]
    bottom.plot(times, nearest, color="#2ca02c", label="nearest depth in ROI (m)")
    bottom.axhline(stop_m, color="#d62728", linestyle="--", label=f"stop threshold {stop_m} m")
    bottom.axhline(clear_m, color="#9467bd", linestyle=":", label=f"clear threshold {clear_m} m")
    bottom.set_xlabel("time (s)")
    bottom.set_ylabel("z-depth (m)")
    bottom.set_title("depth gate")
    bottom.grid(True, alpha=0.3)
    bottom.legend(loc="best")

    figure.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(out, dpi=150)
    plt.close(figure)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", default="results/avoidance_run.csv")
    parser.add_argument("--summary", default="results/avoidance_summary.json")
    parser.add_argument("--plot", default="results/avoidance_run.png")
    parser.add_argument("--obstacle", default="config/obstacle.yaml")
    parser.add_argument("--side-obstacle", default=None)
    parser.add_argument("--mission", default="config/mission_avoidance.yaml")
    parser.add_argument("--depth-config", default="config/depth.yaml")
    parser.add_argument("--targets", default="config/targets.yaml")
    parser.add_argument("--trace", default="results/avoidance_trace.csv")
    parser.add_argument("--mode", choices=("hold", "detour", "global"), default="hold")
    parser.add_argument("--expect-side", choices=("left", "right"), default=None)
    args = parser.parse_args()

    rows = read_rows(Path(args.csv))
    with open(args.obstacle, encoding="utf-8") as handle:
        obstacle = yaml.safe_load(handle)
    side_block = None
    if args.side_obstacle:
        with open(args.side_obstacle, encoding="utf-8") as handle:
            side_block = yaml.safe_load(handle)
    with open(args.mission, encoding="utf-8") as handle:
        mission = yaml.safe_load(handle)
    with open(args.depth_config, encoding="utf-8") as handle:
        gate = yaml.safe_load(handle)
    with open(args.targets, encoding="utf-8") as handle:
        targets = yaml.safe_load(handle)
    target = next(
        (float(item["x"]), float(item["y"])) for item in targets["targets"] if int(item["id"]) == 0
    )

    report = verdict(
        analyse(rows, obstacle, mission, target, side_block),
        obstacle,
        mode=args.mode,
        mission=mission,
        trace_notes=read_trace(Path(args.trace)),
        expect_side=args.expect_side,
    )
    report["gate"] = {
        "stop_distance_m": float(gate["stop_distance_m"]),
        "clear_distance_m": float(gate["clear_distance_m"]),
        "roi_width_fraction": float(gate["roi_width_fraction"]),
        "roi_height_fraction": float(gate["roi_height_fraction"]),
        "source": str(args.depth_config),
    }
    plot(
        report,
        rows,
        Path(args.plot),
        float(gate["stop_distance_m"]),
        float(gate["clear_distance_m"]),
    )

    summary = {key: value for key, value in report.items() if key != "world_track"}
    summary["source_csv"] = str(args.csv)
    Path(args.summary).parent.mkdir(parents=True, exist_ok=True)
    Path(args.summary).write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")

    for name, check in report["checks"].items():
        print(f"  {'PASS' if check['ok'] else 'FAIL'}  {name:34s} {check['detail']}")
    print(f"verdict: {report['verdict']}  (wrote {args.summary} and {args.plot})")
    return 0 if report["verdict"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
