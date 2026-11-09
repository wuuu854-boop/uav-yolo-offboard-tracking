#!/usr/bin/env python3
"""Summarise ``results/landing_error.csv`` into the table that goes in the README.

The CSV is written by the control node during a trial. This script is the only
thing allowed to turn those rows into a claim, so the statistic is defined here
once instead of being recomputed by hand in a chat window.

Usage::

    python3 scripts/benchmark.py --csv results/landing_error.csv
    python3 scripts/benchmark.py --csv results/landing_error.csv --markdown > results/summary.md
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path


def load_rows(path: Path) -> list[dict]:
    with path.open(newline="", encoding="utf-8") as handle:
        return [row for row in csv.DictReader(handle) if row.get("error_m")]


def percentile(values: list[float], q: float) -> float:
    """Linear-interpolated percentile; ``nan`` for an empty list."""
    if not values:
        return float("nan")
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * q / 100.0
    lower, upper = math.floor(position), math.ceil(position)
    if lower == upper:
        return ordered[int(position)]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def _as_float(value: str) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def summarize(rows: list[dict], tolerance: float = 1.5) -> dict:
    errors = [value for row in rows if (value := _as_float(row.get("error_m"))) is not None]
    latencies = [
        value for row in rows if (value := _as_float(row.get("latency_p50_ms"))) is not None
    ]

    by_target: dict[str, list[float]] = {}
    for row in rows:
        error = _as_float(row.get("error_m"))
        target = str(row.get("target_id", "?"))
        if error is not None:
            by_target.setdefault(target, []).append(error)

    successes = sum(1 for value in errors if value <= tolerance)
    return {
        "trials": len(errors),
        "successes": successes,
        "success_rate": (successes / len(errors)) if errors else float("nan"),
        "error_median_m": percentile(errors, 50),
        "error_p95_m": percentile(errors, 95),
        "latency_median_ms": percentile(latencies, 50),
        "tolerance_m": tolerance,
        "by_target": {
            target: {"trials": len(values), "error_median_m": percentile(values, 50)}
            for target, values in sorted(by_target.items())
        },
    }


def to_markdown(summary: dict) -> str:
    if not summary["trials"]:
        return "No trials recorded yet. Run `make e2e` and re-run this script."

    tolerance = summary["tolerance_m"]
    latency = summary["latency_median_ms"]
    latency_text = "n/a" if math.isnan(latency) else f"{latency:.0f}"
    lines = [
        f"| trials | success (<= {tolerance:.1f} m) | error median (m) | error p95 (m) | latency median (ms) |",
        "|---|---|---|---|---|",
        f"| {summary['trials']} | {summary['successes']}/{summary['trials']} "
        f"({summary['success_rate']:.0%}) | {summary['error_median_m']:.2f} "
        f"| {summary['error_p95_m']:.2f} | {latency_text} |",
        "",
        "| target | trials | error median (m) |",
        "|---|---|---|",
    ]
    for target, stats in summary["by_target"].items():
        lines.append(f"| {target} | {stats['trials']} | {stats['error_median_m']:.2f} |")
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", default="results/landing_error.csv")
    parser.add_argument("--tolerance", type=float, default=1.5, help="success threshold in metres")
    parser.add_argument(
        "--markdown", action="store_true", help="print a markdown table instead of JSON"
    )
    args = parser.parse_args()

    path = Path(args.csv)
    if not path.exists():
        print(f"no such file: {path}")
        return 1

    summary = summarize(load_rows(path), tolerance=args.tolerance)
    if args.markdown:
        print(to_markdown(summary))
    else:
        print(json.dumps(summary, indent=2, default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
