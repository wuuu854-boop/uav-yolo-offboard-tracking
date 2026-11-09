#!/usr/bin/env python3
"""Summarise ``results/landing_error.csv`` into the table that goes in the README.

The CSV is written by the control node during a trial. This script is the only
thing allowed to turn those rows into a claim, so the statistic is defined here
once instead of being recomputed by hand in a chat window.

Latency is pooled from ``latency_raw.csv`` when it exists. Averaging per-trial
percentiles is not the same as the percentile of the run, and a p95 built that
way is meaningless — so the fallback says which source it used.

    python3 scripts/benchmark.py --csv results/landing_error.csv
    python3 scripts/benchmark.py --csv results/landing_error.csv --markdown
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

RAW_NAME = "latency_raw.csv"


def load_rows(path: Path) -> list[dict]:
    with path.open(newline="", encoding="utf-8") as handle:
        return [row for row in csv.DictReader(handle) if row.get("error_m")]


def load_raw_latencies(path: Path) -> list[float]:
    """Every individual sample, if the recorder kept them."""
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as handle:
        return [
            float(row["latency_ms"])
            for row in csv.DictReader(handle)
            if row.get("latency_ms") not in (None, "")
        ]


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


def summarize(
    rows: list[dict],
    tolerance: float = 1.5,
    raw_latencies: list[float] | None = None,
) -> dict:
    errors = [value for row in rows if (value := _as_float(row.get("error_m"))) is not None]

    by_target: dict[str, list[float]] = {}
    for row in rows:
        error = _as_float(row.get("error_m"))
        target = str(row.get("target_id", "?"))
        if error is not None:
            by_target.setdefault(target, []).append(error)

    if raw_latencies:
        latency_p50 = percentile(raw_latencies, 50)
        latency_p95 = percentile(raw_latencies, 95)
        latency_source = f"{len(raw_latencies)} raw samples"
    else:
        trial_medians = [
            value for row in rows if (value := _as_float(row.get("latency_p50_ms"))) is not None
        ]
        latency_p50 = percentile(trial_medians, 50)
        latency_p95 = float("nan")
        latency_source = "per-trial medians (no raw file)" if trial_medians else "none"

    successes = sum(1 for value in errors if value <= tolerance)
    return {
        "trials": len(errors),
        "successes": successes,
        "success_rate": (successes / len(errors)) if errors else float("nan"),
        "error_median_m": percentile(errors, 50),
        "error_p95_m": percentile(errors, 95),
        "latency_p50_ms": latency_p50,
        "latency_p95_ms": latency_p95,
        "latency_source": latency_source,
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
    p50 = summary["latency_p50_ms"]
    p95 = summary["latency_p95_ms"]
    p50_text = "n/a" if math.isnan(p50) else f"{p50:.0f}"
    p95_text = "n/a" if math.isnan(p95) else f"{p95:.0f}"

    lines = [
        f"| trials | success (<= {tolerance:.1f} m) | error median (m) | error p95 (m) "
        f"| latency p50 (ms) | latency p95 (ms) |",
        "|---|---|---|---|---|---|",
        f"| {summary['trials']} | {summary['successes']}/{summary['trials']} "
        f"({summary['success_rate']:.0%}) | {summary['error_median_m']:.2f} "
        f"| {summary['error_p95_m']:.2f} | {p50_text} | {p95_text} |",
        "",
        f"Latency source: {summary['latency_source']}.",
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
    parser.add_argument("--raw", default=None, help=f"defaults to {RAW_NAME} next to --csv")
    parser.add_argument("--tolerance", type=float, default=1.5, help="success threshold in metres")
    parser.add_argument(
        "--markdown", action="store_true", help="print a markdown table instead of JSON"
    )
    args = parser.parse_args()

    path = Path(args.csv)
    if not path.exists():
        print(f"no such file: {path}")
        return 1

    raw_path = Path(args.raw) if args.raw else path.with_name(RAW_NAME)
    raw = load_raw_latencies(raw_path) or None

    summary = summarize(load_rows(path), tolerance=args.tolerance, raw_latencies=raw)
    if args.markdown:
        print(to_markdown(summary))
    else:
        print(json.dumps(summary, indent=2, default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
