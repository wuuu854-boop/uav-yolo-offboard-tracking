#!/usr/bin/env python3
"""Aggregate repeated SITL runs into one repeatability report.

The runner saves one summary and one landing row per run as
``results/repeat/<scenario>_<n>_summary.json`` / ``_landing.csv``. This script
does not fly anything; it only reads those files and writes a table.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path


def summarise(records: list[dict]) -> dict:
    """Group per-run records by scenario and compute simple statistics."""
    scenarios: dict[str, list[dict]] = {}
    for record in records:
        scenarios.setdefault(record["scenario"], []).append(record)
    report = {}
    for scenario, runs in sorted(scenarios.items()):
        runs.sort(key=lambda item: item["index"])
        errors = [float(run["final_error_m"]) for run in runs]
        clearances = [float(run["min_clearance_m"]) for run in runs]
        contacts = [int(run["contacts_total"]) for run in runs]
        report[scenario] = {
            "runs": len(runs),
            "passes": sum(1 for run in runs if run["verdict"] == "PASS"),
            "target_error_m": {
                "min": min(errors),
                "max": max(errors),
                "mean": sum(errors) / len(errors),
            },
            "clearance_m": {"min": min(clearances), "max": max(clearances)},
            "contacts_total": sum(contacts),
            "landing_successes": sum(1 for run in runs if run["landing_success"]),
            "landing_error_m": {
                "min": min(float(run["landing_error_m"]) for run in runs),
                "max": max(float(run["landing_error_m"]) for run in runs),
            },
        }
    return report


def load_records(directory: Path) -> list[dict]:
    records = []
    for summary_path in sorted(directory.glob("*_summary.json")):
        stem = summary_path.name[: -len("_summary.json")]
        scenario, _, index_text = stem.rpartition("_")
        landing_path = directory / f"{stem}_landing.csv"
        with summary_path.open(encoding="utf-8") as handle:
            summary = json.load(handle)
        landing = {}
        if landing_path.exists():
            with landing_path.open(encoding="utf-8", newline="") as handle:
                rows = list(csv.DictReader(handle))
            landing = rows[-1] if rows else {}
        records.append(
            {
                "scenario": scenario,
                "index": int(index_text),
                "verdict": summary.get("verdict"),
                "final_error_m": summary["metrics"]["final_error_m"],
                "min_clearance_m": summary["metrics"]["min_clearance_m"],
                "contacts_total": summary["metrics"]["contacts_total"],
                "landing_error_m": float(landing.get("error_m", "nan")),
                "landing_success": str(landing.get("success", "")).lower() == "true",
            }
        )
    return records


def render_markdown(report: dict) -> str:
    lines = [
        "# Repeatability report",
        "",
        "| Scenario | Runs | PASS | Target error min/mean/max (m) | Clearance min/max (m) | Contacts | Landings |",
        "|---|---:|---:|---|---|---:|---:|",
    ]
    for scenario, item in report.items():
        error = item["target_error_m"]
        clearance = item["clearance_m"]
        lines.append(
            f"| {scenario} | {item['runs']} | {item['passes']}/{item['runs']} | "
            f"{error['min']:.3f} / {error['mean']:.3f} / {error['max']:.3f} | "
            f"{clearance['min']:.3f} / {clearance['max']:.3f} | "
            f"{item['contacts_total']} | {item['landing_successes']}/{item['runs']} |"
        )
    lines.append("")
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dir", default="results/repeat")
    parser.add_argument("--out", default="results/repeat_report")
    args = parser.parse_args()
    directory = Path(args.dir)
    records = load_records(directory)
    if not records:
        raise SystemExit(f"no summary files under {directory}")
    report = summarise(records)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    markdown = render_markdown(report)
    out.with_suffix(".md").write_text(markdown, encoding="utf-8")
    print(markdown)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
