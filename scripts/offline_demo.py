#!/usr/bin/env python3
"""Run the control loop with no simulator, write a CSV and (optionally) a plot.

This is the fastest way to check that the geometry and the state machine still
work, on any OS, before spending a day on PX4 + Gazebo. It is not a substitute
for SITL results — it models no dynamics and no controller lag.

    python3 scripts/offline_demo.py
    python3 scripts/offline_demo.py --seed 3 --target 6 4 --plot results/offline_demo.png
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from src.control.offline_sim import run_mission  # noqa: E402


def write_csv(path: Path, samples: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(samples[0]))
        writer.writeheader()
        writer.writerows(samples)


def write_plot(path: Path, samples: list[dict], target: tuple[float, float]) -> bool:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed; skipping the plot (pip install -r requirements-dev.txt)")
        return False

    path.parent.mkdir(parents=True, exist_ok=True)
    east = [row["east_m"] for row in samples]
    north = [row["north_m"] for row in samples]
    errors = [(row["t_s"], row["error_m"]) for row in samples if row["error_m"] != ""]
    final_error = ((east[-1] - target[0]) ** 2 + (north[-1] - target[1]) ** 2) ** 0.5

    figure, (left, right) = plt.subplots(1, 2, figsize=(10, 4.2), dpi=120)

    left.plot(east, north, label="vehicle path")
    left.scatter([0], [0], marker="o", label="start")
    left.scatter([target[0]], [target[1]], marker="x", s=90, label="target (ground truth)")
    left.plot([east[-1], target[0]], [north[-1], target[1]], "k--", lw=0.8)
    left.set_xlabel("east (m)")
    left.set_ylabel("north (m)")
    left.set_title(f"Trajectory — final error {final_error:.2f} m")
    left.axis("equal")
    left.grid(alpha=0.3)
    left.legend(loc="best", fontsize=8)

    if errors:
        right.plot([t for t, _ in errors], [e for _, e in errors])
        right.axhline(1.5, color="tab:red", lw=0.8, ls="--", label="success threshold (1.5 m)")
        right.set_xlabel("simulated time (s)")
        right.set_ylabel("distance to target (m)")
        right.set_title("Range to target")
        right.grid(alpha=0.3)
        right.legend(loc="best", fontsize=8)

    figure.suptitle("Offline control-loop demo (no Gazebo)", fontsize=11)
    figure.tight_layout()
    figure.savefig(path)
    plt.close(figure)
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--target", nargs=2, type=float, default=(5.0, 2.0), metavar=("EAST", "NORTH")
    )
    parser.add_argument("--altitude", type=float, default=10.0, help="cruise altitude in metres")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--csv", default="results/offline_demo.csv")
    parser.add_argument("--plot", default="results/offline_demo.png")
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args()

    target = (args.target[0], args.target[1])
    result = run_mission(target_enu=target, seed=args.seed, takeoff_altitude_m=args.altitude)

    csv_path = REPO_ROOT / args.csv
    write_csv(csv_path, result.samples)

    print(f"target          : ({target[0]:.1f}, {target[1]:.1f}) m")
    print(f"steps           : {result.steps} ({result.steps * 0.1:.1f} s simulated)")
    print(f"final state     : {result.final_state.value}")
    print(f"final error     : {result.final_error_m:.2f} m")
    print(f"samples written : {csv_path.relative_to(REPO_ROOT)}")

    if not args.no_plot and write_plot(REPO_ROOT / args.plot, result.samples, target):
        print(f"plot written    : {args.plot}")

    # A failure here is information, not a crash: report it the way the README does.
    return 0 if result.final_state.value == "IDLE" and result.final_error_m <= 1.5 else 1


if __name__ == "__main__":
    raise SystemExit(main())
