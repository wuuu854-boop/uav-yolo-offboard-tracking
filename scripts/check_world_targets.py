#!/usr/bin/env python3
"""Check that config/targets.yaml agrees with the world file.

The landing-error metric is only meaningful if the ground truth in the YAML is
the same point the simulator renders. This script makes that check mechanical,
so a moved cube cannot silently invalidate every number in the README.

    python3 scripts/check_world_targets.py

Exit code 0 on success, 1 on any mismatch.
"""

from __future__ import annotations

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
WORLD = REPO_ROOT / "sim" / "worlds" / "target_tracking.sdf"
TARGETS = REPO_ROOT / "config" / "targets.yaml"
TOLERANCE_M = 1e-6


def world_poses(path: Path) -> dict[str, tuple[float, float]]:
    root = ET.parse(path).getroot()
    poses: dict[str, tuple[float, float]] = {}
    for model in root.iter("model"):
        name = model.get("name", "")
        pose = model.find("pose")
        if not name.startswith("target_") or pose is None:
            continue
        values = [float(v) for v in pose.text.split()]
        poses[name] = (values[0], values[1])
    return poses


def yaml_poses(path: Path) -> dict[str, tuple[float, float]]:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    return {item["model"]: (float(item["x"]), float(item["y"])) for item in data["targets"]}


def main() -> int:
    world = world_poses(WORLD)
    truth = yaml_poses(TARGETS)

    if not world:
        print(f"no target_* models found in {WORLD}")
        return 1

    failures = 0
    for name, (x, y) in sorted(world.items()):
        if name not in truth:
            print(f"MISSING in {TARGETS.name}: {name} at ({x}, {y})")
            failures += 1
            continue
        tx, ty = truth[name]
        if abs(tx - x) > TOLERANCE_M or abs(ty - y) > TOLERANCE_M:
            print(f"MISMATCH {name}: world ({x}, {y}) vs yaml ({tx}, {ty})")
            failures += 1

    for name in sorted(set(truth) - set(world)):
        print(f"EXTRA in {TARGETS.name}: {name} (not in the world)")
        failures += 1

    if failures:
        print(f"\n{failures} problem(s). Fix the world or the YAML before running trials.")
        return 1

    print(f"OK: {len(world)} targets agree between the world and the YAML")
    return 0


if __name__ == "__main__":
    sys.exit(main())
