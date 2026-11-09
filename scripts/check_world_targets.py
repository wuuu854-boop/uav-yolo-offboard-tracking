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


def structure_problems(path: Path) -> list[str]:
    """Checks that catch the mistakes which silently break a PX4 world."""
    world = ET.parse(path).getroot().find("world")
    if world is None:
        return ["no <world> element"]

    problems: list[str] = []

    plugins = list(world.iter("plugin"))
    if plugins:
        names = ", ".join(plugin.get("filename", "?") for plugin in plugins)
        problems.append(
            f"{len(plugins)} <plugin> element(s) ({names}); "
            "PX4 loads systems from server.config, declaring them again double-loads them"
        )

    physics = world.find("physics")
    if physics is None:
        problems.append("no <physics> element")
    else:
        step = physics.findtext("max_step_size")
        rate = physics.findtext("real_time_update_rate")
        if step != "0.004":
            problems.append(f"max_step_size={step!r}, PX4 worlds use '0.004'")
        if rate != "250":
            problems.append(f"real_time_update_rate={rate!r}, PX4 worlds use '250'")

    if not list(world.iter("light")):
        problems.append("no <light>; the camera would render a black scene")
    if world.find("spherical_coordinates") is None:
        problems.append("no <spherical_coordinates>; NavSat has no origin")

    return problems


def main() -> int:
    structure = structure_problems(WORLD)
    if structure:
        print(f"{WORLD.name} structure:")
        for problem in structure:
            print(f"  - {problem}")

    world = world_poses(WORLD)
    truth = yaml_poses(TARGETS)

    if not world:
        print(f"no target_* models found in {WORLD}")
        return 1

    failures = len(structure)
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

    print(f"OK: {len(world)} targets agree, world structure matches PX4 conventions")
    return 0


if __name__ == "__main__":
    sys.exit(main())
