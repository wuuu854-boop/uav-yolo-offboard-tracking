#!/usr/bin/env python3
"""Check that the simulation assets agree with the configs that quote them.

The landing-error metric is only meaningful if the ground truth in the YAML is
the same point the simulator renders. The avoidance clearance metric has the
same problem with the wall: if it moves in the SDF and not in
``config/obstacle.yaml``, the summary silently measures a different obstacle.
Both checks are mechanical here so they cannot rot.

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
OBSTACLE_WORLD = REPO_ROOT / "sim" / "worlds" / "target_tracking_obstacle.sdf"
DETOUR_WORLD = REPO_ROOT / "sim" / "worlds" / "target_tracking_detour.sdf"
BLOCKED_WORLD = REPO_ROOT / "sim" / "worlds" / "target_tracking_blocked.sdf"
TARGETS = REPO_ROOT / "config" / "targets.yaml"
OBSTACLE = REPO_ROOT / "config" / "obstacle.yaml"
SIDE_OBSTACLE = REPO_ROOT / "config" / "side_obstacle.yaml"
BLOCKED_OBSTACLE = REPO_ROOT / "config" / "blocked_obstacle.yaml"
VEHICLE_MODEL = REPO_ROOT / "sim" / "models" / "x500_mono_cam_down" / "model.sdf"
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


def yaml_poses(path: Path = TARGETS) -> dict[str, tuple[float, float]]:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    return {item["model"]: (float(item["x"]), float(item["y"])) for item in data["targets"]}


def structure_problems(path: Path) -> list[str]:
    """Checks that catch the mistakes which silently break a PX4 world."""
    world = ET.parse(path).getroot().find("world")
    if world is None:
        return ["no <world> element"]

    problems: list[str] = []

    # PX4 v1.15's launcher runs `gz sim -r world.sdf` with no --config, so the
    # world itself has to load the systems. Without Sensors the camera sensor is
    # never instantiated and /camera/image_raw stays empty.
    required = {
        "gz-sim-physics-system": "physics (nothing moves)",
        "gz-sim-sensors-system": "rendering sensors (no camera images)",
        "gz-sim-scene-broadcaster-system": "state publishing (no /world/.../pose)",
    }
    declared = [plugin.get("filename", "") for plugin in world.iter("plugin")]
    for filename, consequence in required.items():
        if not any(filename in name for name in declared):
            problems.append(
                f"missing <plugin ... {filename}>: without it there is no {consequence}"
            )

    duplicates = {name for name in declared if declared.count(name) > 1}
    if duplicates:
        problems.append(f"duplicate system plugin(s): {', '.join(sorted(duplicates))}")

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


def obstacle_geometry(
    path: Path = OBSTACLE_WORLD, name: str = "obstacle_wall"
) -> tuple[list, list]:
    """(pose, collision box size) of one model in a world."""
    for model in ET.parse(path).getroot().iter("model"):
        if model.get("name") != name:
            continue
        pose_node = model.find("pose")
        size_node = model.find(".//box/size")
        if pose_node is None or size_node is None:
            raise ValueError(f"{name} has no <pose> or box <size> in {path}")
        return [float(v) for v in pose_node.text.split()], [
            float(v) for v in size_node.text.split()
        ]
    raise ValueError(f"model {name!r} not found in {path}")


def obstacle_problems(world: Path = OBSTACLE_WORLD, config: Path = OBSTACLE) -> list[str]:
    """The wall must match config/obstacle.yaml and load a contact sensor.

    The contact sensor is part of the evidence, not decoration: without the
    contact system plugin the topic is never created and "0 contacts" would be
    produced by a system that was never loaded.
    """
    if not world.exists():
        return [f"{world.name} is missing"]
    if not config.exists():
        return [f"{config.name} is missing"]

    problems = structure_problems(world)
    data = yaml.safe_load(config.read_text(encoding="utf-8"))
    pose, size = obstacle_geometry(world, data["model"])
    expected_pose = [float(v) for v in data["pose_enu"]]
    expected_size = [float(v) for v in data["size_m"]]
    for axis, (actual, expected) in enumerate(zip(pose, expected_pose, strict=False)):
        if abs(actual - expected) > TOLERANCE_M:
            problems.append(f"obstacle pose[{axis}] {actual} != config {expected}")
    for axis, (actual, expected) in enumerate(zip(size, expected_size, strict=False)):
        if abs(actual - expected) > TOLERANCE_M:
            problems.append(f"obstacle size[{axis}] {actual} != config {expected}")

    root = ET.parse(world).getroot()
    plugins = [plugin.get("filename", "") for plugin in root.iter("plugin")]
    if not any("gz-sim-contact-system" in name for name in plugins):
        problems.append("no gz-sim-contact-system: the wall's contact sensor never publishes")
    if not any(sensor.get("type") == "contact" for sensor in root.iter("sensor")):
        problems.append("no contact sensor on the obstacle: 'no contact' would be unmeasured")
    return problems


def detour_side_block_problems(
    world: Path = DETOUR_WORLD,
    config: Path = SIDE_OBSTACLE,
    wall_config: Path = OBSTACLE,
) -> list[str]:
    """The phase-3 side block must match its config and carry a contact sensor.

    The wall and the contact system are checked by :func:`obstacle_problems`;
    this adds the asymmetric block that makes one side genuinely blocked.
    """
    if not world.exists():
        return [f"{world.name} is missing"]
    if not config.exists():
        return [f"{config.name} is missing"]

    problems = obstacle_problems(world, wall_config)
    side = yaml.safe_load(config.read_text(encoding="utf-8"))
    pose, size = obstacle_geometry(world, side["model"])
    expected_pose = [float(v) for v in side["pose_enu"]]
    expected_size = [float(v) for v in side["size_m"]]
    for axis, (actual, expected) in enumerate(zip(pose, expected_pose, strict=False)):
        if abs(actual - expected) > TOLERANCE_M:
            problems.append(f"side_block pose[{axis}] {actual} != config {expected}")
    for axis, (actual, expected) in enumerate(zip(size, expected_size, strict=False)):
        if abs(actual - expected) > TOLERANCE_M:
            problems.append(f"side_block size[{axis}] {actual} != config {expected}")

    root = ET.parse(world).getroot()
    block = next(
        (model for model in root.iter("model") if model.get("name") == side["model"]), None
    )
    if block is None:
        problems.append(f"model {side['model']!r} not found in {world}")
    elif not any(sensor.get("type") == "contact" for sensor in block.iter("sensor")):
        problems.append("side_block has no contact sensor: 'no contact' would be unmeasured")
    return problems


def blocked_world_problems(
    world: Path = BLOCKED_WORLD, config: Path = BLOCKED_OBSTACLE
) -> list[str]:
    """The blocked world's barrier must match its config and have a contact sensor."""
    if not world.exists():
        return [f"{world.name} is missing"]
    if not config.exists():
        return [f"{config.name} is missing"]

    problems = obstacle_problems(world, OBSTACLE)
    spec = yaml.safe_load(config.read_text(encoding="utf-8"))
    pose, size = obstacle_geometry(world, spec["model"])
    expected_pose = [float(v) for v in spec["pose_enu"]]
    expected_size = [float(v) for v in spec["size_m"]]
    for axis, (actual, expected) in enumerate(zip(pose, expected_pose, strict=False)):
        if abs(actual - expected) > TOLERANCE_M:
            problems.append(f"barrier pose[{axis}] {actual} != config {expected}")
    for axis, (actual, expected) in enumerate(zip(size, expected_size, strict=False)):
        if abs(actual - expected) > TOLERANCE_M:
            problems.append(f"barrier size[{axis}] {actual} != config {expected}")

    root = ET.parse(world).getroot()
    barrier = next(
        (model for model in root.iter("model") if model.get("name") == spec["model"]), None
    )
    if barrier is None:
        problems.append(f"model {spec['model']!r} not found in {world}")
    elif not any(sensor.get("type") == "contact" for sensor in barrier.iter("sensor")):
        problems.append("barrier has no contact sensor: 'no contact' would be unmeasured")
    return problems


def vehicle_model_problems(path: Path = VEHICLE_MODEL) -> list[str]:
    """The installed vehicle model must carry the forward depth camera."""
    if not path.exists():
        return [f"{path.name} is missing"]
    root = ET.parse(path).getroot()
    sensors = [sensor for sensor in root.iter("sensor") if sensor.get("type") == "depth_camera"]
    if not sensors:
        return ["no depth_camera sensor in the vehicle model"]

    problems: list[str] = []
    sensor = sensors[0]
    if sensor.findtext("camera/image/format") != "R_FLOAT32":
        problems.append("depth camera format is not R_FLOAT32; the bridge expects float32 metres")
    if sensor.findtext("camera/image/width") != "640" or (
        sensor.findtext("camera/image/height") != "480"
    ):
        problems.append("depth camera is not 640x480; config/depth.yaml and the ROI assume it")
    if sensor.findtext("topic") != "depth_camera":
        problems.append("depth topic is not 'depth_camera'; scripts/run_avoidance.sh expects it")
    return problems


def main() -> int:
    problems: list[str] = []

    for world in (WORLD, OBSTACLE_WORLD, DETOUR_WORLD, BLOCKED_WORLD):
        problems.extend(f"{world.name}: {problem}" for problem in structure_problems(world))

    truth = yaml_poses(TARGETS)
    for world in (WORLD, OBSTACLE_WORLD, DETOUR_WORLD, BLOCKED_WORLD):
        poses = world_poses(world)
        if not poses:
            problems.append(f"no target_* models found in {world}")
            continue
        for name, (x, y) in sorted(poses.items()):
            if name not in truth:
                problems.append(f"{world.name}: MISSING in {TARGETS.name}: {name} at ({x}, {y})")
                continue
            tx, ty = truth[name]
            if abs(tx - x) > TOLERANCE_M or abs(ty - y) > TOLERANCE_M:
                problems.append(
                    f"{world.name}: MISMATCH {name}: world ({x}, {y}) vs yaml ({tx}, {ty})"
                )

    problems.extend(obstacle_problems())
    problems.extend(detour_side_block_problems())
    problems.extend(blocked_world_problems())
    problems.extend(vehicle_model_problems())

    if problems:
        for problem in problems:
            print(f"  - {problem}")
        print(f"\n{len(problems)} problem(s). Fix the simulation assets before running trials.")
        return 1

    print(
        f"OK: {len(world_poses(WORLD))} targets agree in four worlds, "
        "the wall, the side block and the barrier match their configs, "
        "the vehicle model has the depth camera"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
