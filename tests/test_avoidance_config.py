"""The avoidance scenario's numbers must agree across three configs.

The clearance metric is only meaningful if the wall is really between the spawn
and the waypoint, taller than the flight, and the gate thresholds are inside the
approach. Each of those is a YAML edit away from becoming false, so they are
checked here rather than trusted.
"""

from __future__ import annotations

from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]


def _load(name: str) -> dict:
    with (REPO_ROOT / "config" / name).open(encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def test_the_spawn_and_waypoint_put_the_vehicle_at_target_0():
    mission = _load("mission_avoidance.yaml")
    targets = _load("targets.yaml")
    spawn_east, spawn_north = mission["spawn_offset_enu"]
    waypoint_east, waypoint_north = mission["patrol_waypoints"][0]
    target = next(item for item in targets["targets"] if item["id"] == 0)

    assert spawn_east + waypoint_east == target["x"]
    assert spawn_north + waypoint_north == target["y"]


def test_the_wall_is_between_the_spawn_and_the_waypoint():
    mission = _load("mission_avoidance.yaml")
    obstacle = _load("obstacle.yaml")
    spawn_east = mission["spawn_offset_enu"][0]
    waypoint_east = spawn_east + mission["patrol_waypoints"][0][0]
    wall_east = obstacle["pose_enu"][0]

    assert spawn_east < wall_east < waypoint_east
    # And the wall covers the approach line, not just its centre point.
    spawn_north = mission["spawn_offset_enu"][1]
    waypoint_north = spawn_north + mission["patrol_waypoints"][0][1]
    assert abs(obstacle["pose_enu"][1] - waypoint_north) < obstacle["size_m"][1] / 2.0


def test_the_wall_is_taller_than_the_takeoff_setpoint():
    mission = _load("mission_avoidance.yaml")
    obstacle = _load("obstacle.yaml")

    # A wall below the flight altitude would be cleared without avoidance; the
    # "did not collide" result would then be true for the wrong reason.
    assert obstacle["pose_enu"][2] + obstacle["size_m"][2] / 2.0 > mission["takeoff_altitude_m"]


def test_the_gate_thresholds_are_ordered_and_inside_the_approach():
    mission = _load("mission_avoidance.yaml")
    obstacle = _load("obstacle.yaml")
    gate = _load("depth.yaml")

    assert 0.0 < gate["stop_distance_m"] < gate["clear_distance_m"]
    wall_face = obstacle["pose_enu"][0] - obstacle["size_m"][0] / 2.0
    initial_distance = wall_face - mission["spawn_offset_enu"][0]
    # The gate must not be armed at takeoff and must be able to see the wall.
    assert gate["stop_distance_m"] < initial_distance
    assert gate["max_valid_m"] > initial_distance


def test_the_avoidance_mission_faces_east():
    mission = _load("mission_avoidance.yaml")

    # PX4 yaw is clockwise from north, so 90 is east, which is where the wall is.
    assert mission["heading_yaw_deg"] == 90.0
    assert mission["patrol_waypoints"][0][0] > 0.0
    assert mission["patrol_waypoints"][0][1] == 0.0
