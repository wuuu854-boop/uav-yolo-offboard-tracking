"""The Gazebo worlds and the vehicle model must keep PX4's conventions, not just parse."""

from __future__ import annotations

import shutil

from scripts.check_world_targets import (
    OBSTACLE,
    OBSTACLE_WORLD,
    TARGETS,
    VEHICLE_MODEL,
    WORLD,
    obstacle_problems,
    structure_problems,
    vehicle_model_problems,
    world_poses,
    yaml_poses,
)


def test_repository_world_has_no_structural_problems():
    assert structure_problems(WORLD) == []


def test_repository_obstacle_world_has_no_structural_problems():
    assert structure_problems(OBSTACLE_WORLD) == []


def test_repository_targets_match_the_ground_truth_yaml():
    assert world_poses(WORLD) == yaml_poses(TARGETS)
    assert world_poses(OBSTACLE_WORLD) == yaml_poses(TARGETS)


def test_the_obstacle_and_the_vehicle_model_match_their_configs():
    assert obstacle_problems() == []
    assert vehicle_model_problems() == []


def test_a_wall_moved_in_only_the_world_is_reported(tmp_path):
    world = tmp_path / "obstacle.sdf"
    shutil.copy(OBSTACLE_WORLD, world)
    text = world.read_text(encoding="utf-8")
    world.write_text(text.replace("<pose>-0.8 0 3.0 0 0 0</pose>", "<pose>0.8 0 3.0 0 0 0</pose>"))

    problems = obstacle_problems(world, OBSTACLE)

    assert any("obstacle pose[0]" in problem for problem in problems)


def test_a_missing_contact_system_is_reported(tmp_path):
    world = tmp_path / "obstacle.sdf"
    shutil.copy(OBSTACLE_WORLD, world)
    text = world.read_text(encoding="utf-8")
    text = text.replace(
        '<plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>', ""
    )
    world.write_text(text)

    assert any("contact-system" in problem for problem in obstacle_problems(world, OBSTACLE))


def test_a_vehicle_model_without_a_depth_camera_is_reported(tmp_path):
    model = tmp_path / "model.sdf"
    shutil.copy(WORLD, model)  # any SDF without a depth sensor will do

    assert vehicle_model_problems(model) == ["no depth_camera sensor in the vehicle model"]


def test_a_depth_topic_not_named_depth_camera_is_reported(tmp_path):
    model = tmp_path / "model.sdf"
    shutil.copy(VEHICLE_MODEL, model)
    text = model.read_text(encoding="utf-8")
    model.write_text(text.replace("<topic>depth_camera</topic>", "<topic>other</topic>"))

    assert any("depth topic" in problem for problem in vehicle_model_problems(model))


def test_a_world_without_the_sensors_system_is_reported(tmp_path):
    world = tmp_path / "world.sdf"
    world.write_text(
        """<?xml version="1.0"?>
<sdf version="1.9"><world name="x">
  <physics><max_step_size>0.004</max_step_size><real_time_update_rate>250</real_time_update_rate></physics>
</world></sdf>""",
        encoding="utf-8",
    )
    problems = structure_problems(world)
    assert any("sensors-system" in problem for problem in problems)
    assert any("camera" in problem for problem in problems)


def test_a_duplicate_system_plugin_is_reported(tmp_path):
    world = tmp_path / "world.sdf"
    world.write_text(
        """<?xml version="1.0"?>
<sdf version="1.9"><world name="x">
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <physics><max_step_size>0.004</max_step_size><real_time_update_rate>250</real_time_update_rate></physics>
</world></sdf>""",
        encoding="utf-8",
    )
    assert any("duplicate" in problem for problem in structure_problems(world))


def test_wrong_physics_step_is_reported(tmp_path):
    world = tmp_path / "world.sdf"
    world.write_text(
        """<?xml version="1.0"?>
<sdf version="1.9"><world name="x">
  <physics><max_step_size>0.001</max_step_size><real_time_update_rate>250</real_time_update_rate></physics>
</world></sdf>""",
        encoding="utf-8",
    )
    assert any("max_step_size" in problem for problem in structure_problems(world))
