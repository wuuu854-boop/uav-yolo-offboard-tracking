"""The Gazebo world must keep PX4's conventions, not just parse."""

from __future__ import annotations

from scripts.check_world_targets import (
    TARGETS,
    WORLD,
    structure_problems,
    world_poses,
    yaml_poses,
)


def test_repository_world_has_no_structural_problems():
    assert structure_problems(WORLD) == []


def test_repository_targets_match_the_ground_truth_yaml():
    assert world_poses(WORLD) == yaml_poses(TARGETS)


def test_plugin_tag_is_reported(tmp_path):
    world = tmp_path / "world.sdf"
    world.write_text(
        """<?xml version="1.0"?>
<sdf version="1.9"><world name="x">
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <physics><max_step_size>0.004</max_step_size><real_time_update_rate>250</real_time_update_rate></physics>
</world></sdf>""",
        encoding="utf-8",
    )
    assert any("plugin" in problem for problem in structure_problems(world))


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
