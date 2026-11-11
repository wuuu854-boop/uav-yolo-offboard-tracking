"""Unit tests for the known-map global planner."""

from __future__ import annotations

import math

import pytest

from uav_tracking.planning.global_planner import Box, obstacle_box_from_measurement, plan_path


def _outside(path, obstacles, inflation):
    for x, y in path:
        for box in obstacles:
            assert not box.contains(x, y, inflation), (x, y, box)


def test_a_depth_measurement_becomes_a_conservative_box():
    box = obstacle_box_from_measurement((0.0, 0.0), 2.0, 0.0, 4.0, 0.4)

    assert (box.centre_x, box.centre_y) == pytest.approx((2.0, 0.0))
    assert (box.size_x, box.size_y) == pytest.approx((0.4, 4.0))


def test_a_rotated_measurement_uses_the_axis_aligned_bound():
    box = obstacle_box_from_measurement((0.0, 0.0), 2.0, math.pi / 2, 4.0, 0.4)

    assert (box.centre_x, box.centre_y) == pytest.approx((0.0, 2.0))
    assert (box.size_x, box.size_y) == pytest.approx((4.0, 0.4))


def test_a_measurement_with_a_bad_distance_is_rejected():
    with pytest.raises(ValueError, match="distance_m"):
        obstacle_box_from_measurement((0.0, 0.0), 0.0, 0.0, 4.0, 0.4)


def test_an_empty_map_plans_straight_to_the_goal():
    path = plan_path((0.0, 0.0), (3.0, 0.0), [], (-5.0, 5.0, -5.0, 5.0))

    assert path
    assert path[-1] == pytest.approx((3.0, 0.0))
    assert len(path) <= 3


def test_a_wall_forces_the_path_around_it():
    wall = Box(1.0, 0.0, 0.4, 3.0)
    path = plan_path(
        (0.0, 0.0),
        (2.0, 0.0),
        [wall],
        (-5.0, 5.0, -5.0, 5.0),
        inflation_m=0.5,
    )

    assert path[-1] == pytest.approx((2.0, 0.0))
    assert max(abs(y) for _, y in path) > 1.5
    _outside(path, [wall], 0.5)


def test_a_sealed_box_has_no_path():
    walls = [
        Box(1.0, 0.0, 0.2, 8.0),
        Box(1.0, 1.0, 4.0, 0.2),
        Box(1.0, -1.0, 4.0, 0.2),
    ]
    with pytest.raises(ValueError, match="no path"):
        plan_path((0.0, 0.0), (2.0, 0.0), walls, (-5.0, 5.0, -4.0, 4.0), inflation_m=0.3)


def test_a_start_inside_an_obstacle_is_rejected():
    with pytest.raises(ValueError, match="start"):
        plan_path((0.0, 0.0), (2.0, 0.0), [Box(0.0, 0.0, 1.0, 1.0)], (-5.0, 5.0, -5.0, 5.0))


def test_the_path_ends_exactly_at_the_goal():
    path = plan_path((0.0, 0.0), (2.3, 1.7), [], (-5.0, 5.0, -5.0, 5.0))
    assert path[-1] == pytest.approx((2.3, 1.7))


def test_inflation_is_honoured():
    wall = Box(1.5, 0.0, 0.2, 2.0)
    path = plan_path(
        (0.0, 0.0),
        (3.0, 0.0),
        [wall],
        (-5.0, 5.0, -5.0, 5.0),
        inflation_m=1.0,
    )
    _outside(path, [wall], 1.0)
    assert max(abs(y) for _, y in path) > 2.0


def test_multiple_boxes_are_routed_around():
    boxes = [
        Box(1.5, 0.0, 0.2, 3.0),
        Box(1.0, -1.2, 0.8, 0.8),
        Box(1.0, 1.2, 0.8, 0.8),
    ]
    path = plan_path(
        (0.0, 0.0),
        (3.0, 0.0),
        boxes,
        (-5.0, 5.0, -5.0, 5.0),
        inflation_m=0.5,
    )

    assert path[-1] == pytest.approx((3.0, 0.0))
    _outside(path, boxes, 0.5)


def test_bad_arguments_are_rejected():
    with pytest.raises(ValueError, match="resolution"):
        plan_path((0.0, 0.0), (1.0, 0.0), [], (-2.0, 2.0, -2.0, 2.0), resolution_m=0.0)
    with pytest.raises(ValueError, match="inflation"):
        plan_path((0.0, 0.0), (1.0, 0.0), [], (-2.0, 2.0, -2.0, 2.0), inflation_m=-1.0)
    with pytest.raises(ValueError, match="bounds"):
        plan_path((0.0, 0.0), (1.0, 0.0), [], (2.0, -2.0, -2.0, 2.0))
    with pytest.raises(ValueError, match="finite"):
        plan_path((math.nan, 0.0), (1.0, 0.0), [], (-2.0, 2.0, -2.0, 2.0))
