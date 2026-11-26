"""Tests for corridor coverage quality."""

from __future__ import annotations

import pytest

from uav_tracking.perception.map_gate import OccupancyGrid
from uav_tracking.perception.map_quality import corridor_coverage, coverage_passes


def _ray_grid() -> OccupancyGrid:
    grid = OccupancyGrid(bounds=(-1.0, 6.0, -3.0, 3.0), resolution_m=0.5)
    grid.add_points_with_rays([(4.0, 0.0), (4.0, 0.0), (4.0, 0.0)], (0.0, 0.0))
    return grid


def test_observed_corridor_has_full_coverage():
    result = corridor_coverage(_ray_grid(), (0.0, 0.0), (3.0, 0.0), half_width_m=0.0)
    assert result["coverage"] == pytest.approx(1.0)
    assert coverage_passes(result) is True


def test_unobserved_side_corridor_is_rejected():
    result = corridor_coverage(_ray_grid(), (0.0, 0.0), (0.0, 2.0), half_width_m=0.0)
    assert result["coverage"] < 0.95
    assert coverage_passes(result) is False


def test_goal_behind_wall_is_blocked():
    result = corridor_coverage(_ray_grid(), (0.0, 0.0), (5.0, 0.0), half_width_m=0.0)
    assert result["blocked"] > 0
    assert coverage_passes(result) is False


def test_invalid_half_width_is_rejected():
    with pytest.raises(ValueError, match="half_width"):
        corridor_coverage(_ray_grid(), (0.0, 0.0), (1.0, 0.0), half_width_m=-1.0)
