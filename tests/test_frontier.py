"""Tests for frontier selection on the depth map."""

from __future__ import annotations

import pytest

from uav_tracking.perception.map_gate import OccupancyGrid
from uav_tracking.planning.frontier import choose_frontier, frontier_cells


def _ray_grid() -> OccupancyGrid:
    grid = OccupancyGrid(bounds=(-1.0, 6.0, -3.0, 3.0), resolution_m=0.5)
    grid.add_points_with_rays([(4.0, 0.0), (4.0, 0.0), (4.0, 0.0)], (0.0, 0.0))
    return grid


def test_frontier_is_at_edge_of_observed_corridor():
    grid = _ray_grid()
    frontiers = frontier_cells(grid)
    assert frontiers
    assert all(grid.cell_centre(cell)[0] < 4.0 for cell in frontiers)


def test_choose_frontier_returns_known_free_path():
    grid = _ray_grid()
    result = choose_frontier(grid, (0.0, 0.0), (5.0, 0.0), max_candidates=10)
    assert result["path"]
    assert result["path_length_m"] > 0.0
    assert result["total_cost_m"] >= result["path_length_m"]


def test_no_frontier_is_rejected():
    grid = OccupancyGrid(bounds=(0.0, 2.0, 0.0, 2.0), resolution_m=0.5)
    grid.free.update((i, j) for i in range(grid.width) for j in range(grid.height))
    with pytest.raises(ValueError, match="no observed frontier"):
        choose_frontier(grid, (0.5, 0.5), (1.5, 1.5))
