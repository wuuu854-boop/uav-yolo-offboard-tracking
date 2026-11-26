"""Unit tests for the grid A* planner."""

from __future__ import annotations

import pytest

from uav_tracking.perception.map_gate import OccupancyGrid
from uav_tracking.planning.grid_planner import plan_grid_path


def _grid_with_wall(gap: bool = True) -> OccupancyGrid:
    grid = OccupancyGrid(bounds=(-1.0, 6.0, -2.0, 5.0), resolution_m=0.5)
    wall_x = grid.cell_centre((6, 0))[0]
    for j in range(0, 14):
        y = grid.cell_centre((0, j))[1]
        if gap and 2.0 <= y <= 4.0:
            continue
        for _ in range(3):
            grid.add_points([(wall_x, y)])
    return grid


def test_path_goes_through_the_gap():
    grid = _grid_with_wall(gap=True)
    path = plan_grid_path(grid, (0.0, 3.0), (5.0, 3.0), inflation_cells=1)

    assert path
    assert path[-1] == (5.0, 3.0)
    assert all(grid.index(x, y) is not None for x, y in path)
    assert all(grid.counts.get(grid.index(x, y), 0) < 3 for x, y in path)


def test_full_wall_has_no_path():
    grid = _grid_with_wall(gap=False)
    with pytest.raises(ValueError, match="no path"):
        plan_grid_path(grid, (0.0, 3.0), (5.0, 3.0), inflation_cells=1)


def test_start_inside_wall_is_rejected():
    grid = _grid_with_wall(gap=False)
    with pytest.raises(ValueError, match="start"):
        plan_grid_path(grid, (2.5, 0.0), (5.0, 2.5), inflation_cells=1)


def test_goal_outside_bounds_is_rejected():
    grid = _grid_with_wall()
    with pytest.raises(ValueError, match="outside"):
        plan_grid_path(grid, (0.0, 3.0), (10.0, 3.0), inflation_cells=1)


def _ray_grid() -> OccupancyGrid:
    grid = OccupancyGrid(bounds=(-1.0, 3.0, -1.0, 3.0), resolution_m=0.5)
    grid.add_points_with_rays([(2.0, 0.0), (2.0, 0.0), (2.0, 0.0)], (0.0, 0.0))
    return grid


def test_known_free_path_uses_observed_cells():
    grid = _ray_grid()
    path = plan_grid_path(grid, (0.0, 0.0), (1.0, 0.0), inflation_cells=0, require_known_free=True)
    assert path
    assert path[-1] == (1.0, 0.0)


def test_closing_bridges_a_one_cell_gap():
    grid = OccupancyGrid(bounds=(-1.0, 3.0, -1.0, 3.0), resolution_m=0.5)
    wall_i = 6
    for j in range(0, 8):
        if j == 4:
            continue
        x, y = grid.cell_centre((wall_i, j))
        for _ in range(3):
            grid.add_points([(x, y)])
    gap = grid.cell_centre((wall_i, 4))
    assert grid.index(*gap) not in grid.filtered_cells(2, 0, 3)
    assert grid.index(*gap) in grid.closed_cells(2, 3, 1)


def test_unknown_goal_is_rejected_when_known_free_is_required():
    grid = _ray_grid()
    with pytest.raises(ValueError, match="not observed"):
        plan_grid_path(grid, (0.0, 0.0), (0.0, 2.0), inflation_cells=0, require_known_free=True)
