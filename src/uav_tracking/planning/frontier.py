"""Receding-horizon frontier selection for the depth map.

A single forward frame usually cannot observe a route around a wall. Instead of
planning to the final goal through unknown space, choose an observed-free frontier
cell and plan a known-free path to it. After the vehicle moves, the map grows and
the next frontier is selected. This module only selects the frontier; the flight
integration is separate.
"""

from __future__ import annotations

import math

from uav_tracking.perception.map_gate import OccupancyGrid, Point
from uav_tracking.planning.grid_planner import plan_grid_path


def frontier_cells(
    grid: OccupancyGrid,
    *,
    min_cell_points: int = 2,
    min_component: int = 3,
) -> set[tuple[int, int]]:
    """Observed-free cells with at least one unknown 8-neighbour."""
    occupied = grid.filtered_cells(min_cell_points, 0, min_component)
    unknown = {
        (i, j)
        for i in range(grid.width)
        for j in range(grid.height)
        if (i, j) not in grid.observed_cells()
    }
    neighbours = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1))
    frontiers = set()
    for cell in grid.free:
        if cell in occupied:
            continue
        if any((cell[0] + di, cell[1] + dj) in unknown for di, dj in neighbours):
            frontiers.add(cell)
    return frontiers


def choose_frontier(
    grid: OccupancyGrid,
    start: Point,
    goal: Point,
    *,
    max_candidates: int = 40,
    inflation_cells: int = 2,
    close_radius: int = 1,
) -> dict:
    """Plan known-free to the frontier that best reduces distance to ``goal``."""
    frontier = frontier_cells(grid)
    if not frontier:
        raise ValueError("no observed frontier")
    ranked = sorted(
        frontier,
        key=lambda cell: math.hypot(
            grid.cell_centre(cell)[0] - goal[0],
            grid.cell_centre(cell)[1] - goal[1],
        ),
    )[: max(1, int(max_candidates))]
    best = None
    for cell in ranked:
        target = grid.cell_centre(cell)
        if math.hypot(target[0] - start[0], target[1] - start[1]) <= 1e-9:
            continue
        try:
            path = plan_grid_path(
                grid,
                start,
                target,
                inflation_cells=inflation_cells,
                close_radius=close_radius,
                require_known_free=True,
            )
        except ValueError:
            continue
        length = (
            sum(
                math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
                for i in range(1, len(path))
            )
            if path
            else 0.0
        )
        if path:
            length += math.hypot(path[0][0] - start[0], path[0][1] - start[1])
        total = length + math.hypot(target[0] - goal[0], target[1] - goal[1])
        if best is None or total < best["total_cost_m"]:
            best = {
                "cell": cell,
                "point": target,
                "path": path,
                "path_length_m": length,
                "total_cost_m": total,
            }
    if best is None:
        raise ValueError("no frontier reachable through observed space")
    return best
