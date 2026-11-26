"""Grid A* over the depth-derived occupancy map.

This is the Phase B bridge between the offline map gate and a flight planner: the
same :class:`~uav_tracking.perception.map_gate.OccupancyGrid` cells are inflated,
searched with 8-connected A*, and smoothed with a grid line-of-sight check. It is
still a pure function; no ROS or SITL dependency.
"""

from __future__ import annotations

import heapq
import math

from uav_tracking.perception.map_gate import OccupancyGrid


def _inflate(occupied: set[tuple[int, int]], radius: int) -> set[tuple[int, int]]:
    if radius <= 0:
        return set(occupied)
    inflated: set[tuple[int, int]] = set()
    offsets = [
        (di, dj)
        for di in range(-radius, radius + 1)
        for dj in range(-radius, radius + 1)
        if di * di + dj * dj <= radius * radius
    ]
    for i, j in occupied:
        for di, dj in offsets:
            inflated.add((i + di, j + dj))
    return inflated


def _segment_clear(
    grid: OccupancyGrid,
    blocked: set[tuple[int, int]],
    start: tuple[float, float],
    end: tuple[float, float],
) -> bool:
    distance = math.hypot(end[0] - start[0], end[1] - start[1])
    steps = max(1, int(math.ceil(distance / max(grid.resolution_m / 2.0, 1e-6))))
    for index in range(steps + 1):
        t = index / steps
        cell = grid.index(start[0] + (end[0] - start[0]) * t, start[1] + (end[1] - start[1]) * t)
        if cell is None or cell in blocked:
            return False
    return True


def plan_grid_path(
    grid: OccupancyGrid,
    start: tuple[float, float],
    goal: tuple[float, float],
    *,
    inflation_cells: int = 2,
    min_cell_points: int = 2,
    min_component: int = 3,
) -> list[tuple[float, float]]:
    """Plan around occupied cells; returns points after ``start`` ending at ``goal``."""
    for name, value in (("start", start), ("goal", goal)):
        if len(value) != 2 or not all(math.isfinite(float(v)) for v in value):
            raise ValueError(f"{name} must be two finite coordinates, got {value!r}")
    if inflation_cells < 0:
        raise ValueError("inflation_cells must be non-negative")
    start = (float(start[0]), float(start[1]))
    goal = (float(goal[0]), float(goal[1]))
    start_cell = grid.index(*start)
    goal_cell = grid.index(*goal)
    if start_cell is None:
        raise ValueError("start is outside the grid bounds")
    if goal_cell is None:
        raise ValueError("goal is outside the grid bounds")

    occupied = grid.filtered_cells(min_cell_points, 0, min_component)
    blocked = _inflate(occupied, inflation_cells)
    if start_cell in blocked:
        raise ValueError("start is inside an inflated obstacle")
    if goal_cell in blocked:
        raise ValueError("goal is inside an inflated obstacle")

    res = grid.resolution_m

    def centre(cell: tuple[int, int]) -> tuple[float, float]:
        return grid.cell_centre(cell)

    def heuristic(cell: tuple[int, int]) -> float:
        cx, cy = centre(cell)
        return math.hypot(cx - goal[0], cy - goal[1])

    open_heap: list[tuple[float, tuple[int, int]]] = [(heuristic(start_cell), start_cell)]
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    cost: dict[tuple[int, int], float] = {start_cell: 0.0}
    closed: set[tuple[int, int]] = set()
    directions = [
        (1, 0),
        (-1, 0),
        (0, 1),
        (0, -1),
        (1, 1),
        (1, -1),
        (-1, 1),
        (-1, -1),
    ]

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == goal_cell:
            break
        closed.add(current)
        for di, dj in directions:
            neighbour = (current[0] + di, current[1] + dj)
            if not (0 <= neighbour[0] < grid.width and 0 <= neighbour[1] < grid.height):
                continue
            if neighbour in blocked or neighbour in closed:
                continue
            if di != 0 and dj != 0:
                if (current[0] + di, current[1]) in blocked:
                    continue
                if (current[0], current[1] + dj) in blocked:
                    continue
            new_cost = cost[current] + math.hypot(di * res, dj * res)
            if new_cost >= cost.get(neighbour, math.inf):
                continue
            cost[neighbour] = new_cost
            came_from[neighbour] = current
            heapq.heappush(open_heap, (new_cost + heuristic(neighbour), neighbour))

    if goal_cell not in cost:
        raise ValueError("no path exists inside the grid")
    cells = [goal_cell]
    while cells[-1] != start_cell:
        cells.append(came_from[cells[-1]])
    cells.reverse()
    points = [centre(cell) for cell in cells]
    points[0] = start
    points[-1] = goal

    kept = [points[0]]
    index = 0
    while index < len(points) - 1:
        furthest = index + 1
        for candidate in range(len(points) - 1, index, -1):
            if _segment_clear(grid, blocked, points[index], points[candidate]):
                furthest = candidate
                break
        kept.append(points[furthest])
        index = furthest
    if len(kept) < 2 or kept[-1] != goal:
        kept.append(goal)
    # Drop the start itself unless the caller needs a closed loop.
    return kept[1:]
