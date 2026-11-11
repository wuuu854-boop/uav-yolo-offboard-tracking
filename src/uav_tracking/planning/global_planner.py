"""A small known-map global planner for the SITL demo.

The local avoidance stack reacts to the depth camera. This module is the other
half of the question: given a map of axis-aligned boxes, plan a path before
flying. It is deliberately a grid A* with line-of-sight smoothing, not a
production planner: one static 2-D map, no dynamics, no sensor uncertainty.

ENU convention: +x east, +y north.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Box:
    """Axis-aligned obstacle in world ENU metres."""

    centre_x: float
    centre_y: float
    size_x: float
    size_y: float

    def contains(self, x: float, y: float, inflation_m: float = 0.0) -> bool:
        half_x = self.size_x / 2.0 + float(inflation_m)
        half_y = self.size_y / 2.0 + float(inflation_m)
        return abs(x - self.centre_x) <= half_x and abs(y - self.centre_y) <= half_y


def _blocked(x: float, y: float, obstacles: list[Box], inflation_m: float) -> bool:
    return any(box.contains(x, y, inflation_m) for box in obstacles)


def _segment_clear(
    start: tuple[float, float],
    end: tuple[float, float],
    obstacles: list[Box],
    inflation_m: float,
    step_m: float,
) -> bool:
    distance = math.hypot(end[0] - start[0], end[1] - start[1])
    steps = max(1, int(math.ceil(distance / max(step_m, 1e-6))))
    for index in range(steps + 1):
        t = index / steps
        x = start[0] + (end[0] - start[0]) * t
        y = start[1] + (end[1] - start[1]) * t
        if _blocked(x, y, obstacles, inflation_m):
            return False
    return True


def _simplify(
    points: list[tuple[float, float]],
    obstacles: list[Box],
    inflation_m: float,
    step_m: float,
) -> list[tuple[float, float]]:
    if len(points) <= 2:
        return points
    kept = [points[0]]
    index = 0
    while index < len(points) - 1:
        furthest = index + 1
        for candidate in range(len(points) - 1, index, -1):
            if _segment_clear(points[index], points[candidate], obstacles, inflation_m, step_m):
                furthest = candidate
                break
        kept.append(points[furthest])
        index = furthest
    return kept


def plan_path(
    start: tuple[float, float],
    goal: tuple[float, float],
    obstacles: list[Box],
    bounds: tuple[float, float, float, float],
    *,
    resolution_m: float = 0.25,
    inflation_m: float = 0.5,
) -> list[tuple[float, float]]:
    """Plan from ``start`` to ``goal`` around inflated axis-aligned boxes.

    Returns a list of ENU points beginning after the start and ending exactly at
    the goal. Raises ``ValueError`` when the start/goal is blocked or no path
    exists inside ``bounds`` (xmin, xmax, ymin, ymax).
    """
    for name, value in (("start", start), ("goal", goal)):
        if len(value) != 2 or not all(math.isfinite(float(v)) for v in value):
            raise ValueError(f"{name} must be two finite coordinates, got {value!r}")
    if len(bounds) != 4 or not all(math.isfinite(float(v)) for v in bounds):
        raise ValueError(f"bounds must be four finite values, got {bounds!r}")
    if not math.isfinite(float(resolution_m)) or resolution_m <= 0.0:
        raise ValueError(f"resolution_m must be positive and finite, got {resolution_m!r}")
    if not math.isfinite(float(inflation_m)) or inflation_m < 0.0:
        raise ValueError(f"inflation_m must be non-negative and finite, got {inflation_m!r}")

    start = (float(start[0]), float(start[1]))
    goal = (float(goal[0]), float(goal[1]))
    x_min, x_max, y_min, y_max = (float(v) for v in bounds)
    if x_min >= x_max or y_min >= y_max:
        raise ValueError("bounds must have min < max on both axes")
    if _blocked(*start, obstacles, inflation_m):
        raise ValueError("start is inside an inflated obstacle")
    if _blocked(*goal, obstacles, inflation_m):
        raise ValueError("goal is inside an inflated obstacle")

    resolution = float(resolution_m)
    nx = max(2, int(math.ceil((x_max - x_min) / resolution)))
    ny = max(2, int(math.ceil((y_max - y_min) / resolution)))
    step_x = (x_max - x_min) / nx
    step_y = (y_max - y_min) / ny

    def centre(cell: tuple[int, int]) -> tuple[float, float]:
        return (x_min + (cell[0] + 0.5) * step_x, y_min + (cell[1] + 0.5) * step_y)

    def cell_of(point: tuple[float, float]) -> tuple[int, int]:
        i = min(max(int((point[0] - x_min) / step_x), 0), nx - 1)
        j = min(max(int((point[1] - y_min) / step_y), 0), ny - 1)
        return (i, j)

    blocked: set[tuple[int, int]] = set()
    for i in range(nx):
        for j in range(ny):
            if _blocked(*centre((i, j)), obstacles, inflation_m):
                blocked.add((i, j))

    start_cell = cell_of(start)
    goal_cell = cell_of(goal)
    if start_cell in blocked or goal_cell in blocked:
        raise ValueError("start or goal grid cell is inside an inflated obstacle")

    def heuristic(cell: tuple[int, int]) -> float:
        return math.hypot(centre(cell)[0] - goal[0], centre(cell)[1] - goal[1])

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
            if not (0 <= neighbour[0] < nx and 0 <= neighbour[1] < ny):
                continue
            if neighbour in blocked or neighbour in closed:
                continue
            if di != 0 and dj != 0:
                # No corner cutting: both cardinal cells must be open.
                if (current[0] + di, current[1]) in blocked:
                    continue
                if (current[0], current[1] + dj) in blocked:
                    continue
            new_cost = cost[current] + math.hypot(di * step_x, dj * step_y)
            if new_cost >= cost.get(neighbour, math.inf):
                continue
            cost[neighbour] = new_cost
            came_from[neighbour] = current
            heapq.heappush(open_heap, (new_cost + heuristic(neighbour), neighbour))

    if goal_cell not in cost:
        raise ValueError("no path exists inside the bounds")

    cells = [goal_cell]
    while cells[-1] != start_cell:
        cells.append(came_from[cells[-1]])
    cells.reverse()
    points = [centre(cell) for cell in cells]
    points[0] = start
    points[-1] = goal
    points = _simplify(points, obstacles, inflation_m, resolution)
    if len(points) < 2 or points[-1] != goal:
        points.append(goal)
    return points[1:]
