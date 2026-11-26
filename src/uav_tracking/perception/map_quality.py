"""Map coverage quality: is the route actually observed?

Known-free planning already blocks unseen cells, but it does not tell the
operator *how much* of the intended corridor was seen. This module samples the
straight corridor and reports the fraction of samples that are observed free.
"""

from __future__ import annotations

import math

from uav_tracking.perception.map_gate import OccupancyGrid, Point


def corridor_coverage(
    grid: OccupancyGrid,
    start: Point,
    goal: Point,
    *,
    half_width_m: float = 0.5,
    step_m: float | None = None,
) -> dict:
    """Fraction of a straight corridor whose cells are observed free."""
    if not (math.isfinite(start[0]) and math.isfinite(start[1])):
        raise ValueError("start must be finite")
    if not (math.isfinite(goal[0]) and math.isfinite(goal[1])):
        raise ValueError("goal must be finite")
    if half_width_m < 0.0:
        raise ValueError("half_width_m must be non-negative")
    step = float(step_m if step_m is not None else grid.resolution_m)
    if step <= 0.0:
        raise ValueError("step_m must be positive")

    distance = math.hypot(goal[0] - start[0], goal[1] - start[1])
    if distance <= step:
        samples = [start]
    else:
        count = int(math.ceil(distance / step))
        samples = [
            (
                start[0] + (goal[0] - start[0]) * i / count,
                start[1] + (goal[1] - start[1]) * i / count,
            )
            for i in range(count + 1)
        ]
    if half_width_m > 0.0:
        length = max(distance, 1e-9)
        nx = -(goal[1] - start[1]) / length
        ny = (goal[0] - start[0]) / length
        offsets = (-half_width_m, 0.0, half_width_m)
        samples = [(x + nx * offset, y + ny * offset) for x, y in samples for offset in offsets]

    observed_free = grid.free
    occupied = set(grid.counts)
    free = 0
    blocked = 0
    unknown = 0
    for x, y in samples:
        cell = grid.index(x, y)
        if cell is None:
            unknown += 1
        elif cell in occupied:
            blocked += 1
        elif cell in observed_free:
            free += 1
        else:
            unknown += 1
    total = len(samples)
    return {
        "samples": total,
        "free": free,
        "blocked": blocked,
        "unknown": unknown,
        "coverage": free / total if total else 0.0,
    }


def coverage_passes(result: dict, *, min_coverage: float = 0.95) -> bool:
    if not 0.0 <= min_coverage <= 1.0:
        raise ValueError("min_coverage must be in [0, 1]")
    return (
        int(result["samples"]) > 0
        and float(result["coverage"]) >= min_coverage
        and int(result["blocked"]) == 0
    )
