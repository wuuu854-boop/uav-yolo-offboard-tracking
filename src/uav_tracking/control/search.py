"""Lawnmower search pattern for a lost target.

When the detector loses the target, hovering forever is a plan that only works
if the target comes back on its own. A real mission searches: cover the area
around the last known position in lanes spaced so that consecutive passes
overlap, then give up or return.

Pure geometry and state, so the coverage can be checked without a simulator.
"""

from __future__ import annotations

import math
from collections.abc import Sequence


def coverage_spacing(footprint_width_m: float, overlap: float = 0.25) -> float:
    """Lane spacing that keeps ``overlap`` fraction of the camera footprint.

    With a 11.9 m footprint and 0.25 overlap the lanes are 8.9 m apart, which is
    the same as the footprint height — so a square-ish patch is covered without
    gaps even when the vehicle drifts a little.
    """
    if footprint_width_m <= 0.0:
        raise ValueError("footprint_width_m must be positive")
    if not 0.0 <= overlap < 1.0:
        raise ValueError("overlap must be in [0, 1)")
    return footprint_width_m * (1.0 - overlap)


def lawnmower(
    centre: tuple[float, float],
    width_m: float,
    height_m: float,
    spacing_m: float,
) -> list[tuple[float, float]]:
    """Waypoints covering a rectangle, alternating direction per lane.

    The first waypoint is the south-west corner and the last the north-west one,
    so the vehicle starts from wherever it is and sweeps the area once.
    """
    if width_m <= 0.0 or height_m <= 0.0:
        raise ValueError("width_m and height_m must be positive")
    if spacing_m <= 0.0:
        raise ValueError("spacing_m must be positive")

    lanes = max(1, math.ceil(height_m / spacing_m))
    south = centre[1] - height_m / 2.0
    # lanes + 1 sweep lines, so both edges of the area are covered.
    lane_gap = height_m / lanes
    west, east = centre[0] - width_m / 2.0, centre[0] + width_m / 2.0

    waypoints: list[tuple[float, float]] = []
    for lane in range(lanes + 1):
        north = south + lane * lane_gap
        if lane % 2 == 0:
            waypoints.append((west, north))
            waypoints.append((east, north))
        else:
            waypoints.append((east, north))
            waypoints.append((west, north))
    return waypoints


class SearchPattern:
    """Walks the waypoint list, advancing when the vehicle is close enough."""

    def __init__(self, waypoints: Sequence[tuple[float, float]], tolerance_m: float = 1.0) -> None:
        if not waypoints:
            raise ValueError("waypoints must not be empty")
        if tolerance_m <= 0.0:
            raise ValueError("tolerance_m must be positive")
        self.waypoints = [(float(east), float(north)) for east, north in waypoints]
        self.tolerance_m = float(tolerance_m)
        self.index = 0

    @property
    def done(self) -> bool:
        return self.index >= len(self.waypoints)

    def current(self) -> tuple[float, float] | None:
        return None if self.done else self.waypoints[self.index]

    def update(self, position: tuple[float, float]) -> tuple[float, float] | None:
        """Return the waypoint to fly to now, advancing past reached ones."""
        if self.done:
            return None
        target = self.waypoints[self.index]
        reached = math.hypot(target[0] - position[0], target[1] - position[1]) <= self.tolerance_m
        if reached:
            self.index += 1
        return self.current()
