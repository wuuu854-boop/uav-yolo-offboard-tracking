"""Keep the vehicle inside a circle around where it took off.

PX4 has its own geofence and failsafes, but a fence in the controller is the one
that stops a bad target estimate before it becomes a setpoint: a detector that
locks onto a distant object can produce a position tens of metres away, and the
fence is what turns that into "stay at the boundary" instead of "fly there".

Deliberately a circle around home, not a polygon: the mission is a local
approach, and a circle has one parameter a reviewer can check against the
mission's stated range.
"""

from __future__ import annotations

import math


def clamp_to_fence(
    target: tuple[float, float],
    home: tuple[float, float],
    radius_m: float,
) -> tuple[float, float]:
    """Return ``target`` pulled onto the boundary if it is outside the fence.

    ``radius_m <= 0`` disables the fence and returns the target unchanged.
    """
    if radius_m <= 0.0:
        return target

    delta_east = target[0] - home[0]
    delta_north = target[1] - home[1]
    distance = math.hypot(delta_east, delta_north)
    if distance <= radius_m or distance == 0.0:
        return target

    scale = radius_m / distance
    return (home[0] + delta_east * scale, home[1] + delta_north * scale)
