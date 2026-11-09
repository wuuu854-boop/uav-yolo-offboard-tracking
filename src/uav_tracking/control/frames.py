"""Frame conversions between PX4 (NED) and ROS / Gazebo (ENU).

These two conventions disagree in two ways at once, which is why they get their
own module and their own tests:

* axes:    NED ``(north, east, down)`` vs ENU ``(east, north, up)``
* heading: PX4 yaw is measured **clockwise from north**,
           ROS REP-103 yaw is measured **counter-clockwise from east**

So ``yaw_enu = pi/2 - yaw_ned``. Skipping the heading conversion gives a constant
90 degree error that looks like a sensor-mounting problem and costs hours.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np


def wrap_to_pi(angle: float) -> float:
    """Wrap an angle to ``(-pi, pi]``."""
    wrapped = math.fmod(angle + math.pi, 2.0 * math.pi)
    if wrapped <= 0.0:
        wrapped += 2.0 * math.pi
    return wrapped - math.pi


def quaternion_to_heading_ned(quaternion: Sequence[float]) -> float:
    """Heading (rad) from a Hamilton quaternion ``(w, x, y, z)``.

    The result is PX4's convention: 0 = north, positive clockwise (towards east).
    """
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,):
        raise ValueError(f"quaternion must have 4 components, got shape {q.shape}")
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def heading_ned_to_yaw_enu(heading_ned: float) -> float:
    """Convert a PX4 heading to a ROS REP-103 yaw (0 = east, CCW positive)."""
    return wrap_to_pi(math.pi / 2.0 - heading_ned)


def ned_position_to_enu(position_ned: Sequence[float]) -> tuple[float, float, float]:
    """``(north, east, down)`` -> ``(east, north, up)``, metres."""
    if len(position_ned) < 3:
        raise ValueError("position_ned needs three components")
    north, east, down = (float(v) for v in position_ned[:3])
    return east, north, -down
