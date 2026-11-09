"""Pixel -> ground projection for a nadir (downward-looking) camera.

Frame conventions (keep in sync with ``docs/architecture.md``):

* Body frame, PX4 FRD: ``+x`` forward, ``+y`` right, ``+z`` down.
* World frame: ENU, ``+x`` east, ``+y`` north, ``+z`` up.
* Yaw follows ROS REP-103: ``0`` rad faces east, ``+pi/2`` faces north.
* Camera: optical axis points down (body ``+z``). With ``image_up_is_forward``
  (the default) the image "up" direction is body forward, therefore::

      image right (+u) -> body +y (right)
      image down  (+v) -> body -x (backward)

The projection assumes flat ground and a level vehicle. Roll and pitch are not
compensated here; the impact of that assumption is measured and discussed in
``docs/architecture.md``.

Everything in this module is a pure function so it can be unit-tested without
ROS, a simulator, or a model file.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np


def _as_intrinsics(K: Sequence[Sequence[float]]) -> np.ndarray:
    """Validate and normalise a 3x3 camera matrix."""
    matrix = np.asarray(K, dtype=float)
    if matrix.shape != (3, 3):
        raise ValueError(f"K must be a 3x3 matrix, got shape {matrix.shape}")
    if not np.all(np.isfinite(matrix)):
        raise ValueError("K contains non-finite values")
    if matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0:
        raise ValueError("fx and fy must be positive")
    return matrix


def pixel_to_ground(
    u: float,
    v: float,
    h: float,
    K: Sequence[Sequence[float]],
    yaw: float = 0.0,
    image_up_is_forward: bool = True,
) -> tuple[float, float]:
    """Project a pixel to the ground plane.

    Args:
        u: Pixel column, origin top-left.
        v: Pixel row, origin top-left.
        h: Camera height above the ground in metres (must be > 0).
        K: 3x3 camera matrix ``[[fx, 0, cx], [0, fy, cy], [0, 0, 1]]``.
        yaw: Vehicle yaw in radians, ROS REP-103 (0 = east, pi/2 = north).
        image_up_is_forward: Camera mounting. ``True`` means the top of the
            image points to the vehicle's forward direction.

    Returns:
        ``(east, north)`` offset in metres, relative to the camera position.

    Raises:
        ValueError: if the intrinsics or the height are invalid.
    """
    matrix = _as_intrinsics(K)
    if not math.isfinite(h) or h <= 0.0:
        raise ValueError(f"height must be a positive finite number, got {h!r}")

    fx, fy = matrix[0, 0], matrix[1, 1]
    cx, cy = matrix[0, 2], matrix[1, 2]

    right = (float(u) - cx) * h / fx
    down = (float(v) - cy) * h / fy

    if image_up_is_forward:
        forward = -down
    else:
        forward = down

    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    east = forward * cos_yaw + right * sin_yaw
    north = forward * sin_yaw - right * cos_yaw
    return float(east), float(north)


def estimate_target_enu(
    drone_enu: Sequence[float],
    u: float,
    v: float,
    h: float,
    K: Sequence[Sequence[float]],
    yaw: float = 0.0,
    image_up_is_forward: bool = True,
) -> tuple[float, float, float]:
    """Absolute target position in the world ENU frame (z = 0, flat ground)."""
    if len(drone_enu) < 2:
        raise ValueError("drone_enu needs at least (east, north)")
    east, north = pixel_to_ground(u, v, h, K, yaw=yaw, image_up_is_forward=image_up_is_forward)
    return float(drone_enu[0]) + east, float(drone_enu[1]) + north, 0.0


def ground_sample_distance(h: float, K: Sequence[Sequence[float]]) -> float:
    """Ground sampling distance in metres per pixel (useful for range budgeting)."""
    matrix = _as_intrinsics(K)
    if h <= 0.0:
        raise ValueError("height must be positive")
    return float(h / matrix[0, 0])
