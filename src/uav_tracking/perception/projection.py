"""Pixel -> ground projection for a nadir (downward-looking) camera.

Frame conventions (keep in sync with ``docs/architecture.md``):

* Body frame, PX4 FRD: ``+x`` forward, ``+y`` right, ``+z`` down.
* World frame: ENU, ``+x`` east, ``+y`` north, ``+z`` up.
* Yaw follows ROS REP-103: ``0`` rad faces east, ``+pi/2`` faces north.
* Camera: optical axis points down (body ``+z``). With ``image_up_is_forward``
  (the default) the image "up" direction is body forward, therefore::

      image right (+u) -> body +y (right)
      image down  (+v) -> body -x (backward)

The projection assumes flat ground. It used to assume a level vehicle as well;
that is now optional, because a vehicle accelerating at 1 m/s pitches, and 10 deg
of pitch at 5 m moves the ground point by ``5*tan(10 deg) ~= 0.88 m`` - the same
order as the landing error it caused (see ``docs/model.md``).

Everything in this module is a pure function so it can be unit-tested without
ROS, a simulator, or a model file.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np


def _dcm_enu_from_body(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Rotation from body FRD to world ENU.

    ``roll`` and ``pitch`` are the vehicle's PX4 (FRD) angles in radians, and
    ``yaw`` is the ENU yaw (REP-103, 0 = east). The conversion is the standard
    aerospace DCM composed with the NED->ENU axis swap, which is the only place
    that swap appears.
    """
    yaw_ned = math.pi / 2.0 - yaw
    cy, sy = math.cos(yaw_ned), math.sin(yaw_ned)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)

    about_z = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    about_y = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    about_x = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ned_from_body = about_z @ about_y @ about_x

    # v_enu = (north, east, -down) for a v_ned = (n, e, d)
    flip = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
    return flip @ ned_from_body


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
    roll: float = 0.0,
    pitch: float = 0.0,
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
        roll: Vehicle roll in radians, PX4 body frame (positive = right side down).
        pitch: Vehicle pitch in radians, PX4 body frame (positive = nose up).

    Returns:
        ``(east, north)`` offset in metres, relative to the camera position.

    Raises:
        ValueError: if the intrinsics or the height are invalid, or if the ray
            points at or above the horizon.
    """
    matrix = _as_intrinsics(K)
    if not math.isfinite(h) or h <= 0.0:
        raise ValueError(f"height must be a positive finite number, got {h!r}")
    if not (math.isfinite(roll) and math.isfinite(pitch)):
        raise ValueError("roll and pitch must be finite")

    fx, fy = matrix[0, 0], matrix[1, 1]
    cx, cy = matrix[0, 2], matrix[1, 2]

    # Direction of the pixel's ray in body FRD: down is +z, the optical axis is
    # +z, and the pixel offsets are tangents against it.
    right = (float(u) - cx) / fx
    down_offset = (float(v) - cy) / fy
    forward = -down_offset if image_up_is_forward else down_offset
    ray_body = np.array([forward, right, 1.0])

    ray_enu = _dcm_enu_from_body(roll, pitch, yaw) @ ray_body
    if ray_enu[2] >= -1e-9:
        raise ValueError(
            "the pixel's ray does not hit the ground with this attitude "
            f"(direction z = {ray_enu[2]:.3f})"
        )

    # The camera is h above the ground, so the ray needs h / (-dz) metres to
    # reach it.
    scale = h / -ray_enu[2]
    return float(ray_enu[0] * scale), float(ray_enu[1] * scale)


def camera_offset_enu(offset_body: Sequence[float], yaw: float) -> tuple[float, float, float]:
    """Body FLU offset -> ENU offset, rotated by yaw.

    Gazebo models use FLU (x forward, y left, z up), while the projection works
    in ENU. Mixing those two is the third most common frame bug, so the
    conversion lives in one place and is unit tested.
    """
    if len(offset_body) < 3:
        raise ValueError("offset_body needs three components (forward, left, up)")
    forward, left, up = (float(value) for value in offset_body[:3])
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    east = forward * cos_yaw - left * sin_yaw
    north = forward * sin_yaw + left * cos_yaw
    return east, north, up


def estimate_target_enu(
    drone_enu: Sequence[float],
    u: float,
    v: float,
    h: float,
    K: Sequence[Sequence[float]],
    yaw: float = 0.0,
    image_up_is_forward: bool = True,
    offset_body: Sequence[float] = (0.0, 0.0, 0.0),
    roll: float = 0.0,
    pitch: float = 0.0,
) -> tuple[float, float, float]:
    """Absolute target position in the world ENU frame (z = 0, flat ground).

    ``offset_body`` is the camera position relative to the vehicle origin in
    body FLU metres; ``h`` is the vehicle altitude, not the camera altitude.

    ``roll``/``pitch`` are the vehicle's PX4 body angles. Leaving them at zero
    keeps the level-flight behaviour this function had before they existed; a
    caller that has attitude should pass it, because 10 deg of pitch at 5 m is
    almost a metre of ground error.
    """
    if len(drone_enu) < 2:
        raise ValueError("drone_enu needs at least (east, north)")

    offset_east, offset_north, offset_up = camera_offset_enu(offset_body, yaw)
    camera_height = h + offset_up
    if camera_height <= 0.0:
        raise ValueError(f"camera height must be positive, got {camera_height!r}")

    east, north = pixel_to_ground(
        u,
        v,
        camera_height,
        K,
        yaw=yaw,
        image_up_is_forward=image_up_is_forward,
        roll=roll,
        pitch=pitch,
    )
    return float(drone_enu[0]) + offset_east + east, float(drone_enu[1]) + offset_north + north, 0.0


def ground_point_to_pixel(
    point_enu: Sequence[float],
    drone_enu: Sequence[float],
    h: float,
    K: Sequence[Sequence[float]],
    yaw: float = 0.0,
    roll: float = 0.0,
    pitch: float = 0.0,
    image_up_is_forward: bool = True,
    offset_body: Sequence[float] = (0.0, 0.0, 0.0),
) -> tuple[float, float]:
    """Project a world point onto the image; the inverse of estimate_target_enu.

    Used to label real frames: take the target's known ground position, project
    the cube's corners, and the bounding box is the label.

    Raises:
        ValueError: if the point is at or behind the camera plane.
    """
    matrix = _as_intrinsics(K)
    if len(point_enu) < 2 or len(drone_enu) < 2:
        raise ValueError("point_enu and drone_enu need at least (east, north)")

    offset_east, offset_north, offset_up = camera_offset_enu(offset_body, yaw)
    camera = np.array(
        [
            float(drone_enu[0]) + offset_east,
            float(drone_enu[1]) + offset_north,
            float(h) + offset_up,
        ]
    )
    point = np.array([float(point_enu[0]), float(point_enu[1]), 0.0])
    # body = M^T @ world, with M the same body->ENU rotation the projection uses
    delta_body = _dcm_enu_from_body(roll, pitch, yaw).T @ (point - camera)

    forward, right, down = delta_body
    if down <= 1e-6:
        raise ValueError(f"point is at or behind the camera plane (down = {down:.3f})")

    fx, fy = matrix[0, 0], matrix[1, 1]
    cx, cy = matrix[0, 2], matrix[1, 2]
    u = cx + fx * (right / down)
    v = cy - fy * (forward / down) if image_up_is_forward else cy + fy * (forward / down)
    return float(u), float(v)


def ground_sample_distance(h: float, K: Sequence[Sequence[float]]) -> float:
    """Ground sampling distance in metres per pixel (useful for range budgeting)."""
    matrix = _as_intrinsics(K)
    if h <= 0.0:
        raise ValueError("height must be positive")
    return float(h / matrix[0, 0])
