"""Offline depth-map quality gate.

This is the pre-flight check that was missing when the first occupancy-map
attempts failed: take a recorded depth frame and the pose where it was captured,
project sampled pixels into local ENU, and compare the points with known boxes.

The camera convention was measured against a real wall/pillar frame. The image row
increases upward, and the fitted projection uses::

    y_cam = (cy - v) / f * z
    down  = z * sin(tilt) + y_cam * cos(tilt)
    fwd   = mount_forward + z * cos(tilt) - y_cam * sin(tilt)

The fitted frame/pose pair puts ~84% of the sampled points within 1 m of the
known boxes; the opposite sign puts only ~56% and was the bug behind the failed
occupancy-map attempts.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CameraIntrinsics:
    width: int = 640
    height: int = 480
    horizontal_fov_rad: float = 1.2
    down_tilt_rad: float = math.radians(30.0)
    mount_forward_m: float = 0.20
    mount_down_m: float = 0.02

    @property
    def focal_px(self) -> float:
        return (self.width / 2.0) / math.tan(self.horizontal_fov_rad / 2.0)


Point = tuple[float, float]
BoxSpec = tuple[float, float, float, float]


def project_depth(
    depth: np.ndarray,
    position_enu: tuple[float, float, float],
    yaw_rad: float,
    *,
    camera: CameraIntrinsics | None = None,
    sample_px: int = 12,
    min_depth_m: float = 0.3,
    max_depth_m: float = 12.0,
    min_height_m: float = 2.0,
    max_height_m: float = 5.5,
) -> list[Point]:
    """Project sampled z-depth pixels into local ENU, keeping a height band."""
    camera = camera or CameraIntrinsics()
    if depth.ndim != 2:
        raise ValueError(f"depth must be 2-D, got {depth.shape}")
    if depth.shape != (camera.height, camera.width):
        raise ValueError(
            f"depth shape {depth.shape} does not match camera ({camera.height}, {camera.width})"
        )
    if sample_px < 1:
        raise ValueError("sample_px must be at least 1")
    for value in (*position_enu, yaw_rad):
        if not math.isfinite(float(value)):
            raise ValueError("pose and yaw must be finite")
    if not (0.0 <= min_depth_m < max_depth_m):
        raise ValueError("min_depth_m must be < max_depth_m and non-negative")
    if not (0.0 <= min_height_m < max_height_m):
        raise ValueError("min_height_m must be < max_height_m and non-negative")

    east, north, altitude = (float(v) for v in position_enu)
    yaw = float(yaw_rad)
    focal = camera.focal_px
    cx = camera.width / 2.0
    cy = camera.height / 2.0
    cos_tilt = math.cos(camera.down_tilt_rad)
    sin_tilt = math.sin(camera.down_tilt_rad)
    forward_unit = (math.cos(yaw), math.sin(yaw))
    right_unit = (math.sin(yaw), -math.cos(yaw))

    points: list[Point] = []
    for v in range(0, camera.height, sample_px):
        for u in range(0, camera.width, sample_px):
            z = float(depth[v, u])
            if not math.isfinite(z) or z < min_depth_m or z > max_depth_m:
                continue
            x_cam = (u + 0.5 - cx) / focal * z
            y_cam = (cy - (v + 0.5)) / focal * z
            down = z * sin_tilt + y_cam * cos_tilt
            world_z = altitude - camera.mount_down_m - down
            if world_z < min_height_m or world_z > max_height_m:
                continue
            forward = camera.mount_forward_m + z * cos_tilt - y_cam * sin_tilt
            right = x_cam
            points.append(
                (
                    east + forward * forward_unit[0] + right * right_unit[0],
                    north + forward * forward_unit[1] + right * right_unit[1],
                )
            )
    return points


def point_to_box_distance(x: float, y: float, box: BoxSpec) -> float:
    cx, cy, sx, sy = (float(v) for v in box)
    gap_x = max(abs(x - cx) - sx / 2.0, 0.0)
    gap_y = max(abs(y - cy) - sy / 2.0, 0.0)
    return math.hypot(gap_x, gap_y)


def nearest_box_distance(x: float, y: float, boxes: list[BoxSpec]) -> float:
    if not boxes:
        raise ValueError("at least one box is required")
    return min(point_to_box_distance(x, y, box) for box in boxes)


def summarise(
    points: list[Point],
    boxes: list[BoxSpec],
    tolerance_m: float = 1.0,
) -> dict:
    if not boxes:
        raise ValueError("at least one box is required")
    if tolerance_m <= 0.0:
        raise ValueError("tolerance_m must be positive")
    distances = np.array([nearest_box_distance(x, y, boxes) for x, y in points])
    return {
        "points": len(points),
        "near_fraction": float(np.mean(distances <= tolerance_m)) if len(points) else 0.0,
        "median_distance_m": float(np.median(distances)) if len(points) else math.inf,
        "tolerance_m": float(tolerance_m),
    }


def passes(
    summary: dict,
    *,
    min_points: int = 100,
    min_near_fraction: float = 0.70,
    max_median_distance_m: float = 1.0,
) -> bool:
    return (
        int(summary["points"]) >= int(min_points)
        and float(summary["near_fraction"]) >= float(min_near_fraction)
        and float(summary["median_distance_m"]) <= float(max_median_distance_m)
    )
