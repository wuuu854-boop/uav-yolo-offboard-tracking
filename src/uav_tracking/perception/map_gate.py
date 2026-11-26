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
    min_height_m: float = 2.5,
    max_height_m: float = 5.5,
    roi_fraction: float = 1.0,
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
    if not (0.0 < roi_fraction <= 1.0):
        raise ValueError("roi_fraction must be in (0, 1]")

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
    u0 = int(round((1.0 - roi_fraction) * camera.width / 2.0))
    u1 = camera.width - u0
    v0 = int(round((1.0 - roi_fraction) * camera.height / 2.0))
    v1 = camera.height - v0
    for v in range(v0, v1, sample_px):
        for u in range(u0, u1, sample_px):
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


class OccupancyGrid:
    """Small local ENU count grid built from projected depth points."""

    def __init__(
        self,
        bounds: tuple[float, float, float, float] = (-10.0, 10.0, -10.0, 10.0),
        resolution_m: float = 0.25,
    ) -> None:
        x_min, x_max, y_min, y_max = (float(v) for v in bounds)
        if x_min >= x_max or y_min >= y_max:
            raise ValueError("bounds must have min < max on both axes")
        if not math.isfinite(resolution_m) or resolution_m <= 0.0:
            raise ValueError(f"resolution_m must be positive, got {resolution_m!r}")
        self.bounds = (x_min, x_max, y_min, y_max)
        self.resolution_m = float(resolution_m)
        self.width = int(math.ceil((x_max - x_min) / self.resolution_m))
        self.height = int(math.ceil((y_max - y_min) / self.resolution_m))
        self.counts: dict[tuple[int, int], int] = {}
        self.free: set[tuple[int, int]] = set()

    def index(self, x: float, y: float) -> tuple[int, int] | None:
        x_min, x_max, y_min, y_max = self.bounds
        if not (x_min <= x < x_max and y_min <= y < y_max):
            return None
        i = min(int((x - x_min) / self.resolution_m), self.width - 1)
        j = min(int((y - y_min) / self.resolution_m), self.height - 1)
        return i, j

    def add_points(self, points: list[Point]) -> None:
        for x, y in points:
            cell = self.index(x, y)
            if cell is not None:
                self.counts[cell] = self.counts.get(cell, 0) + 1

    def add_points_with_rays(self, points: list[Point], sensor_xy: Point) -> None:
        """Add hits and mark the free cells along each sensor ray.

        Cells before the endpoint are observed free; the endpoint is a hit. This
        is what lets the planner distinguish "unoccupied because unseen" from
        "observed free".
        """
        self.add_points(points)
        sensor_cell = self.index(*sensor_xy)
        if sensor_cell is not None:
            self.free.add(sensor_cell)
        for x, y in points:
            distance = math.hypot(x - sensor_xy[0], y - sensor_xy[1])
            steps = max(1, int(math.ceil(distance / max(self.resolution_m / 2.0, 1e-6))))
            for index in range(steps):
                t = index / steps
                cell = self.index(
                    sensor_xy[0] + (x - sensor_xy[0]) * t,
                    sensor_xy[1] + (y - sensor_xy[1]) * t,
                )
                if cell is not None:
                    self.free.add(cell)

    def observed_cells(self) -> set[tuple[int, int]]:
        return set(self.counts) | self.free

    def occupied_cells(self, min_points: int = 2) -> set[tuple[int, int]]:
        if min_points < 1:
            raise ValueError("min_points must be at least 1")
        return {cell for cell, count in self.counts.items() if count >= min_points}

    def filtered_cells(
        self,
        min_points: int = 2,
        min_neighbours: int = 0,
        min_component: int = 1,
    ) -> set[tuple[int, int]]:
        """Occupied cells, optionally dropping sparse cells/small components."""
        occupied = self.occupied_cells(min_points)
        if min_neighbours > 0:
            neighbours = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1))
            occupied = {
                cell
                for cell in occupied
                if sum((cell[0] + di, cell[1] + dj) in occupied for di, dj in neighbours)
                >= min_neighbours
            }
        if min_component <= 1 or not occupied:
            return occupied
        kept: set[tuple[int, int]] = set()
        unseen = set(occupied)
        neighbours = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1))
        while unseen:
            seed = unseen.pop()
            component = {seed}
            stack = [seed]
            while stack:
                cell = stack.pop()
                for di, dj in neighbours:
                    other = (cell[0] + di, cell[1] + dj)
                    if other in unseen:
                        unseen.remove(other)
                        component.add(other)
                        stack.append(other)
            if len(component) >= min_component:
                kept.update(component)
        return kept

    def closed_cells(
        self,
        min_points: int = 2,
        min_component: int = 3,
        radius: int = 1,
    ) -> set[tuple[int, int]]:
        """Morphological closing: bridge small gaps in a wall/column.

        A depth wall is physically continuous; a few missing cells in a sparse
        frame should not become a doorway. Closing with a 1-2 cell disk bridges
        such gaps without inventing large obstacles.
        """
        if radius < 0:
            raise ValueError("radius must be non-negative")
        occupied = self.filtered_cells(min_points, 0, min_component)
        if radius == 0 or not occupied:
            return occupied
        offsets = [
            (di, dj)
            for di in range(-radius, radius + 1)
            for dj in range(-radius, radius + 1)
            if di * di + dj * dj <= radius * radius
        ]
        dilated = {(i + di, j + dj) for i, j in occupied for di, dj in offsets}
        # A dilation is intentionally conservative here: it bridges a one-cell
        # sparse gap and slightly thickens the wall. The planner adds its own
        # inflation on top, so the path stays outside this conservative surface.
        return dilated | occupied

    def cell_centre(self, cell: tuple[int, int]) -> Point:
        x_min, _, y_min, _ = self.bounds
        return (
            x_min + (cell[0] + 0.5) * self.resolution_m,
            y_min + (cell[1] + 0.5) * self.resolution_m,
        )

    def to_boxes(self, min_points: int = 2) -> list[BoxSpec]:
        boxes = []
        for cell in sorted(self.occupied_cells(min_points)):
            x, y = self.cell_centre(cell)
            boxes.append((x, y, self.resolution_m, self.resolution_m))
        return boxes

    def cell_is_occupied(self, x: float, y: float, min_points: int = 2) -> bool:
        cell = self.index(x, y)
        return cell is not None and self.counts.get(cell, 0) >= min_points


def grid_quality(
    grid: OccupancyGrid,
    boxes: list[BoxSpec],
    *,
    start_xy: Point,
    min_points: int = 2,
    min_neighbours: int = 0,
    min_component: int = 1,
    tolerance_m: float = 0.75,
) -> dict:
    """Check occupied cells against known boxes and a start point."""
    if not boxes:
        raise ValueError("at least one box is required")
    occupied = grid.filtered_cells(min_points, min_neighbours, min_component)
    if not occupied:
        return {
            "cells": 0,
            "precision": 0.0,
            "boxes_covered": 0,
            "boxes_total": len(boxes),
            "start_free": not grid.cell_is_occupied(*start_xy, min_points),
        }
    centres = [grid.cell_centre(cell) for cell in occupied]
    near = [nearest_box_distance(x, y, boxes) <= tolerance_m for x, y in centres]
    covered = 0
    for box in boxes:
        if any(point_to_box_distance(x, y, box) <= tolerance_m for x, y in centres):
            covered += 1
    return {
        "cells": len(occupied),
        "precision": sum(near) / len(occupied),
        "boxes_covered": covered,
        "boxes_total": len(boxes),
        "start_free": not grid.cell_is_occupied(*start_xy, min_points),
        "tolerance_m": float(tolerance_m),
    }
