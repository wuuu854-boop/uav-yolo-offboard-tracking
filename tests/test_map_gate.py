"""Unit tests for the offline depth-map quality gate."""

from __future__ import annotations

import math

import numpy as np
import pytest

from uav_tracking.perception.map_gate import (
    CameraIntrinsics,
    OccupancyGrid,
    grid_quality,
    nearest_box_distance,
    passes,
    project_depth,
    summarise,
)

CAMERA = CameraIntrinsics()


def _plane(depth_m: float) -> np.ndarray:
    return np.full((CAMERA.height, CAMERA.width), depth_m, dtype=np.float32)


def test_forward_plane_projects_ahead():
    points = project_depth(_plane(4.0), (0.0, 0.0, 5.0), 0.0, sample_px=32)

    assert points
    assert min(x for x, _ in points) > 2.5
    assert max(x for x, _ in points) < 5.0


def test_yaw_ninety_projects_north():
    points = project_depth(_plane(4.0), (0.0, 0.0, 5.0), math.pi / 2.0, sample_px=32)

    assert points
    assert min(y for _, y in points) > 2.5
    assert max(y for _, y in points) < 5.0


def test_bad_shape_is_rejected():
    with pytest.raises(ValueError, match="shape"):
        project_depth(np.zeros((10, 10), dtype=np.float32), (0.0, 0.0, 5.0), 0.0)


def test_summarise_counts_near_fraction_and_median():
    summary = summarise([(1.0, 0.0), (3.0, 0.0)], [(0.0, 0.0, 2.0, 2.0)], 0.1)
    assert summary["points"] == 2
    assert summary["near_fraction"] == pytest.approx(0.5)
    assert summary["median_distance_m"] == pytest.approx(1.0)


def test_passes_requires_count_fraction_and_median():
    summary = {"points": 200, "near_fraction": 0.8, "median_distance_m": 0.5}
    assert passes(summary) is True
    assert passes({"points": 10, "near_fraction": 0.9, "median_distance_m": 0.2}) is False
    assert passes({"points": 200, "near_fraction": 0.5, "median_distance_m": 0.2}) is False
    assert passes({"points": 200, "near_fraction": 0.9, "median_distance_m": 2.0}) is False


def test_nearest_box_distance_requires_boxes():
    with pytest.raises(ValueError, match="box"):
        nearest_box_distance(0.0, 0.0, [])


def test_occupancy_grid_counts_and_quality():
    grid = OccupancyGrid(bounds=(-2.0, 2.0, -2.0, 2.0), resolution_m=0.5)
    grid.add_points([(1.1, 0.0), (1.2, 0.1), (1.1, 0.2), (0.0, 0.0)])
    assert grid.occupied_cells(min_points=2) == {(6, 4)}
    assert grid.cell_is_occupied(1.1, 0.0) is True
    assert grid.cell_is_occupied(0.0, 0.0) is False
    quality = grid_quality(grid, [(1.0, 0.0, 0.2, 0.2)], start_xy=(0.0, 0.0))
    assert quality["cells"] == 1
    assert quality["precision"] == pytest.approx(1.0)
    assert quality["boxes_covered"] == 1
    assert quality["start_free"] is True
