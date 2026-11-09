"""Pixel-to-ground projection: the geometry every other result depends on."""

from __future__ import annotations

import math

import numpy as np
import pytest

from src.perception.projection import (
    estimate_target_enu,
    ground_sample_distance,
    pixel_to_ground,
)

K = np.array([[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]])
HEIGHT_M = 5.0


def test_centre_pixel_maps_to_ground_under_the_vehicle():
    east, north = pixel_to_ground(320, 240, HEIGHT_M, K, yaw=0.0)
    assert east == pytest.approx(0.0, abs=1e-9)
    assert north == pytest.approx(0.0, abs=1e-9)


def test_right_of_centre_is_to_the_right_of_the_vehicle():
    # 60 px right, h=5 m, fx=600 -> 0.5 m to the body +y (right).
    # Facing east (yaw=0), "right" is south, i.e. north = -0.5.
    east, north = pixel_to_ground(380, 240, HEIGHT_M, K, yaw=0.0)
    assert east == pytest.approx(0.0, abs=1e-9)
    assert north == pytest.approx(-0.5, abs=1e-9)


def test_below_centre_is_behind_the_vehicle():
    # 60 px below centre, image_up_is_forward=True -> body -x (backward).
    east, north = pixel_to_ground(320, 300, HEIGHT_M, K, yaw=0.0)
    assert east == pytest.approx(-0.5, abs=1e-9)
    assert north == pytest.approx(0.0, abs=1e-9)


def test_yaw_90_rotates_the_offset_into_the_world_frame():
    # Facing north (yaw=pi/2), the vehicle's right side points east.
    east, north = pixel_to_ground(380, 240, HEIGHT_M, K, yaw=math.pi / 2)
    assert east == pytest.approx(0.5, abs=1e-9)
    assert north == pytest.approx(0.0, abs=1e-9)


def test_flipped_camera_mounting_mirrors_the_forward_axis():
    east, north = pixel_to_ground(320, 300, HEIGHT_M, K, yaw=0.0, image_up_is_forward=False)
    assert east == pytest.approx(0.5, abs=1e-9)
    assert north == pytest.approx(0.0, abs=1e-9)


def test_altitude_scales_the_offset_linearly():
    low_east, _ = pixel_to_ground(380, 240, 2.5, K, yaw=0.0)
    high_east, _ = pixel_to_ground(380, 240, 5.0, K, yaw=0.0)
    assert low_east == pytest.approx(0.0, abs=1e-9)
    assert high_east == pytest.approx(0.0, abs=1e-9)
    _, low_north = pixel_to_ground(380, 240, 2.5, K, yaw=0.0)
    assert low_north == pytest.approx(-0.25, abs=1e-9)


def test_non_positive_height_is_rejected():
    with pytest.raises(ValueError):
        pixel_to_ground(320, 240, 0.0, K)
    with pytest.raises(ValueError):
        pixel_to_ground(320, 240, -1.0, K)


def test_bad_intrinsics_are_rejected():
    with pytest.raises(ValueError):
        pixel_to_ground(320, 240, 5.0, np.eye(2))
    with pytest.raises(ValueError):
        pixel_to_ground(320, 240, 5.0, np.array([[0.0, 0, 320], [0, 600, 240], [0, 0, 1]]))


def test_target_enu_adds_the_vehicle_position():
    east, north, up = estimate_target_enu((10.0, 5.0), 380, 240, HEIGHT_M, K, yaw=0.0)
    assert east == pytest.approx(10.0, abs=1e-9)
    assert north == pytest.approx(4.5, abs=1e-9)
    assert up == 0.0


def test_ground_sample_distance_matches_the_range_budget():
    # 5 m altitude, fx=600 -> 8.3 mm per pixel
    assert ground_sample_distance(HEIGHT_M, K) == pytest.approx(5.0 / 600.0, rel=1e-12)
