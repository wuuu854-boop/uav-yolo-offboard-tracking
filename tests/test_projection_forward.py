"""Forward projection: a ground point back to a pixel, and the round trip."""

from __future__ import annotations

import math

import pytest

from uav_tracking.perception.projection import (
    estimate_target_enu,
    ground_point_to_pixel,
    pixel_to_ground,
)

K = ((538.4, 0.0, 640.0), (0.0, 538.4, 480.0), (0.0, 0.0, 1.0))
HEIGHT = 5.0


@pytest.mark.parametrize("u,v", [(640.0, 480.0), (500.0, 400.0), (800.0, 600.0)])
def test_pixel_ground_pixel_round_trips(u, v):
    east, north = pixel_to_ground(u, v, HEIGHT, K, yaw=0.3, roll=0.05, pitch=-0.04)

    back_u, back_v = ground_point_to_pixel(
        (east, north), (0.0, 0.0), HEIGHT, K, yaw=0.3, roll=0.05, pitch=-0.04
    )

    assert back_u == pytest.approx(u, abs=1e-6)
    assert back_v == pytest.approx(v, abs=1e-6)


def test_the_principal_point_maps_to_the_camera_position():
    # centre pixel at level attitude: the ground point is directly below
    u, v = ground_point_to_pixel((0.0, 0.0), (0.0, 0.0), HEIGHT, K)

    assert u == pytest.approx(640.0, abs=1e-6)
    assert v == pytest.approx(480.0, abs=1e-6)


def test_a_point_under_the_vehicle_projects_to_the_principal_point():
    east, north, _ = estimate_target_enu((0.0, 0.0), 640.0, 480.0, HEIGHT, K)
    u, v = ground_point_to_pixel((east, north), (0.0, 0.0), HEIGHT, K)

    assert u == pytest.approx(640.0, abs=1e-6)
    assert v == pytest.approx(480.0, abs=1e-6)


def test_a_point_behind_the_camera_is_rejected():
    with pytest.raises(ValueError, match="behind the camera"):
        ground_point_to_pixel((0.0, 0.0), (0.0, 0.0), -1.0, K)


def test_pitch_moves_the_projection_like_a_real_camera():
    level_u, level_v = ground_point_to_pixel((0.0, 0.0), (0.0, 0.0), HEIGHT, K)
    pitched_u, pitched_v = ground_point_to_pixel(
        (0.0, 0.0), (0.0, 0.0), HEIGHT, K, pitch=math.radians(10.0)
    )

    # nose up: the ground below the vehicle moves down in the image (v grows)
    assert pitched_v > level_v
    assert pitched_u == pytest.approx(level_u)
