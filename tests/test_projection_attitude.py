"""Attitude compensation: pitch and roll move the ground point predictably."""

from __future__ import annotations

import math

import pytest

from uav_tracking.perception.projection import estimate_target_enu, pixel_to_ground

K = ((538.4, 0.0, 640.0), (0.0, 538.4, 480.0), (0.0, 0.0, 1.0))
CX, CY = 640.0, 480.0
HEIGHT = 5.0


def test_a_level_vehicle_projects_the_principal_point_to_its_own_position():
    east, north = pixel_to_ground(CX, CY, HEIGHT, K, yaw=0.0)

    assert east == pytest.approx(0.0, abs=1e-9)
    assert north == pytest.approx(0.0, abs=1e-9)


def test_nose_up_pitch_moves_the_footprint_forward():
    pitch = math.radians(10.0)
    east, north = pixel_to_ground(CX, CY, HEIGHT, K, yaw=0.0, pitch=pitch)

    # facing east, nose up: the camera axis tilts towards the nose
    assert east == pytest.approx(HEIGHT * math.tan(pitch), rel=1e-3)
    assert north == pytest.approx(0.0, abs=1e-6)


def test_right_wing_down_roll_moves_the_footprint_left_of_the_heading():
    roll = math.radians(10.0)
    east, north = pixel_to_ground(CX, CY, HEIGHT, K, yaw=0.0, roll=roll)

    # facing east, right wing down: the down direction tilts to the left, which
    # is north
    assert north == pytest.approx(HEIGHT * math.tan(roll), rel=1e-3)
    assert east == pytest.approx(0.0, abs=1e-6)


def test_attitude_rotates_with_yaw():
    pitch = math.radians(10.0)
    east, north = pixel_to_ground(CX, CY, HEIGHT, K, yaw=math.pi / 2.0, pitch=pitch)

    # facing north, nose up: the footprint moves north
    assert north == pytest.approx(HEIGHT * math.tan(pitch), rel=1e-3)
    assert east == pytest.approx(0.0, abs=1e-6)


def test_a_ray_at_the_horizon_is_rejected():
    with pytest.raises(ValueError, match="does not hit the ground"):
        pixel_to_ground(CX, CY, HEIGHT, K, yaw=0.0, pitch=-math.pi / 2.0)


def test_a_steep_pitch_still_hits_the_ground():
    # 45 deg nose up at 5 m: the footprint moves h*tan(45) = 5 m forward and the
    # slant range grows, which is the 1/cos factor
    east, north = pixel_to_ground(CX, CY, HEIGHT, K, yaw=0.0, pitch=math.radians(45.0))

    assert east == pytest.approx(HEIGHT, rel=1e-3)
    assert north == pytest.approx(0.0, abs=1e-6)


def test_estimate_target_enu_applies_the_attitude_it_is_given():
    level = estimate_target_enu((0.0, 0.0), CX, CY, HEIGHT, K)
    pitched = estimate_target_enu((0.0, 0.0), CX, CY, HEIGHT, K, pitch=math.radians(10.0))

    assert pitched[0] - level[0] == pytest.approx(HEIGHT * math.tan(math.radians(10.0)), rel=1e-3)
    assert pitched[1] == pytest.approx(level[1], abs=1e-6)


def test_estimate_target_enu_keeps_the_old_behaviour_by_default():
    # no roll/pitch arguments: identical to the level-flight formula. A +u offset
    # is to the right, which at yaw 0 (facing east) is south, so it changes north.
    east, north, _ = estimate_target_enu((2.0, 3.0), CX + 100.0, CY, HEIGHT, K, yaw=0.0)

    assert east == pytest.approx(2.0)
    assert north == pytest.approx(3.0 - 100.0 * HEIGHT / 538.4, rel=1e-6)

    # a +v offset is backwards, which is west at yaw 0
    east, north, _ = estimate_target_enu((2.0, 3.0), CX, CY + 100.0, HEIGHT, K, yaw=0.0)

    assert east == pytest.approx(2.0 - 100.0 * HEIGHT / 538.4, rel=1e-6)
    assert north == pytest.approx(3.0)
