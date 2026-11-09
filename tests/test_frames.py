"""NED <-> ENU conversions. These bugs are silent, so they get explicit tests."""

from __future__ import annotations

import math

import numpy as np
import pytest

from uav_tracking.control.frames import (
    heading_ned_to_yaw_enu,
    ned_position_to_enu,
    quaternion_to_heading_ned,
    wrap_to_pi,
)


def _yaw_quaternion(yaw_rad: float) -> tuple[float, float, float, float]:
    return (math.cos(yaw_rad / 2.0), 0.0, 0.0, math.sin(yaw_rad / 2.0))


def test_identity_quaternion_is_heading_north():
    assert quaternion_to_heading_ned((1.0, 0.0, 0.0, 0.0)) == pytest.approx(0.0)


def test_90_degree_ned_heading_is_east():
    assert quaternion_to_heading_ned(_yaw_quaternion(math.pi / 2)) == pytest.approx(math.pi / 2)


def test_north_heading_is_90_degrees_in_enu():
    # Pointing north: PX4 heading 0, ROS yaw +pi/2.
    assert heading_ned_to_yaw_enu(0.0) == pytest.approx(math.pi / 2)


def test_east_heading_is_zero_in_enu():
    # Pointing east: PX4 heading +pi/2, ROS yaw 0.
    assert heading_ned_to_yaw_enu(math.pi / 2) == pytest.approx(0.0)


def test_west_heading_is_pi_in_enu():
    assert abs(heading_ned_to_yaw_enu(-math.pi / 2)) == pytest.approx(math.pi)


def test_position_axes_are_swapped_and_z_is_flipped():
    # PX4: 10 m north, 3 m east, 2 m down -> ENU: 3 m east, 10 m north, 2 m up.
    east, north, up = ned_position_to_enu([10.0, 3.0, 2.0])
    assert (east, north, up) == (3.0, 10.0, -2.0)


def test_wrap_to_pi_keeps_values_in_range():
    assert wrap_to_pi(3 * math.pi) == pytest.approx(math.pi)
    assert wrap_to_pi(-3 * math.pi) == pytest.approx(math.pi)
    assert wrap_to_pi(0.5) == pytest.approx(0.5)


def test_quaternion_of_wrong_length_is_rejected():
    with pytest.raises(ValueError):
        quaternion_to_heading_ned(np.array([1.0, 0.0, 0.0]))
