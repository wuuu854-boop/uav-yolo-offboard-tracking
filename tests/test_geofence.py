"""The fence: a target outside the circle is pulled onto it, not rejected."""

from __future__ import annotations

import math

import pytest

from uav_tracking.control.geofence import clamp_to_fence


def test_a_target_inside_the_fence_is_unchanged():
    assert clamp_to_fence((3.0, 4.0), (0.0, 0.0), 10.0) == (3.0, 4.0)


def test_a_target_exactly_on_the_boundary_is_unchanged():
    assert clamp_to_fence((0.0, 10.0), (0.0, 0.0), 10.0) == (0.0, 10.0)


def test_a_target_outside_is_pulled_onto_the_boundary():
    clamped = clamp_to_fence((30.0, 40.0), (0.0, 0.0), 10.0)
    assert clamped == pytest.approx((6.0, 8.0))
    assert math.hypot(*clamped) == pytest.approx(10.0)


def test_the_direction_is_preserved():
    east, north = clamp_to_fence((0.0, 100.0), (0.0, 0.0), 5.0)
    assert east == pytest.approx(0.0)
    assert north == pytest.approx(5.0)


def test_the_fence_is_centred_on_home_not_the_origin():
    # home 20 m east: a target 3 m further east is inside a 5 m fence
    assert clamp_to_fence((23.0, 0.0), (20.0, 0.0), 5.0) == (23.0, 0.0)
    outside = clamp_to_fence((40.0, 0.0), (20.0, 0.0), 5.0)
    assert outside == pytest.approx((25.0, 0.0))


def test_a_non_positive_radius_disables_the_fence():
    assert clamp_to_fence((100.0, 0.0), (0.0, 0.0), 0.0) == (100.0, 0.0)
    assert clamp_to_fence((100.0, 0.0), (0.0, 0.0), -1.0) == (100.0, 0.0)


def test_a_target_at_home_is_returned_as_is():
    assert clamp_to_fence((1.0, 2.0), (1.0, 2.0), 5.0) == (1.0, 2.0)
