"""The speed limit the README claims, tested."""

from __future__ import annotations

import math

import pytest

from src.control.limiter import limited_step


def test_far_target_moves_exactly_one_step():
    start = (0.0, 0.0)
    end = limited_step(start, (10.0, 0.0), max_step=0.1)
    assert end == pytest.approx((0.1, 0.0))


def test_diagonal_move_is_bounded_by_the_speed_limit():
    # Per-axis clipping would give sqrt(0.1^2 + 0.1^2) = 0.141, i.e. 41 % too fast.
    end = limited_step((0.0, 0.0), (10.0, 10.0), max_step=0.1)
    assert math.hypot(*end) == pytest.approx(0.1, abs=1e-12)


def test_reachable_target_is_returned_exactly():
    assert limited_step((1.0, 1.0), (1.05, 1.0), max_step=0.1) == pytest.approx((1.05, 1.0))


def test_zero_distance_is_a_no_op():
    assert limited_step((2.0, 3.0), (2.0, 3.0), max_step=0.1) == pytest.approx((2.0, 3.0))


def test_works_in_one_dimension_for_altitude():
    assert limited_step((5.0,), (0.0,), max_step=0.5) == pytest.approx((4.5,))


def test_rejects_a_non_positive_step():
    with pytest.raises(ValueError):
        limited_step((0.0, 0.0), (1.0, 1.0), max_step=0.0)


def test_rejects_mismatched_dimensions():
    with pytest.raises(ValueError):
        limited_step((0.0, 0.0), (1.0,), max_step=1.0)
