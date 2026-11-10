"""The patrol queue: order, advancement, and the empty cases."""

from __future__ import annotations

import pytest

from uav_tracking.control.patrol import PatrolPlan


def test_an_empty_plan_is_done_immediately():
    plan = PatrolPlan()

    assert plan.done()
    assert plan.current() is None
    assert plan.remaining() == 0


def test_waypoints_come_back_in_order():
    plan = PatrolPlan.from_config([[3.5, 0.0], [23.5, 0.0]])

    assert plan.current() == (3.5, 0.0)
    plan.advance()
    assert plan.current() == (23.5, 0.0)
    plan.advance()
    assert plan.done()
    assert plan.current() is None


def test_remaining_counts_what_is_still_ahead():
    plan = PatrolPlan.from_config([[1.0, 1.0], [2.0, 2.0], [3.0, 3.0]])

    assert plan.remaining() == 3
    plan.advance()
    assert plan.remaining() == 2


def test_advancing_past_the_end_is_harmless():
    plan = PatrolPlan.from_config([[1.0, 1.0]])
    plan.advance()
    plan.advance()

    assert plan.done()
    assert plan.index == 1


def test_a_malformed_waypoint_is_rejected():
    with pytest.raises(ValueError, match="two values"):
        PatrolPlan.from_config([[1.0, 2.0, 3.0]])


def test_values_are_coerced_to_float():
    plan = PatrolPlan.from_config([[1, 2]])

    assert plan.current() == (1.0, 2.0)
    assert isinstance(plan.current()[0], float)
