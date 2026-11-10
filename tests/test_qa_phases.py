"""Phase classification and per-phase detection rates."""

from __future__ import annotations

import pytest

from uav_tracking.control.qa import phase_of, summarise_phases


def test_the_ground_is_ground():
    assert phase_of(0.2, 0.0) == "ground"


def test_rising_is_climb_and_sinking_is_descent():
    assert phase_of(3.0, 0.6) == "climb"
    assert phase_of(3.0, -0.6) == "descent"


def test_gentle_vertical_motion_still_counts_as_level():
    assert phase_of(5.0, 0.1) == "level"
    assert phase_of(5.0, -0.1) == "level"


def test_phases_are_counted_separately():
    # ground, climb, climb, level, descent, ground
    samples = [
        (0.0, 0.1, False),
        (1.0, 4.0, False),
        (2.0, 5.0, True),
        (3.0, 5.0, True),
        (4.0, 1.0, False),
        (5.0, 0.1, False),
    ]

    phases = summarise_phases(samples)

    assert phases["ground"]["frames"] == 2
    assert phases["climb"]["frames"] == 2
    assert phases["level"]["frames"] == 1
    assert phases["level"]["detection_rate"] == pytest.approx(1.0)
    assert phases["descent"]["frames"] == 1


def test_the_first_sample_has_no_vertical_speed():
    phases = summarise_phases([(0.0, 5.0, True)])

    assert phases["level"]["detection_rate"] == pytest.approx(1.0)


def test_a_duplicate_timestamp_does_not_divide_by_zero():
    phases = summarise_phases([(0.0, 5.0, True), (0.0, 5.0, False)])

    assert phases["level"]["frames"] == 2
