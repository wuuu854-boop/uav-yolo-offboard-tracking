"""Search geometry: lanes that overlap, and a walker that advances on arrival."""

from __future__ import annotations

import pytest

from uav_tracking.control.search import SearchPattern, coverage_spacing, lawnmower


def test_spacing_keeps_the_requested_overlap():
    assert coverage_spacing(11.9, overlap=0.25) == pytest.approx(8.925)
    assert coverage_spacing(10.0, overlap=0.0) == pytest.approx(10.0)


def test_spacing_rejects_impossible_parameters():
    with pytest.raises(ValueError):
        coverage_spacing(0.0)
    with pytest.raises(ValueError):
        coverage_spacing(10.0, overlap=1.0)


def test_lawnmower_stays_inside_the_rectangle():
    west, east, south, north = 0.0, 20.0, -10.0, 10.0
    waypoints = lawnmower(
        centre=((west + east) / 2.0, (south + north) / 2.0),
        width_m=east - west,
        height_m=north - south,
        spacing_m=5.0,
    )

    for point_east, point_north in waypoints:
        assert west <= point_east <= east
        assert south <= point_north <= north


def test_lane_separation_never_exceeds_the_spacing():
    waypoints = lawnmower(centre=(0.0, 0.0), width_m=20.0, height_m=30.0, spacing_m=8.0)

    lanes = sorted({round(north, 6) for _, north in waypoints})
    for lower, upper in zip(lanes, lanes[1:], strict=False):
        assert upper - lower <= 8.0 + 1e-9


def test_the_sweep_starts_west_and_finishes_east():
    waypoints = lawnmower(centre=(0.0, 0.0), width_m=20.0, height_m=20.0, spacing_m=10.0)

    # Three lanes at -10, 0, +10; even lanes run west->east, so the sweep ends
    # at the north-east corner.
    assert waypoints[0] == pytest.approx((-10.0, -10.0))
    assert waypoints[-1] == pytest.approx((10.0, 10.0))


def test_a_single_lane_area_still_spans_both_edges():
    waypoints = lawnmower(centre=(0.0, 0.0), width_m=10.0, height_m=1.0, spacing_m=10.0)

    # Two lane lines (south and north edge), two waypoints each.
    assert len(waypoints) == 4
    assert {round(north, 6) for _, north in waypoints} == {-0.5, 0.5}


def test_lawnmower_rejects_bad_sizes():
    with pytest.raises(ValueError):
        lawnmower((0.0, 0.0), width_m=0.0, height_m=1.0, spacing_m=1.0)
    with pytest.raises(ValueError):
        lawnmower((0.0, 0.0), width_m=1.0, height_m=1.0, spacing_m=0.0)


def test_the_walker_advances_only_when_close_enough():
    pattern = SearchPattern([(0.0, 0.0), (10.0, 0.0)], tolerance_m=1.0)

    assert pattern.update((5.0, 0.0)) == (0.0, 0.0)  # still flying to the first
    assert pattern.update((0.4, 0.0)) == (10.0, 0.0)  # reached, next one
    assert pattern.index == 1


def test_the_walker_reports_done_at_the_end():
    pattern = SearchPattern([(0.0, 0.0)], tolerance_m=1.0)

    assert pattern.update((0.0, 0.0)) is None
    assert pattern.done
    assert pattern.current() is None


def test_the_walker_rejects_empty_input_and_a_zero_tolerance():
    with pytest.raises(ValueError):
        SearchPattern([])
    with pytest.raises(ValueError):
        SearchPattern([(0.0, 0.0)], tolerance_m=0.0)
