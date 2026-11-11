"""Unit tests for the avoidance report's geometry and verdict.

The report is what turns a CSV into the numbers quoted in the docs. A wrong
point-to-box distance or a verdict that ignores a contact would quietly bless a
bad run, so both are tested on synthetic rows.
"""

from __future__ import annotations

import math

import pytest

from scripts.avoidance_report import (
    analyse,
    longest_stop_run,
    point_to_aabb_distance,
    point_to_box_distance,
    verdict,
)

OBSTACLE = {
    "model": "obstacle_wall",
    "pose_enu": [-0.8, 0.0, 3.0],
    "size_m": [0.2, 3.0, 6.0],
    "min_clearance_m": 1.0,
}
MISSION = {"spawn_offset_enu": [-7.5, 0.0]}
BOUNDS = (-0.9, -0.7, -1.5, 1.5)


def test_distance_inside_the_box_is_zero():
    assert point_to_aabb_distance(-0.8, 0.0, BOUNDS) == 0.0


def test_distance_to_a_face_is_the_perpendicular_gap():
    assert point_to_aabb_distance(-3.9, 0.0, BOUNDS) == 3.0


def test_distance_to_a_corner_includes_both_axes():
    assert point_to_aabb_distance(-2.9, 2.5, BOUNDS) == math.hypot(2.0, 1.0)


def test_distance_to_a_rotated_box():
    # A 2 x 0.2 box rotated 90 degrees is long on y, so (0, 2) is inside and
    # (2, 0) is 1.9 m away.
    assert point_to_box_distance(0.0, 0.5, (0.0, 0.0), (2.0, 0.2), 90.0) == 0.0
    assert point_to_box_distance(2.0, 0.0, (0.0, 0.0), (2.0, 0.2), 90.0) == pytest.approx(1.9)


def test_longest_stop_run_picks_the_longest_of_several():
    rows = [{"stop": False}, {"stop": True}, {"stop": True}, {"stop": False}, {"stop": True}]

    assert longest_stop_run(rows) == (1, 2)


def test_longest_stop_run_handles_a_run_that_reaches_the_end():
    rows = [{"stop": False}, {"stop": True}, {"stop": True}]

    assert longest_stop_run(rows) == (1, 2)


def _rows(stopped: bool = True, contacts: int = 0) -> list[dict]:
    rows = []
    for second in range(30):
        rows.append(
            {
                "t_s": float(second),
                "east_m": 4.5,
                "north_m": 0.0,
                "altitude_m": 5.0,
                "yaw_deg": 0.0,
                "nearest_m": 2.3,
                "stop": bool(stopped and second >= 10),
                "contacts_total": contacts,
                "depth_frames": second * 10,
            }
        )
    return rows


def test_a_clean_run_passes_every_check():
    report = verdict(analyse(_rows(), OBSTACLE, MISSION), OBSTACLE)

    assert report["verdict"] == "PASS"
    assert report["metrics"]["min_clearance_m"] == 2.1
    assert report["metrics"]["settled_drift_m"] == 0.0


def test_a_contact_fails_the_run_even_when_the_geometry_is_fine():
    report = verdict(analyse(_rows(contacts=2), OBSTACLE, MISSION), OBSTACLE)

    assert report["verdict"] == "FAIL"
    assert report["checks"]["no_contact"]["ok"] is False


def test_a_run_that_never_stops_fails():
    report = verdict(analyse(_rows(stopped=False), OBSTACLE, MISSION), OBSTACLE)

    assert report["verdict"] == "FAIL"
    assert report["checks"]["stop_engaged"]["ok"] is False


def _detour_rows(
    stop_released: bool = True,
    contact_count: int = 0,
    lateral_m: float = 7.0,
    side: str = "right",
):
    """A synthetic path: approach, sideways leg, diagonal return to target_0."""
    sign = 1.0 if side == "left" else -1.0
    rows = []
    for second in range(30):
        if second < 10:
            east, north = 0.5 * second, 0.0
        elif second < 20:
            east, north = 5.0, sign * lateral_m * (second - 9) / 10.0
        elif second < 25:
            east, north = 5.0 + 2.5 * (second - 19) / 5.0, sign * lateral_m
        else:
            east, north = 7.5, sign * lateral_m * (1.0 - (second - 24) / 5.0)
        rows.append(
            {
                "t_s": float(second),
                "east_m": east,
                "north_m": north,
                "altitude_m": 5.0,
                "yaw_deg": 0.0,
                "nearest_m": 2.3,
                "stop": 8 <= second <= (15 if stop_released else 29),
                "clear_side": side,
                "contacts_total": contact_count,
                "depth_frames": second * 10,
            }
        )
    return rows


DETOUR_MISSION = {"detour_min_lateral_m": 6.5, "detour_min_clearance_m": 0.4}


def test_a_detour_that_goes_around_and_reaches_the_target_passes():
    report = verdict(
        analyse(_detour_rows(), OBSTACLE, MISSION),
        OBSTACLE,
        mode="detour",
        mission=DETOUR_MISSION,
        trace_notes=["detour: detour started"],
    )

    assert report["verdict"] == "PASS"
    assert report["metrics"]["max_abs_north_m"] == 7.0
    assert report["metrics"]["final_error_m"] == 0.0


def test_a_detour_that_never_releases_the_gate_fails():
    report = verdict(
        analyse(_detour_rows(stop_released=False), OBSTACLE, MISSION),
        OBSTACLE,
        mode="detour",
        mission=DETOUR_MISSION,
    )

    assert report["verdict"] == "FAIL"
    assert report["checks"]["gate_released"]["ok"] is False


def test_a_detour_that_moves_less_than_the_minimum_lateral_distance_fails():
    report = verdict(
        analyse(_detour_rows(lateral_m=2.0), OBSTACLE, MISSION),
        OBSTACLE,
        mode="detour",
        mission=DETOUR_MISSION,
    )

    assert report["checks"]["lateral_leg_at_least_min"]["ok"] is False


def test_a_detour_with_a_contact_fails():
    report = verdict(
        analyse(_detour_rows(contact_count=1), OBSTACLE, MISSION),
        OBSTACLE,
        mode="detour",
        mission=DETOUR_MISSION,
    )

    assert report["checks"]["no_contact"]["ok"] is False


def _global_rows(contacts: int = 0) -> list[dict]:
    rows = _rows(stopped=False, contacts=contacts)
    for row in rows:
        row["east_m"] = 7.5
    return rows


def test_a_clean_global_run_passes():
    report = verdict(analyse(_global_rows(), OBSTACLE, MISSION), OBSTACLE, mode="global")

    assert report["verdict"] == "PASS"
    assert report["metrics"]["final_error_m"] == pytest.approx(0.0)


def test_a_global_run_with_a_contact_fails():
    report = verdict(analyse(_global_rows(contacts=1), OBSTACLE, MISSION), OBSTACLE, mode="global")

    assert report["checks"]["no_contact"]["ok"] is False


def test_an_unknown_report_mode_is_rejected():
    with pytest.raises(ValueError, match="unknown report mode"):
        verdict(analyse(_rows(), OBSTACLE, MISSION), OBSTACLE, mode="teleport")


def test_an_auto_detour_that_chose_the_expected_side_passes():
    report = verdict(
        analyse(_detour_rows(side="left"), OBSTACLE, MISSION),
        OBSTACLE,
        mode="detour",
        mission=DETOUR_MISSION,
        expect_side="left",
    )

    assert report["verdict"] == "PASS"
    assert report["metrics"]["clear_side_mode"] == "left"
    assert report["checks"]["side_selected"]["ok"] is True


def test_an_auto_detour_that_chose_the_other_side_fails():
    report = verdict(
        analyse(_detour_rows(side="right"), OBSTACLE, MISSION),
        OBSTACLE,
        mode="detour",
        mission=DETOUR_MISSION,
        expect_side="left",
    )

    assert report["verdict"] == "FAIL"
    assert report["checks"]["side_selected"]["ok"] is False


def test_an_auto_detour_whose_motion_is_on_the_wrong_side_fails():
    # The depth node says "left" but the path goes south (right): the check is
    # about the motion, not only about the label.
    rows = _detour_rows(side="left")
    for row in rows:
        row["clear_side"] = "right"

    report = verdict(
        analyse(rows, OBSTACLE, MISSION),
        OBSTACLE,
        mode="detour",
        mission=DETOUR_MISSION,
        expect_side="right",
    )

    # The label matches the expectation, but the path went +north, so the
    # direction does not match "right".
    assert report["checks"]["side_selected"]["ok"] is True
    assert report["checks"]["lateral_direction_matches_side"]["ok"] is False
