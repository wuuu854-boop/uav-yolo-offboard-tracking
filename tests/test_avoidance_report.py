"""Unit tests for the avoidance report's geometry and verdict.

The report is what turns a CSV into the numbers quoted in the docs. A wrong
point-to-box distance or a verdict that ignores a contact would quietly bless a
bad run, so both are tested on synthetic rows.
"""

from __future__ import annotations

import math

from scripts.avoidance_report import analyse, longest_stop_run, point_to_aabb_distance, verdict

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
