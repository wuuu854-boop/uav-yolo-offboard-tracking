"""Offline simulation: geometry and mission logic without a simulator."""

from __future__ import annotations

import math

import pytest

from src.control.offline_sim import IMAGE_HEIGHT, IMAGE_WIDTH, project_target, run_mission
from src.perception.projection import pixel_to_ground

CAMERA_MATRIX = ((554.26, 0.0, 320.0), (0.0, 554.26, 240.0), (0.0, 0.0, 1.0))


def test_project_target_is_the_inverse_of_pixel_to_ground():
    for target_east, target_north in [(2.0, 0.0), (0.0, -3.0), (4.0, 4.0)]:
        u, v = project_target((0.0, 0.0), (target_east, target_north), 5.0, CAMERA_MATRIX)
        east, north = pixel_to_ground(u, v, 5.0, CAMERA_MATRIX)
        assert east == pytest.approx(target_east, abs=1e-6)
        assert north == pytest.approx(target_north, abs=1e-6)


def test_projection_respects_yaw():
    # Facing north (pi/2), a target to the north is straight ahead: v < cy.
    u, v = project_target((0.0, 0.0), (0.0, 3.0), 5.0, CAMERA_MATRIX, yaw=math.pi / 2)
    assert u == pytest.approx(320.0, abs=1e-6)
    assert v < 240.0


def test_centre_pixel_means_target_is_under_the_vehicle():
    u, v = project_target((4.0, 4.0), (4.0, 4.0), 5.0, CAMERA_MATRIX)
    assert (u, v) == pytest.approx((320.0, 240.0))


def test_zero_altitude_is_rejected():
    with pytest.raises(ValueError):
        project_target((0.0, 0.0), (1.0, 1.0), 0.0)


def test_mission_reaches_the_target_and_lands():
    result = run_mission(seed=0)
    assert result.final_state.value == "IDLE"
    # The mission stops when it is inside reach_tolerance_m (1.0 m by default),
    # so the meaningful assertion is the README success threshold, not zero.
    assert result.final_error_m <= 1.5
    assert result.steps < 600  # ~60 s simulated at 10 Hz


def test_mission_is_deterministic_for_a_fixed_seed():
    first = run_mission(seed=7)
    second = run_mission(seed=7)
    assert first.final_error_m == pytest.approx(second.final_error_m)
    assert first.steps == second.steps


def test_samples_cover_takeoff_and_landing_states():
    states = {row["state"] for row in run_mission(seed=0).samples}
    assert {"TAKEOFF", "GOTO", "HOVER", "LAND"} <= states


def test_image_bounds_are_used_by_the_camera_model():
    # A target far to the side falls outside a 640x480 frame.
    u, _ = project_target((0.0, 0.0), (0.0, 20.0), 5.0, CAMERA_MATRIX)
    assert u > IMAGE_WIDTH or u < 0
    assert IMAGE_HEIGHT == 480
