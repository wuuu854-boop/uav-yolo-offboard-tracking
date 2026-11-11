"""Unit tests for the L-shaped detour geometry and phase transitions."""

from __future__ import annotations

import math

import pytest

from uav_tracking.control.detour import DetourConfig, DetourPhase, DetourPlanner


def _planner(**overrides) -> DetourPlanner:
    defaults = {
        "enabled": True,
        "side": "right",
        "offset_m": 3.0,
        "forward_m": 3.0,
        "reach_tolerance_m": 0.5,
        "clear_hold_s": 1.0,
        "wait_timeout_s": 5.0,
        "max_attempts": 2,
    }
    defaults.update(overrides)
    return DetourPlanner(DetourConfig(**defaults))


def test_a_disabled_planner_never_moves():
    planner = DetourPlanner(DetourConfig(enabled=False))

    decision = planner.update((0.0, 0.0), (10.0, 0.0), stop=True, now_s=0.0)

    assert decision.active is False
    assert decision.phase is DetourPhase.INACTIVE


def test_a_clear_path_does_not_start_a_detour():
    planner = _planner()

    decision = planner.update((0.0, 0.0), (10.0, 0.0), stop=False, now_s=0.0)

    assert decision.active is False
    assert planner.attempts == 0


def test_a_stop_starts_a_rightward_lateral_leg():
    planner = _planner()

    decision = planner.update((0.0, 0.0), (10.0, 0.0), stop=True, now_s=0.0)

    assert decision.active is True
    assert decision.phase is DetourPhase.LATERAL
    # Facing east, right is south (-north).
    assert decision.waypoint == pytest.approx((0.0, -3.0))
    assert planner.attempts == 1


def test_left_is_the_mirror_of_right():
    planner = _planner(side="left")

    decision = planner.update((0.0, 0.0), (10.0, 0.0), stop=True, now_s=0.0)

    assert decision.waypoint == pytest.approx((0.0, 3.0))


def test_the_side_can_come_from_the_depth_choice():
    planner = _planner(side="right")

    decision = planner.update((0.0, 0.0), (10.0, 0.0), stop=True, now_s=0.0, side="left")

    assert decision.waypoint == pytest.approx((0.0, 3.0))
    assert decision.side == "left"
    assert planner.side == "left"


def test_an_unknown_side_falls_back_to_the_configured_side():
    planner = _planner(side="right")

    decision = planner.update((0.0, 0.0), (10.0, 0.0), stop=True, now_s=0.0, side="unknown")

    assert decision.waypoint == pytest.approx((0.0, -3.0))
    assert decision.side == "right"


def test_the_lateral_leg_is_perpendicular_to_a_diagonal_approach():
    planner = _planner()

    decision = planner.update((0.0, 0.0), (10.0, 10.0), stop=True, now_s=0.0)

    assert decision.waypoint == pytest.approx((2.1213, -2.1213), abs=1e-3)


def test_a_zero_length_target_vector_uses_the_last_known_direction():
    planner = _planner()
    planner.update((0.0, 0.0), (10.0, 0.0), stop=False, now_s=0.0)  # learns east

    decision = planner.update((0.0, 0.0), (0.0, 0.0), stop=True, now_s=0.1)

    assert decision.waypoint == pytest.approx((0.0, -3.0))


def test_the_lateral_leg_continues_even_while_the_gate_still_stops():
    planner = _planner()
    planner.update((0.0, 0.0), (10.0, 0.0), stop=True, now_s=0.0)

    decision = planner.update((0.0, -2.0), (10.0, 0.0), stop=True, now_s=1.0)

    assert decision.active is True
    assert decision.phase is DetourPhase.LATERAL


def test_reaching_the_lateral_point_turns_forward():
    planner = _planner()
    planner.update((0.0, 0.0), (10.0, 0.0), stop=True, now_s=0.0)

    decision = planner.update((0.0, -2.7), (10.0, 0.0), stop=True, now_s=1.0)

    assert decision.phase is DetourPhase.FORWARD
    assert decision.waypoint == pytest.approx((3.0, -2.7))


def test_the_forward_leg_continues_until_the_waypoint():
    planner = _planner()
    planner.update((0.0, 0.0), (10.0, 0.0), stop=True, now_s=0.0)
    planner.update((0.0, -2.7), (10.0, 0.0), stop=True, now_s=1.0)  # LATERAL -> FORWARD

    decision = planner.update((1.5, -2.7), (10.0, 0.0), stop=True, now_s=2.0)

    assert decision.phase is DetourPhase.FORWARD
    assert decision.waypoint == pytest.approx((3.0, -2.7))


def test_reaching_the_forward_point_waits_for_a_clear_gate():
    planner = _planner()
    planner.update((0.0, 0.0), (10.0, 0.0), stop=True, now_s=0.0)
    planner.update((0.0, -2.7), (10.0, 0.0), stop=True, now_s=1.0)

    decision = planner.update((2.7, -2.7), (10.0, 0.0), stop=True, now_s=2.0)

    assert decision.phase is DetourPhase.WAITING
    assert decision.waypoint == pytest.approx((3.0, -2.7))


def test_a_clear_gate_at_the_forward_point_resumes_the_waypoint():
    planner = _planner()
    planner.update((0.0, 0.0), (10.0, 0.0), stop=True, now_s=0.0)
    planner.update((0.0, -2.7), (10.0, 0.0), stop=True, now_s=1.0)
    planner.update((2.7, -2.7), (10.0, 0.0), stop=True, now_s=2.0)

    planner.update((2.7, -2.7), (10.0, 0.0), stop=False, now_s=3.0)
    decision = planner.update((2.7, -2.7), (10.0, 0.0), stop=False, now_s=4.1)

    assert decision.active is False
    assert "complete" in decision.note
    assert planner.phase is DetourPhase.INACTIVE


def test_waiting_too_long_extends_the_forward_leg_once():
    planner = _planner()
    planner.update((0.0, 0.0), (10.0, 0.0), stop=True, now_s=0.0)
    planner.update((0.0, -2.7), (10.0, 0.0), stop=True, now_s=1.0)
    planner.update((2.7, -2.7), (10.0, 0.0), stop=True, now_s=2.0)  # WAITING

    decision = planner.update((2.7, -2.7), (10.0, 0.0), stop=True, now_s=7.1)

    assert decision.phase is DetourPhase.FORWARD
    assert decision.waypoint == pytest.approx((5.7, -2.7))
    assert planner.attempts == 2


def test_the_last_attempt_holds_instead_of_extending_forever():
    planner = _planner(max_attempts=1)
    planner.update((0.0, 0.0), (10.0, 0.0), stop=True, now_s=0.0)
    planner.update((0.0, -2.7), (10.0, 0.0), stop=True, now_s=1.0)
    planner.update((2.7, -2.7), (10.0, 0.0), stop=True, now_s=2.0)

    decision = planner.update((2.7, -2.7), (10.0, 0.0), stop=True, now_s=7.1)

    assert decision.phase is DetourPhase.HOLDING
    assert decision.waypoint == pytest.approx((2.7, -2.7))
    assert "exhausted" in decision.note


def test_a_held_detour_recovers_when_the_gate_stays_clear():
    planner = _planner(max_attempts=1)
    planner.update((0.0, 0.0), (10.0, 0.0), stop=True, now_s=0.0)
    planner.update((0.0, -2.7), (10.0, 0.0), stop=True, now_s=1.0)
    planner.update((2.7, -2.7), (10.0, 0.0), stop=True, now_s=2.0)
    planner.update((2.7, -2.7), (10.0, 0.0), stop=True, now_s=7.1)  # HOLDING

    planner.update((2.7, -2.7), (10.0, 0.0), stop=False, now_s=8.0)
    decision = planner.update((2.7, -2.7), (10.0, 0.0), stop=False, now_s=9.1)

    assert decision.active is False
    assert "recovered" in decision.note
    assert planner.phase is DetourPhase.INACTIVE


def test_reset_forgets_the_leg():
    planner = _planner()
    planner.update((0.0, 0.0), (10.0, 0.0), stop=True, now_s=0.0)

    planner.reset()

    assert planner.phase is DetourPhase.INACTIVE
    assert planner.attempts == 0
    assert planner.waypoint is None


def test_a_bad_side_is_rejected():
    with pytest.raises(ValueError, match="side"):
        DetourConfig(side="up")


def test_the_config_rejects_non_positive_geometry():
    for field in ("offset_m", "forward_m", "reach_tolerance_m", "clear_hold_s"):
        with pytest.raises(ValueError, match=field):
            DetourConfig(**{field: 0.0})


def test_the_config_rejects_a_zero_attempt_count():
    with pytest.raises(ValueError, match="max_attempts"):
        DetourConfig(max_attempts=0)


def test_the_config_rejects_a_negative_wait_timeout():
    with pytest.raises(ValueError, match="wait_timeout_s"):
        DetourConfig(wait_timeout_s=-1.0)


def test_non_finite_inputs_are_rejected():
    planner = _planner()
    with pytest.raises(ValueError, match="now_s"):
        planner.update((0.0, 0.0), (1.0, 0.0), stop=True, now_s=math.nan)

    planner = _planner()
    with pytest.raises(ValueError, match="position"):
        planner.update((math.inf, 0.0), (1.0, 0.0), stop=True, now_s=0.0)
