"""Unit tests for the L-shaped detour geometry and phase transitions."""

from __future__ import annotations

import math

import pytest

from uav_tracking.control.detour import (
    DetourConfig,
    DetourPhase,
    DetourPlanner,
    adaptive_backoff_step,
)


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


def test_a_configured_start_delay_holds_before_committing_to_a_side():
    planner = _planner(start_delay_s=1.0)

    first = planner.update((0.0, 0.0), (10.0, 0.0), stop=True, now_s=0.0)
    early = planner.update((0.0, 0.0), (10.0, 0.0), stop=True, now_s=0.5)
    late = planner.update((0.0, 0.0), (10.0, 0.0), stop=True, now_s=1.1)

    assert first.active is False
    assert "confirming" in first.note
    assert early.active is False
    assert late.active is True
    assert late.waypoint == pytest.approx((0.0, -3.0))


def test_a_clearance_between_stop_attempts_restarts_the_confirmation():
    planner = _planner(start_delay_s=1.0)
    planner.update((0.0, 0.0), (10.0, 0.0), stop=True, now_s=0.0)
    planner.update((0.0, 0.0), (10.0, 0.0), stop=False, now_s=0.5)

    decision = planner.update((0.0, 0.0), (10.0, 0.0), stop=True, now_s=0.6)

    assert decision.active is False
    assert "confirming" in decision.note


def test_both_blocked_with_backoff_starts_away_from_the_obstacle():
    planner = _planner(backoff_m=2.0)

    decision = planner.update((0.0, 0.0), (10.0, 0.0), stop=True, now_s=0.0, side="blocked")

    assert decision.active is True
    assert decision.phase is DetourPhase.BACKOFF
    assert decision.waypoint == pytest.approx((-2.0, 0.0))


def test_backoff_that_has_not_arrived_keeps_going():
    planner = _planner(backoff_m=2.0)
    planner.update((0.0, 0.0), (10.0, 0.0), stop=True, now_s=0.0, side="blocked")

    decision = planner.update((-1.0, 0.0), (10.0, 0.0), stop=True, now_s=0.5, side="blocked")

    assert decision.active is True
    assert decision.phase is DetourPhase.BACKOFF
    assert decision.waypoint == pytest.approx((-2.0, 0.0))


def test_backoff_that_ends_still_blocked_holds_at_the_backoff_point():
    planner = _planner(backoff_m=2.0)
    planner.update((0.0, 0.0), (10.0, 0.0), stop=True, now_s=0.0, side="blocked")

    decision = planner.update((-2.0, 0.0), (10.0, 0.0), stop=True, now_s=1.0, side="blocked")

    assert decision.active is True
    assert decision.phase is DetourPhase.HOLDING
    assert decision.waypoint == pytest.approx((-2.0, 0.0))
    assert "still blocked" in decision.note


def test_backoff_that_ends_with_a_clear_side_starts_the_lateral_leg():
    planner = _planner(backoff_m=2.0)
    planner.update((0.0, 0.0), (10.0, 0.0), stop=True, now_s=0.0, side="blocked")

    decision = planner.update((-2.0, 0.0), (10.0, 0.0), stop=True, now_s=1.0, side="left")

    assert decision.active is True
    assert decision.phase is DetourPhase.LATERAL
    assert decision.side == "left"
    assert decision.waypoint == pytest.approx((-2.0, 3.0))


def test_holding_after_backoff_recovers_when_a_side_later_clears():
    planner = _planner(backoff_m=2.0)
    planner.update((0.0, 0.0), (10.0, 0.0), stop=True, now_s=0.0, side="blocked")
    planner.update((-2.0, 0.0), (10.0, 0.0), stop=True, now_s=1.0, side="blocked")

    decision = planner.update((-2.0, 0.0), (10.0, 0.0), stop=True, now_s=2.0, side="left")

    assert decision.active is True
    assert decision.phase is DetourPhase.LATERAL
    assert decision.side == "left"


def test_backoff_zero_keeps_the_hold_instead_of_moving():
    planner = _planner()

    decision = planner.update((0.0, 0.0), (10.0, 0.0), stop=True, now_s=0.0, side="blocked")

    assert decision.active is False
    assert decision.note == "both sides blocked: holding"


def test_a_second_backoff_step_extends_the_sweep():
    planner = _planner(backoff_m=2.0, backoff_max_attempts=2)
    planner.update((0.0, 0.0), (10.0, 0.0), stop=True, now_s=0.0, side="blocked")

    decision = planner.update((-2.0, 0.0), (10.0, 0.0), stop=True, now_s=1.0, side="blocked")

    assert decision.active is True
    assert decision.phase is DetourPhase.BACKOFF
    assert decision.waypoint == pytest.approx((-4.0, 0.0))
    assert "backing off again (2/2)" in decision.note


def test_the_sweep_stops_at_its_attempt_budget():
    planner = _planner(backoff_m=2.0, backoff_max_attempts=2)
    planner.update((0.0, 0.0), (10.0, 0.0), stop=True, now_s=0.0, side="blocked")
    planner.update((-2.0, 0.0), (10.0, 0.0), stop=True, now_s=1.0, side="blocked")

    decision = planner.update((-4.0, 0.0), (10.0, 0.0), stop=True, now_s=2.0, side="blocked")

    assert decision.active is True
    assert decision.phase is DetourPhase.HOLDING
    assert "still blocked" in decision.note


def test_a_side_that_clears_at_the_second_step_starts_the_lateral_leg():
    planner = _planner(backoff_m=2.0, backoff_max_attempts=2)
    planner.update((0.0, 0.0), (10.0, 0.0), stop=True, now_s=0.0, side="blocked")
    planner.update((-2.0, 0.0), (10.0, 0.0), stop=True, now_s=1.0, side="blocked")

    decision = planner.update((-4.0, 0.0), (10.0, 0.0), stop=True, now_s=2.0, side="left")

    assert decision.active is True
    assert decision.phase is DetourPhase.LATERAL
    assert decision.side == "left"
    assert decision.waypoint == pytest.approx((-4.0, 3.0))


def test_an_exhausted_backoff_does_not_resume_just_because_the_gate_cleared():
    planner = _planner(backoff_m=2.0, backoff_max_attempts=1)
    planner.update((0.0, 0.0), (10.0, 0.0), stop=True, now_s=0.0, side="blocked")
    planner.update((-2.0, 0.0), (10.0, 0.0), stop=True, now_s=1.0, side="blocked")

    decision = planner.update((-2.0, 0.0), (10.0, 0.0), stop=False, now_s=2.0, side="blocked")

    assert decision.active is True
    assert decision.phase is DetourPhase.HOLDING
    assert "backoff exhausted" in decision.note


def test_an_exhausted_backoff_still_recovers_if_a_side_clears():
    planner = _planner(backoff_m=2.0, backoff_max_attempts=1)
    planner.update((0.0, 0.0), (10.0, 0.0), stop=True, now_s=0.0, side="blocked")
    planner.update((-2.0, 0.0), (10.0, 0.0), stop=True, now_s=1.0, side="blocked")

    decision = planner.update((-2.0, 0.0), (10.0, 0.0), stop=True, now_s=2.0, side="left")

    assert decision.active is True
    assert decision.phase is DetourPhase.LATERAL
    assert decision.side == "left"


def test_the_backoff_attempt_budget_cannot_be_zero():
    with pytest.raises(ValueError, match="backoff_max_attempts"):
        DetourConfig(backoff_max_attempts=0)


def test_adaptive_step_lifts_the_better_side_to_the_target():
    assert adaptive_backoff_step(2.0, 3.0, target_m=6.0, max_step_m=2.0) == pytest.approx(2.0)
    assert adaptive_backoff_step(5.0, 5.5, target_m=6.0) == pytest.approx(0.5)


def test_adaptive_step_is_none_when_the_side_is_already_clear_or_values_are_missing():
    assert adaptive_backoff_step(6.0, 6.5, target_m=6.0) is None
    assert adaptive_backoff_step(math.nan, 3.0, target_m=6.0) is None


def test_adaptive_step_rejects_bad_bounds():
    with pytest.raises(ValueError, match="min_step_m"):
        adaptive_backoff_step(1.0, 3.0, target_m=6.0, min_step_m=0.0)
    with pytest.raises(ValueError, match="max_step_m"):
        adaptive_backoff_step(1.0, 3.0, target_m=6.0, min_step_m=1.0, max_step_m=0.5)


def test_an_adaptive_step_changes_the_backoff_leg():
    planner = _planner(backoff_m=0.0, backoff_max_attempts=2)
    planner.set_backoff_step(1.5)

    decision = planner.update((0.0, 0.0), (10.0, 0.0), stop=True, now_s=0.0, side="blocked")

    assert decision.phase is DetourPhase.BACKOFF
    assert decision.waypoint == pytest.approx((-1.5, 0.0))
    assert "1.50 m" in decision.note


def test_setting_no_adaptive_step_falls_back_to_the_config():
    planner = _planner(backoff_m=0.5)
    planner.set_backoff_step(2.0)
    planner.set_backoff_step(None)

    decision = planner.update((0.0, 0.0), (10.0, 0.0), stop=True, now_s=0.0, side="blocked")

    assert decision.waypoint == pytest.approx((-0.5, 0.0))


def test_a_bad_adaptive_step_is_rejected():
    planner = _planner()
    with pytest.raises(ValueError, match="backoff step"):
        planner.set_backoff_step(0.0)


def test_a_spent_backoff_can_side_step_and_recover():
    planner = _planner(
        backoff_m=0.2,
        backoff_max_attempts=1,
        search_step_m=1.5,
        search_max_attempts=1,
    )
    planner.update((0.0, 0.0), (10.0, 0.0), stop=True, now_s=0.0, side="blocked")
    planner.update(
        (-0.2, 0.0), (10.0, 0.0), stop=True, now_s=1.0, side="blocked", search_side="left"
    )

    decision = planner.update(
        (-0.2, 1.5), (10.0, 0.0), stop=True, now_s=2.0, side="left", search_side="left"
    )

    assert decision.active is True
    assert decision.phase is DetourPhase.LATERAL
    assert decision.side == "left"


def test_the_side_step_starts_after_the_backoff_budget():
    planner = _planner(
        backoff_m=0.2,
        backoff_max_attempts=1,
        search_step_m=1.5,
        search_max_attempts=1,
    )
    planner.update((0.0, 0.0), (10.0, 0.0), stop=True, now_s=0.0, side="blocked")

    decision = planner.update(
        (-0.2, 0.0), (10.0, 0.0), stop=True, now_s=1.0, side="blocked", search_side="left"
    )

    assert decision.phase is DetourPhase.SEARCH
    assert decision.waypoint == pytest.approx((-0.2, 1.5))
    assert "side-search left" in decision.note


def test_a_spent_side_search_holds():
    planner = _planner(
        backoff_m=0.2,
        backoff_max_attempts=1,
        search_step_m=1.5,
        search_max_attempts=1,
    )
    planner.update((0.0, 0.0), (10.0, 0.0), stop=True, now_s=0.0, side="blocked")
    planner.update(
        (-0.2, 0.0), (10.0, 0.0), stop=True, now_s=1.0, side="blocked", search_side="left"
    )

    decision = planner.update(
        (-0.2, 1.5), (10.0, 0.0), stop=True, now_s=2.0, side="blocked", search_side="left"
    )

    assert decision.phase is DetourPhase.HOLDING
    assert "side-search exhausted" in decision.note


def test_search_is_disabled_by_default_and_holds_after_the_backoff():
    planner = _planner(backoff_m=0.2, backoff_max_attempts=1)
    planner.update((0.0, 0.0), (10.0, 0.0), stop=True, now_s=0.0, side="blocked")

    decision = planner.update((-0.2, 0.0), (10.0, 0.0), stop=True, now_s=1.0, side="blocked")

    assert decision.phase is DetourPhase.HOLDING
    assert "still blocked" in decision.note


def test_blocked_with_no_backoff_can_side_step_directly():
    planner = _planner(backoff_m=0.0, search_step_m=2.0, search_max_attempts=2)

    decision = planner.update(
        (0.0, 0.0), (10.0, 0.0), stop=True, now_s=0.0, side="blocked", search_side="left"
    )

    assert decision.phase is DetourPhase.SEARCH
    assert decision.waypoint == pytest.approx((0.0, 2.0))
    assert "side-search left" in decision.note


def test_a_side_step_not_yet_reached_keeps_searching():
    planner = _planner(
        backoff_m=0.2,
        backoff_max_attempts=1,
        search_step_m=1.5,
        search_max_attempts=1,
    )
    planner.update((0.0, 0.0), (10.0, 0.0), stop=True, now_s=0.0, side="blocked")
    planner.update(
        (-0.2, 0.0), (10.0, 0.0), stop=True, now_s=1.0, side="blocked", search_side="left"
    )

    decision = planner.update(
        (-0.2, 0.5), (10.0, 0.0), stop=True, now_s=2.0, side="blocked", search_side="left"
    )

    assert decision.phase is DetourPhase.SEARCH
    assert "side-search left" in decision.note


def test_a_second_side_step_extends_the_search():
    planner = _planner(
        backoff_m=0.2,
        backoff_max_attempts=1,
        search_step_m=1.5,
        search_max_attempts=2,
    )
    planner.update((0.0, 0.0), (10.0, 0.0), stop=True, now_s=0.0, side="blocked")
    planner.update(
        (-0.2, 0.0), (10.0, 0.0), stop=True, now_s=1.0, side="blocked", search_side="left"
    )

    decision = planner.update(
        (-0.2, 1.5), (10.0, 0.0), stop=True, now_s=2.0, side="blocked", search_side="left"
    )

    assert decision.phase is DetourPhase.SEARCH
    assert decision.waypoint == pytest.approx((-0.2, 3.0))
    assert "again (2/2)" in decision.note


def test_a_bad_search_side_is_rejected():
    planner = _planner()
    with pytest.raises(ValueError, match="search_side"):
        planner.update(
            (0.0, 0.0), (10.0, 0.0), stop=True, now_s=0.0, side="blocked", search_side="up"
        )


def test_negative_search_settings_are_rejected():
    with pytest.raises(ValueError, match="search_step_m"):
        DetourConfig(search_step_m=-0.1)
    with pytest.raises(ValueError, match="search_max_attempts"):
        DetourConfig(search_max_attempts=-1)


def test_the_backoff_cannot_be_negative():
    with pytest.raises(ValueError, match="backoff_m"):
        DetourConfig(backoff_m=-0.1)


def test_the_start_delay_cannot_be_negative():
    with pytest.raises(ValueError, match="start_delay_s"):
        DetourConfig(start_delay_s=-0.1)


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
