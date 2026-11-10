"""Mission state machine and target filter behaviour."""

from __future__ import annotations

import pytest

from uav_tracking.control.state_machine import (
    MissionConfig,
    MissionStateMachine,
    Observation,
    State,
    TargetJumpFilter,
)


def _arm_and_take_off(sm: MissionStateMachine) -> None:
    assert sm.step(Observation(0.0, 0.0, armed=False)).command == "arm"
    assert sm.step(Observation(1.0, 0.0, armed=True)).state is State.TAKEOFF


def test_nominal_mission_reaches_landing():
    sm = MissionStateMachine(MissionConfig(hover_duration_s=10.0))
    _arm_and_take_off(sm)

    assert sm.step(Observation(2.0, 4.0, armed=True)).state is State.TAKEOFF
    assert sm.step(Observation(3.0, 4.8, armed=True)).state is State.GOTO
    assert sm.step(Observation(4.0, 5.0, armed=True, distance_to_target_m=3.0)).command == "goto"
    assert sm.step(Observation(5.0, 5.0, armed=True, distance_to_target_m=0.8)).state is State.HOVER

    # 9 s of hover is not enough, 10.5 s is.
    assert (
        sm.step(Observation(14.0, 5.0, armed=True, distance_to_target_m=0.8)).state is State.HOVER
    )
    assert sm.step(Observation(15.5, 5.0, armed=True, distance_to_target_m=0.8)).state is State.LAND
    assert sm.step(Observation(20.0, 0.1, armed=True)).state is State.IDLE


def test_target_loss_hovers_after_the_timeout_and_resumes_when_reacquired():
    sm = MissionStateMachine(MissionConfig(target_loss_hover_s=2.0))
    _arm_and_take_off(sm)
    sm.step(Observation(3.0, 5.0, armed=True))

    assert sm.step(Observation(10.0, 5.0, armed=True, distance_to_target_m=5.0)).command == "goto"
    assert sm.step(Observation(10.5, 5.0, armed=True, distance_to_target_m=None)).command == "goto"
    # 1.9 s lost: still below the 2.0 s threshold.
    assert sm.step(Observation(12.4, 5.0, armed=True, distance_to_target_m=None)).command == "goto"
    # 2.1 s lost: hover, and the reason is recorded in the note.
    decision = sm.step(Observation(12.6, 5.0, armed=True, distance_to_target_m=None))
    assert decision.state is State.HOVER
    assert decision.note == "target_lost"
    # Target back: resume the approach instead of landing.
    assert sm.step(Observation(13.0, 5.0, armed=True, distance_to_target_m=4.0)).state is State.GOTO


def test_does_not_leave_idle_before_arming():
    sm = MissionStateMachine()
    for t in range(5):
        decision = sm.step(Observation(float(t), 0.0, armed=False))
        assert decision.state is State.IDLE
        assert decision.command == "arm"


def test_landing_returns_to_idle_even_if_still_armed_below_threshold():
    sm = MissionStateMachine()
    _arm_and_take_off(sm)
    sm.step(Observation(2.0, 5.0, armed=True))
    sm.step(Observation(3.0, 5.0, armed=True, distance_to_target_m=0.2))
    sm.step(Observation(20.0, 5.0, armed=True, distance_to_target_m=0.2))
    assert sm.state is State.LAND
    assert sm.step(Observation(21.0, 0.1, armed=True)).state is State.IDLE


def test_jump_filter_accepts_first_fix_and_rejects_teleports():
    jump_filter = TargetJumpFilter(jump_limit_m=3.0)
    assert jump_filter.update((10.0, 0.0)) == (10.0, 0.0)
    # 10 m jump: rejected, last accepted position is kept.
    assert jump_filter.update((20.0, 0.0)) == (10.0, 0.0)
    assert jump_filter.rejected == 1
    # 1 m move: accepted.
    assert jump_filter.update((11.0, 0.0)) == (11.0, 0.0)
    assert jump_filter.rejected == 1


def test_jump_filter_holds_last_position_when_detection_disappears():
    jump_filter = TargetJumpFilter(jump_limit_m=3.0)
    jump_filter.update((1.0, 2.0))
    assert jump_filter.update(None) == (1.0, 2.0)


def test_jump_filter_rejects_a_non_positive_limit():
    with pytest.raises(ValueError):
        TargetJumpFilter(0.0)


def test_hover_does_not_land_when_landing_is_disabled():
    sm = MissionStateMachine(MissionConfig(hover_duration_s=10.0, land_after_hover=False))
    _arm_and_take_off(sm)
    sm.step(Observation(3.0, 5.0, armed=True))
    sm.step(Observation(4.0, 5.0, armed=True, distance_to_target_m=0.2))

    assert (
        sm.step(Observation(30.0, 5.0, armed=True, distance_to_target_m=0.2)).state is State.HOVER
    )


def test_an_in_flight_disarm_stops_the_mission():
    sm = MissionStateMachine()
    _arm_and_take_off(sm)
    sm.step(Observation(3.0, 5.0, armed=True))

    decision = sm.step(Observation(4.0, 5.0, armed=False))

    assert decision.state is State.IDLE
    assert decision.command == "disarm"
    assert decision.note == "vehicle disarmed in flight"


def test_an_in_flight_disarm_during_goto_does_not_keep_flying():
    sm = MissionStateMachine()
    _arm_and_take_off(sm)
    sm.step(Observation(3.0, 5.0, armed=True))
    assert sm.step(Observation(4.0, 5.0, armed=True, distance_to_target_m=4.0)).command == "goto"

    assert sm.step(Observation(5.0, 5.0, armed=False)).state is State.IDLE


def test_a_long_target_loss_starts_a_search():
    sm = MissionStateMachine(MissionConfig(target_loss_hover_s=1.0, search_after_s=2.0))
    _arm_and_take_off(sm)
    sm.step(Observation(3.0, 5.0, armed=True))

    sm.step(Observation(4.0, 5.0, armed=True, distance_to_target_m=4.0))
    sm.step(Observation(5.1, 5.0, armed=True, distance_to_target_m=None))
    # lost for 1.1 s -> hover starts here (t=6.2)
    assert (
        sm.step(Observation(6.2, 5.0, armed=True, distance_to_target_m=None)).state is State.HOVER
    )
    # 1.9 s of hovering is not enough
    assert (
        sm.step(Observation(8.1, 5.0, armed=True, distance_to_target_m=None)).state is State.HOVER
    )

    decision = sm.step(Observation(8.4, 5.0, armed=True, distance_to_target_m=None))
    assert decision.state is State.SEARCH
    assert decision.command == "search"


def test_finding_the_target_during_the_search_resumes_the_approach():
    sm = MissionStateMachine(MissionConfig(target_loss_hover_s=1.0, search_after_s=2.0))
    _arm_and_take_off(sm)
    sm.step(Observation(3.0, 5.0, armed=True))
    sm.step(Observation(4.0, 5.0, armed=True, distance_to_target_m=4.0))
    sm.step(Observation(5.1, 5.0, armed=True, distance_to_target_m=None))
    sm.step(Observation(6.2, 5.0, armed=True, distance_to_target_m=None))
    sm.step(Observation(8.4, 5.0, armed=True, distance_to_target_m=None))
    assert sm.state is State.SEARCH

    decision = sm.step(Observation(9.0, 5.0, armed=True, distance_to_target_m=3.0))
    assert decision.state is State.GOTO
    assert decision.note == "target reacquired during search"


def test_a_mission_that_never_converges_lands_on_timeout():
    sm = MissionStateMachine(MissionConfig(max_mission_s=30.0))
    _arm_and_take_off(sm)
    sm.step(Observation(3.0, 5.0, armed=True))

    # airborne since t=1; the target is never reached
    sm.step(Observation(20.0, 5.0, armed=True, distance_to_target_m=8.0))
    assert sm.state is State.GOTO

    decision = sm.step(Observation(31.1, 5.0, armed=True, distance_to_target_m=8.0))
    assert decision.state is State.LAND
    assert decision.note == "mission timeout"


def test_a_timeout_of_zero_never_triggers():
    sm = MissionStateMachine(MissionConfig(max_mission_s=0.0))
    _arm_and_take_off(sm)
    sm.step(Observation(3.0, 5.0, armed=True))
    for t in range(10, 400, 10):
        sm.step(Observation(float(t), 5.0, armed=True, distance_to_target_m=8.0))
    assert sm.state is State.GOTO


def test_the_timeout_budget_restarts_after_a_disarm():
    sm = MissionStateMachine(MissionConfig(max_mission_s=10.0))
    _arm_and_take_off(sm)
    sm.step(Observation(3.0, 5.0, armed=True))
    sm.step(Observation(9.0, 5.0, armed=True, distance_to_target_m=8.0))

    sm.step(Observation(10.0, 5.0, armed=False))
    assert sm.state is State.IDLE

    sm.step(Observation(11.0, 5.0, armed=True))  # new mission, budget restarts
    sm.step(Observation(15.0, 5.0, armed=True, distance_to_target_m=8.0))
    assert sm.state is State.GOTO


def test_arrival_needs_three_consecutive_frames_when_configured():
    sm = MissionStateMachine(MissionConfig(confirm_frames=3))
    _arm_and_take_off(sm)
    sm.step(Observation(3.0, 5.0, armed=True))

    sm.step(Observation(4.0, 5.0, armed=True, distance_to_target_m=0.5))
    sm.step(Observation(4.2, 5.0, armed=True, distance_to_target_m=0.5))
    assert sm.state is State.GOTO  # two frames are not enough

    decision = sm.step(Observation(4.4, 5.0, armed=True, distance_to_target_m=0.5))
    assert decision.state is State.HOVER
    assert decision.note == "arrived"


def test_a_missed_frame_resets_the_confirmation():
    sm = MissionStateMachine(MissionConfig(confirm_frames=3, target_loss_hover_s=5.0))
    _arm_and_take_off(sm)
    sm.step(Observation(3.0, 5.0, armed=True))

    sm.step(Observation(4.0, 5.0, armed=True, distance_to_target_m=0.5))
    sm.step(Observation(4.2, 5.0, armed=True, distance_to_target_m=0.5))
    sm.step(Observation(4.4, 5.0, armed=True, distance_to_target_m=None))  # reset
    sm.step(Observation(4.6, 5.0, armed=True, distance_to_target_m=0.5))
    sm.step(Observation(4.8, 5.0, armed=True, distance_to_target_m=0.5))
    assert sm.state is State.GOTO

    assert sm.step(Observation(5.0, 5.0, armed=True, distance_to_target_m=0.5)).state is State.HOVER


def test_reacquiring_after_a_loss_needs_the_same_confirmation():
    sm = MissionStateMachine(MissionConfig(confirm_frames=2, target_loss_hover_s=1.0))
    _arm_and_take_off(sm)
    sm.step(Observation(3.0, 5.0, armed=True))
    sm.step(Observation(4.0, 5.0, armed=True, distance_to_target_m=5.0))
    sm.step(Observation(5.1, 5.0, armed=True, distance_to_target_m=None))
    sm.step(Observation(6.2, 5.0, armed=True, distance_to_target_m=None))
    assert sm.state is State.HOVER

    sm.step(Observation(7.0, 5.0, armed=True, distance_to_target_m=4.0))
    assert sm.state is State.HOVER  # one frame does not reacquire
    assert sm.step(Observation(7.2, 5.0, armed=True, distance_to_target_m=4.0)).state is State.GOTO


def test_a_pending_goal_turns_the_arrival_hover_into_the_next_leg():
    sm = MissionStateMachine(MissionConfig(hover_duration_s=1.0))
    _arm_and_take_off(sm)
    sm.step(Observation(3.0, 5.0, armed=True))
    sm.step(Observation(4.0, 5.0, armed=True, distance_to_target_m=0.5))
    assert sm.state is State.HOVER

    decision = sm.step(
        Observation(5.6, 5.0, armed=True, distance_to_target_m=0.5, goal_pending=True)
    )

    assert decision.state is State.GOTO
    assert decision.note == "next waypoint"


def test_without_a_pending_goal_the_same_hover_lands():
    sm = MissionStateMachine(MissionConfig(hover_duration_s=1.0))
    _arm_and_take_off(sm)
    sm.step(Observation(3.0, 5.0, armed=True))
    sm.step(Observation(4.0, 5.0, armed=True, distance_to_target_m=0.5))

    decision = sm.step(
        Observation(5.6, 5.0, armed=True, distance_to_target_m=0.5, goal_pending=False)
    )

    assert decision.state is State.LAND
    assert decision.note == "hover complete"
