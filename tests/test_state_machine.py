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
