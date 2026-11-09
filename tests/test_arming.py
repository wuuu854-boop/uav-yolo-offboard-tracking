"""Arming retry: bounded attempts, then a diagnosis instead of silence."""

from __future__ import annotations

import pytest

from src.control.arming import ArmingSequencer


def test_first_step_requests_mode_and_arming():
    step = ArmingSequencer().step(now_s=0.0, armed=False)
    assert (step.request_mode, step.request_arm, step.attempts) == (True, True, 1)


def test_a_second_call_before_the_retry_sends_nothing():
    sequencer = ArmingSequencer(retry_s=2.0)
    sequencer.step(now_s=0.0, armed=False)
    step = sequencer.step(now_s=0.5, armed=False)
    assert (step.request_mode, step.request_arm) == (False, False)


def test_after_the_retry_only_arming_is_resent():
    sequencer = ArmingSequencer(retry_s=2.0)
    sequencer.step(now_s=0.0, armed=False)
    step = sequencer.step(now_s=2.1, armed=False)
    assert (step.request_mode, step.request_arm, step.attempts) == (False, True, 2)


def test_a_single_warning_after_the_grace_period():
    sequencer = ArmingSequencer(retry_s=1.0, give_up_after_s=5.0)
    warnings = [sequencer.step(now_s=float(t), armed=False).warn for t in range(0, 12)]
    assert warnings.count(True) == 1
    assert warnings.index(True) >= 5


def test_arming_resets_the_sequence():
    sequencer = ArmingSequencer(retry_s=1.0)
    sequencer.step(now_s=0.0, armed=False)
    assert sequencer.step(now_s=1.5, armed=True).request_arm is False
    # A later disarm starts from scratch: mode and arming again.
    step = sequencer.step(now_s=10.0, armed=False)
    assert (step.request_mode, step.request_arm, step.attempts) == (True, True, 1)


def test_invalid_configuration_is_rejected():
    with pytest.raises(ValueError):
        ArmingSequencer(retry_s=0.0)
    with pytest.raises(ValueError):
        ArmingSequencer(give_up_after_s=0.0)
