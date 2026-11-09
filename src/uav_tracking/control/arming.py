"""Arming and mode-switch retry logic.

Requesting arming once is fragile: the mode switch can be rejected, a preflight
check can clear a second later, and the node then sits silent forever. This
sequencer retries a bounded number of times and then says something useful
instead of leaving the operator to guess.

Pure Python so the timing can be tested without a vehicle.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ArmingStep:
    """What to send on this control tick."""

    request_mode: bool = False
    request_arm: bool = False
    warn: bool = False
    attempts: int = 0


class ArmingSequencer:
    """Request Offboard mode once, then retry arming every ``retry_s`` seconds."""

    def __init__(self, retry_s: float = 2.0, give_up_after_s: float = 20.0) -> None:
        if retry_s <= 0.0:
            raise ValueError("retry_s must be positive")
        if give_up_after_s <= 0.0:
            raise ValueError("give_up_after_s must be positive")
        self.retry_s = float(retry_s)
        self.give_up_after_s = float(give_up_after_s)
        self._started_s: float | None = None
        self._last_attempt_s: float | None = None
        self._mode_sent = False
        self._attempts = 0
        self._warned = False

    def reset(self) -> None:
        self._started_s = None
        self._last_attempt_s = None
        self._mode_sent = False
        self._attempts = 0
        self._warned = False

    def step(self, now_s: float, armed: bool) -> ArmingStep:
        if armed:
            self.reset()
            return ArmingStep()

        if self._started_s is None:
            self._started_s = now_s

        due = self._last_attempt_s is None or (now_s - self._last_attempt_s) >= self.retry_s
        if not due:
            return ArmingStep(attempts=self._attempts)

        request_mode = not self._mode_sent
        self._mode_sent = True
        self._last_attempt_s = now_s
        self._attempts += 1

        waited = now_s - self._started_s
        warn = (not self._warned) and self._attempts > 1 and waited >= self.give_up_after_s
        if warn:
            self._warned = True

        return ArmingStep(
            request_mode=request_mode,
            request_arm=True,
            warn=warn,
            attempts=self._attempts,
        )
