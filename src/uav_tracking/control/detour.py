"""A minimal L-shaped detour around a static obstacle.

Phase 1 stopped and waited. Phase 2 is the smallest behaviour that gets past the
same wall: when the depth gate says stop, fly sideways past the wall's edge, fly
forward past its face, then resume the waypoint. An L is the minimum that does
not cut the corner: a single diagonal back to a target standing 0.8 m behind the
wall would pass within ~0.2 m of its edge.

It is still not a planner: one obstacle, one fixed side, no map. The geometry and
the phase transitions below are pure and unit-tested; the controller only feeds
them the current position, the target, the gate decision and the clock.

ENU convention: +x east, +y north. For a vehicle facing the target, "right" is
``(dy, -dx)``; flying east, right is south.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum


class DetourPhase(str, Enum):
    INACTIVE = "INACTIVE"
    BACKOFF = "BACKOFF"
    LATERAL = "LATERAL"
    FORWARD = "FORWARD"
    WAITING = "WAITING"
    HOLDING = "HOLDING"


@dataclass(frozen=True)
class DetourConfig:
    """Tunables, loaded from the mission YAML."""

    enabled: bool = False
    side: str = "right"
    offset_m: float = 3.0
    forward_m: float = 3.0
    reach_tolerance_m: float = 0.5
    clear_hold_s: float = 1.0
    wait_timeout_s: float = 5.0
    max_attempts: int = 2
    # Hold still for this long after the gate first stops before committing to a
    # side. The side label can take a few frames to settle as the vehicle pitches
    # to a stop; starting on the first frame picked the wrong side in SITL.
    start_delay_s: float = 0.0
    # If both side strips are blocked, back away from the obstacle by this much
    # and re-evaluate. 0 keeps the phase-4 behaviour (hold without moving).
    backoff_m: float = 0.0
    # How many backoff_m steps to try before holding. 1 is the phase-5 single
    # retry; 2+ is the phase-6 distance sweep.
    backoff_max_attempts: int = 1

    def __post_init__(self) -> None:
        side = str(self.side).lower()
        if side not in ("left", "right"):
            raise ValueError(f"detour side must be 'left' or 'right', got {self.side!r}")
        object.__setattr__(self, "side", side)

        for name in (
            "offset_m",
            "forward_m",
            "reach_tolerance_m",
            "clear_hold_s",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be positive and finite, got {value!r}")

        wait = float(self.wait_timeout_s)
        if not math.isfinite(wait) or wait < 0.0:
            raise ValueError(f"wait_timeout_s must be finite and non-negative, got {wait!r}")
        delay = float(self.start_delay_s)
        if not math.isfinite(delay) or delay < 0.0:
            raise ValueError(f"start_delay_s must be finite and non-negative, got {delay!r}")
        backoff = float(self.backoff_m)
        if not math.isfinite(backoff) or backoff < 0.0:
            raise ValueError(f"backoff_m must be finite and non-negative, got {backoff!r}")
        if int(self.backoff_max_attempts) < 1:
            raise ValueError("backoff_max_attempts must be at least 1")
        if int(self.max_attempts) < 1:
            raise ValueError("max_attempts must be at least 1")

        object.__setattr__(self, "offset_m", float(self.offset_m))
        object.__setattr__(self, "forward_m", float(self.forward_m))
        object.__setattr__(self, "reach_tolerance_m", float(self.reach_tolerance_m))
        object.__setattr__(self, "clear_hold_s", float(self.clear_hold_s))
        object.__setattr__(self, "wait_timeout_s", wait)
        object.__setattr__(self, "start_delay_s", delay)
        object.__setattr__(self, "backoff_m", backoff)
        object.__setattr__(self, "backoff_max_attempts", int(self.backoff_max_attempts))
        object.__setattr__(self, "max_attempts", int(self.max_attempts))


@dataclass(frozen=True)
class DetourDecision:
    """What the controller should do with the horizontal setpoint this tick."""

    active: bool
    phase: DetourPhase
    waypoint: tuple[float, float] | None = None
    note: str = ""
    side: str | None = None


def _unit(vector: tuple[float, float]) -> tuple[float, float] | None:
    norm = math.hypot(vector[0], vector[1])
    if norm < 1e-9:
        return None
    return (vector[0] / norm, vector[1] / norm)


def _perpendicular(direction: tuple[float, float], side: str) -> tuple[float, float]:
    dx, dy = direction
    if side == "right":
        return (dy, -dx)
    return (-dy, dx)


@dataclass
class DetourPlanner:
    """Lateral leg, forward leg, then resume once the gate stays clear."""

    config: DetourConfig = field(default_factory=DetourConfig)
    phase: DetourPhase = DetourPhase.INACTIVE
    attempts: int = 0
    side: str | None = None

    def __post_init__(self) -> None:
        self._direction: tuple[float, float] = (1.0, 0.0)
        # The forward leg must stay on the approach axis, not point at the
        # target from the lateral point: that diagonal is what cuts the corner.
        self._approach: tuple[float, float] = (1.0, 0.0)
        self._lateral: tuple[float, float] = (0.0, -1.0)
        self._waypoint: tuple[float, float] | None = None
        self._clear_since: float | None = None
        self._wait_since: float | None = None
        self._stop_since: float | None = None
        self._backoff_attempts = 0
        self._backoff_hold = False

    @property
    def enabled(self) -> bool:
        return bool(self.config.enabled)

    @property
    def waypoint(self) -> tuple[float, float] | None:
        return self._waypoint

    def reset(self) -> None:
        self.phase = DetourPhase.INACTIVE
        self.attempts = 0
        self.side = None
        self._waypoint = None
        self._clear_since = None
        self._wait_since = None
        self._stop_since = None
        self._backoff_attempts = 0
        self._backoff_hold = False

    def update(
        self,
        position: tuple[float, float],
        target: tuple[float, float],
        stop: bool,
        now_s: float,
        side: str | None = None,
    ) -> DetourDecision:
        """Feed one control tick; get the detour waypoint or an inactive decision.

        ``side`` is the free-space choice from the depth node (``left``/``right``/
        ``unknown``). It is used only when a detour starts; ``unknown`` and any
        other value fall back to ``config.side``.
        """
        if not self.enabled:
            return DetourDecision(False, DetourPhase.INACTIVE, None, "detour disabled")

        now = float(now_s)
        if not math.isfinite(now):
            raise ValueError(f"now_s must be finite, got {now_s!r}")
        here = (float(position[0]), float(position[1]))
        goal = (float(target[0]), float(target[1]))
        for name, value in (("position", here), ("target", goal)):
            if not all(math.isfinite(component) for component in value):
                raise ValueError(f"{name} must be finite, got {value!r}")

        direction = _unit((goal[0] - here[0], goal[1] - here[1]))
        if direction is not None:
            self._direction = direction

        if self.phase is DetourPhase.INACTIVE:
            if not stop:
                self._stop_since = None
                return DetourDecision(False, DetourPhase.INACTIVE, None, "path clear")
            if self._stop_since is None:
                self._stop_since = now
            if now - self._stop_since < self.config.start_delay_s:
                return DetourDecision(
                    False,
                    DetourPhase.INACTIVE,
                    None,
                    f"confirming stop {now - self._stop_since:.1f}s",
                )
            if side == "blocked":
                if self.config.backoff_m > 0.0:
                    self._start_backoff(here)
                    return self._active("both sides blocked: backing off")
                return DetourDecision(
                    False,
                    DetourPhase.INACTIVE,
                    None,
                    "both sides blocked: holding",
                )
            chosen = side if side in ("left", "right") else self.config.side
            self._start(here, chosen)
            return self._active("detour started: lateral leg")

        assert self._waypoint is not None
        if self.phase is DetourPhase.BACKOFF:
            if self._reached(here):
                if side in ("left", "right"):
                    self._start(here, side)
                    return self._active("backoff done: lateral leg")
                if self._backoff_attempts < self.config.backoff_max_attempts:
                    self._backoff_attempts += 1
                    self._waypoint = (
                        here[0] - self.config.backoff_m * self._approach[0],
                        here[1] - self.config.backoff_m * self._approach[1],
                    )
                    return self._active(
                        "still blocked: backing off again "
                        f"({self._backoff_attempts}/{self.config.backoff_max_attempts})"
                    )
                self.phase = DetourPhase.HOLDING
                self._waypoint = here
                self._clear_since = None
                self._backoff_hold = True
                return self._active("backoff done: both sides still blocked, holding")
            return self._active("backing off from blocked sides")
        if self.phase is DetourPhase.LATERAL:
            if self._reached(here):
                self._begin_forward(here)
                return self._active("detour lateral leg done: forward leg")
            return self._active("detour lateral leg")

        if self.phase is DetourPhase.FORWARD:
            if self._reached(here):
                self.phase = DetourPhase.WAITING
                self._wait_since = now
                self._clear_since = None
                return self._active("detour forward leg done: waiting for a clear gate")
            return self._active("detour forward leg")

        if self.phase is DetourPhase.WAITING:
            clear_for = self._update_clear_since(stop, now)
            if clear_for >= self.config.clear_hold_s:
                self.reset()
                return DetourDecision(
                    False,
                    DetourPhase.INACTIVE,
                    None,
                    f"detour complete: gate clear for {clear_for:.1f} s",
                )
            assert self._wait_since is not None
            if now - self._wait_since >= self.config.wait_timeout_s:
                if self.attempts >= self.config.max_attempts:
                    self.phase = DetourPhase.HOLDING
                    self._waypoint = here
                    return self._active("detour exhausted, holding")
                self.attempts += 1
                self._begin_forward(here)
                return self._active("detour extending forward")
            return self._active("detour waiting for a clear gate")

        # HOLDING: the forward-extension budget is spent, or both sides were
        # still blocked after backing off. Hold; recover if a side later clears
        # or the gate stays clear.
        if self.side is None and stop and side in ("left", "right"):
            self._start(here, side)
            return self._active("detour recovered after backoff: lateral leg")
        if self._backoff_hold:
            # The gate can read clear simply because the vehicle backed away;
            # resuming the approach would drive it straight back into the same
            # obstacle. Hold until a side clears or the mission times out.
            self._waypoint = here
            return self._active("backoff exhausted: holding")
        clear_for = self._update_clear_since(stop, now)
        if clear_for >= self.config.clear_hold_s:
            self.reset()
            return DetourDecision(
                False, DetourPhase.INACTIVE, None, "detour recovered after holding"
            )
        self._waypoint = here
        return self._active("detour exhausted, holding")

    def _reached(self, position: tuple[float, float]) -> bool:
        assert self._waypoint is not None
        return (
            math.hypot(position[0] - self._waypoint[0], position[1] - self._waypoint[1])
            <= self.config.reach_tolerance_m
        )

    def _update_clear_since(self, stop: bool, now: float) -> float:
        if stop:
            self._clear_since = None
            return 0.0
        if self._clear_since is None:
            self._clear_since = now
        return now - self._clear_since

    def _start_backoff(self, position: tuple[float, float]) -> None:
        self.side = None
        self._approach = self._direction
        self._lateral = _perpendicular(self._approach, self.config.side)
        self.phase = DetourPhase.BACKOFF
        self.attempts = 1
        self._backoff_attempts = 1
        self._backoff_hold = False
        self._wait_since = None
        self._clear_since = None
        self._waypoint = (
            position[0] - self.config.backoff_m * self._approach[0],
            position[1] - self.config.backoff_m * self._approach[1],
        )

    def _start(self, position: tuple[float, float], side: str) -> None:
        self.side = side
        self._approach = self._direction
        self._lateral = _perpendicular(self._approach, side)
        self.phase = DetourPhase.LATERAL
        self.attempts = 1
        self._wait_since = None
        self._clear_since = None
        self._waypoint = (
            position[0] + self.config.offset_m * self._lateral[0],
            position[1] + self.config.offset_m * self._lateral[1],
        )

    def _begin_forward(self, position: tuple[float, float]) -> None:
        self.phase = DetourPhase.FORWARD
        self._wait_since = None
        self._clear_since = None
        self._waypoint = (
            position[0] + self.config.forward_m * self._approach[0],
            position[1] + self.config.forward_m * self._approach[1],
        )

    def _active(self, note: str) -> DetourDecision:
        return DetourDecision(True, self.phase, self._waypoint, note, self.side)
