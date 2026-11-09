"""Mission state machine and target filter.

Pure Python on purpose: the safety logic is the part most worth testing, and it
must be testable on a CI runner that has no ROS, no Gazebo and no model file.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum


class State(str, Enum):
    """Mission states. Values match the strings used in logs and CSVs."""

    IDLE = "IDLE"
    TAKEOFF = "TAKEOFF"
    GOTO = "GOTO"
    HOVER = "HOVER"
    LAND = "LAND"


@dataclass(frozen=True)
class MissionConfig:
    """Tunables, loaded from ``config/mission.yaml``."""

    takeoff_altitude_m: float = 5.0
    reach_tolerance_m: float = 1.0
    hover_duration_s: float = 10.0
    target_loss_hover_s: float = 2.0
    setpoint_jump_limit_m: float = 3.0
    max_speed_mps: float = 1.0
    land_after_hover: bool = True
    control_rate_hz: float = 50.0
    target_selection: str = "nearest_centre"


@dataclass(frozen=True)
class Observation:
    """One controller tick of inputs."""

    now_s: float
    altitude_m: float
    armed: bool
    distance_to_target_m: float | None = None


@dataclass(frozen=True)
class Decision:
    """What the state machine wants the Offboard node to do this tick."""

    state: State
    command: str
    note: str = ""


class MissionStateMachine:
    """Deterministic mission logic; call :meth:`step` at the control rate.

    ``HOVER`` has two entry reasons, and they exit differently:

    * ``arrived``      - reached the target, land after ``hover_duration_s``.
    * ``target_lost``  - the target disappeared, resume ``GOTO`` when it returns.
    """

    def __init__(self, cfg: MissionConfig | None = None) -> None:
        self.cfg = cfg or MissionConfig()
        self.state = State.IDLE
        self._hover_reason = ""
        self._hover_started_s = 0.0
        self._lost_since_s: float | None = None

    def step(self, obs: Observation) -> Decision:
        if self.state is State.IDLE:
            if not obs.armed:
                return Decision(self.state, "arm", "waiting for arming")
            self.state = State.TAKEOFF
            return Decision(self.state, "takeoff", f"climb to {self.cfg.takeoff_altitude_m:.1f} m")

        if self.state is State.TAKEOFF:
            if obs.altitude_m >= self.cfg.takeoff_altitude_m - 0.3:
                self.state = State.GOTO
                return Decision(self.state, "goto", "takeoff altitude reached")
            return Decision(self.state, "takeoff", "")

        if self.state is State.GOTO:
            if obs.distance_to_target_m is None:
                if self._lost_since_s is None:
                    self._lost_since_s = obs.now_s
                lost_for = obs.now_s - self._lost_since_s
                if lost_for >= self.cfg.target_loss_hover_s:
                    return self._enter_hover(obs, "target_lost")
                return Decision(self.state, "goto", f"target missing {lost_for:.1f}s")
            self._lost_since_s = None
            if obs.distance_to_target_m <= self.cfg.reach_tolerance_m:
                return self._enter_hover(obs, "arrived")
            return Decision(self.state, "goto", "")

        if self.state is State.HOVER:
            # Two ways into HOVER with two different exits on purpose: after a
            # target-loss hover the mission resumes when the target reappears,
            # whereas after arriving it lands on a timer. Collapsing them into
            # one state would make a single missed frame end the mission.
            if self._hover_reason == "target_lost" and obs.distance_to_target_m is not None:
                self.state = State.GOTO
                self._lost_since_s = None
                return Decision(self.state, "goto", "target reacquired")
            if self._hover_reason == "arrived" and self.cfg.land_after_hover:
                if obs.now_s - self._hover_started_s >= self.cfg.hover_duration_s:
                    self.state = State.LAND
                    return Decision(self.state, "land", "hover complete")
            return Decision(self.state, "hover", self._hover_reason)

        # State.LAND
        if obs.altitude_m <= 0.2 or not obs.armed:
            self.state = State.IDLE
            self._hover_reason = ""
            return Decision(self.state, "disarm", "landed")
        return Decision(self.state, "land", "")

    def _enter_hover(self, obs: Observation, reason: str) -> Decision:
        self.state = State.HOVER
        self._hover_reason = reason
        self._hover_started_s = obs.now_s
        return Decision(self.state, "hover", reason)


class TargetJumpFilter:
    """Reject implausible target jumps and hold the last accepted position.

    A single bad detection can otherwise move the setpoint by several metres;
    on a real vehicle that is a safety issue, in simulation it silently ruins
    the landing-error metric.
    """

    def __init__(self, jump_limit_m: float) -> None:
        if jump_limit_m <= 0.0:
            raise ValueError("jump_limit_m must be positive")
        self.jump_limit_m = float(jump_limit_m)
        self._last: tuple[float, float] | None = None
        self.rejected = 0

    @property
    def last(self) -> tuple[float, float] | None:
        return self._last

    def reset(self) -> None:
        self._last = None
        self.rejected = 0

    def update(self, xy: tuple[float, float] | None) -> tuple[float, float] | None:
        """Feed one observation, get the position that may be used for control."""
        if xy is None:
            return self._last

        candidate = (float(xy[0]), float(xy[1]))
        if self._last is None:
            self._last = candidate
            return self._last

        distance = math.hypot(candidate[0] - self._last[0], candidate[1] - self._last[1])
        if distance > self.jump_limit_m:
            self.rejected += 1
            return self._last

        self._last = candidate
        return self._last
