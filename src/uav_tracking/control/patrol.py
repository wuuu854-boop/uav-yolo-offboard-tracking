"""A queue of waypoints for a multi-target mission.

The single-target mission ends where it started: take off, fly to one detection,
land. A patrol visits several positions in order, and the mission only lands once
the queue is empty. This module is the queue itself — no ROS, no state machine —
so the ordering rules can be tested without a simulator.

Waypoints are in the same local ENU frame the controller uses (metres east and
north of the spawn point), which is what the mission config carries.
"""

from __future__ import annotations

from collections.abc import Iterable, Sequence
from dataclasses import dataclass, field


@dataclass
class PatrolPlan:
    """Ordered waypoints, advanced one arrival at a time."""

    waypoints: tuple[tuple[float, float], ...] = ()
    index: int = field(default=0, init=False)

    @classmethod
    def from_config(cls, waypoints: Iterable[Sequence[float]]) -> PatrolPlan:
        """Build a plan from ``[[east, north], ...]``, rejecting malformed pairs."""
        parsed = []
        for waypoint in waypoints:
            values = tuple(float(value) for value in waypoint)
            if len(values) != 2:
                raise ValueError(f"a waypoint needs two values (east, north), got {waypoint!r}")
            parsed.append(values)
        return cls(waypoints=tuple(parsed))

    def current(self) -> tuple[float, float] | None:
        """The waypoint to fly to now, or ``None`` when the plan is finished."""
        if self.index >= len(self.waypoints):
            return None
        return self.waypoints[self.index]

    def advance(self) -> None:
        """Mark the current waypoint as visited."""
        if self.index < len(self.waypoints):
            self.index += 1

    def done(self) -> bool:
        return self.index >= len(self.waypoints)

    def remaining(self) -> int:
        return max(0, len(self.waypoints) - self.index)
