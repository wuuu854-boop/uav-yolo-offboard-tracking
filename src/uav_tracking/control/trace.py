"""A log of mission state changes, for diagnosing a run after the fact.

The results CSV records one row per landing: an error, a latency, a verdict. When
a trial goes wrong that row says nothing about why — which state it was in, when
it gave up, whether it searched. This writes one line per *change*, so a failed
trial has a story instead of a number.

Deliberately small: only transitions, not every control tick. A 30 s trial at
50 Hz would be 1500 rows of noise; it produces a handful of meaningful ones.
"""

from __future__ import annotations

import csv
from pathlib import Path

HEADER = ["t_s", "state", "note", "east_m", "north_m", "altitude_m"]


class StateTrace:
    """Append a row whenever the mission state or its reason changes."""

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self._last: tuple[str, str] | None = None

    def write(
        self,
        now_s: float,
        state: str,
        note: str = "",
        position: tuple[float, float] | None = None,
        altitude_m: float | None = None,
    ) -> bool:
        """Record the state if it differs from the last one. Returns True if written."""
        key = (state, note)
        if key == self._last:
            return False
        self._last = key

        east, north = position if position is not None else (0.0, 0.0)
        row = {
            "t_s": round(now_s, 2),
            "state": state,
            "note": note,
            "east_m": round(east, 2),
            "north_m": round(north, 2),
            "altitude_m": "" if altitude_m is None else round(altitude_m, 2),
        }
        self._append(row)
        return True

    def reset(self) -> None:
        """Forget the last state, so a replay starts with a fresh row."""
        self._last = None

    def _append(self, row: dict) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        write_header = not self.path.exists() or self.path.stat().st_size == 0
        with self.path.open("a", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=HEADER)
            if write_header:
                writer.writeheader()
            writer.writerow(row)
