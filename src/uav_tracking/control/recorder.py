"""Append one row per mission trial to ``results/landing_error.csv``.

The CSV is the only evidence behind the numbers quoted in the README and in the
resume, so it is written by code, never by hand.
"""

from __future__ import annotations

import csv
from pathlib import Path

import numpy as np

HEADER = [
    "trial",
    "target_id",
    "error_m",
    "latency_p50_ms",
    "latency_p95_ms",
    "success",
    "notes",
]

RAW_NAME = "latency_raw.csv"


class MissionRecorder:
    """Collects latencies during a trial and writes one summary row on landing."""

    def __init__(
        self,
        path: str | Path,
        ground_truth: dict[int, tuple[float, float]],
        success_tolerance_m: float = 1.5,
    ) -> None:
        if not ground_truth:
            raise ValueError("ground_truth must contain at least one target")
        self.path = Path(path)
        self.ground_truth = ground_truth
        self.success_tolerance_m = float(success_tolerance_m)
        self._latencies_ms: list[float] = []
        self._active = False
        self._trial = self._existing_rows()

    @property
    def trial(self) -> int:
        """Index that the *next* trial will use (1-based)."""
        return self._trial

    def _existing_rows(self) -> int:
        if not self.path.exists():
            return 1
        with self.path.open(newline="", encoding="utf-8") as handle:
            rows = list(csv.reader(handle))
        return max(len(rows) - 1, 0) + 1

    def start_trial(self) -> None:
        self._latencies_ms.clear()
        self._active = True

    def add_latency(self, latency_ms: float) -> None:
        if self._active and np.isfinite(latency_ms):
            self._latencies_ms.append(float(latency_ms))

    def _nearest_target(self, position_enu: tuple[float, float]) -> tuple[int, float]:
        best_id, best_distance = -1, float("inf")
        for target_id, (east, north) in self.ground_truth.items():
            distance = float(np.hypot(position_enu[0] - east, position_enu[1] - north))
            if distance < best_distance:
                best_id, best_distance = int(target_id), distance
        return best_id, best_distance

    def finish_trial(self, position_enu: tuple[float, float], notes: str = "") -> dict:
        target_id, error_m = self._nearest_target(position_enu)

        if self._latencies_ms:
            p50 = float(np.percentile(self._latencies_ms, 50))
            p95 = float(np.percentile(self._latencies_ms, 95))
        else:
            p50 = p95 = float("nan")

        success = bool(error_m <= self.success_tolerance_m)
        row = {
            "trial": self._trial,
            "target_id": target_id,
            "error_m": round(error_m, 3),
            "latency_p50_ms": round(p50, 1) if np.isfinite(p50) else "",
            "latency_p95_ms": round(p95, 1) if np.isfinite(p95) else "",
            "success": success,
            "notes": notes,
        }
        self._append(row)
        self._append_raw(self._trial)
        self._trial += 1
        self._active = False
        return row

    def _append_raw(self, trial: int) -> None:
        """Keep the individual samples.

        The summary row stores per-trial percentiles, and the percentile of
        percentiles is not the pooled percentile. Pooling the raw samples is the
        only way to quote a p95 for the whole run honestly.
        """
        if not self._latencies_ms:
            return
        path = self.path.with_name(RAW_NAME)
        write_header = not path.exists() or path.stat().st_size == 0
        with path.open("a", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            if write_header:
                writer.writerow(["trial", "latency_ms"])
            for latency in self._latencies_ms:
                writer.writerow([trial, round(latency, 3)])

    def _append(self, row: dict) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        write_header = not self.path.exists() or self.path.stat().st_size == 0
        with self.path.open("a", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=HEADER)
            if write_header:
                writer.writeheader()
            writer.writerow(row)
