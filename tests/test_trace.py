"""State trace: one row per change, so a failed trial can be explained."""

from __future__ import annotations

import csv

from uav_tracking.control.trace import HEADER, StateTrace


def _rows(path):
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def test_a_transition_is_written_once(tmp_path):
    trace = StateTrace(tmp_path / "state_trace.csv")

    assert trace.write(0.0, "IDLE", "waiting for arming")
    assert not trace.write(0.05, "IDLE", "waiting for arming")  # same state, no row
    assert trace.write(1.0, "TAKEOFF", "climb to 5.0 m")

    rows = _rows(tmp_path / "state_trace.csv")
    assert [row["state"] for row in rows] == ["IDLE", "TAKEOFF"]
    assert list(rows[0]) == HEADER


def test_a_new_note_on_the_same_state_is_a_new_row(tmp_path):
    trace = StateTrace(tmp_path / "state_trace.csv")
    trace.write(0.0, "GOTO", "")
    trace.write(2.0, "GOTO", "target missing 2.0s")

    assert len(_rows(tmp_path / "state_trace.csv")) == 2


def test_position_and_altitude_are_recorded(tmp_path):
    trace = StateTrace(tmp_path / "state_trace.csv")
    trace.write(3.0, "GOTO", "", position=(1.234, -5.678), altitude_m=4.95)

    row = _rows(tmp_path / "state_trace.csv")[0]
    assert row["east_m"] == "1.23"
    assert row["north_m"] == "-5.68"
    assert row["altitude_m"] == "4.95"


def test_altitude_is_optional(tmp_path):
    trace = StateTrace(tmp_path / "state_trace.csv")
    trace.write(0.0, "IDLE")

    assert _rows(tmp_path / "state_trace.csv")[0]["altitude_m"] == ""


def test_a_second_run_appends_rather_than_overwrites(tmp_path):
    path = tmp_path / "state_trace.csv"
    StateTrace(path).write(0.0, "IDLE")
    StateTrace(path).write(0.0, "IDLE")  # a new process starts at the same state

    rows = _rows(path)
    assert len(rows) == 2
    assert rows[0]["state"] == rows[1]["state"] == "IDLE"


def test_reset_allows_the_same_state_again(tmp_path):
    trace = StateTrace(tmp_path / "state_trace.csv")
    trace.write(0.0, "IDLE")
    trace.reset()
    assert trace.write(0.0, "IDLE")

    assert len(_rows(tmp_path / "state_trace.csv")) == 2
