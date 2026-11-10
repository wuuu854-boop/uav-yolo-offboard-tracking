"""Mission recorder: the CSV that the README numbers are quoted from."""

from __future__ import annotations

import csv

import pytest

from uav_tracking.control.recorder import MissionRecorder, localise_targets

GROUND_TRUTH = {0: (10.0, 0.0), 1: (0.0, 0.0)}


def test_one_row_per_trial_with_metrics(tmp_path):
    path = tmp_path / "landing_error.csv"
    recorder = MissionRecorder(path, GROUND_TRUTH)

    recorder.start_trial()
    for latency in (100.0, 120.0, 140.0):
        recorder.add_latency(latency)
    row = recorder.finish_trial((9.2, 0.0), notes="arrived")

    assert row["trial"] == 1
    assert row["target_id"] == 0
    assert row["error_m"] == pytest.approx(0.8, abs=1e-3)
    assert row["success"] is True
    assert row["latency_p50_ms"] == pytest.approx(120.0, abs=0.5)

    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    assert len(rows) == 1
    assert rows[0]["notes"] == "arrived"


def test_trials_append_and_numbering_survives_a_restart(tmp_path):
    path = tmp_path / "landing_error.csv"
    recorder = MissionRecorder(path, GROUND_TRUTH)

    recorder.start_trial()
    recorder.finish_trial((5.0, 0.0))  # far from both targets -> failure
    recorder.start_trial()
    recorder.finish_trial((0.3, 0.0))

    # A new process must continue the numbering instead of overwriting trial 1.
    restarted = MissionRecorder(path, GROUND_TRUTH)
    assert restarted.trial == 3
    restarted.start_trial()
    restarted.finish_trial((10.0, 0.0))

    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    assert [row["trial"] for row in rows] == ["1", "2", "3"]
    assert rows[0]["success"] == "False"
    assert rows[1]["success"] == "True"


def test_missing_latency_is_written_as_empty_not_nan(tmp_path):
    path = tmp_path / "landing_error.csv"
    recorder = MissionRecorder(path, GROUND_TRUTH)
    recorder.start_trial()
    recorder.finish_trial((10.0, 0.0))

    with path.open(newline="", encoding="utf-8") as handle:
        row = next(csv.DictReader(handle))
    assert row["latency_p50_ms"] == ""


def test_recorder_requires_ground_truth(tmp_path):
    with pytest.raises(ValueError):
        MissionRecorder(tmp_path / "x.csv", {})


def test_ground_truth_is_shifted_into_the_local_frame():
    world = {0: (0.0, 0.0), 1: (20.0, 0.0)}
    local = localise_targets(world, (-3.5, 0.0))

    # the vehicle spawned 3.5 m west of target 0, so target 0 is 3.5 m east of it
    assert local[0] == pytest.approx((3.5, 0.0))
    assert local[1] == pytest.approx((23.5, 0.0))


def test_localising_with_no_spawn_offset_changes_nothing():
    world = {0: (1.0, 2.0)}
    assert localise_targets(world, (0.0, 0.0)) == world


def test_latency_samples_are_kept_next_to_the_summary(tmp_path):
    path = tmp_path / "landing_error.csv"
    recorder = MissionRecorder(path, GROUND_TRUTH)
    recorder.start_trial()
    for latency in (100.0, 120.0, 140.0):
        recorder.add_latency(latency)
    recorder.finish_trial((0.1, 0.0))

    raw = tmp_path / "latency_raw.csv"
    lines = raw.read_text(encoding="utf-8").strip().splitlines()
    assert lines[0] == "trial,latency_ms"
    assert len(lines) == 4
    assert lines[1].startswith("1,100")


def test_without_latency_samples_no_raw_file_is_created(tmp_path):
    path = tmp_path / "landing_error.csv"
    recorder = MissionRecorder(path, GROUND_TRUTH)
    recorder.start_trial()
    recorder.finish_trial((0.1, 0.0))
    assert not (tmp_path / "latency_raw.csv").exists()
