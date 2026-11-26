"""Unit tests for depth-frame pose metadata."""

from __future__ import annotations

from pathlib import Path

import pytest

from uav_tracking.perception.frame_meta import (
    FrameMeta,
    read_frame_meta,
    sidecar_path,
    write_frame_meta,
)


def _meta(**overrides) -> FrameMeta:
    values = dict(
        source="avoidance",
        t_s=20.2,
        east_m=4.04,
        north_m=-0.14,
        altitude_m=4.97,
        yaw_deg=-0.2,
        nearest_m=2.24,
        stop_event=2,
        width=640,
        height=480,
    )
    values.update(overrides)
    return FrameMeta(**values)


def test_roundtrip(tmp_path: Path):
    frame = tmp_path / "frame.npy"
    path = write_frame_meta(frame, _meta())
    assert path == tmp_path / "frame.npy.json"
    loaded = read_frame_meta(path)
    assert loaded == _meta()


def test_sidecar_path_keeps_extension():
    assert sidecar_path("results/a_depth_frame.npy").name == "a_depth_frame.npy.json"


def test_rejects_non_finite_pose():
    with pytest.raises(ValueError, match="finite"):
        _meta(east_m=float("nan"))


def test_rejects_negative_stop_event():
    with pytest.raises(ValueError, match="stop_event"):
        _meta(stop_event=-1)


def test_rejects_zero_size():
    with pytest.raises(ValueError, match="positive"):
        _meta(width=0)
