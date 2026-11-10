"""Roll and pitch extraction from a PX4 quaternion."""

from __future__ import annotations

import math

import pytest

from uav_tracking.control.frames import quaternion_to_roll_pitch


def _from_euler(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """Quaternion (w, x, y, z) for a ZYX (yaw, pitch, roll) rotation."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def test_identity_has_no_attitude():
    assert quaternion_to_roll_pitch((1.0, 0.0, 0.0, 0.0)) == pytest.approx((0.0, 0.0))


def test_a_yaw_only_rotation_has_no_roll_or_pitch():
    q = _from_euler(0.0, 0.0, math.radians(75.0))

    roll, pitch = quaternion_to_roll_pitch(q)

    assert roll == pytest.approx(0.0, abs=1e-9)
    assert pitch == pytest.approx(0.0, abs=1e-9)


def test_roll_is_recovered():
    q = _from_euler(math.radians(30.0), 0.0, 0.0)

    roll, pitch = quaternion_to_roll_pitch(q)

    assert roll == pytest.approx(math.radians(30.0), rel=1e-9)
    assert pitch == pytest.approx(0.0, abs=1e-9)


def test_pitch_is_recovered():
    q = _from_euler(0.0, math.radians(20.0), 0.0)

    roll, pitch = quaternion_to_roll_pitch(q)

    assert roll == pytest.approx(0.0, abs=1e-9)
    assert pitch == pytest.approx(math.radians(20.0), rel=1e-9)


def test_combined_attitude_round_trips():
    roll_in, pitch_in = math.radians(-12.0), math.radians(8.0)
    q = _from_euler(roll_in, pitch_in, math.radians(140.0))

    roll, pitch = quaternion_to_roll_pitch(q)

    assert roll == pytest.approx(roll_in, rel=1e-6)
    assert pitch == pytest.approx(pitch_in, rel=1e-6)


def test_a_bad_quaternion_is_rejected():
    with pytest.raises(ValueError, match="4 components"):
        quaternion_to_roll_pitch((1.0, 0.0, 0.0))
