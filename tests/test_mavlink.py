"""The read-only MAVLink mirror: encoding, and the bytes coming back."""

from __future__ import annotations

import pytest

pytest.importorskip("pymavlink")

from pymavlink import mavutil  # noqa: E402

from uav_tracking.control.mavlink import Telemetry, decode, encode  # noqa: E402

SNAPSHOT = Telemetry(
    armed=True,
    north_m=3.5,
    east_m=-1.25,
    down_m=-4.9,
    vn_mps=0.4,
    ve_mps=-0.1,
    vd_mps=0.0,
    roll_rad=0.02,
    pitch_rad=-0.03,
    yaw_rad=1.57,
    heading_deg=90.0,
)


def test_three_messages_come_out_in_order():
    messages = decode(b"".join(encode(SNAPSHOT)))

    assert [message.get_type() for message in messages] == [
        "HEARTBEAT",
        "ATTITUDE",
        "LOCAL_POSITION_NED",
    ]


def test_the_heartbeat_says_armed_and_px4():
    heartbeat = decode(b"".join(encode(SNAPSHOT)))[0]

    assert heartbeat.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4
    assert heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    assert heartbeat.custom_mode == 6  # offboard


def test_a_disarmed_snapshot_says_so():
    disarmed = Telemetry(**{**SNAPSHOT.__dict__, "armed": False})
    heartbeat = decode(b"".join(encode(disarmed)))[0]

    assert not heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    assert heartbeat.custom_mode == 0


def test_position_keeps_its_ned_meaning():
    position = decode(b"".join(encode(SNAPSHOT)))[2]

    assert position.x == pytest.approx(SNAPSHOT.north_m, abs=1e-3)
    assert position.y == pytest.approx(SNAPSHOT.east_m, abs=1e-3)
    assert position.z == pytest.approx(SNAPSHOT.down_m, abs=1e-3)


def test_attitude_round_trips():
    attitude = decode(b"".join(encode(SNAPSHOT)))[1]

    assert attitude.roll == pytest.approx(SNAPSHOT.roll_rad, abs=1e-6)
    assert attitude.pitch == pytest.approx(SNAPSHOT.pitch_rad, abs=1e-6)
    assert attitude.yaw == pytest.approx(SNAPSHOT.yaw_rad, abs=1e-6)


def test_the_stream_survives_a_udp_round_trip():
    import socket

    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(("127.0.0.1", 0))
    receiver.settimeout(2.0)
    port = receiver.getsockname()[1]

    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sender.sendto(b"".join(encode(SNAPSHOT)), ("127.0.0.1", port))
    payload, _ = receiver.recvfrom(4096)
    receiver.close()
    sender.close()

    assert len(decode(payload)) == 3
