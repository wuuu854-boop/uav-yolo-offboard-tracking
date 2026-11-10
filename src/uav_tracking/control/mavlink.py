"""MAVLink telemetry that mirrors what the vehicle is doing.

The control path is uXRCE-DDS, but field tooling — a ground station, a companion
computer, a log collector — speaks MAVLink. This module encodes the three
messages that describe a vehicle without needing a GPS fix or the autopilot's
own MAVLink stream: heartbeat, attitude, and local position in NED.

It is read-only on purpose. Sending commands over a second link is how a
demonstration turns into an incident; nothing here can arm, take off or move the
vehicle.

Encoding is separated from ROS so the bytes can be decoded and checked in a test.
"""

from __future__ import annotations

from dataclasses import dataclass

from pymavlink import mavutil

# ArduPilot/PX4 custom mode for "offboard" differs; the number is what a ground
# station displays, so it is passed through rather than derived here.
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6


@dataclass(frozen=True)
class Telemetry:
    """One snapshot, in the frames MAVLink expects (NED, degrees)."""

    armed: bool
    north_m: float
    east_m: float
    down_m: float
    vn_mps: float
    ve_mps: float
    vd_mps: float
    roll_rad: float
    pitch_rad: float
    yaw_rad: float
    heading_deg: float


def _mav(src_system: int = 1, src_component: int = 1) -> mavutil.mavlink.MAVLink:
    """A MAVLink codec; the file object it normally writes to is unused."""
    return mavutil.mavlink.MAVLink(None, srcSystem=src_system, srcComponent=src_component)


def encode(snapshot: Telemetry, seq: int = 0, src_system: int = 1) -> list[bytes]:
    """The three messages as bytes, in the order a ground station expects."""
    mav = _mav(src_system)
    mav.seq = seq
    messages = [
        mavutil.mavlink.MAVLink_heartbeat_message(
            type=mavutil.mavlink.MAV_TYPE_QUADROTOR,
            autopilot=mavutil.mavlink.MAV_AUTOPILOT_PX4,
            base_mode=(mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED if snapshot.armed else 0),
            custom_mode=PX4_CUSTOM_MAIN_MODE_OFFBOARD if snapshot.armed else 0,
            system_status=mavutil.mavlink.MAV_STATE_ACTIVE
            if snapshot.armed
            else mavutil.mavlink.MAV_STATE_STANDBY,
            mavlink_version=3,
        ),
        mavutil.mavlink.MAVLink_attitude_message(
            time_boot_ms=0,
            roll=snapshot.roll_rad,
            pitch=snapshot.pitch_rad,
            yaw=snapshot.yaw_rad,
            rollspeed=0.0,
            pitchspeed=0.0,
            yawspeed=0.0,
        ),
        mavutil.mavlink.MAVLink_local_position_ned_message(
            time_boot_ms=0,
            x=snapshot.north_m,
            y=snapshot.east_m,
            z=snapshot.down_m,
            vx=snapshot.vn_mps,
            vy=snapshot.ve_mps,
            vz=snapshot.vd_mps,
        ),
    ]
    return [message.pack(mav) for message in messages]


def decode(raw: bytes) -> list:
    """Messages parsed back out of a byte stream; used by the tests and tools."""
    parser = mavutil.mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
    parsed = []
    for byte in raw:
        message = parser.parse_char(bytes([byte]))
        if message is not None:
            parsed.append(message)
    return parsed
