"""Offline control-loop simulation: no ROS, no Gazebo, no model file.

Why this exists: a visitor (or CI) should be able to verify the geometry and the
mission logic in one command, on any operating system, before installing a
5 GB simulator. It exercises the *real* modules — ``projection``, ``TargetJumpFilter``
and ``MissionStateMachine`` — with a synthetic camera and a point-mass vehicle.

It is explicitly **not** a substitute for the SITL results: it models no
dynamics, no controller lag and no perception failure. The README labels it as
such.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

from uav_tracking.control.state_machine import (
    MissionConfig,
    MissionStateMachine,
    Observation,
    State,
    TargetJumpFilter,
)
from uav_tracking.perception.projection import estimate_target_enu

IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 960

# Same intrinsics as config/camera.yaml, derived from PX4's x500_mono_cam_down
# model (1280x960, horizontal FOV 1.74 rad).
CAMERA_MATRIX = (
    (538.4, 0.0, 640.0),
    (0.0, 538.4, 480.0),
    (0.0, 0.0, 1.0),
)

# The camera sits 10 cm above base_link in that model; using the same call as
# the flight node keeps the demo honest about the geometry it exercises.
CAMERA_OFFSET_BODY = (0.0, 0.0, 0.10)


def project_target(
    uav_enu: tuple[float, float],
    target_enu: tuple[float, float],
    altitude: float,
    camera_matrix: tuple[tuple[float, float, float], ...] = CAMERA_MATRIX,
    yaw: float = 0.0,
    image_up_is_forward: bool = True,
) -> tuple[float, float]:
    """Inverse of :func:`pixel_to_ground`: world target -> pixel (may be off-screen)."""
    if altitude <= 0.0:
        raise ValueError("altitude must be positive")

    delta_east = target_enu[0] - uav_enu[0]
    delta_north = target_enu[1] - uav_enu[1]

    # Transpose of the rotation used in pixel_to_ground.
    forward = delta_east * math.cos(yaw) + delta_north * math.sin(yaw)
    right = delta_east * math.sin(yaw) - delta_north * math.cos(yaw)
    down = -forward if image_up_is_forward else forward

    fx, fy = camera_matrix[0][0], camera_matrix[1][1]
    cx, cy = camera_matrix[0][2], camera_matrix[1][2]
    return cx + right * fx / altitude, cy + down * fy / altitude


@dataclass
class DemoResult:
    steps: int
    final_error_m: float
    final_state: State
    samples: list[dict] = field(default_factory=list)


def run_mission(
    target_enu: tuple[float, float] = (5.0, 2.0),
    seed: int = 0,
    dt: float = 0.1,
    max_time_s: float = 120.0,
    pixel_noise_px: float = 1.0,
    takeoff_altitude_m: float = 10.0,
    cfg: MissionConfig | None = None,
) -> DemoResult:
    """Fly the simulated vehicle to ``target_enu`` using the production state machine.

    The default 10 m altitude is chosen so the target starts inside the camera
    frame: at 5 m the visible patch is 11.9 x 8.9 m, so a target 5 m ahead is
    already off the top of the image and the loop would look broken for a reason
    that has nothing to do with the logic under test.
    """
    mission = cfg or MissionConfig(takeoff_altitude_m=takeoff_altitude_m)
    state_machine = MissionStateMachine(mission)
    jump_filter = TargetJumpFilter(mission.setpoint_jump_limit_m)
    rng = random.Random(seed)

    east, north, altitude = 0.0, 0.0, 0.0
    yaw = 0.0
    armed = False
    started = False
    step = 0
    samples: list[dict] = []

    for step in range(1, int(max_time_s / dt) + 1):
        now = step * dt
        if now >= 2.0:  # simulate the operator arming after two seconds
            armed = True

        estimate = None
        if altitude > 0.3:
            u, v = project_target((east, north), target_enu, altitude, yaw=yaw)
            if 0 <= u < IMAGE_WIDTH and 0 <= v < IMAGE_HEIGHT:
                u += rng.gauss(0.0, pixel_noise_px)
                v += rng.gauss(0.0, pixel_noise_px)
                target_east, target_north, _ = estimate_target_enu(
                    (east, north),
                    u,
                    v,
                    altitude,
                    CAMERA_MATRIX,
                    yaw=yaw,
                    offset_body=CAMERA_OFFSET_BODY,
                )
                estimate = jump_filter.update((target_east, target_north))

        distance = None
        if estimate is not None:
            distance = math.hypot(estimate[0] - east, estimate[1] - north)

        decision = state_machine.step(
            Observation(now_s=now, altitude_m=altitude, armed=armed, distance_to_target_m=distance)
        )

        if decision.command == "takeoff":
            altitude = min(altitude + mission.max_speed_mps * dt, mission.takeoff_altitude_m)
        elif decision.command == "goto" and estimate is not None:
            to_east, to_north = estimate[0] - east, estimate[1] - north
            remaining = math.hypot(to_east, to_north)
            stride = min(mission.max_speed_mps * dt, remaining)
            if remaining > 1e-9:
                east += to_east / remaining * stride
                north += to_north / remaining * stride
        elif decision.command == "land":
            altitude = max(altitude - mission.max_speed_mps * dt, 0.0)

        if decision.state is not State.IDLE:
            started = True

        samples.append(
            {
                "t_s": round(now, 2),
                "east_m": round(east, 3),
                "north_m": round(north, 3),
                "error_m": round(distance, 3) if distance is not None else "",
                "state": decision.state.value,
            }
        )

        # Stop only after the mission actually ran and came back to IDLE; the
        # vehicle legitimately sits in IDLE while it waits to be armed.
        if started and decision.state is State.IDLE:
            break

    final_error = math.hypot(east - target_enu[0], north - target_enu[1])
    return DemoResult(
        steps=step,
        final_error_m=final_error,
        final_state=state_machine.state,
        samples=samples,
    )
