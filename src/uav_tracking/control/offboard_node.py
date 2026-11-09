#!/usr/bin/env python3
"""PX4 Offboard node: detections -> ground projection -> trajectory setpoint.

Subscribes
    px4_msgs/VehicleOdometry      /fmu/out/vehicle_odometry
    uav_interfaces/TargetArray    /detections
Publishes
    px4_msgs/OffboardControlMode  /fmu/in/offboard_control_mode
    px4_msgs/TrajectorySetpoint   /fmu/in/trajectory_setpoint
    px4_msgs/VehicleCommand       /fmu/in/vehicle_command
    geometry_msgs/PoseStamped     /target_pose        (debug / RViz, ENU)

The arming and mode-switch sequence follows the official ``px4_ros_com``
offboard example; check it against your PX4 version before flying.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import yaml

# Running as a script: put the package root (src/) on the path.
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import rclpy  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from px4_msgs.msg import (  # noqa: E402
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleOdometry,
    VehicleStatus,
)
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402

from uav_interfaces.msg import TargetArray  # noqa: E402
from uav_tracking.control.arming import ArmingSequencer  # noqa: E402
from uav_tracking.control.frames import (  # noqa: E402
    heading_ned_to_yaw_enu,
    ned_position_to_enu,
    quaternion_to_heading_ned,
)
from uav_tracking.control.limiter import limited_step  # noqa: E402
from uav_tracking.control.recorder import MissionRecorder  # noqa: E402
from uav_tracking.control.selection import STRATEGIES, select_target  # noqa: E402
from uav_tracking.control.state_machine import (  # noqa: E402
    MissionConfig,
    MissionStateMachine,
    Observation,
    State,
    TargetJumpFilter,
)
from uav_tracking.perception.projection import estimate_target_enu  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[3]

VEHICLE_CMD_DO_SET_MODE = 176
VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6


class OffboardNode(Node):
    def __init__(
        self, mission_cfg: MissionConfig, camera_cfg: dict, recorder: MissionRecorder
    ) -> None:
        # Fail at startup, before any ROS entity exists: a typo in the config
        # would otherwise surface as a crash on the first frame with two
        # detections, which is the worst possible moment.
        if mission_cfg.target_selection not in STRATEGIES:
            raise ValueError(
                f"unknown target_selection {mission_cfg.target_selection!r}; "
                f"expected one of {STRATEGIES}"
            )
        super().__init__("uav_offboard")
        self.cfg = mission_cfg
        self.recorder = recorder
        self._previous_state = State.IDLE
        self.image_up_is_forward = bool(camera_cfg.get("image_up_is_forward", True))
        self.mount_offset = tuple(camera_cfg.get("mount_offset_xyz", (0.0, 0.0, 0.0)))
        self.K = np.array(
            [
                [camera_cfg["fx"], 0.0, camera_cfg["cx"]],
                [0.0, camera_cfg["fy"], camera_cfg["cy"]],
                [0.0, 0.0, 1.0],
            ],
            dtype=float,
        )

        self.state_machine = MissionStateMachine(mission_cfg)
        self.target_filter = TargetJumpFilter(mission_cfg.setpoint_jump_limit_m)

        # Vehicle state, ENU + altitude (positive up).
        self.east = 0.0
        self.north = 0.0
        self.altitude = 0.0
        self.yaw = 0.0
        self.armed = False

        # Setpoint held in ENU + altitude, converted to NED when published.
        self.setpoint_east = 0.0
        self.setpoint_north = 0.0
        self.setpoint_alt = float(mission_cfg.takeoff_altitude_m)

        self.latest_detections: TargetArray | None = None
        self.latest_detection_s: float | None = None
        self.last_pixel: tuple[float, float] | None = None
        self.update_count = 0

        self.arming = ArmingSequencer()

        # PX4 publishes /fmu/out/* as best-effort (SensorDataQoS). A default
        # reliable subscription would never match and the node would sit there
        # with no odometry, no arming and no error message - the worst kind of
        # silent failure.
        px4_qos = qos_profile_sensor_data
        self.create_subscription(
            VehicleOdometry, "/fmu/out/vehicle_odometry", self.on_odometry, px4_qos
        )
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status", self.on_status, px4_qos)
        self.create_subscription(TargetArray, "/detections", self.on_detections, 10)

        self.offboard_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", 10
        )
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10
        )
        self.command_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)
        self.target_pub = self.create_publisher(PoseStamped, "/target_pose", 10)

        self.control_rate_hz = float(mission_cfg.control_rate_hz)
        self.dt = 1.0 / self.control_rate_hz
        self.timer = self.create_timer(self.dt, self.on_timer)
        self.get_logger().info("offboard node ready, waiting for odometry")

    # ------------------------------------------------------------------ inputs
    def on_odometry(self, msg: VehicleOdometry) -> None:
        # PX4 odometry is NED: convert once, here, and stay in ENU everywhere else.
        self.east, self.north, self.altitude = ned_position_to_enu(msg.position)
        self.yaw = heading_ned_to_yaw_enu(quaternion_to_heading_ned(msg.q))
        self.update_count += 1

    def on_status(self, msg: VehicleStatus) -> None:
        # px4_msgs/VehicleStatus: arming_state == 2 means ARMED.
        self.armed = int(msg.arming_state) == 2

    def on_detections(self, msg: TargetArray) -> None:
        self.latest_detections = msg
        self.latest_detection_s = self.get_clock().now().nanoseconds / 1e9

    # ------------------------------------------------------------- perception
    def _project_best_detection(self) -> tuple[float, float] | None:
        """Selected detection -> filtered world ENU position, or None."""
        if not self.latest_detections or not self.latest_detections.targets:
            self.target_filter.update(None)
            return None

        message = self.latest_detections
        first = message.targets[0]
        chosen = select_target(
            [
                {
                    "confidence": target.confidence,
                    "bbox_cx": target.bbox_cx,
                    "bbox_cy": target.bbox_cy,
                }
                for target in message.targets
            ],
            strategy=self.cfg.target_selection,
            image_size=(int(first.image_width), int(first.image_height)),
            previous=self.last_pixel,
        )
        if chosen is None:
            self.target_filter.update(None)
            return None

        self.last_pixel = (chosen["bbox_cx"], chosen["bbox_cy"])
        height = max(self.altitude, 0.3)  # guard against a zero/negative altitude sample

        east, north, _ = estimate_target_enu(
            (self.east, self.north),
            chosen["bbox_cx"],
            chosen["bbox_cy"],
            height,
            self.K,
            yaw=self.yaw,
            image_up_is_forward=self.image_up_is_forward,
            offset_body=self.mount_offset,
        )
        return self.target_filter.update((east, north))

    def _target_is_stale(self) -> bool:
        """True when no detection arrived recently enough to act on.

        Uses the receipt time, not the image stamp: a frozen publisher keeps
        re-sending the same stamp, and the vehicle should hover rather than
        chase a frame that is no longer being produced.
        """
        if self.latest_detection_s is None:
            return True
        now = self.get_clock().now().nanoseconds / 1e9
        return (now - self.latest_detection_s) > self.cfg.target_loss_hover_s

    def _detection_stamp_s(self) -> float:
        """Sim-time stamp of the newest detection image, in seconds."""
        stamp = self.latest_detections.header.stamp
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    # ---------------------------------------------------------------- control
    def on_timer(self) -> None:
        """One control tick.

        Order matters: read the freshest estimate, ask the state machine what it
        wants, record any transition, then emit exactly one command. Doing the
        bookkeeping before the command keeps the CSV row attributable to the
        state that produced it.

        This runs at ``control_rate_hz`` (50 Hz) while detection runs at 5 Hz on
        purpose. Inference jitter must not reach the setpoint stream; the
        setpoint is integrated from the last known target instead of being
        rewritten by every frame.
        """
        now_s = self.get_clock().now().nanoseconds / 1e9
        target = None if self._target_is_stale() else self._project_best_detection()
        distance = None
        if target is not None:
            distance = math.hypot(target[0] - self.east, target[1] - self.north)
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "map"
            pose.pose.position.x = target[0]
            pose.pose.position.y = target[1]
            pose.pose.position.z = 0.0
            self.target_pub.publish(pose)

            # Latency is measured against the image stamp, not the receipt time,
            # because the transport delay is exactly what is being measured. The
            # bounds reject the first frames after a clock jump or a bag replay
            # start, where the difference is meaningless.
            latency_ms = (now_s - self._detection_stamp_s()) * 1000.0
            if 0.0 <= latency_ms < 5000.0:
                self.recorder.add_latency(latency_ms)

        decision = self.state_machine.step(
            Observation(
                now_s=now_s,
                altitude_m=self.altitude,
                armed=self.armed,
                distance_to_target_m=distance,
            )
        )

        if decision.state is State.LAND and self._previous_state is not State.LAND:
            row = self.recorder.finish_trial((self.east, self.north), notes=decision.note)
            self.get_logger().info(
                f"trial {row['trial']}: target {row['target_id']}, "
                f"error {row['error_m']:.2f} m, success={row['success']}"
            )
        if decision.command == "takeoff" and self._previous_state is not State.TAKEOFF:
            self.recorder.start_trial()
        self._previous_state = decision.state

        if decision.command == "arm":
            self._request_offboard_and_arm(now_s)
        elif decision.command == "takeoff":
            self._move_towards(self.east, self.north, self.cfg.takeoff_altitude_m)
        elif decision.command == "goto" and target is not None:
            self._move_towards(target[0], target[1], self.cfg.takeoff_altitude_m)
        elif decision.command == "land":
            self._move_towards(self.east, self.north, max(self.altitude - 0.3, 0.0))

        self._publish_setpoint()

    def _move_towards(self, east: float, north: float, altitude: float) -> None:
        """Rate-limited setpoint move, honouring ``max_speed_mps`` in any direction.

        Landing passes ``altitude - 0.3`` each tick; the rate limiter turns that
        into a constant descent instead of a step, so the vehicle does not chase
        a target that is always 0.3 m below it.
        """
        step = self.cfg.max_speed_mps * self.dt
        self.setpoint_east, self.setpoint_north = limited_step(
            (self.setpoint_east, self.setpoint_north), (east, north), step
        )
        (self.setpoint_alt,) = limited_step((self.setpoint_alt,), (altitude,), step)

    def _publish_setpoint(self) -> None:
        # PX4 drops out of OFFBOARD if the setpoint stream stops, and it refuses
        # to enter the mode until the stream has been running. So this is sent on
        # every tick from startup: before arming, during the mission, after
        # landing. Do not gate it behind the state machine.
        #
        # Timestamps are microseconds since boot (px4_msgs convention), not the
        # ROS clock in nanoseconds, even though the values come from the same
        # clock here.
        timestamp = int(self.get_clock().now().nanoseconds / 1000)

        mode = OffboardControlMode()
        mode.position = True
        mode.timestamp = timestamp
        self.offboard_pub.publish(mode)

        setpoint = TrajectorySetpoint()
        setpoint.position = [
            float(self.setpoint_north),  # NED x
            float(self.setpoint_east),  # NED y
            float(-self.setpoint_alt),  # NED z (down positive)
        ]
        # Heading is commanded to north (0) rather than left as NaN on purpose:
        # a fixed heading keeps the camera's image-up direction constant between
        # trials, which is what makes image_up_is_forward comparable. Send NaN
        # instead if the vehicle should keep whatever heading it already has.
        setpoint.yaw = 0.0
        setpoint.timestamp = timestamp
        self.setpoint_pub.publish(setpoint)

    def _send_command(self, command_id: int, param1: float = 0.0, param2: float = 0.0) -> None:
        command = VehicleCommand()
        command.command = command_id
        command.param1 = float(param1)
        command.param2 = float(param2)
        command.target_system = 1
        command.target_component = 1
        command.source_system = 1
        command.source_component = 1
        command.from_external = True
        command.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(command)

    def _request_offboard_and_arm(self, now_s: float) -> None:
        """Retry on a timer instead of firing once and going quiet."""
        step = self.arming.step(now_s, self.armed)
        if step.request_mode:
            self._send_command(
                VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=float(PX4_CUSTOM_MAIN_MODE_OFFBOARD)
            )
            self.get_logger().info("requested OFFBOARD mode")
        if step.request_arm:
            self._send_command(VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            self.get_logger().info(f"requested arming (attempt {step.attempts})")
        if step.warn:
            self.get_logger().error(
                "still not armed after repeated attempts: read the preflight messages in "
                "`ros2 topic echo /fmu/out/vehicle_status` and see docs/troubleshooting.md #5 and #6"
            )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mission", default="config/mission.yaml")
    parser.add_argument("--camera", default="config/camera.yaml")
    parser.add_argument("--targets", default="config/targets.yaml")
    parser.add_argument("--results", default="results/landing_error.csv")
    args = parser.parse_args()

    with open(args.mission, encoding="utf-8") as handle:
        mission_raw = yaml.safe_load(handle)
    with open(args.camera, encoding="utf-8") as handle:
        camera_raw = yaml.safe_load(handle)
    with open(args.targets, encoding="utf-8") as handle:
        targets_raw = yaml.safe_load(handle)

    ground_truth = {
        int(item["id"]): (float(item["x"]), float(item["y"])) for item in targets_raw["targets"]
    }
    recorder = MissionRecorder(args.results, ground_truth)

    mission_cfg = MissionConfig(
        takeoff_altitude_m=mission_raw.get("takeoff_altitude_m", 5.0),
        reach_tolerance_m=mission_raw.get("reach_tolerance_m", 1.0),
        hover_duration_s=mission_raw.get("hover_duration_s", 10.0),
        target_loss_hover_s=mission_raw.get("target_loss_hover_s", 2.0),
        setpoint_jump_limit_m=mission_raw.get("setpoint_jump_limit_m", 3.0),
        max_speed_mps=mission_raw.get("max_speed_mps", 1.0),
        land_after_hover=mission_raw.get("land_after_hover", True),
        control_rate_hz=mission_raw.get("control_rate_hz", 50.0),
        target_selection=mission_raw.get("target_selection", "nearest_centre"),
    )

    rclpy.init()
    node = OffboardNode(mission_cfg, camera_raw, recorder)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
