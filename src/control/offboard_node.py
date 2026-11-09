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

from src.control.recorder import MissionRecorder  # noqa: E402
from src.control.state_machine import (  # noqa: E402
    MissionConfig,
    MissionStateMachine,
    Observation,
    State,
    TargetJumpFilter,
)
from src.perception.projection import estimate_target_enu  # noqa: E402
from uav_interfaces.msg import TargetArray  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[2]

VEHICLE_CMD_DO_SET_MODE = 176
VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6


def quaternion_to_yaw(q: np.ndarray) -> float:
    """Yaw (rad, NED, 0 = north) from a Hamilton quaternion (w, x, y, z)."""
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class OffboardNode(Node):
    def __init__(
        self, mission_cfg: MissionConfig, camera_cfg: dict, recorder: MissionRecorder
    ) -> None:
        super().__init__("uav_offboard")
        self.cfg = mission_cfg
        self.recorder = recorder
        self._previous_state = State.IDLE
        self.image_up_is_forward = bool(camera_cfg.get("image_up_is_forward", True))
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
        self.update_count = 0

        self.offboard_mode_requested = False
        self.arm_requested = False

        self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry", self.on_odometry, 10)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status", self.on_status, 10)
        self.create_subscription(TargetArray, "/detections", self.on_detections, 10)

        self.offboard_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", 10
        )
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10
        )
        self.command_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)
        self.target_pub = self.create_publisher(PoseStamped, "/target_pose", 10)

        self.control_rate_hz = 50.0
        self.dt = 1.0 / self.control_rate_hz
        self.timer = self.create_timer(self.dt, self.on_timer)
        self.get_logger().info("offboard node ready, waiting for odometry")

    # ------------------------------------------------------------------ inputs
    def on_odometry(self, msg: VehicleOdometry) -> None:
        # PX4 odometry is NED: x north, y east, z down.
        self.north = float(msg.position[0])
        self.east = float(msg.position[1])
        self.altitude = -float(msg.position[2])
        self.yaw = quaternion_to_yaw(np.asarray(msg.q, dtype=float))
        self.update_count += 1

    def on_status(self, msg: VehicleStatus) -> None:
        # px4_msgs/VehicleStatus: arming_state == 2 means ARMED.
        self.armed = int(msg.arming_state) == 2

    def on_detections(self, msg: TargetArray) -> None:
        self.latest_detections = msg
        self.latest_detection_s = self.get_clock().now().nanoseconds / 1e9

    # ------------------------------------------------------------- perception
    def _project_best_detection(self) -> tuple[float, float] | None:
        """Best detection -> filtered world ENU position, or None."""
        if not self.latest_detections or not self.latest_detections.targets:
            self.target_filter.update(None)
            return None

        newest = max(self.latest_detections.targets, key=lambda t: t.confidence)
        height = max(self.altitude, 0.3)  # guard against a zero/negative altitude sample

        east, north, _ = estimate_target_enu(
            (self.east, self.north),
            newest.bbox_cx,
            newest.bbox_cy,
            height,
            self.K,
            yaw=self.yaw,
            image_up_is_forward=self.image_up_is_forward,
        )
        return self.target_filter.update((east, north))

    def _target_is_stale(self) -> bool:
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
            self._request_offboard_and_arm()
        elif decision.command == "takeoff":
            self._move_towards(self.east, self.north, self.cfg.takeoff_altitude_m)
        elif decision.command == "goto" and target is not None:
            self._move_towards(target[0], target[1], self.cfg.takeoff_altitude_m)
        elif decision.command == "land":
            self._move_towards(self.east, self.north, max(self.altitude - 0.3, 0.0))

        self._publish_setpoint()

    def _move_towards(self, east: float, north: float, altitude: float) -> None:
        """Rate-limited setpoint move, honouring ``max_speed_mps``."""
        step = self.cfg.max_speed_mps * self.dt
        self.setpoint_east += float(np.clip(east - self.setpoint_east, -step, step))
        self.setpoint_north += float(np.clip(north - self.setpoint_north, -step, step))
        self.setpoint_alt += float(np.clip(altitude - self.setpoint_alt, -step, step))

    def _publish_setpoint(self) -> None:
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
        setpoint.yaw = 0.0
        setpoint.timestamp = timestamp
        self.setpoint_pub.publish(setpoint)

    def _request_offboard_and_arm(self) -> None:
        timestamp = int(self.get_clock().now().nanoseconds / 1000)

        if not self.offboard_mode_requested:
            command = VehicleCommand()
            command.command = VEHICLE_CMD_DO_SET_MODE
            command.param1 = 1.0  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            command.param2 = float(PX4_CUSTOM_MAIN_MODE_OFFBOARD)
            command.target_system = 1
            command.target_component = 1
            command.source_system = 1
            command.source_component = 1
            command.from_external = True
            command.timestamp = timestamp
            self.command_pub.publish(command)
            self.offboard_mode_requested = True
            self.get_logger().info("requested OFFBOARD mode")

        if not self.arm_requested:
            command = VehicleCommand()
            command.command = VEHICLE_CMD_COMPONENT_ARM_DISARM
            command.param1 = 1.0
            command.target_system = 1
            command.target_component = 1
            command.source_system = 1
            command.source_component = 1
            command.from_external = True
            command.timestamp = timestamp
            self.command_pub.publish(command)
            self.arm_requested = True
            self.get_logger().info("requested arming")


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
