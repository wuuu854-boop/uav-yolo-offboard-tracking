#!/usr/bin/env python3
"""PX4 Offboard node: detections -> ground projection -> trajectory setpoint.

Subscribes
    px4_msgs/VehicleOdometry      /fmu/out/vehicle_odometry
    uav_interfaces/TargetArray    /detections
    std_msgs/Bool                 /avoidance/stop     (hold horizontal cruise)
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
import os
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
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from std_msgs.msg import Bool, Float32, Float32MultiArray, String  # noqa: E402

from uav_interfaces.msg import TargetArray  # noqa: E402
from uav_tracking.control.arming import ArmingSequencer  # noqa: E402
from uav_tracking.control.detour import (  # noqa: E402
    DetourConfig,
    DetourDecision,
    DetourPhase,
    DetourPlanner,
    adaptive_backoff_step,
)
from uav_tracking.control.frames import (  # noqa: E402
    heading_ned_to_yaw_enu,
    ned_position_to_enu,
    quaternion_to_heading_ned,
    quaternion_to_roll_pitch,
)
from uav_tracking.control.geofence import clamp_to_fence  # noqa: E402
from uav_tracking.control.limiter import limited_step  # noqa: E402
from uav_tracking.control.patrol import PatrolPlan  # noqa: E402
from uav_tracking.control.recorder import MissionRecorder, localise_targets  # noqa: E402
from uav_tracking.control.search import SearchPattern, lawnmower  # noqa: E402
from uav_tracking.control.selection import STRATEGIES, select_target  # noqa: E402
from uav_tracking.control.state_machine import (  # noqa: E402
    MissionConfig,
    MissionStateMachine,
    Observation,
    State,
    TargetJumpFilter,
)
from uav_tracking.control.trace import StateTrace  # noqa: E402
from uav_tracking.perception.depth import StopWatchdog  # noqa: E402
from uav_tracking.perception.projection import estimate_target_enu  # noqa: E402
from uav_tracking.planning.global_planner import (  # noqa: E402
    Box,
    obstacle_box_from_measurement,
    plan_path,
)

REPO_ROOT = Path(__file__).resolve().parents[3]

VEHICLE_CMD_DO_SET_MODE = 176
VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6


class OffboardNode(Node):
    def __init__(
        self,
        mission_cfg: MissionConfig,
        camera_cfg: dict,
        recorder: MissionRecorder,
        trace: StateTrace | None = None,
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
        self.trace = trace
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
        # Attitude, because a projection that assumes a level camera loses about
        # a metre per 10 deg of pitch (docs/model.md).
        self.roll = 0.0
        self.pitch = 0.0
        self.armed = False

        # Setpoint held in ENU + altitude, converted to NED when published.
        self.setpoint_east = 0.0
        self.setpoint_north = 0.0
        self.setpoint_alt = float(mission_cfg.takeoff_altitude_m)

        self.latest_detections: TargetArray | None = None
        self.latest_detection_s: float | None = None
        self.last_pixel: tuple[float, float] | None = None
        self.locked_track_id: int | None = None
        self.search: SearchPattern | None = None
        # Empty unless the mission config lists waypoints: the single-target
        # mission never touches it.
        self.patrol = PatrolPlan.from_config(mission_cfg.patrol_waypoints)
        self.update_count = 0

        self.arming = ArmingSequencer()
        # The depth node publishes a stop decision at ~10 Hz. The watchdog is
        # inactive until the first message, so the tracking mission (which does
        # not start that node) is unchanged; after that, silence means stop.
        self.avoidance = StopWatchdog(mission_cfg.avoidance_timeout_s)
        self.avoidance_holding = False
        # Latched on the tick the stop engages. Holding (self.east, self.north)
        # every tick would follow the vehicle's own drift instead of correcting
        # it; one fixed point is what makes "stopped" measurable. It is the
        # vehicle position, not the setpoint: the setpoint can be a metre ahead
        # during cruise, and freezing it would let the vehicle fly to it.
        self.avoidance_hold: tuple[float, float] | None = None
        # Phase 2: a lateral leg around a static obstacle. Disabled by default;
        # the tracking missions must keep their exact behaviour.
        self.detour = DetourPlanner(
            DetourConfig(
                enabled=mission_cfg.detour_enabled,
                side=mission_cfg.detour_side,
                offset_m=mission_cfg.detour_offset_m,
                forward_m=mission_cfg.detour_forward_m,
                reach_tolerance_m=mission_cfg.detour_reach_tolerance_m,
                clear_hold_s=mission_cfg.detour_clear_hold_s,
                wait_timeout_s=mission_cfg.detour_wait_timeout_s,
                max_attempts=mission_cfg.detour_max_attempts,
                start_delay_s=mission_cfg.detour_start_delay_s,
                backoff_m=mission_cfg.detour_backoff_m,
                backoff_max_attempts=mission_cfg.detour_backoff_max_attempts,
                search_step_m=mission_cfg.detour_search_step_m,
                search_max_attempts=mission_cfg.detour_search_max_attempts,
            )
        )
        self._detour_phase = DetourPhase.INACTIVE
        # The depth node also says which side looks freer. "unknown" means no
        # usable return on either side; the planner then uses detour_side.
        self.clear_side = "unknown"
        self.global_path: list[tuple[float, float]] = []
        self._global_goal: tuple[float, float] | None = None
        self.nearest_m = math.nan
        self._perception_box: tuple[float, float, float, float] | None = None
        self._perception_done = False
        self.side_left_m = math.nan
        self.side_right_m = math.nan
        # Once a side has been blocked during a stop, hold for the rest of that
        # stop. The label can flicker as the vehicle pitches to a halt; a detour
        # must not start on a one-frame "left" between two "blocked" frames.
        self._blocked_seen = False

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
        self.create_subscription(Bool, "/avoidance/stop", self.on_avoidance_stop, 10)
        self.create_subscription(Float32, "/avoidance/nearest_m", self.on_nearest, 10)
        self.create_subscription(String, "/avoidance/clear_side", self.on_clear_side, 10)
        self.create_subscription(
            Float32MultiArray, "/avoidance/side_clearance", self.on_side_clearance, 10
        )

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
        self.roll, self.pitch = quaternion_to_roll_pitch(msg.q)
        self.update_count += 1

    def on_status(self, msg: VehicleStatus) -> None:
        # px4_msgs/VehicleStatus: arming_state == 2 means ARMED.
        self.armed = int(msg.arming_state) == 2

    def on_detections(self, msg: TargetArray) -> None:
        self.latest_detections = msg
        self.latest_detection_s = self.get_clock().now().nanoseconds / 1e9

    def on_avoidance_stop(self, msg: Bool) -> None:
        now_s = self.get_clock().now().nanoseconds / 1e9
        self.avoidance.update(bool(msg.data), now_s)

    def on_nearest(self, msg: Float32) -> None:
        self.nearest_m = float(msg.data)

    def on_clear_side(self, msg: String) -> None:
        side = str(msg.data).strip().lower()
        if side in ("left", "right", "blocked", "unknown"):
            self.clear_side = side

    def on_side_clearance(self, msg: Float32MultiArray) -> None:
        values = [float(value) for value in msg.data]
        if len(values) >= 2:
            self.side_left_m, self.side_right_m = values[0], values[1]

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
                    "track_id": target.track_id,
                }
                for target in message.targets
            ],
            strategy=self.cfg.target_selection,
            image_size=(int(first.image_width), int(first.image_height)),
            previous=self.last_pixel,
            locked_id=self.locked_track_id,
        )
        if chosen is None:
            self.target_filter.update(None)
            return None

        # Commit to the chosen identity so the next frame follows the same object
        # instead of re-deciding between two plausible ones.
        if int(chosen.get("track_id", 0)) > 0:
            self.locked_track_id = int(chosen["track_id"])

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
            roll=self.roll,
            pitch=self.pitch,
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
        # One-shot perception-to-planning: the first cruise stop that is not the
        # takeoff ground gives a measured obstacle in front. Turn it into a
        # conservative box, replan, then ignore the reactive stop for this
        # mission so the planned path can fly.
        if (
            self.cfg.global_perception
            and not self._perception_done
            and self.altitude >= 4.0
            and self.avoidance.active(now_s)
            and math.isfinite(self.nearest_m)
            and 0.5 < self.nearest_m < 4.0
        ):
            box = obstacle_box_from_measurement(
                (self.east, self.north),
                self.nearest_m,
                self.yaw,
                self.cfg.global_perception_width_m,
                self.cfg.global_perception_thickness_m,
            )
            self._perception_box = (box.centre_x, box.centre_y, box.size_x, box.size_y)
            self._perception_done = True
            self._global_goal = None
            self.get_logger().info(
                "perception obstacle: "
                f"nearest {self.nearest_m:.2f} m -> box at "
                f"({box.centre_x:.2f}, {box.centre_y:.2f}) "
                f"{box.size_x:.2f} x {box.size_y:.2f} m; replanning"
            )
        estimate = None if self._target_is_stale() else self._project_best_detection()
        # Patrol legs are waypoint-driven on purpose. The first version let a
        # detection override the goal, and the vehicle kept turning back to the
        # target it had just visited, because that target was still in view. The
        # single-target mission keeps following detections.
        target = estimate if self.patrol.done() else self.patrol.current()
        move_target = target
        if self.cfg.global_planning and target is not None:
            goal = (float(target[0]), float(target[1]))
            if self._global_goal != goal:
                obstacles = [Box(*spec) for spec in self.cfg.global_obstacles]
                if self._perception_box is not None:
                    obstacles.append(Box(*self._perception_box))
                plan_goal = self.cfg.global_pre_goal or goal
                try:
                    self.global_path = plan_path(
                        (self.east, self.north),
                        plan_goal,
                        obstacles,
                        self.cfg.global_bounds,
                        resolution_m=self.cfg.global_resolution_m,
                        inflation_m=self.cfg.global_inflation_m,
                    )
                    if plan_goal != goal:
                        self.global_path.append(goal)
                    self.get_logger().info(
                        f"global path: {len(self.global_path)} waypoint(s) around "
                        f"{len(obstacles)} obstacle(s)"
                    )
                except ValueError as error:
                    self.get_logger().error(f"global plan failed: {error}; flying direct")
                    self.global_path = []
                self._global_goal = goal
            while (
                self.global_path
                and math.hypot(
                    self.global_path[0][0] - self.east,
                    self.global_path[0][1] - self.north,
                )
                <= self.cfg.global_waypoint_tolerance_m
            ):
                self.global_path.pop(0)
            if self.global_path:
                move_target = self.global_path[0]
        if estimate is None:
            # A lost target releases the identity lock: the next sighting may be
            # a different object, and holding the id would make the mission
            # refuse to follow anything.
            self.locked_track_id = None
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

        if estimate is not None:
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
                goal_pending=not self.patrol.done(),
            )
        )

        if decision.note == "next waypoint":
            self.patrol.advance()
            if self.patrol.done():
                decision = self.state_machine.request_land("patrol complete")
            else:
                self.get_logger().info(
                    f"patrol: waypoint {self.patrol.index} of {len(self.patrol.waypoints)}"
                )

        # The gate overrides horizontal cruise only. Takeoff, hover and landing
        # keep their commands: refusing to climb or refusing to land because a
        # wall is in front would turn an obstacle into a different incident.
        stop_enabled = self.cfg.avoidance_stop_enabled and not (
            self.cfg.global_perception and self._perception_done
        )
        avoiding = stop_enabled and self.avoidance.active(now_s)
        if not avoiding:
            self._blocked_seen = False
        elif self.detour.phase is DetourPhase.INACTIVE and self.clear_side == "blocked":
            # Latch only while the planner has not committed. Once it is backing
            # off or holding, use the live label so a clear side can recover.
            self._blocked_seen = True
        elif self.detour.phase is not DetourPhase.INACTIVE:
            self._blocked_seen = False
        side_for_planner = None
        if self.cfg.detour_auto_side:
            side_for_planner = "blocked" if self._blocked_seen else self.clear_side
        if self.cfg.detour_adaptive_backoff and side_for_planner == "blocked":
            step = adaptive_backoff_step(
                self.side_left_m,
                self.side_right_m,
                target_m=self.cfg.detour_backoff_target_m,
                min_step_m=self.cfg.detour_backoff_min_step_m,
                max_step_m=self.cfg.detour_backoff_max_step_m,
            )
            self.detour.set_backoff_step(step)
        else:
            self.detour.set_backoff_step(None)
        search_side = None
        if math.isfinite(self.side_left_m) and math.isfinite(self.side_right_m):
            search_side = "left" if self.side_left_m >= self.side_right_m else "right"
        detour = DetourDecision(False, DetourPhase.INACTIVE)
        if decision.command == "goto" and target is not None:
            detour = self.detour.update(
                (self.east, self.north),
                target,
                avoiding,
                now_s,
                side=side_for_planner,
                search_side=search_side,
            )
        else:
            self.detour.reset()

        if detour.active:
            if detour.phase is not self._detour_phase:
                if detour.phase is DetourPhase.HOLDING:
                    self.get_logger().error(f"detour cannot clear the obstacle: {detour.note}")
                elif detour.phase is DetourPhase.BACKOFF:
                    self.get_logger().warn(f"obstacle within stop threshold: {detour.note}")
                elif detour.phase in (DetourPhase.LATERAL, DetourPhase.FORWARD):
                    self.get_logger().warn(
                        f"obstacle within stop threshold: {detour.note} to the "
                        f"{detour.side or self.cfg.detour_side}, waypoint {detour.waypoint}"
                    )
                else:
                    self.get_logger().info(detour.note)
            self.avoidance_hold = None
            self.avoidance_holding = False
        elif self._detour_phase is not DetourPhase.INACTIVE:
            self.get_logger().info(f"detour finished: {detour.note}")
        self._detour_phase = detour.phase

        holding = decision.command == "goto" and avoiding and not detour.active
        if holding and not self.avoidance_holding:
            self.avoidance_hold = (self.east, self.north)
            self.get_logger().warn(
                "obstacle within stop threshold: holding position instead of advancing"
            )
        elif not holding and self.avoidance_holding:
            self.avoidance_hold = None
            if decision.command == "goto":
                self.get_logger().info("obstacle clear: resuming the approach")
            else:
                # The mission timed out or was told to land. The gate did not
                # clear; the state change released it. Saying "resuming" here
                # would misreport why the vehicle started moving again.
                self.get_logger().info(
                    f"stop gate released by {decision.state.value}, not by a clear reading"
                )
        self.avoidance_holding = holding

        if decision.state is State.LAND and self._previous_state is not State.LAND:
            row = self.recorder.finish_trial((self.east, self.north), notes=decision.note)
            self.get_logger().info(
                f"trial {row['trial']}: target {row['target_id']}, "
                f"error {row['error_m']:.2f} m, success={row['success']}"
            )
        if decision.command == "takeoff" and self._previous_state is not State.TAKEOFF:
            self.recorder.start_trial()

        # One row per state change: when a trial goes wrong this is the only
        # record of which state it was in and why it moved. A held approach or a
        # detour is a change of reason, not of state, so it replaces the empty
        # GOTO note.
        note = decision.note
        if detour.active:
            note = f"detour: {detour.note}"
        elif detour.note == "both sides blocked: holding":
            note = "blocked: holding"
        elif holding:
            note = "avoidance stop"
        if self.trace is not None:
            self.trace.write(
                now_s,
                decision.state.value,
                note,
                position=(self.east, self.north),
                altitude_m=self.altitude,
            )

        self._previous_state = decision.state

        if decision.command == "arm":
            self._request_offboard_and_arm(now_s)
        elif decision.command == "takeoff":
            self._move_towards(self.east, self.north, self.cfg.takeoff_altitude_m)
        elif decision.command == "goto" and target is not None:
            if detour.active and detour.waypoint is not None:
                self._move_towards(
                    detour.waypoint[0], detour.waypoint[1], self.cfg.takeoff_altitude_m
                )
            elif holding:
                assert self.avoidance_hold is not None
                hold_east, hold_north = self.avoidance_hold
                self._move_towards(hold_east, hold_north, self.cfg.takeoff_altitude_m)
            else:
                self._move_towards(move_target[0], move_target[1], self.cfg.takeoff_altitude_m)
        elif decision.command == "hover":
            # Hold where the vehicle actually is. Leaving the setpoint where GOTO
            # left it makes the vehicle finish closing a lag of up to a metre
            # during the hover, which is what the landing error then measures.
            self._move_towards(self.east, self.north, self.cfg.takeoff_altitude_m)
        elif decision.command == "search":
            self._fly_search_pattern()
        elif decision.command == "land":
            self._move_towards(self.east, self.north, max(self.altitude - 0.3, 0.0))

        if decision.command != "search":
            # Leaving the search drops the pattern, so a later loss starts a new
            # sweep from wherever the vehicle is then.
            self.search = None

        self._publish_setpoint()

    def _fly_search_pattern(self) -> None:
        """Cover the area around the point where the target was last seen."""
        if self.search is None:
            self.search = SearchPattern(
                lawnmower(
                    centre=(self.east, self.north),
                    width_m=self.cfg.search_area_m,
                    height_m=self.cfg.search_area_m,
                    spacing_m=self.cfg.search_spacing_m,
                ),
                tolerance_m=self.cfg.reach_tolerance_m,
            )
            self.get_logger().info(f"searching: {len(self.search.waypoints)} waypoints")

        waypoint = self.search.update((self.east, self.north))
        if waypoint is None:
            # Pattern exhausted without a detection: hold position rather than
            # wander. Landing here would be a decision the operator did not ask
            # for; the state machine stays in SEARCH until the target returns.
            self._move_towards(self.east, self.north, self.cfg.takeoff_altitude_m)
            return
        self._move_towards(waypoint[0], waypoint[1], self.cfg.takeoff_altitude_m)

    def _move_towards(self, east: float, north: float, altitude: float) -> None:
        """Rate-limited setpoint move, honouring ``max_speed_mps`` in any direction.

        Every horizontal command goes through here, so the fence is enforced in
        one place: a bad estimate becomes a boundary setpoint, not a flight away.

        Landing passes ``altitude - 0.3`` each tick; the rate limiter turns that
        into a constant descent instead of a step, so the vehicle does not chase
        a target that is always 0.3 m below it.
        """
        east, north = clamp_to_fence((east, north), (0.0, 0.0), self.cfg.geofence_radius_m)
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
        # Heading is commanded rather than left as NaN on purpose: a fixed
        # heading keeps the camera's image-up direction constant between trials,
        # which is what makes image_up_is_forward comparable. The avoidance
        # mission sets 90 deg (east) so its forward depth camera faces the wall.
        # NaN would let PX4 keep whatever heading it had, which is not something
        # a camera-based decision can rely on.
        setpoint.yaw = math.radians(self.cfg.heading_yaw_deg)
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
    parser.add_argument("--trace", default="results/state_trace.csv")
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

    # PX4's local frame starts where the vehicle spawned; the targets are in world
    # coordinates. The spawn offset comes from the mission config and can be
    # overridden per run, because each trial spawns somewhere else.
    spawn_east, spawn_north = mission_raw.get("spawn_offset_enu", [0.0, 0.0])
    override = os.environ.get("UAV_SPAWN_ENU")
    if override:
        spawn_east, spawn_north = (float(value) for value in override.split(","))
    ground_truth = localise_targets(ground_truth, (spawn_east, spawn_north))
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
        search_after_s=mission_raw.get("search_after_s", 8.0),
        search_area_m=mission_raw.get("search_area_m", 20.0),
        search_spacing_m=mission_raw.get("search_spacing_m", 8.9),
        max_mission_s=mission_raw.get("max_mission_s", 0.0),
        geofence_radius_m=mission_raw.get("geofence_radius_m", 30.0),
        confirm_frames=int(mission_raw.get("confirm_frames", 3)),
        heading_yaw_deg=float(mission_raw.get("heading_yaw_deg", 0.0)),
        avoidance_timeout_s=mission_raw.get("avoidance_timeout_s", 1.0),
        avoidance_stop_enabled=bool(mission_raw.get("avoidance_stop_enabled", True)),
        detour_enabled=bool(mission_raw.get("detour_enabled", False)),
        detour_side=str(mission_raw.get("detour_side", "right")),
        detour_auto_side=bool(mission_raw.get("detour_auto_side", False)),
        detour_offset_m=float(mission_raw.get("detour_offset_m", 3.0)),
        detour_forward_m=float(mission_raw.get("detour_forward_m", 3.0)),
        detour_reach_tolerance_m=float(mission_raw.get("detour_reach_tolerance_m", 0.5)),
        detour_min_lateral_m=float(mission_raw.get("detour_min_lateral_m", 2.5)),
        detour_clear_hold_s=float(mission_raw.get("detour_clear_hold_s", 1.0)),
        detour_wait_timeout_s=float(mission_raw.get("detour_wait_timeout_s", 5.0)),
        detour_max_attempts=int(mission_raw.get("detour_max_attempts", 2)),
        detour_start_delay_s=float(mission_raw.get("detour_start_delay_s", 0.0)),
        detour_backoff_m=float(mission_raw.get("detour_backoff_m", 0.0)),
        detour_backoff_max_attempts=int(mission_raw.get("detour_backoff_max_attempts", 1)),
        detour_adaptive_backoff=bool(mission_raw.get("detour_adaptive_backoff", False)),
        detour_backoff_target_m=float(mission_raw.get("detour_backoff_target_m", 0.0)),
        detour_backoff_min_step_m=float(mission_raw.get("detour_backoff_min_step_m", 0.2)),
        detour_backoff_max_step_m=float(mission_raw.get("detour_backoff_max_step_m", 2.0)),
        detour_search_step_m=float(mission_raw.get("detour_search_step_m", 0.0)),
        detour_search_max_attempts=int(mission_raw.get("detour_search_max_attempts", 0)),
        global_planning=bool(mission_raw.get("global_planning", False)),
        global_bounds=tuple(
            float(v) for v in mission_raw.get("global_bounds", (-10.0, 10.0, -10.0, 10.0))
        ),
        global_obstacles=tuple(
            tuple(float(v) for v in spec) for spec in mission_raw.get("global_obstacles", ())
        ),
        global_resolution_m=float(mission_raw.get("global_resolution_m", 0.25)),
        global_inflation_m=float(mission_raw.get("global_inflation_m", 0.8)),
        global_waypoint_tolerance_m=float(mission_raw.get("global_waypoint_tolerance_m", 0.5)),
        global_pre_goal=(
            tuple(float(v) for v in mission_raw["global_pre_goal"])
            if mission_raw.get("global_pre_goal")
            else None
        ),
        global_perception=bool(mission_raw.get("global_perception", False)),
        global_perception_width_m=float(mission_raw.get("global_perception_width_m", 4.0)),
        global_perception_thickness_m=float(mission_raw.get("global_perception_thickness_m", 0.4)),
        patrol_waypoints=tuple(
            (float(pair[0]), float(pair[1])) for pair in mission_raw.get("patrol_waypoints", ())
        ),
    )

    rclpy.init()
    node = OffboardNode(mission_cfg, camera_raw, recorder, StateTrace(args.trace))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
