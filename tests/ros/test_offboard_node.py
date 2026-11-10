"""Integration test for the offboard node.

The node is the one piece with no unit coverage: it needs rclpy, the PX4 message
types and a clock. This test feeds it fake ``/fmu/out`` traffic and a synthetic
detection, then checks what it publishes — which is the only way to know that the
frame conversions, the selection and the projection are wired together.

Runs in the `ROS build` workflow, where `px4_msgs` and `uav_interfaces` exist.
"""

from __future__ import annotations

import time
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip("rclpy")
# Built by the ROS workflow; without it these tests cannot be collected at all.
pytest.importorskip("uav_interfaces")

from geometry_msgs.msg import PoseStamped  # noqa: E402
from px4_msgs.msg import (  # noqa: E402
    TrajectorySetpoint,
    VehicleCommand,
    VehicleOdometry,
    VehicleStatus,
)
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402

from uav_interfaces.msg import Target, TargetArray  # noqa: E402
from uav_tracking.control.offboard_node import OffboardNode  # noqa: E402
from uav_tracking.control.recorder import MissionRecorder  # noqa: E402
from uav_tracking.control.state_machine import MissionConfig  # noqa: E402
from uav_tracking.control.trace import StateTrace  # noqa: E402

CAMERA = {
    "fx": 538.4,
    "fy": 538.4,
    "cx": 640.0,
    "cy": 480.0,
    "image_up_is_forward": True,
    "mount_offset_xyz": [0.0, 0.0, 0.10],
}
ALTITUDE_M = 5.0
ARMED = 2
DISARMED = 1


def _odometry(
    north: float = 0.0, east: float = 0.0, altitude: float = ALTITUDE_M
) -> VehicleOdometry:
    message = VehicleOdometry()
    message.position = [float(north), float(east), float(-altitude)]  # NED, z down
    message.q = [1.0, 0.0, 0.0, 0.0]  # heading 0 in NED = facing north
    message.timestamp = int(time.time() * 1e6)
    return message


def _status(armed: bool) -> VehicleStatus:
    message = VehicleStatus()
    message.arming_state = ARMED if armed else DISARMED
    return message


def _detections(u: float, v: float, width: int = 1280, height: int = 960) -> TargetArray:
    target = Target()
    target.label = "target"
    target.confidence = 0.9
    target.bbox_cx = u
    target.bbox_cy = v
    target.bbox_w = 60.0
    target.bbox_h = 60.0
    target.image_width = width
    target.image_height = height

    array = TargetArray()
    array.targets.append(target)
    return array


@pytest.fixture(scope="module")
def rig(tmp_path_factory):
    rclpy.init()

    cfg = MissionConfig(takeoff_altitude_m=ALTITUDE_M, target_loss_hover_s=2.0)
    results_dir = tmp_path_factory.mktemp("offboard")
    recorder = MissionRecorder(results_dir / "landing_error.csv", {0: (0.0, 0.0)})
    trace_path = results_dir / "state_trace.csv"
    node = OffboardNode(cfg, CAMERA, recorder, StateTrace(trace_path))
    helper = Node("offboard_test_helper")

    poses: list = []
    setpoints: list = []
    commands: list = []
    helper.create_subscription(PoseStamped, "/target_pose", poses.append, 10)
    helper.create_subscription(
        TrajectorySetpoint, "/fmu/in/trajectory_setpoint", setpoints.append, 10
    )
    helper.create_subscription(VehicleCommand, "/fmu/in/vehicle_command", commands.append, 10)

    odometry_pub = helper.create_publisher(
        VehicleOdometry, "/fmu/out/vehicle_odometry", qos_profile_sensor_data
    )
    status_pub = helper.create_publisher(
        VehicleStatus, "/fmu/out/vehicle_status", qos_profile_sensor_data
    )
    detection_pub = helper.create_publisher(TargetArray, "/detections", 10)

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(helper)

    state = SimpleNamespace(armed=False)

    def tick(detection: tuple[float, float] | None = None) -> None:
        odometry_pub.publish(_odometry())
        status_pub.publish(_status(state.armed))
        if detection is not None:
            detection_pub.publish(_detections(*detection))
        executor.spin_once(timeout_sec=0.02)

    yield SimpleNamespace(
        executor=executor,
        node=node,
        helper=helper,
        poses=poses,
        setpoints=setpoints,
        commands=commands,
        state=state,
        tick=tick,
        trace_path=trace_path,
    )

    executor.shutdown()
    node.destroy_node()
    helper.destroy_node()
    rclpy.try_shutdown()


def _pump(rig, until, timeout_s: float = 8.0, detection: tuple[float, float] | None = None) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rig.tick(detection)
        if until():
            return True
    return False


def test_an_unknown_selection_strategy_is_rejected_at_construction(tmp_path):
    # Raised before any ROS entity is created, so this needs no rclpy context.
    recorder = MissionRecorder(tmp_path / "landing_error.csv", {0: (0.0, 0.0)})
    cfg = MissionConfig(target_selection="biggest")

    with pytest.raises(ValueError, match="target_selection"):
        OffboardNode(cfg, CAMERA, recorder)


def test_a_disarmed_vehicle_is_asked_for_offboard_mode_and_arming(rig):
    # The sequencer sends mode + arm on its first tick and retries the arm
    # every 2 s. It must be visible before anything else happens.
    assert _pump(rig, lambda: any(command.command == 176 for command in rig.commands)), (
        "no OFFBOARD mode request"
    )
    assert _pump(rig, lambda: any(command.command == 400 for command in rig.commands)), (
        "no arming request"
    )


def test_the_setpoint_stream_keeps_running(rig):
    # PX4 leaves OFFBOARD when this stops, so it must run before and after arming.
    count = len(rig.setpoints)
    assert _pump(rig, lambda: len(rig.setpoints) > count + 5), "setpoint stream stalled"

    rig.state.armed = True
    count = len(rig.setpoints)
    assert _pump(rig, lambda: len(rig.setpoints) > count + 5), (
        "setpoint stream stalled after arming"
    )


def test_setpoint_z_is_ned_down_for_a_positive_altitude(rig):
    assert _pump(rig, lambda: len(rig.setpoints) > 0)
    # takeoff altitude 5 m -> z = -5 in NED. A sign error here would climb down.
    assert rig.setpoints[-1].position[2] == pytest.approx(-ALTITUDE_M, abs=0.01)


def test_a_centred_detection_projects_to_the_vehicle_position(rig):
    rig.poses.clear()
    assert _pump(rig, lambda: bool(rig.poses), detection=(CAMERA["cx"], CAMERA["cy"])), (
        "no /target_pose published"
    )

    pose = rig.poses[-1].pose.position
    assert pose.x == pytest.approx(0.0, abs=0.05)
    assert pose.y == pytest.approx(0.0, abs=0.05)


def test_a_pixel_right_of_centre_maps_to_the_right_of_the_heading(rig):
    rig.poses.clear()
    # 200 px right of centre, height 5.1 m (5 m + the 0.10 m mount offset),
    # fx 538.4 -> 1.89 m to the vehicle's right. Heading north puts right east.
    assert _pump(rig, lambda: bool(rig.poses), detection=(CAMERA["cx"] + 200.0, CAMERA["cy"]))

    pose = rig.poses[-1].pose.position
    assert pose.x == pytest.approx(200.0 * (ALTITUDE_M + 0.10) / CAMERA["fx"], abs=0.1)
    assert pose.y == pytest.approx(0.0, abs=0.1)


def test_the_state_trace_records_the_transitions(rig):
    # Arm the vehicle so the machine leaves IDLE, then read the trace back.
    assert _pump(rig, lambda: True, timeout_s=0.5)
    rig.state.armed = True
    assert _pump(rig, lambda: True, timeout_s=0.5)

    lines = rig.trace_path.read_text(encoding="utf-8").strip().splitlines()
    assert lines[0].startswith("t_s,state,note")
    states = [line.split(",")[1] for line in lines[1:]]
    assert "IDLE" in states
    assert "TAKEOFF" in states


def test_no_detection_means_no_target_pose(rig):
    # Let the last detection go stale (2 s timeout) while keeping PX4 alive.
    deadline = time.monotonic() + 2.6
    while time.monotonic() < deadline:
        rig.tick(detection=None)

    rig.poses.clear()
    assert not _pump(rig, lambda: bool(rig.poses), timeout_s=1.0, detection=None)
