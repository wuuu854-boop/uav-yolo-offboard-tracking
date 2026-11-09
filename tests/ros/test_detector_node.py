"""Integration smoke test for the detector node.

Needs a sourced ROS 2 environment, so it is skipped everywhere else. It runs in
the `ROS build` workflow, which builds `uav_interfaces` first.

What it proves that the unit tests cannot: the node subscribes to an Image,
converts it, runs the backend and publishes a `TargetArray` that an ordinary ROS
subscriber receives.

rclpy is initialised once per module on purpose: shutting it down and calling
``init()`` again in the same process segfaults (exit 139) on Humble.
"""

from __future__ import annotations

import time

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")

from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402

from src.perception.detector_node import DetectorNode  # noqa: E402
from uav_interfaces.msg import TargetArray  # noqa: E402

WIDTH, HEIGHT = 1280, 960
CONFIG = {
    "backend": "baseline",
    "publish_rate_hz": 50,
    "baseline_hsv_lower": [35, 80, 80],
    "baseline_hsv_upper": [85, 255, 255],
    "baseline_min_area_px": 150,
}


def _image_with_square(centre_u: int, centre_v: int, side: int = 160) -> Image:
    frame = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
    if side > 0:
        top, left = centre_v - side // 2, centre_u - side // 2
        frame[top : top + side, left : left + side] = (0, 255, 0)  # BGR green

    message = Image()
    message.height, message.width = HEIGHT, WIDTH
    message.encoding = "bgr8"
    message.step = WIDTH * 3
    message.data = frame.tobytes()
    message.header.frame_id = "camera_link"
    return message


def _pump_until(executor, publisher, image, received, predicate, timeout_s: float = 8.0) -> bool:
    """Publish until a message satisfying ``predicate`` arrives, or time out.

    A predicate rather than "any message" because a late message from the
    previous test can still be queued; matching the content makes the test
    independent of that race.
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        publisher.publish(image)
        executor.spin_once(timeout_sec=0.05)
        if received and predicate(received[-1]):
            return True
    return False


@pytest.fixture(scope="module")
def rig():
    rclpy.init()
    detector = DetectorNode(CONFIG)
    helper = Node("detector_test_helper")

    received: list[TargetArray] = []
    helper.create_subscription(TargetArray, "/detections", received.append, 10)
    publisher = helper.create_publisher(Image, "/camera/image_raw", 10)

    executor = SingleThreadedExecutor()
    executor.add_node(detector)
    executor.add_node(helper)

    yield executor, publisher, received

    executor.shutdown()
    detector.destroy_node()
    helper.destroy_node()
    rclpy.try_shutdown()


def test_baseline_backend_publishes_a_detection(rig):
    executor, publisher, received = rig
    received.clear()

    arrived = _pump_until(
        executor,
        publisher,
        _image_with_square(700, 420),
        received,
        predicate=lambda message: len(message.targets) == 1,
    )
    assert arrived, "no TargetArray with a detection within the timeout"

    target = received[-1].targets[0]
    assert target.bbox_cx == pytest.approx(700, abs=8)
    assert target.bbox_cy == pytest.approx(420, abs=8)
    assert target.image_width == WIDTH
    assert target.image_height == HEIGHT


def test_empty_frame_publishes_an_empty_array(rig):
    executor, publisher, received = rig
    received.clear()

    arrived = _pump_until(
        executor,
        publisher,
        _image_with_square(640, 480, side=0),
        received,
        predicate=lambda message: len(message.targets) == 0,
    )
    assert arrived, "no empty TargetArray within the timeout"
