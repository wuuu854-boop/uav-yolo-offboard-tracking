"""Integration smoke test for the depth node.

Needs a sourced ROS 2 environment, so it is skipped everywhere else. It proves
the wiring the unit tests cannot: an ``Image`` of encoding 32FC1 arrives, the
node computes the nearest depth in its ROI and an ordinary subscriber receives
``/avoidance/stop`` and ``/avoidance/nearest_m``.
"""

from __future__ import annotations

import math
import time

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")

from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402
from std_msgs.msg import Bool, Float32  # noqa: E402

from uav_tracking.perception.depth_node import DepthNode  # noqa: E402

WIDTH, HEIGHT = 64, 48
CONFIG = {
    "stop_distance_m": 2.0,
    "clear_distance_m": 2.6,
    "roi_width_fraction": 0.6,
    "roi_height_fraction": 0.5,
    "min_valid_m": 0.25,
    "max_valid_m": 12.0,
    "stale_timeout_s": 0.5,
    "fail_safe_on_stale": True,
    "publish_rate_hz": 20.0,
}


def _depth_image(foreground_m: float | None) -> Image:
    depth = np.full((HEIGHT, WIDTH), 10.0, dtype=np.float32)
    if foreground_m is not None:
        depth[HEIGHT // 3 : 2 * HEIGHT // 3, WIDTH // 3 : 2 * WIDTH // 3] = foreground_m

    message = Image()
    message.height, message.width = HEIGHT, WIDTH
    message.encoding = "32FC1"
    message.step = WIDTH * 4
    message.data = depth.tobytes()
    message.header.frame_id = "depth_cam_link"
    return message


def _pump_until(executor, publisher, image, received, predicate, timeout_s: float = 8.0) -> bool:
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
    node = DepthNode(CONFIG)
    helper = Node("depth_test_helper")

    stops: list[Bool] = []
    distances: list[Float32] = []
    helper.create_subscription(Bool, "/avoidance/stop", stops.append, 10)
    helper.create_subscription(Float32, "/avoidance/nearest_m", distances.append, 10)
    publisher = helper.create_publisher(Image, "/camera/depth/image_raw", 10)

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(helper)

    yield executor, publisher, stops, distances

    executor.shutdown()
    node.destroy_node()
    helper.destroy_node()
    rclpy.try_shutdown()


def test_a_close_surface_publishes_stop_and_its_distance(rig):
    executor, publisher, stops, distances = rig
    stops.clear()
    distances.clear()

    assert _pump_until(
        executor,
        publisher,
        _depth_image(1.0),
        stops,
        predicate=lambda message: bool(message.data),
    ), "no stop was published for a 1.0 m surface"

    # The stop and the distance are published in the same timer callback as two
    # messages; one spin can deliver the stop before the distance. Keep spinning
    # until the distance arrives instead of assuming their order.
    assert _pump_until(
        executor,
        publisher,
        _depth_image(1.0),
        distances,
        predicate=lambda message: math.isfinite(float(message.data)),
        timeout_s=3.0,
    ), "no nearest distance was published"

    assert distances[-1].data == pytest.approx(1.0, abs=0.05)


def test_a_clear_frame_releases_the_stop(rig):
    executor, publisher, stops, distances = rig
    stops.clear()

    # 10 m is beyond the clear threshold, so the gate must release.
    assert _pump_until(
        executor,
        publisher,
        _depth_image(None),
        stops,
        predicate=lambda message: not bool(message.data),
    ), "the gate did not release at 10 m"

    assert _pump_until(
        executor,
        publisher,
        _depth_image(None),
        distances,
        predicate=lambda message: math.isfinite(float(message.data)) and message.data > 5.0,
        timeout_s=3.0,
    ), "the released gate did not publish the far distance"
