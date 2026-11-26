#!/usr/bin/env python3
"""Receding-horizon map planner: depth frames -> grid -> frontier/goal path.

The node accumulates observed-free space and publishes `/map/path` (flat x,y) plus
`/map/ready`. Until the final goal is observed, it publishes a known-free path to
an observed frontier; the offboard follows it, moves, and the map grows.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

import rclpy  # noqa: E402
from px4_msgs.msg import VehicleOdometry  # noqa: E402
from rclpy.executors import ExternalShutdownException  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402
from std_msgs.msg import Bool, Float32MultiArray, String  # noqa: E402

from uav_tracking.control.frames import (  # noqa: E402
    heading_ned_to_yaw_enu,
    ned_position_to_enu,
    quaternion_to_heading_ned,
)
from uav_tracking.perception.depth import depth_from_bytes  # noqa: E402
from uav_tracking.perception.map_gate import OccupancyGrid, project_depth  # noqa: E402
from uav_tracking.perception.map_quality import corridor_coverage, coverage_passes  # noqa: E402
from uav_tracking.planning.frontier import choose_frontier  # noqa: E402
from uav_tracking.planning.grid_planner import plan_grid_path  # noqa: E402


class MapPlanner(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("map_planner")
        self.args = args
        self.east = 0.0
        self.north = 0.0
        self.altitude = 0.0
        self.yaw = 0.0
        self.have_odometry = False
        self.grid = OccupancyGrid(bounds=args.bounds, resolution_m=args.resolution)
        self.points = 0
        self._last_plan_ns = 0
        self._last_ready = False
        self._last_status = ""
        self._paths = 0

        self.ready_pub = self.create_publisher(Bool, "/map/ready", 10)
        self.path_pub = self.create_publisher(Float32MultiArray, "/map/path", 10)
        self.status_pub = self.create_publisher(String, "/map/status", 10)
        self.create_subscription(
            VehicleOdometry, "/fmu/out/vehicle_odometry", self.on_odometry, qos_profile_sensor_data
        )
        self.create_subscription(Image, args.frame_topic, self.on_depth, qos_profile_sensor_data)
        self.get_logger().info(
            f"map planner: goal ({args.goal[0]:.1f}, {args.goal[1]:.1f}), "
            f"min altitude {args.min_altitude:.1f} m"
        )

    def on_odometry(self, msg: VehicleOdometry) -> None:
        self.east, self.north, self.altitude = ned_position_to_enu(msg.position)
        self.yaw = heading_ned_to_yaw_enu(quaternion_to_heading_ned(msg.q))
        self.have_odometry = True

    def _publish(self, ready: bool, status: str) -> None:
        self.ready_pub.publish(Bool(data=ready))
        self.status_pub.publish(String(data=status))
        if status != self._last_status or ready != self._last_ready:
            self.get_logger().info(("map ready: " if ready else "map not ready: ") + status)
            self._last_status = status
            self._last_ready = ready

    def _publish_path(self, path: list[tuple[float, float]], status: str) -> None:
        flat: list[float] = []
        for x, y in path:
            flat.extend((float(x), float(y)))
        self.path_pub.publish(Float32MultiArray(data=flat))
        self._paths += 1
        self._publish(True, status)

    def on_depth(self, msg: Image) -> None:
        if not self.have_odometry or self.altitude < self.args.min_altitude:
            return
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_plan_ns < int(1e9 / max(self.args.rate, 0.1)):
            return
        self._last_plan_ns = now_ns
        try:
            depth = depth_from_bytes(msg.data, msg.width, msg.height)
        except ValueError:
            return
        points = project_depth(
            depth,
            (self.east, self.north, self.altitude),
            self.yaw,
            sample_px=self.args.sample_px,
            max_depth_m=self.args.max_depth,
            min_height_m=self.args.min_height,
            roi_fraction=self.args.roi_fraction,
        )
        self.grid.add_points_with_rays(points, (self.east, self.north))
        self.points += len(points)
        if self.points < self.args.min_points:
            self._publish(False, f"only {self.points} points")
            return

        goal = tuple(self.args.goal)
        coverage = corridor_coverage(self.grid, (self.east, self.north), goal, half_width_m=0.3)
        if coverage_passes(coverage, min_coverage=self.args.goal_coverage):
            try:
                path = plan_grid_path(
                    self.grid,
                    (self.east, self.north),
                    goal,
                    inflation_cells=self.args.inflation_cells,
                    close_radius=self.args.close_radius,
                    require_known_free=True,
                )
            except ValueError:
                path = []
            if path:
                self._publish_path(
                    path,
                    f"GOAL path {len(path)} pts, coverage {coverage['coverage']:.2f}, "
                    f"{self.points} points",
                )
                return

        try:
            frontier = choose_frontier(
                self.grid,
                (self.east, self.north),
                goal,
                max_candidates=self.args.max_frontiers,
                inflation_cells=self.args.inflation_cells,
                close_radius=self.args.close_radius,
            )
        except ValueError as exc:
            self._publish(False, f"no frontier: {exc}; coverage {coverage['coverage']:.2f}")
            return
        self._publish_path(
            frontier["path"],
            f"FRONTIER {frontier['point'][0]:.2f},{frontier['point'][1]:.2f} "
            f"{frontier['path_length_m']:.2f} m, coverage {coverage['coverage']:.2f}",
        )


def _xy(text: str) -> tuple[float, float]:
    values = tuple(float(v) for v in text.split(","))
    if len(values) != 2:
        raise argparse.ArgumentTypeError("point must be x,y")
    return values


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--goal", type=_xy, default=(5.7, 0.0))
    parser.add_argument("--frame-topic", default="/camera/depth/image_raw")
    parser.add_argument("--bounds", type=float, nargs=4, default=(-10.0, 10.0, -10.0, 10.0))
    parser.add_argument("--resolution", type=float, default=0.25)
    parser.add_argument("--inflation-cells", type=int, default=1)
    parser.add_argument("--close-radius", type=int, default=1)
    parser.add_argument("--min-cell-points", type=int, default=2)
    parser.add_argument("--min-component", type=int, default=3)
    parser.add_argument("--min-points", type=int, default=300)
    parser.add_argument("--sample-px", type=int, default=12)
    parser.add_argument("--roi-fraction", type=float, default=0.6)
    parser.add_argument("--max-depth", type=float, default=8.0)
    parser.add_argument("--min-height", type=float, default=2.5)
    parser.add_argument("--min-altitude", type=float, default=4.0)
    parser.add_argument("--rate", type=float, default=2.0)
    parser.add_argument("--goal-coverage", type=float, default=0.95)
    parser.add_argument("--max-frontiers", type=int, default=50)
    args = parser.parse_args()
    args.bounds = tuple(float(v) for v in args.bounds)
    if not (args.bounds[0] < args.bounds[1] and args.bounds[2] < args.bounds[3]):
        parser.error("bounds must be xmin,xmax,ymin,ymax")

    rclpy.init()
    node = MapPlanner(args)
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
