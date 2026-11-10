#!/usr/bin/env python3
"""Move a Gazebo model along a straight line, so the tracker has something moving.

The world ships three static targets; the tracker gained a motion model, and a
model is only worth trusting after it has seen a moving box. This script drives
one model with ``gz service set_pose`` at a fixed speed and ping-pongs between
two end points.

    python3 scripts/move_target.py --model target_0 --start 0,0 --end 0,3 --speed 0.4
"""

from __future__ import annotations

import argparse
import math
import subprocess
import time


def _set_pose(world: str, model: str, x: float, y: float, z: float) -> bool:
    request = f'name: "{model}", position: {{x: {x}, y: {y}, z: {z}}}'
    result = subprocess.run(
        [
            "gz",
            "service",
            "-s",
            f"/world/{world}/set_pose",
            "--reqtype",
            "gz.msgs.Pose",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "1000",
            "--req",
            request,
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    return "true" in result.stdout.lower() or result.returncode == 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", default="target_tracking")
    parser.add_argument("--model", default="target_0")
    parser.add_argument("--start", default="0,0", help="world ENU start x,y")
    parser.add_argument("--end", default="0,3", help="world ENU end x,y")
    parser.add_argument("--speed", type=float, default=0.4, help="metres per second")
    parser.add_argument("--height", type=float, default=0.05, help="model z")
    parser.add_argument("--duration", type=float, default=120.0, help="seconds to keep moving")
    parser.add_argument("--rate", type=float, default=10.0, help="updates per second")
    args = parser.parse_args()

    start = tuple(float(value) for value in args.start.split(","))
    end = tuple(float(value) for value in args.end.split(","))
    length = math.hypot(end[0] - start[0], end[1] - start[1])
    if length <= 0.0 or args.speed <= 0.0:
        print("start, end and speed must describe a non-zero path")
        return 2

    period = args.rate
    step = args.speed / period
    travelled = 0.0
    direction = 1.0
    started = time.monotonic()
    sent = 0

    while time.monotonic() - started < args.duration:
        fraction = travelled / length
        x = start[0] + (end[0] - start[0]) * fraction
        y = start[1] + (end[1] - start[1]) * fraction
        if _set_pose(args.world, args.model, x, y, args.height):
            sent += 1
        travelled += step * direction
        if travelled >= length:
            travelled = length
            direction = -1.0
        elif travelled <= 0.0:
            travelled = 0.0
            direction = 1.0
        time.sleep(1.0 / period)

    print(f"moved {args.model}: {sent} poses, {args.speed} m/s, {length:.1f} m path")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
