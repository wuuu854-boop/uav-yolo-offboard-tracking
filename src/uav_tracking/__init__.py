"""UAV target-tracking pipeline: perception, projection and mission control.

The modules under ``perception`` and ``control`` are deliberately free of ROS
imports where possible, so the geometry and the state machine can be tested on a
machine without a simulator.
"""
