# Setup log

Fill this in as you go. It is the difference between "it worked on my machine"
and a repository someone can reproduce.

## Environment actually used

| Item | Value | Command to verify |
|---|---|---|
| OS | Ubuntu 22.04.x | `lsb_release -a` |
| Kernel | | `uname -r` |
| CPU | | `lscpu \| grep "Model name"` |
| RAM | | `free -h` |
| GPU | none / model | `lspci \| grep -i vga` |
| ROS 2 | Humble | `echo $ROS_DISTRO` |
| Python | 3.10.x | `python3 --version` |
| PX4 | v1.15.x | `git -C $PX4_DIR describe --tags` |
| Gazebo | gz-sim 8 | `gz sim --versions` |
| ONNX Runtime | 1.17+ | `python3 -c "import onnxruntime; print(onnxruntime.__version__)"` |
| OpenCV | 4.9+ | `python3 -c "import cv2; print(cv2.__version__)"` |

## Chronological log

Append one entry per session. Keep the failing command and the fix — future you
will hit the same wall.

### YYYY-MM-DD — session title

```
$ command that failed
error text
```

Cause:
Fix:

## Known environment traps

| Symptom | Cause | Fix |
|---|---|---|
| `ModuleNotFoundError: rclpy` inside the venv | venv created without `--system-site-packages` | recreate: `python3 -m venv --system-site-packages .venv` |
| `gz: command not found` | Gazebo Harmonic not installed / wrong PX4 target | install `gz-harmonic`, use `make px4_sitl gz_x500` |
| Camera topic empty | wrong model name in `sim.launch.py` | `gz topic -l \| grep image` |
| `colcon build` cannot find `uav_interfaces` | running outside the repo root | run from the repository root, `src/` must contain the package |
| Latency in the thousands of ms | `use_sim_time` not set | pass `--ros-args -p use_sim_time:=true` |
