# Setup log

The environment this project was actually brought up in, and the order it
happened. The difference between "it worked on my machine" and a repository
someone can reproduce is this file.

## Environment actually used

| Item | Value | Command to verify |
|---|---|---|
| OS | Ubuntu 22.04.5 LTS on WSL2 | `lsb_release -ds` |
| Kernel | 6.18.33.2-microsoft-standard-WSL2 | `uname -r` |
| CPU | Intel i7-14650HX, 24 threads | `nproc` |
| RAM | 15 GiB visible to WSL2 (32 GiB host) | `free -h` |
| GPU | WSL2 d3d12 passthrough | `ls /usr/lib/wsl/lib \| head` |
| ROS 2 | Humble | `echo $ROS_DISTRO` |
| Python | 3.10.12 | `python3 --version` |
| PX4 | v1.15.0 | `git -C $PX4_DIR describe --tags` |
| Gazebo | gz-sim 8.15.0 | `gz sim --versions` |
| ONNX Runtime | not installed (baseline backend used) | `python3 -c "import onnxruntime"` |
| OpenCV | 4.10 (pip, pinned `<4.11`) | `python3 -c "import cv2; print(cv2.__version__)"` |

The distro lives on a second physical disk (`wsl --install --location D:\WSL\...`)
so the ~12 GB toolchain does not consume the system drive.

## 2026-11-09 — first SITL bring-up (WSL2, China network)

What worked, in order, and what each failure cost. Times are wall clock.

```
12:00  wsl --install -d Ubuntu-22.04 --location D:\WSL\Ubuntu-22.04 --no-launch
       # no elevation needed: WSL itself was already installed (2.7.12)
12:10  apt install git/cmake/build-essential ...                  OK
12:20  ROS 2 Humble repo: raw.githubusercontent.com unreachable
       → curl of ros.key fails with "Connection reset by peer"
       fix: deb [trusted=yes] http://packages.ros.org/ros2/ubuntu jammy main
       # the key is only a signature check; the mirror itself was reachable
13:00  ros-humble-ros-gz-bridge download stalls at 12 KB/s, twice
       fix: point apt at the host proxy for the two foreign hosts only
       Acquire::http::Proxy::packages.osrfoundation.org "http://<host>:7890"
       Acquire::http::Proxy::packages.ros.org "http://<host>:7890"
       measured: 17 KB/s direct vs 2.7 MB/s through the proxy
13:40  gz-harmonic installed                                      OK
14:00  git clone PX4 v1.15 + submodules                           OK (proxied)
14:20  Tools/setup/ubuntu.sh --no-nuttx: pip stalls on matplotlib
       fix: /etc/pip.conf → https://pypi.tuna.tsinghua.edu.cn/simple
14:40  make px4_sitl_default -j24                                  OK (667 MB build)
15:00  colcon build px4_msgs + uav_interfaces                      OK (2 min)
15:10  pip install -e . fails: "build backend is missing build_editable"
       fix: python3 -m pip install --upgrade pip setuptools wheel
       # Ubuntu 22.04 ships pip 22.0, older than PEP 660 support
15:20  pip then backtracks opencv for minutes (numpy<2 vs opencv 4.13)
       fix: pin opencv-python<4.11 in requirements.txt
15:30  Micro-XRCE-DDS-Agent v2.4.3 builds                         OK
15:40  PX4 SITL boots, uXRCE-DDS connects, /fmu/out/* appears      OK
15:50  model x500_mono_cam_down spawns, IMU publishes              OK
16:00  no camera topic at all
       fix: the world must declare the system plugins (skill 03)
16:10  /camera/image_raw ~30 Hz after ros_gz_bridge                OK
16:20  run_e2e reaches "waiting for landing row" but no row
       cause: PX4 preflight "barometer/compass missing", EKF never starts
       status: unresolved — see below
```

### Unresolved at the end of this session

PX4 refuses to arm. The gz side publishes `imu`, `air_pressure`, `navsat` and
`camera` under `/world/target_tracking/model/x500_mono_cam_down_0/...`, but PX4
reports the barometer and compass missing and `ekf2 missing data`. The suspected
cause is the instance suffix: Gazebo names the spawned model
`x500_mono_cam_down_0`, while PX4 builds its subscription names from
`PX4_SIM_MODEL` (`x500_mono_cam_down`). Next step is to check
`PX4_GZ_MODEL_NAME` and the magnetometer topic, then re-run `make e2e`.

### What this session cost

About four hours, of which roughly half was network, a quarter was two
independent installers fighting over the dpkg lock, and a quarter was the
camera. The camera fault was worth the time: it produced `skills/03` and
corrected a rule this repository had documented backwards.

## 2026-11-10 — safety constraints, and a 5.6 m lesson

The second session began as cleanup and ended with three changes to the control
loop.

```
09:00  docker CI job wired up -> pyproject.toml and requirements.txt had drifted
       (opencv-python vs opencv-python-headless) -> scripts/check_dependencies.py
10:30  three validation runs: 0.82 m, 5.57 m, 1.31 m
11:00  the 5.57 m run reacquired a single frame during the search, flew to it,
       and the trace called it "arrived"
11:20  confirm_frames: arrival, reacquisition and end-of-loss-hover now need
       three consecutive frames with a target
11:40  re-runs with the fence and the confirmation: 0.94 / 1.01 / 1.49 m
12:00  a loss + search + recovery exceeds a 120 s trial window -> default 180 s
```

What the 5.57 m run taught: the projection was fine; the mistake was believing
one frame. The trace shows `HOVER(target_lost) → SEARCH → GOTO` at a point 3.7 m
from the target, and nothing in the mission could tell that apart from a real
arrival. That is now a state-machine property with a unit test, not a tuning
parameter someone remembers.

## Adding a session

Append one `## YYYY-MM-DD — title` section. Keep the failing command, the error
text, and the fix. The error text is the part that makes it findable later.
