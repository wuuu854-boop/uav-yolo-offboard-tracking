#!/usr/bin/env bash
# Check the environment and install Python dependencies.
# This script never runs sudo for you: it prints the apt command instead, so you
# can read it before it touches your machine.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

echo "== 1/5 checking OS and ROS =="
if [[ -r /etc/os-release ]]; then
  # shellcheck disable=SC1091
  source /etc/os-release
  echo "OS: ${PRETTY_NAME:-unknown}"
  if [[ "${VERSION_ID:-}" != "22.04" ]]; then
    echo "WARNING: this project is verified on Ubuntu 22.04 only (found ${VERSION_ID:-unknown})."
  fi
fi

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ERROR: ROS 2 environment not sourced."
  echo "  source /opt/ros/humble/setup.bash"
  exit 1
fi
if [[ "$ROS_DISTRO" != "humble" ]]; then
  echo "WARNING: expected ROS 2 Humble, found '$ROS_DISTRO'."
fi

echo "== 2/5 checking python =="
python3 - <<'PY'
import sys
if sys.version_info[:2] != (3, 10):
    raise SystemExit(f"ERROR: ROS 2 Humble requires Python 3.10, found {sys.version.split()[0]}")
print("python:", sys.version.split()[0])
PY

echo "== 3/5 checking apt packages (install manually if missing) =="
MISSING=()
for pkg in ros-humble-cv-bridge ros-humble-ros-gz-bridge python3-venv; do
  if ! dpkg -s "$pkg" >/dev/null 2>&1; then MISSING+=("$pkg"); fi
done
if ((${#MISSING[@]})); then
  echo "Missing packages: ${MISSING[*]}"
  echo "  sudo apt install -y ${MISSING[*]}"
fi

echo "== 4/5 creating venv =="
# --system-site-packages is required so that `import rclpy` keeps working.
if [[ ! -d .venv ]]; then
  python3 -m venv --system-site-packages .venv
fi
# shellcheck disable=SC1091
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements-dev.txt

echo "== 5/5 checking PX4 =="
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
if [[ -d "$PX4_DIR" ]]; then
  echo "PX4 found at $PX4_DIR"
  git -C "$PX4_DIR" describe --tags --always 2>/dev/null || true
else
  echo "PX4 not found at $PX4_DIR"
  echo "  git clone https://github.com/PX4/PX4-Autopilot.git --recursive \"$PX4_DIR\""
  echo "  cd \"$PX4_DIR\" && git checkout v1.15.0 && make px4_sitl   # first build takes a while"
fi

cat <<'EOF'

Next steps:
  1. cp .env.example .env   and set PX4_DIR
  2. cp sim/worlds/target_tracking.sdf "$PX4_DIR/Tools/simulation/gz/worlds/"
  3. source .venv/bin/activate
  4. colcon build --packages-select uav_interfaces && source install/setup.bash
  5. make sim      (terminal A)
     make detect   (terminal B)
EOF
