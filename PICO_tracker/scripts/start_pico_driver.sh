#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"

if [[ "${PIXI_PROJECT_ROOT:-}" != "$repo_root" ]]; then
  echo "请先执行: cd $repo_root && pixi shell" >&2
  exit 2
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-120}"
export EXO_REQUESTED_ROS_DOMAIN_ID="${EXO_REQUESTED_ROS_DOMAIN_ID:-$ROS_DOMAIN_ID}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export ROS2CLI_DISABLE_DAEMON="${ROS2CLI_DISABLE_DAEMON:-1}"

set +u
source "$repo_root/install/local_setup.bash"
set -u

python - <<'PY'
import sys
import rclpy

if sys.version_info[:2] != (3, 11):
    raise SystemExit(f"PICO requires Python 3.11, got {sys.version.split()[0]}")
if "/opt/ros/humble" in rclpy.__file__:
    raise SystemExit(f"ROS environment contaminated by system rclpy: {rclpy.__file__}")
PY

command -v adb >/dev/null || { echo "adb not found" >&2; exit 2; }
adb forward tcp:9999 tcp:9999

echo "PICO driver starting (domain=$ROS_DOMAIN_ID)"
exec ros2 launch pico_bridge start_pico_bridge.launch.py "$@"
