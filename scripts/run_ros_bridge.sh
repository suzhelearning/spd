#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
bridge="$repo_root/.pixi/tools/zenoh-bridge-ros2dds/1.10.0/zenoh-bridge-ros2dds"
usage() {
  echo "Usage: run_ros_bridge.sh publisher|spd"
  echo "  publisher: domain 120, connects to SPD_ZENOH_CONNECT (default tcp/127.0.0.1:7447)"
  echo "  spd:       domain 121, listens on SPD_ZENOH_LISTEN (default tcp/127.0.0.1:7447)"
  echo "Runs only the bridge in the foreground; Ctrl-C stops only this bridge."
}
if [[ $# != 1 ]]; then usage >&2; exit 2; fi
case "$1" in
  publisher) domain=120; endpoint="${SPD_ZENOH_CONNECT:-tcp/127.0.0.1:7447}"; direction="--connect" ;;
  spd) domain=121; endpoint="${SPD_ZENOH_LISTEN:-tcp/127.0.0.1:7447}"; direction="--listen" ;;
  --help|-h) usage; exit 0 ;;
  *) usage >&2; exit 2 ;;
esac
[[ "$endpoint" == tcp/* ]] || { echo "Bridge endpoint must use tcp/: $endpoint" >&2; exit 2; }
[[ -x "$bridge" ]] || {
  echo "Missing bridge: $bridge; run bash scripts/install_ros_bridge.sh" >&2
  exit 1
}
export ROS_DOMAIN_ID="$domain"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_STATIC_PEERS=""
export ROS_DISTRO=jazzy
# Explicit unicast participant ports work even when Linux loopback has no MULTICAST flag.
export CYCLONEDDS_URI="$repo_root/config/ros2dds-cyclone.xml"
echo "Starting ROS2DDS bridge: domain=$domain $direction=$endpoint"
exec "$bridge" -c "$repo_root/config/ros2dds.json5" --domain "$domain" \
  --ros-automatic-discovery-range LOCALHOST --ros-static-peers '' \
  "$direction" "$endpoint"
