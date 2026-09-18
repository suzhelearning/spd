#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
session_name="spd-ros"
socket="$repo_root/.pixi/spd-ros.tmux.sock"
mode="detach"
output="${SPD_EPISODE_OUTPUT:-$repo_root/episodes}"
headless=0
max_enable_delta="0.15"

usage() {
  echo "Usage: start_spd_ros.sh [--attach] [--headless] [--output PATH] [--max-enable-delta-rad RAD]"
  echo "Starts only the SPD ROS2DDS bridge (domain 121) and MuJoCo subscriber viewer."
}
while (($#)); do
  case "$1" in
    --attach) mode="attach"; shift ;;
    --headless) headless=1; shift ;;
    --output|--max-enable-delta-rad)
      [[ $# -ge 2 && -n "$2" ]] || { echo "Missing value for $1" >&2; exit 2; }
      if [[ "$1" == --output ]]; then output="$2"; else max_enable_delta="$2"; fi
      shift 2 ;;
    --help|-h) usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done
command -v pixi >/dev/null || { echo "pixi is required" >&2; exit 1; }
tmux_bin="$(command -v tmux || true)"
if [[ -z "$tmux_bin" ]]; then
  tmux_bin="$repo_root/.pixi/envs/default/bin/tmux"
  export TERMINFO="$repo_root/.pixi/envs/default/share/terminfo"
fi
[[ -x "$tmux_bin" ]] || { echo "tmux is required; run pixi install" >&2; exit 1; }
[[ -x "$repo_root/.pixi/tools/zenoh-bridge-ros2dds/1.10.0/zenoh-bridge-ros2dds" ]] || {
  echo "Missing ROS2DDS bridge; run bash scripts/install_ros_bridge.sh" >&2; exit 1;
}
[[ -f "$repo_root/.ros/install/setup.sh" ]] || {
  echo "Missing ROS interfaces; run pixi run ros-build-interfaces" >&2; exit 1;
}
if "$tmux_bin" -S "$socket" has-session -t "=$session_name" 2>/dev/null; then
  echo "refusing duplicate session: $session_name" >&2
  exit 1
fi
mkdir -p "$output"
viewer_inner="source .ros/install/setup.sh && export ROS_DOMAIN_ID=121 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_STATIC_PEERS='' && exec python -m spd_vr.ros_viewer --output $(printf '%q' "$output") --max-enable-delta-rad $(printf '%q' "$max_enable_delta")"
if ((headless)); then viewer_inner+=" --headless"; fi
bridge="cd $(printf '%q' "$repo_root") && exec env SPD_ZENOH_LISTEN=$(printf '%q' "${SPD_ZENOH_LISTEN:-tcp/127.0.0.1:7447}") bash scripts/run_ros_bridge.sh spd"
viewer="cd $(printf '%q' "$repo_root") && exec pixi run -e ros-jazzy bash -c $(printf '%q' "$viewer_inner")"
"$tmux_bin" -S "$socket" new-session -d -s "$session_name" -n bridge "$bridge"
trap '"$tmux_bin" -S "$socket" kill-session -t "=$session_name" 2>/dev/null || true' ERR
"$tmux_bin" -S "$socket" set-option -t "=$session_name:" remain-on-exit on >/dev/null
"$tmux_bin" -S "$socket" new-window -t "=$session_name:" -n viewer "$viewer"
trap - ERR
echo "Launched SPD subscriber session: $session_name (ROS domain 121)"
echo "No publisher, PICO input, IK, or hardware controller was started."
echo "Control terminal / viewer: e=enable/disable, c=clear control; viewer r=start, s=save, d=discard"
printf 'Inspect/attach: %q -S %q attach-session -t %q\n' "$tmux_bin" "$socket" "=$session_name"
if [[ "$mode" == "attach" ]]; then
  exec "$tmux_bin" -S "$socket" attach-session -t "=$session_name"
fi
