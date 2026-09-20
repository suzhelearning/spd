#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
session_name="spd-ros"
socket="$repo_root/.pixi/spd-ros.tmux.sock"
mode="detach"
output="${SPD_EPISODE_OUTPUT:-$repo_root/episodes}"
headless=0
max_enable_delta="0.15"
scene=""
task=""
seed="0"

usage() {
  echo "Usage: start_spd_ros.sh [--attach] [--headless] [--output PATH] [--scene NAME] [--task SCENE/TASK] [--seed N] [--max-enable-delta-rad RAD]"
  echo "Starts only the MuJoCo subscriber viewer (direct Fast DDS, domain 120)."
}
while (($#)); do
  case "$1" in
    --attach) mode="attach"; shift ;;
    --headless) headless=1; shift ;;
    --output|--max-enable-delta-rad|--scene|--task|--seed)
      [[ $# -ge 2 && -n "$2" ]] || { echo "Missing value for $1" >&2; exit 2; }
      case "$1" in
        --output) output="$2" ;;
        --max-enable-delta-rad) max_enable_delta="$2" ;;
        --scene) scene="$2" ;;
        --task) task="$2" ;;
        --seed) seed="$2" ;;
      esac
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
[[ -f "$repo_root/.ros/install/setup.sh" ]] || {
  echo "Missing ROS interfaces; run pixi run ros-build-interfaces" >&2; exit 1;
}
if "$tmux_bin" -S "$socket" has-session -t "=$session_name" 2>/dev/null; then
  echo "refusing duplicate session: $session_name" >&2
  exit 1
fi
mkdir -p "$output"
viewer_inner="source .ros/install/setup.sh && export ROS_DOMAIN_ID=120 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_STATIC_PEERS='' && exec python -m spd_vr.ros_viewer --output $(printf '%q' "$output") --max-enable-delta-rad $(printf '%q' "$max_enable_delta")"
if ((headless)); then viewer_inner+=" --headless"; fi
viewer_inner+=" --seed $(printf '%q' "$seed")"
if [[ -n "$scene" ]]; then viewer_inner+=" --scene $(printf '%q' "$scene")"; fi
if [[ -n "$task" ]]; then viewer_inner+=" --task $(printf '%q' "$task")"; fi
viewer="cd $(printf '%q' "$repo_root") && exec pixi run -e ros-jazzy bash -c $(printf '%q' "$viewer_inner")"
"$tmux_bin" -S "$socket" new-session -d -s "$session_name" -n viewer "$viewer"
trap '"$tmux_bin" -S "$socket" kill-session -t "=$session_name" 2>/dev/null || true' ERR
"$tmux_bin" -S "$socket" set-option -t "=$session_name:" remain-on-exit on >/dev/null
trap - ERR
echo "Launched SPD subscriber session: $session_name (direct Fast DDS, ROS domain 120)"
echo "No publisher, PICO input, IK, or hardware controller was started."
echo "Control terminal / viewer: e=enable/disable, c=clear control; viewer r=start, s=save, d=discard"
printf 'Inspect/attach: %q -S %q attach-session -t %q\n' "$tmux_bin" "$socket" "=$session_name"
if [[ "$mode" == "attach" ]]; then
  exec "$tmux_bin" -S "$socket" attach-session -t "=$session_name"
fi
