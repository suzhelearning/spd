#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
output="${SPD_EPISODE_OUTPUT:-}"
headless=0
scene=""
task=""
seed=""
collection_config="$repo_root/config/collect_sim.yaml"
max_frames=""

table_distance=""
usage() {
  echo "Usage: start_spd_sim.sh [--headless] [--collection-config PATH] [--output PATH] [--max-frames N] [--scene NAME] [--task SCENE/TASK] [--seed N] [--table-distance METRES]"
  echo "Runs only the MuJoCo subscriber in this terminal; Ctrl+C exits."
  echo "Table scenes ask for the robot-base-to-near-edge distance unless --table-distance is given."
  echo "Omit --task for a random task at startup and after each completed episode."
  echo "--scene restricts random selection; --scene hardware_free disables task scenes; --seed reproduces the sequence."
  echo "Upstream: r calibrates, s starts following. SPD: r starts/checkpoints; s pauses/resumes."
  echo "Paused d rewinds and automatically resumes; r then r confirms successful saving."
  echo "Entries blend for 1 second; r/s/d ignored during the transition. No discard key."
  echo "Focus this terminal or viewer. Tap only: key autorepeat can confirm save. q/Ctrl+C exits."
}
while (($#)); do
  case "$1" in
    --headless) headless=1; shift ;;
    --output|--collection-config|--max-frames|--scene|--task|--seed|--table-distance)
      [[ $# -ge 2 && -n "$2" ]] || { echo "Missing value for $1" >&2; exit 2; }
      case "$1" in
        --output) output="$2" ;;
        --collection-config) collection_config="$2" ;;
        --max-frames) max_frames="$2" ;;
        --scene) scene="$2" ;;
        --task) task="$2" ;;
        --seed) seed="$2" ;;
        --table-distance) table_distance="$2" ;;
      esac
      shift 2 ;;
    --help|-h) usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done
command -v pixi >/dev/null || { echo "pixi is required" >&2; exit 1; }
[[ -f "$repo_root/.ros/install/setup.sh" ]] || {
  echo "Missing ROS interfaces; run pixi run ros-build-interfaces" >&2; exit 1;
}
viewer_args=(--collection-config "$collection_config")
if [[ -n "$seed" ]]; then viewer_args+=(--seed "$seed"); fi
if [[ -n "$output" ]]; then viewer_args+=(--output "$output"); fi
if [[ -n "$max_frames" ]]; then viewer_args+=(--max-frames "$max_frames"); fi
if ((headless)); then viewer_args+=(--headless); fi
if [[ -n "$scene" ]]; then viewer_args+=(--scene "$scene"); fi
if [[ -n "$task" ]]; then viewer_args+=(--task "$task"); fi
if [[ -n "$table_distance" ]]; then viewer_args+=(--table-distance "$table_distance"); fi
if [[ "${CONDA_PREFIX:-}" != "$repo_root/.pixi/envs/ros-jazzy" ]]; then
  exec pixi run --manifest-path "$repo_root/pixi.toml" -e ros-jazzy \
    bash "$repo_root/bash/start_spd_sim.sh" "${viewer_args[@]}"
fi
echo "Starting SPD subscriber in the foreground (Fast DDS, ROS domain 120)."
echo "Local R/S/D: r start/checkpoint; s pause/resume; paused d rewind+resume; paused r then r saves."
echo "Upstream stays running independently. One-second entry transitions are recorded and labelled."
cd "$repo_root"
# Generated colcon setup scripts do not support nounset.
set +u
source "$repo_root/.ros/install/setup.sh"
set -u
export ROS_DOMAIN_ID=120 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_STATIC_PEERS=""
exec python -m simulation.ros_viewer "${viewer_args[@]}"
