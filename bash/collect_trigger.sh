#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
for argument in "$@"; do
  case "$argument" in
    --help|-h)
      echo "Usage: collect_trigger.sh [--command start|save|discard|status] [--timeout SECONDS]"
      echo "Controls the existing SPD collector only (Fast DDS, localhost, ROS domain 120)."
      echo "Interactive: r=start, s=save-success, d=discard, q/Ctrl+C=exit client only."
      echo "--command works without a TTY; --timeout must be positive finite seconds (default 30)."
      echo "Acceptance is not completion. Unknown outcomes are not retried automatically."
      exit 0
      ;;
  esac
done

if [[ "${CONDA_PREFIX:-}" != "$repo_root/.pixi/envs/ros-jazzy" ]]; then
  command -v pixi >/dev/null || { echo "pixi is required" >&2; exit 1; }
  exec pixi run --manifest-path "$repo_root/pixi.toml" -e ros-jazzy \
    bash "$repo_root/bash/collect_trigger.sh" "$@"
fi
[[ -f "$repo_root/.ros/install/setup.sh" ]] || {
  echo "Missing ROS interfaces; run pixi run ros-build-interfaces" >&2; exit 1;
}
cd "$repo_root"
# Generated colcon setup scripts do not support nounset.
set +u
source "$repo_root/.ros/install/setup.sh"
set -u
export ROS_DOMAIN_ID=120 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_STATIC_PEERS=""
exec python -m spd_vr.data_collector.trigger "$@"
