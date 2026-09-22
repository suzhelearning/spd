#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
for argument in "$@"; do
  case "$argument" in
    --help|-h)
      echo "Usage: collect_trigger.sh [--command start|save|discard|checkpoint|pause|resume|revert|skip|status] [--timeout SECONDS]"
      echo "Controls the existing SPD collector only (Fast DDS, localhost, ROS domain 120)."
      echo "Interactive: r=checkpoint, s=pause/resume, d=revert (no checkpoint: d twice skips), g=start, f=save."
      echo "q/Ctrl+C exits this client only. Resume never authorizes motion; use local e with a fresh command first."
      echo "Focus this terminal; tap only. Held-key autorepeat can confirm skip; no long-press detection."
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
exec python -m data_collector.trigger "$@"
