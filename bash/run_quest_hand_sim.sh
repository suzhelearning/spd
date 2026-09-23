#!/usr/bin/env bash
# Quest v1 uses the same FLU/OpenXR wire format as the upstream bare-hand input.
# Reuse calibration, DLS/Ruckig, Hand2 and ROS publishing; simulation only.
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
teleop_root="${TIANJI_TELEOP_ROOT:-$(dirname "$repo_root")/tianji_teleop-ros2}"
launcher="$teleop_root/bash/run_pico_hand_sim.sh"
if [[ ! -f "$launcher" ]]; then
  echo "Missing teleoperation workspace: $launcher" >&2
  echo "Set TIANJI_TELEOP_ROOT to the built tianji_teleop-ros2 workspace." >&2
  exit 1
fi
exec bash "$launcher" "$@"
