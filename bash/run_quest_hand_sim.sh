#!/usr/bin/env bash
# Quest v1 and PICO use the same FLU/OpenXR wire format.
# Both use this workspace's calibration, DLS/Ruckig, Hand2 and ROS publisher.
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
exec bash "$repo_root/bash/run_pico_hand_sim.sh" "$@"
