#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
# Collection runs inside the existing simulation owner, never a second writer.
# Use this OR spd-sim. To control an already running instance, use collect_trigger.sh.
exec bash "$repo_root/bash/start_spd_sim.sh" "$@"
