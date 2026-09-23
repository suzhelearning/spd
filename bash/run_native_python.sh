#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
cd "$repo_root"
[[ -x .ros/install/lib/spd_native/spd_executor ]] || {
  echo "Missing native runtime; run pixi run spd-native-build" >&2; exit 1;
}
set +u
source .ros/install/setup.sh
set -u
exec python "$@"
