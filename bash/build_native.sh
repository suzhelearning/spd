#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
cd "$repo_root"
: "${CONDA_PREFIX:?Run through pixi run spd-native-build}"
export CMAKE_PREFIX_PATH="$CONDA_PREFIX${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
exec colcon build --base-paths src/interfaces/tianji_spd_interfaces src/spd_native \
  --build-base .ros/build --install-base .ros/install --merge-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython_EXECUTABLE="$CONDA_PREFIX/bin/python"
