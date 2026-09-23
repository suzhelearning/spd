#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
if [[ "${CONDA_PREFIX:-}" != "$root/.pixi/envs/ros-jazzy" ]]; then
  exec env -u PYTHONPATH -u PYTHONHOME -u LD_LIBRARY_PATH -u LD_PRELOAD \
    -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH \
    -u ROS_DISTRO -u ROS_VERSION \
    pixi run --locked --manifest-path "$root/pixi.toml" -e ros-jazzy \
    bash "$root/bash/start_spd_sim.sh" "$@"
fi
[[ -x "$root/.ros/install/lib/spd_native/spd_executor" ]] || {
  echo 'Missing native runtime; run pixi run spd-teleop-build (or spd-native-build for external input).' >&2
  exit 1
}
set +u
source "$root/.ros/install/setup.sh"
set -u
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
exec "$root/.ros/install/lib/spd_native/spd_executor" "$@"
