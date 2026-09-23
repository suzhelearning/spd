#!/usr/bin/env bash
# Build local simulation-only teleop workers without mixing their native ABIs.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
clean=(env -u PYTHONPATH -u PYTHONHOME -u LD_LIBRARY_PATH -u LD_PRELOAD
  -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH
  -u CMAKE_ARGS -u CPPFLAGS -u CFLAGS -u CXXFLAGS -u LDFLAGS)
arm=("${clean[@]}" pixi run --locked --manifest-path "$root/pixi.toml" -e teleop-native)
prefix="$root/.teleop/install"
"${arm[@]}" cmake -S "$root/src/teleop_native" -B "$root/.teleop/build/arm" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$prefix"
"${arm[@]}" cmake --build "$root/.teleop/build/arm" --parallel 2
"${arm[@]}" cmake --install "$root/.teleop/build/arm"
hand="$root/tools/wuji_hand_native/.pixi/envs/default"
"${clean[@]}" pixi install --locked --manifest-path "$root/tools/wuji_hand_native/pixi.toml"
"${clean[@]}" CONDA_PREFIX="$hand" "$root/.pixi/envs/teleop-native/bin/cmake" \
  -S "$root/src/pico2_hands/native/hand/optimizer" -B "$root/.teleop/build/hand" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=/usr/bin/g++ \
  -DCMAKE_MAKE_PROGRAM="$root/.pixi/envs/teleop-native/bin/ninja" \
  -DCMAKE_PREFIX_PATH="$hand" -DCMAKE_INSTALL_PREFIX="$prefix" \
  -DCMAKE_BUILD_RPATH="$hand/lib" -DCMAKE_INSTALL_RPATH="$hand/lib"
"${clean[@]}" "$root/.pixi/envs/teleop-native/bin/cmake" --build "$root/.teleop/build/hand" --parallel 1
"${clean[@]}" "$root/.pixi/envs/teleop-native/bin/cmake" --install "$root/.teleop/build/hand"
# The simulator owns the single ROS interface definition and generated bindings.
exec "${clean[@]}" pixi run --locked --manifest-path "$root/pixi.toml" -e ros-jazzy \
  bash "$root/bash/build_native.sh"
