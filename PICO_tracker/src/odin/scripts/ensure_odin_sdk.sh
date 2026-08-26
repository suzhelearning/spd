#!/usr/bin/env bash
# Build the Odin vendor SDK static lib (libodin_sdk.a) if it is not already built.
#
# odin_ros_driver_rev1's CMakeLists expects module/sdk_api/build/sdk/libodin_sdk.a
# to exist BEFORE colcon builds the package (it does not build the SDK itself).
# module/sdk_api is a symlink to ../../odin-sdk2, so we build odin-sdk2 in place.
#
# Idempotent: if the lib already exists, this is a no-op. Builds natively, so it
# works on both x86_64 and aarch64 (RK3588). Run from the workspace root.
set -euo pipefail

SDK_DIR="src/odin/odin-sdk2"
SDK_LIB="${SDK_DIR}/build/sdk/libodin_sdk.a"

if [[ ! -d "${SDK_DIR}" ]]; then
  echo "[ensure_odin_sdk] ${SDK_DIR} not found — odin-lite source not installed, skipping."
  exit 0
fi

newer_source=""
if [[ -f "${SDK_LIB}" ]]; then
  newer_source="$(find "${SDK_DIR}" -path "${SDK_DIR}/build" -prune -o \
    -type f \( -name '*.cpp' -o -name '*.c' -o -name '*.hpp' -o -name '*.h' \
    -o -name 'CMakeLists.txt' \) -newer "${SDK_LIB}" -print -quit)"
fi

if [[ -f "${SDK_LIB}" && -z "${newer_source}" ]]; then
  echo "[ensure_odin_sdk] ${SDK_LIB} already built — skipping."
  exit 0
fi

echo "[ensure_odin_sdk] building Odin SDK (missing or sources changed)..."
cmake -S "${SDK_DIR}" -B "${SDK_DIR}/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "${SDK_DIR}/build" -j"$(nproc)"
echo "[ensure_odin_sdk] done: ${SDK_LIB}"
