#!/usr/bin/env bash
set -euo pipefail

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# ros_fold is 3 levels up from script location (src/ros_driver/script -> ros_fold)
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

FOUND_SDK_PREFIX=""
FOUND_SDK_LIB=""
CANDIDATE_PREFIXES=()

append_prefix() {
    local value="$1"
    [[ -z "${value}" ]] && return
    for existing in "${CANDIDATE_PREFIXES[@]}"; do
        if [[ "${existing}" == "${value}" ]]; then
            return
        fi
    done
    CANDIDATE_PREFIXES+=("${value}")
}

check_sdk_prefix() {
    local prefix="$1"
    [[ -z "${prefix}" ]] && return 1
    local header="${prefix}/include/odin_lidar_api.h"
    if [[ ! -f "${header}" ]]; then
        return 1
    fi
    local lib=""
    for libdir in lib lib64 lib/x86_64-linux-gnu build/sdk; do
        if [[ -f "${prefix}/${libdir}/libodin_sdk.a" ]]; then
            lib="${prefix}/${libdir}/libodin_sdk.a"
            break
        fi
    done
    if [[ -z "${lib}" ]]; then
        return 1
    fi
    FOUND_SDK_PREFIX="${prefix}"
    FOUND_SDK_LIB="${lib}"
    return 0
}

build_sdk_module() {
    local sdk_dir="$1"
    local install_prefix="$2"
    
    echo "========================================"
    echo "Building sdk_api module..."
    echo "Install prefix: ${install_prefix}"
    echo "========================================"
    
    local sdk_build_dir="${sdk_dir}/build"
    
    cmake -S "${sdk_dir}" -B "${sdk_build_dir}" \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="${install_prefix}"
    
    cmake --build "${sdk_build_dir}" -j$(nproc)
    cmake --install "${sdk_build_dir}"
    
    echo "SDK installed to: ${install_prefix}"
}

ensure_sdk() {
    # Check if sdk_api submodule exists
    SDK_API_MODULE="${SCRIPT_DIR}/../module/sdk_api"
    
    # Set default SDK install prefix if not specified
    if [[ -z "${ODIN_SDK_PREFIX:-}" ]]; then
        export ODIN_SDK_PREFIX="${WORKSPACE_DIR}/devel"
    fi
    
    if [[ -f "${SDK_API_MODULE}/CMakeLists.txt" ]] && [[ -f "${SDK_API_MODULE}/sdk/CMakeLists.txt" ]]; then
        # Build and install SDK module first
        build_sdk_module "${SDK_API_MODULE}" "${ODIN_SDK_PREFIX}"
        return
    fi
    
    # SDK submodule not found, check for pre-installed SDK
    CANDIDATE_PREFIXES=()
    append_prefix "${ODIN_SDK_PREFIX:-}"
    append_prefix "/usr/local"
    append_prefix "/usr"

    for prefix in "${CANDIDATE_PREFIXES[@]}"; do
        if check_sdk_prefix "${prefix}"; then
            export ODIN_SDK_PREFIX="${FOUND_SDK_PREFIX}"
            echo "Using pre-installed ODIN SDK from ${FOUND_SDK_PREFIX} (lib: ${FOUND_SDK_LIB})"
            return
        fi
    done

    cat <<EOF >&2
ODIN SDK not found. Either:
  1. Initialize the sdk_api submodule: git submodule update --init module/sdk_api
  2. Install SDK system-wide (e.g. sudo make install)
  3. Set ODIN_SDK_PREFIX=/path/to/install
EOF
    exit 1
}

if [[ ! -d "${WORKSPACE_DIR}/src" ]]; then
    echo "ROS workspace not found under ${WORKSPACE_DIR}/src." >&2
    exit 1
fi

if [[ -n "${ROS1_SETUP:-}" ]]; then
    # shellcheck disable=SC1090
    source "${ROS1_SETUP}"
fi

ensure_sdk

if ! command -v catkin_make >/dev/null 2>&1; then
    echo "catkin_make command not found. Source your ROS 1 environment or set ROS1_SETUP=/path/to/setup.bash." >&2
    exit 1
fi

export ROS_VERSION=1

# Build with all outputs inside WORKSPACE_DIR
cd "${WORKSPACE_DIR}"
catkin_make \
    --directory "${WORKSPACE_DIR}" \
    --build "${WORKSPACE_DIR}/build" \
    "$@"

echo "Build complete. Artifacts in ${WORKSPACE_DIR}/{build,devel}"
