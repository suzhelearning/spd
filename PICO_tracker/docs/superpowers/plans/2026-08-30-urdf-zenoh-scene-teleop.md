# URDF-First Zenoh Scene Teleoperation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 以完整 `tianji_wuji2.urdf` 为唯一模型来源，实现真实 PICO 光学手追踪到 PC MuJoCo 完整天际双臂与双 Wuji2 手的 ROS-free、仿真-only 场景遥操作链。

**Architecture:** 一个确定性 Python 编译器从同一 URDF 生成 54-DoF `unified_plant.xml` 与 14-DoF `arm_ik.xml`，视觉保留厂家 STL，碰撞使用带质量门的 CoACD 多凸分解。运行时由 C++ `pico_zenoh_bridge`、C++ `tianji_zenoh_ik`、Python `spd_vr_viewer` 三个进程组成；桥接进程监听本机 Zenoh peer，IK 与 viewer 显式连接，viewer 是唯一 MuJoCo plant 所有者。

**Tech Stack:** Python 3.11、MuJoCo、NumPy、SciPy、PyYAML、trimesh、CoACD 1.0.14、eclipse-zenoh 1.10.0、C++17、Eigen、qpOASES、zenoh-pico 1.10.0、PXREARobotSDK、GoogleTest、pytest、Pixi、tmux。

**Spec:** `PICO_tracker/docs/superpowers/specs/2026-08-30-urdf-zenoh-scene-teleop-design.md`

## Global Constraints

- 唯一结构与物理模型来源是 `/home/current/syz/spd/assets/tianji_wuji2/tianji_wuji2.urdf`；现有 Tianji/Wuji MJCF 不得作为编译输入。
- 生成的完整 plant 必须恰好包含 54 个 revolute DoF；arm IK projection 必须恰好包含 14 个 revolute DoF，并暴露 `l_wrist_target`、`r_wrist_target`。
- 厂家 STL 视觉网格不得降采样；碰撞最多 16 个凸块/链接、最多 64 个顶点/凸块、固定 seed 0，失败不得回退为 primitive 或单 hull。
- CoACD 构建与测试统一使用 `OMP_NUM_THREADS=1`，确保同一 seed/输入/参数下的 piece ordering 与 output hash 稳定。
- 双手碰撞代理 p95 双向表面距离不得超过 1.5 mm；臂/基座不得超过 3 mm。
- tracking wire 固定为 1,540-byte little-endian v1；arm-target wire 固定为现有 272-byte v2；control wire 固定为本计划定义的 40-byte little-endian v1。
- Zenoh key 固定为 `spd/vr/v1/tracking`、`spd/vr/v1/arm_targets`、`spd/vr/v1/control`、`spd/vr/v1/status/{bridge,ik,viewer}`。
- `pico_zenoh_bridge` 以 peer 模式监听 `tcp/127.0.0.1:7447`；IK 与 viewer 以 peer 模式连接该 endpoint；不得启动 `zenohd`。
- 对齐窗口固定为连续 10 帧；相邻接受帧最大平移 0.02 m、最大旋转 0.15 rad；输入 stale 阈值固定为 50 ms。
- IK 频率 200 Hz、physics 480 Hz、PC render 60 Hz；render 不能成为 physics 时钟。
- 新 live path 不得启动或 import ROS/DDS；不得发送任何 Tianji/Wuji2 物理硬件命令。
- 首个场景只有完整机器人、灯光、相机和 ground plane；不加载任务物体、评分或录制流程。
- clean cutover：最终删除旧 hybrid `tianji_wuji2_spd.xml`、旧 live UDP arm wiring 与旧六窗口启动逻辑；不得保留兼容别名。
- 每个协议 callback 只做有界复制/解码与 latest-only 发布；不得在 callback 中运行 CoACD、IK、retarget 或 MuJoCo step。
- 每一任务只运行本任务列出的聚焦检查；完整端到端与性能检查统一在 Task 12、Task 13 执行。

---

## File Structure

### 新建的 Python 文件

- `PICO_tracker/src/spd_vr/spd_vr/model_compiler/urdf_model.py`：URDF dataclass、图验证、arm/hand 分类与确定性顺序。
- `PICO_tracker/src/spd_vr/spd_vr/model_compiler/collision.py`：CoACD 参数、缓存、凸块验证和双向 p95 质量门。
- `PICO_tracker/src/spd_vr/spd_vr/model_compiler/mjcf.py`：从已验证 URDF 图生成完整 plant 与 arm projection。
- `PICO_tracker/src/spd_vr/spd_vr/model_compiler/artifacts.py`：manifest、hash、atomic artifact commit 与 runtime 校验。
- `PICO_tracker/src/spd_vr/spd_vr/model_compiler/cli.py`：`spd-model` 编排入口。
- `PICO_tracker/src/spd_vr/spd_vr/tracking_protocol.py`：1,540-byte tracking v1 Python codec 与 stream gate。
- `PICO_tracker/src/spd_vr/spd_vr/control_protocol.py`：40-byte control v1 Python codec 与幂等序列 gate。
- `PICO_tracker/src/spd_vr/spd_vr/zenoh_transport.py`：Python Zenoh peer、latest-only mailbox、可靠 control publisher。
- `PICO_tracker/src/spd_vr/spd_vr/session_state.py`：START/PAUSE/RESUME/REALIGN/RESET/SHUTDOWN 状态机。
- `PICO_tracker/src/spd_vr/spd_vr/viewer_window.py`：GLFW MuJoCo window、相机交互、键盘映射、HUD overlay。
- `PICO_tracker/src/spd_vr/spd_vr/viewer.py`：唯一 plant owner、480 Hz physics、60 Hz render、Wuji retarget 与 status。
- `PICO_tracker/src/spd_vr/spd_vr/preflight.py`：ADB/SDK/model/hash/display/port/session 前置检查。
- `PICO_tracker/src/spd_vr/spd_vr/control_cli.py`：发送可靠 control command。
- `PICO_tracker/src/spd_vr/spd_vr/status_cli.py`：采集三进程 status 并输出诊断 JSON。
- `PICO_tracker/scripts/verify_protocol_interop.py`：调用 C++ fixture tool 完成双向跨语言协议验证。
- `PICO_tracker/scripts/test_spd_teleop_e2e.py`：真实 Zenoh session 的硬件-free 三进程验收。
- `PICO_tracker/scripts/accept_real_pico.py`：真实 PICO 速率、状态、命令与人工动作验收记录。

### 新建的 C++ 文件

- `TJ_arm_control/include/tianji_qp_ik/tracking_protocol.hpp`、`TJ_arm_control/src/tracking_protocol.cpp`：tracking v1 canonical C++ codec。
- `TJ_arm_control/include/tianji_qp_ik/control_protocol.hpp`、`TJ_arm_control/src/control_protocol.cpp`：control v1 canonical C++ codec。
- `TJ_arm_control/include/tianji_qp_ik/zenoh_session.hpp`、`TJ_arm_control/src/zenoh_session.cpp`：zenoh-pico RAII session/publisher/subscriber。
- `TJ_arm_control/include/tianji_qp_ik/pico_bridge_core.hpp`、`TJ_arm_control/src/pico_bridge_core.cpp`：SDK 事件、设备选择、有界队列、custom frame 配对与 epoch。
- `TJ_arm_control/include/tianji_qp_ik/pico_bridge_runtime.hpp`、`TJ_arm_control/src/pico_bridge_runtime.cpp`：复用 bridge worker、tracking/control/status Zenoh 路径。
- `TJ_arm_control/include/tianji_qp_ik/zenoh_ik_core.hpp`、`TJ_arm_control/src/zenoh_ik_core.cpp`：独立双侧对齐、200 Hz QP、HOLD 与 arm-target 生成。
- `TJ_arm_control/include/tianji_qp_ik/generated_model_manifest.hpp`、`TJ_arm_control/src/generated_model_manifest.cpp`：C++ manifest/hash/site/joint 校验。
- `TJ_arm_control/apps/pico_zenoh_bridge.cpp`：PXREA SDK lifecycle 与 tracking/status publication。
- `TJ_arm_control/apps/tianji_zenoh_ik.cpp`：tracking/control subscription、IK loop 与 target/status publication。
- `TJ_arm_control/apps/spd_protocol_fixture_tool.cpp`：跨语言 fixture stdin/stdout 工具。
- `TJ_arm_control/apps/spd_fake_pxrea_bridge.cpp`：仅验证使用，以 fake PXREA callback 驱动真实 bridge core 与 Zenoh peer。

### 修改/删除的现有文件

- `PICO_tracker/pixi.toml`、`PICO_tracker/pixi.lock`：固定新依赖与 operator tasks。
- `.gitmodules`、`third_party/zenoh-pico/`：固定 zenoh-pico 1.10.0 submodule。
- `TJ_arm_control/CMakeLists.txt`：native protocol、bridge、IK、fixture、测试目标。
- `TJ_arm_control/include/tianji_qp_ik/arm_target_protocol.hpp`、`TJ_arm_control/src/arm_target_protocol.cpp`：补齐 HOLD reason，不改变 272-byte layout。
- `TJ_arm_control/include/tianji_qp_ik/pico_wrist_alignment.hpp`、`TJ_arm_control/src/pico_wrist_alignment.cpp`：独立侧状态与保留 last-valid target 语义。
- `PICO_tracker/src/spd_vr/spd_vr/arm_target_protocol.py`：与 C++ HOLD enum/golden vector 同步。
- `wuji-retargeting/wuji_retargeting/{robot.py,retarget.py,opt/base.py}`：从完整 authoritative URDF 构造 manifest 指定的 20-DoF 单手 Pinocchio reduced model。
- `PICO_tracker/src/spd_vr/config/wuji2_pico_{left,right}.yaml`、`PICO_tracker/src/spd_vr/spd_vr/retarget_pair.py`：删除 standalone hand URDF/MJCF live 依赖，按 generated manifest 初始化双手 retarget。
- `PICO_tracker/src/spd_vr/spd_vr/model_builder.py`：缩减为新 compiler CLI 的兼容模块入口；不保留旧 hybrid 构建路径。
- `PICO_tracker/src/spd_vr/spd_vr/manifest.py`：改为新 `model_manifest.yaml` runtime validator。
- `PICO_tracker/src/spd_vr/spd_vr/simulator.py`：新 artifact 默认值、显式 reset、移除 UDP receiver。
- `PICO_tracker/src/spd_vr/spd_vr/runtime.py`：保留 hardware-free episode/mock 用途，删除 ROS live branch 与 UDP 参数。
- `PICO_tracker/src/spd_vr/setup.py`、`PICO_tracker/src/spd_vr/package.xml`：安装新 artifacts/CLI；新 CLI 无 ROS dependency。
- `PICO_tracker/scripts/start_spd_vr.sh`、`PICO_tracker/scripts/stop_spd_vr.sh`：严格三窗口 Zenoh live lifecycle。
- `PICO_tracker/README.zh-CN.md`、`PICO_tracker/README.md`：记录真实 operator 命令、键位、依赖、故障语义。
- 删除 `PICO_tracker/src/spd_vr/spd_vr/ros_input.py`、对应 ROS mailbox tests、旧 generated `tianji_wuji2_spd.xml`、`joint_manifest.yaml`、`sim_actuator_calibration.yaml`。

---

### Task 1: Pin dependencies and native build boundary

**Files:**
- Modify: `PICO_tracker/pixi.toml`
- Modify: `PICO_tracker/pixi.lock`
- Modify: `.gitmodules`
- Create: `third_party/zenoh-pico/` as git submodule at tag `1.10.0`
- Modify: `TJ_arm_control/CMakeLists.txt`
- Test: `PICO_tracker/src/spd_vr/test/test_dependency_contract.py`

**Interfaces:**
- Consumes: the workstation SDK root from `PXREA_SDK_ROOT`, default `/opt/apps/roboticsservice/SDK`.
- Produces: Pixi tasks `build-teleop-native` and `test-teleop-native`; CMake option `TIANJI_BUILD_ZENOH_TELEOP`; aggregate custom target `teleop_native`; imported target `zenohpico::lib` available to later native tasks.

- [ ] **Step 1: Write the dependency contract test**

```python
def test_teleop_dependencies_are_pinned():
    text = Path("pixi.toml").read_text(encoding="utf-8")
    assert 'eclipse-zenoh = "==1.10.0"' in text
    assert 'coacd = "==1.0.14"' in text
    assert 'trimesh = ">=4.8,<5"' in text
    assert "build-teleop-native" in text
    assert "TIANJI_BUILD_ZENOH_TELEOP=ON" in text
```

- [ ] **Step 2: Run the test and verify the missing dependency/task failure**

Run: `cd PICO_tracker && pixi run python -m pytest src/spd_vr/test/test_dependency_contract.py -q`

Expected: FAIL because the three pinned dependencies and native task are absent.

- [ ] **Step 3: Add pinned Python dependencies and operator build tasks**

Add `trimesh >=4.8,<5` to `[dependencies]`; add the following exact PyPI entries and tasks:

```toml
[pypi-dependencies]
wuji-retargeting = { path = "../wuji-retargeting", editable = true }
eclipse-zenoh = "==1.10.0"
coacd = "==1.0.14"

[tasks]
build-teleop-native = "cmake -S ../TJ_arm_control -B ../TJ_arm_control/build-teleop -G Ninja -DTIANJI_BUILD_ZENOH_TELEOP=ON -DPXREA_SDK_ROOT=${PXREA_SDK_ROOT:-/opt/apps/roboticsservice/SDK} -DCMAKE_PREFIX_PATH=$CONDA_PREFIX && cmake --build ../TJ_arm_control/build-teleop --target teleop_native"
test-teleop-native = "ctest --test-dir ../TJ_arm_control/build-teleop --output-on-failure -R 'tracking_protocol|control_protocol|arm_target_protocol|pico_bridge_core|zenoh_ik_core|generated_model_manifest|mujoco_robot|pico_wrist_alignment'"
```

Preserve existing Pixi tasks; do not remove ROS dependencies needed by unrelated legacy packages.

- [ ] **Step 4: Add the pinned zenoh-pico source dependency**

Run from repository root:

```bash
git submodule add https://github.com/eclipse-zenoh/zenoh-pico.git third_party/zenoh-pico
git -C third_party/zenoh-pico checkout 1.10.0
git add .gitmodules third_party/zenoh-pico
```

In `TJ_arm_control/CMakeLists.txt`, under `TIANJI_BUILD_ZENOH_TELEOP`, set `ZENOH_PICO_SOURCE_DIR` to `${CMAKE_CURRENT_SOURCE_DIR}/../third_party/zenoh-pico`, fail if its `CMakeLists.txt` is absent, then configure only the required transport surface before `add_subdirectory`:

```cmake
set(Z_FEATURE_PUBLICATION 1 CACHE STRING "" FORCE)
set(Z_FEATURE_SUBSCRIPTION 1 CACHE STRING "" FORCE)
set(Z_FEATURE_MULTI_THREAD 1 CACHE STRING "" FORCE)
set(Z_FEATURE_LINK_TCP 1 CACHE STRING "" FORCE)
set(Z_FEATURE_UNICAST_TRANSPORT 1 CACHE STRING "" FORCE)
set(Z_FEATURE_UNICAST_PEER 1 CACHE STRING "" FORCE)
set(Z_FEATURE_QUERY 0 CACHE STRING "" FORCE)
set(Z_FEATURE_QUERYABLE 0 CACHE STRING "" FORCE)
set(Z_FEATURE_LIVELINESS 0 CACHE STRING "" FORCE)
set(Z_FEATURE_SCOUTING 0 CACHE STRING "" FORCE)
set(Z_FEATURE_LINK_UDP_MULTICAST 0 CACHE STRING "" FORCE)
set(Z_FEATURE_LINK_UDP_UNICAST 0 CACHE STRING "" FORCE)
set(Z_FEATURE_MULTICAST_TRANSPORT 0 CACHE STRING "" FORCE)
set(Z_FEATURE_LINK_TLS 0 CACHE STRING "" FORCE)
set(Z_FEATURE_LINK_WS 0 CACHE STRING "" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_TOOLS OFF CACHE BOOL "" FORCE)
set(_tianji_build_testing "${BUILD_TESTING}")
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
add_subdirectory("${ZENOH_PICO_SOURCE_DIR}" "${CMAKE_BINARY_DIR}/zenoh-pico" EXCLUDE_FROM_ALL)
set(BUILD_TESTING "${_tianji_build_testing}" CACHE BOOL "" FORCE)
```

Create `add_custom_target(teleop_native)` inside the feature block. Each later protocol/bridge/IK/fake-bridge task adds its finished executable and fixture dependencies to this aggregate; at Task 1 it is intentionally empty but configure/build succeeds. Do not use `FetchContent` or a network-dependent configure step.

- [ ] **Step 5: Add strict SDK discovery**

Implement exact discovery behavior:

```cmake
set(PXREA_SDK_ROOT "$ENV{PXREA_SDK_ROOT}" CACHE PATH "PXREA RoboticsService SDK root")
if(NOT PXREA_SDK_ROOT)
  set(PXREA_SDK_ROOT "/opt/apps/roboticsservice/SDK")
endif()
find_path(PXREA_INCLUDE_DIR PXREARobotSDK.h PATHS "${PXREA_SDK_ROOT}/include" NO_DEFAULT_PATH)
find_library(PXREA_LIBRARY PXREARobotSDK PATHS "${PXREA_SDK_ROOT}/x64" NO_DEFAULT_PATH)
if(TIANJI_BUILD_ZENOH_TELEOP AND (NOT PXREA_INCLUDE_DIR OR NOT PXREA_LIBRARY))
  message(FATAL_ERROR "PXREARobotSDK header/library not found under ${PXREA_SDK_ROOT}")
endif()
message(STATUS "PXREA header: ${PXREA_INCLUDE_DIR}/PXREARobotSDK.h")
message(STATUS "PXREA library: ${PXREA_LIBRARY}")
```

- [ ] **Step 6: Run focused dependency and configure checks**

Run:

```bash
cd PICO_tracker
pixi install
pixi run python -m pytest src/spd_vr/test/test_dependency_contract.py -q
pixi run cmake -S ../TJ_arm_control -B ../TJ_arm_control/build-teleop -G Ninja -DTIANJI_BUILD_ZENOH_TELEOP=ON -DPXREA_SDK_ROOT=/opt/apps/roboticsservice/SDK -DCMAKE_PREFIX_PATH=$CONDA_PREFIX
```

Expected: dependency test PASS; configure prints exact discovered SDK header/library and creates `zenohpico::lib` without downloading source.

- [ ] **Step 7: Commit dependency boundary**

```bash
git add PICO_tracker/pixi.toml PICO_tracker/pixi.lock PICO_tracker/src/spd_vr/test/test_dependency_contract.py TJ_arm_control/CMakeLists.txt .gitmodules third_party/zenoh-pico
git commit -m "build: pin zenoh teleoperation dependencies"
```

---

### Task 2: Implement canonical binary protocols and golden interop

**Files:**
- Create: `TJ_arm_control/include/tianji_qp_ik/tracking_protocol.hpp`
- Create: `TJ_arm_control/src/tracking_protocol.cpp`
- Create: `TJ_arm_control/include/tianji_qp_ik/control_protocol.hpp`
- Create: `TJ_arm_control/src/control_protocol.cpp`
- Create: `TJ_arm_control/include/tianji_qp_ik/zenoh_keys.hpp`
- Modify: `TJ_arm_control/include/tianji_qp_ik/arm_target_protocol.hpp`
- Modify: `TJ_arm_control/src/arm_target_protocol.cpp`
- Create: `TJ_arm_control/apps/spd_protocol_fixture_tool.cpp`
- Create: `TJ_arm_control/tests/test_tracking_protocol.cpp`
- Create: `TJ_arm_control/tests/test_control_protocol.cpp`
- Modify: `TJ_arm_control/tests/test_arm_target_protocol.cpp`
- Create: `PICO_tracker/src/spd_vr/spd_vr/tracking_protocol.py`
- Create: `PICO_tracker/src/spd_vr/spd_vr/control_protocol.py`
- Create: `PICO_tracker/src/spd_vr/spd_vr/zenoh_keys.py`
- Modify: `PICO_tracker/src/spd_vr/spd_vr/arm_target_protocol.py`
- Create: `PICO_tracker/src/spd_vr/test/test_tracking_protocol.py`
- Create: `PICO_tracker/src/spd_vr/test/test_control_protocol.py`
- Modify: `PICO_tracker/src/spd_vr/test/test_arm_target_protocol.py`
- Create: `PICO_tracker/src/spd_vr/test/fixtures/tracking_v1.hex`
- Create: `PICO_tracker/src/spd_vr/test/fixtures/control_v1.hex`
- Modify: `PICO_tracker/src/spd_vr/test/fixtures/arm_target_v2.hex`
- Create: `PICO_tracker/scripts/verify_protocol_interop.py`
- Modify: `TJ_arm_control/CMakeLists.txt`

**Interfaces:**
- Produces C++: `encodeTrackingPacket`, `decodeTrackingPacket`, `TrackingStreamGate`, `encodeControlPacket`, `decodeControlPacket`, `ControlSequenceGate`, and header-only Zenoh key constants.
- Produces Python: `encode_tracking_packet`, `decode_tracking_packet`, `TrackingStreamDecoder`, `encode_control_packet`, `decode_control_packet`, `ControlSequenceGate`, and matching key constants.
- Produces wire constants: `TRACKING_PACKET_SIZE = 1540`, `CONTROL_PACKET_SIZE = 40`, `PACKET_SIZE = 272` for arm targets.

- [ ] **Step 1: Write failing C++ size, CRC, semantic and stream tests**

Define tests that construct identity quaternions for all active hand joints and verify:

```cpp
static_assert(tianji_qp_ik::kTrackingPacketSize == 1540U);
EXPECT_EQ(encoded.size(), 1540U);
EXPECT_EQ(encoded[0], static_cast<std::uint8_t>('S'));
EXPECT_EQ(decoded.frame->left_hand[1][0], 0.125F);
EXPECT_EQ(gate.evaluate(decoded.frame.value()).reason,
          TrackingStreamRejectReason::kNone);
```
Require the C++ and Python key constants to equal `spd/vr/v1/tracking`, `spd/vr/v1/arm_targets`, `spd/vr/v1/control`, `spd/vr/v1/status/bridge`, `spd/vr/v1/status/ik`, and `spd/vr/v1/status/viewer` exactly.

Cover wrong magic/version/declared size/CRC, NaN/Inf, non-positive scale/epoch, quaternion norm error above `1e-3`, timestamp rollback, duplicate sequence, epoch transition, non-zero reserved bytes, invalid control enum and duplicate control sequence.

- [ ] **Step 2: Run native protocol tests and capture missing-target failure**

Run: `cd PICO_tracker && pixi run cmake --build ../TJ_arm_control/build-teleop --target test_tracking_protocol test_control_protocol test_arm_target_protocol`

Expected: FAIL because the new source/targets do not exist.

- [ ] **Step 3: Implement the exact C++ protocol types**

Use these public declarations without packed structs or `reinterpret_cast` wire reads:

```cpp
inline constexpr std::size_t kTrackingPacketSize = 1540U;
inline constexpr std::uint16_t kTrackingLeftActive = 1U << 0U;
inline constexpr std::uint16_t kTrackingRightActive = 1U << 1U;
inline constexpr std::uint16_t kTrackingHeadValid = 1U << 2U;
using TrackingPose = std::array<float, 7>;
using TrackingHand = std::array<TrackingPose, 26>;
struct TrackingFrame {
  std::uint64_t sequence{0};
  std::uint64_t tracking_epoch{0};
  std::int64_t source_timestamp_ns{0};
  std::int64_t bridge_monotonic_ns{0};
  std::uint16_t flags{0};
  float left_scale{1.0F};
  float right_scale{1.0F};
  TrackingPose head_pose{};
  TrackingHand left_hand{};
  TrackingHand right_hand{};
};
enum class ControlCommand : std::uint16_t {
  kStart = 1, kPause = 2, kResume = 3,
  kRealign = 4, kReset = 5, kShutdown = 6,
};
struct ControlFrame {
  std::uint64_t sequence{0};
  std::int64_t monotonic_timestamp_ns{0};
  ControlCommand command{ControlCommand::kStart};
};
```

The tracking byte map is fixed and shared by both implementations:

```text
0..3      magic "SVT1"
4..5      uint16 version = 1
6..7      uint16 flags
8..11     uint32 payload_size = 1540
12..15    uint32 crc32 over bytes [16,1540)
16..23    uint64 sequence
24..31    uint64 tracking_epoch
32..39    int64 source_timestamp_ns
40..47    int64 bridge_monotonic_ns
48..51    float32 left_scale
52..55    float32 right_scale
56..83    7 float32 head pose: xyz + quaternion_xyzw
84..811   26 × 7 float32 left hand poses
812..1539 26 × 7 float32 right hand poses
```

The control byte map is fixed and shared by both implementations:

```text
0..3   magic \"SVTC\"
4..5   uint16 version = 1
6..7   uint16 command
8..11  uint32 payload_size = 40
12..15 uint32 crc32 over bytes [16,40)
16..23 uint64 sequence
24..31 int64 monotonic_timestamp_ns
32..39 uint64 reserved = 0
```

Write every integer/float explicitly little-endian. Tracking CRC is over bytes `[16,1540)` and control CRC over bytes `[16,40)`. Reject unknown flag bits and require positive epoch/timestamps, positive finite scales and every pose scalar finite. Require each active-side and valid-head quaternion norm within `1e-3`, normalize accepted values once, require an invalid head pose to be seven exact zeros, and permit finite placeholder poses for inactive hands. Reject a smaller epoch, or non-increasing sequence/source timestamp within one epoch; only an increased epoch resets sequence/timestamp history.

- [ ] **Step 4: Extend arm-target HOLD reasons without changing layout**

Use identical values in C++ and Python:

```text
NONE=0, INPUT_STALE=1, SOLVER_FAILURE=2, PAUSED=3,
INACTIVE=4, UNALIGNED=5, INVALID_INPUT=6, DISCONNECTED=7
```

Keep valid-bit semantics: a side is valid if and only if its reason is `NONE`. Regenerate the existing v2 fixture with one valid side and one `INACTIVE` side.

- [ ] **Step 5: Implement Python codecs with matching immutable dataclasses**

Use `struct.pack_into`/`unpack_from`, the existing table-free CRC32 implementation, `math.isfinite`, and immutable tuple storage. The Python tracking decoder must expose NumPy-free protocol objects; viewer conversion to NumPy belongs in Task 8.

```python
@dataclass(frozen=True)
class TrackingFrame:
    sequence: int
    tracking_epoch: int
    source_timestamp_ns: int
    bridge_monotonic_ns: int
    flags: int
    left_scale: float
    right_scale: float
    head_pose: tuple[float, ...]
    left_hand: tuple[tuple[float, ...], ...]
    right_hand: tuple[tuple[float, ...], ...]
```

`ControlSequenceGate.accept(frame)` returns `False` for a duplicate sequence and raises `ControlProtocolError("out_of_order")` for a smaller sequence. Duplicate IDs are idempotent and never execute twice.

- [ ] **Step 6: Check in deterministic golden vectors and fixture tool**

`spd_protocol_fixture_tool` supports exact commands:

```text
encode-tracking
encode-control
encode-arm-target
decode-tracking
decode-control
decode-arm-target
```

Encode commands write raw bytes to stdout from fixed values; decode commands read raw stdin and write one compact JSON object. `verify_protocol_interop.py` decodes each C++ byte stream with Python, encodes each Python fixture and feeds it to the C++ decoder, then compares every semantic field.
Add `spd_protocol_fixture_tool` as the first dependency of the `teleop_native` aggregate.


- [ ] **Step 7: Run focused protocol verification**

Run:

```bash
cd PICO_tracker
pixi run cmake --build ../TJ_arm_control/build-teleop --target test_tracking_protocol test_control_protocol test_arm_target_protocol spd_protocol_fixture_tool
pixi run ctest --test-dir ../TJ_arm_control/build-teleop --output-on-failure -R 'tracking_protocol|control_protocol|arm_target_protocol'
pixi run python -m pytest src/spd_vr/test/test_tracking_protocol.py src/spd_vr/test/test_control_protocol.py src/spd_vr/test/test_arm_target_protocol.py -q
pixi run python scripts/verify_protocol_interop.py --tool ../TJ_arm_control/build-teleop/spd_protocol_fixture_tool
```

Expected: all C++ tests PASS, all Python tests PASS, interop script prints `tracking=ok control=ok arm_target=ok`.

- [ ] **Step 8: Commit protocols**

```bash
git add TJ_arm_control PICO_tracker/src/spd_vr/spd_vr/tracking_protocol.py PICO_tracker/src/spd_vr/spd_vr/control_protocol.py PICO_tracker/src/spd_vr/spd_vr/zenoh_keys.py PICO_tracker/src/spd_vr/spd_vr/arm_target_protocol.py PICO_tracker/src/spd_vr/test PICO_tracker/scripts/verify_protocol_interop.py
git commit -m "feat: add zenoh teleoperation wire contracts"
```

---

### Task 3: Parse and validate the authoritative URDF graph

**Files:**
- Create: `PICO_tracker/src/spd_vr/spd_vr/model_compiler/__init__.py`
- Create: `PICO_tracker/src/spd_vr/spd_vr/model_compiler/urdf_model.py`
- Create: `PICO_tracker/src/spd_vr/test/test_urdf_model.py`

**Interfaces:**
- Produces: `load_urdf(path: Path) -> UrdfModel`; `UrdfModel.full_joint_order`; `UrdfModel.arm_joint_order`; `UrdfModel.parent_child_exclusions`; `UrdfModel.source_meshes`; `UrdfModel.omitted_debug_visuals`.
- `UrdfModel` is the sole input to collision and MJCF generation tasks.

- [ ] **Step 1: Write graph and validation tests**

Test the real URDF plus minimal temporary malformed URDF documents:

```python
model = load_urdf(AUTHORITATIVE_URDF)
assert len(model.links) == 80
assert len(model.joints) == 79
assert len(model.revolute_joints) == 54
assert len(model.arm_joint_order) == 14
assert model.root_link == "Link_Base"
assert model.link_chain("l_wrist")[-1] == "l_wrist"
assert model.link_chain("r_wrist")[-1] == "r_wrist"
assert len(model.source_meshes) == 62
assert all(joint.damping == 0.0 for joint in model.revolute_joints)
assert len(model.omitted_debug_visuals) == 24
assert sum(link.inertial is None for link in model.links.values()) == 14
assert {
    link.name for link in model.links.values()
    if link.inertial is not None and link.inertial.source == "fixed_point_mass"
} == {"TCP_Link_L", "TCP_Link_R"}
```

Malformed cases must assert exact error codes for multiple roots, duplicate name, missing parent, disconnected graph, missing mesh, non-finite origin/scale/inertia/material/damping, out-of-range RGBA, negative damping, non-positive mass, non-physical inertia, missing inertial on a revolute child, missing inertial on a fixed link with geometry, invalid fixed-point-mass eligibility, unknown primitive visual, any primitive collision, missing/inverted revolute limit, absent wrist and wrong DoF count.

- [ ] **Step 2: Run the graph tests and verify import failure**

Run: `cd PICO_tracker && pixi run python -m pytest src/spd_vr/test/test_urdf_model.py -q`

Expected: FAIL because `spd_vr.model_compiler.urdf_model` is absent.

- [ ] **Step 3: Implement immutable URDF dataclasses**

Define focused types:

```python
@dataclass(frozen=True)
class Transform:
    xyz: tuple[float, float, float]
    quat_wxyz: tuple[float, float, float, float]

@dataclass(frozen=True)
class UrdfMesh:
    path: Path
    scale: tuple[float, float, float]
    origin: Transform
    rgba: tuple[float, float, float, float] | None

@dataclass(frozen=True)
class UrdfInertial:
    mass: float
    origin: Transform
    inertia: tuple[float, float, float, float, float, float] | None
    source: Literal["urdf", "fixed_point_mass"]

@dataclass(frozen=True)
class UrdfLink:
    name: str
    inertial: UrdfInertial | None
    visuals: tuple[UrdfMesh, ...]
    collisions: tuple[UrdfMesh, ...]
    omitted_debug_visuals: tuple[str, ...]

@dataclass(frozen=True)
class UrdfJoint:
    name: str
    joint_type: str
    parent: str
    child: str
    origin: Transform
    axis: tuple[float, float, float]
    lower: float | None
    upper: float | None
    damping: float
    effort: float | None
    velocity: float | None

@dataclass(frozen=True)
class UrdfModel:
    source_path: Path
    root_link: str
    links: Mapping[str, UrdfLink]
    joints: tuple[UrdfJoint, ...]
    children_by_link: Mapping[str, tuple[UrdfJoint, ...]]
```
Resolve `package://`, `file://` and relative mesh references against the URDF directory; reject paths outside the workspace asset tree. Parse rpy to normalized `wxyz` once. Parse optional joint dynamics and use the URDF-defined zero default when absent; all 54 real revolute joints therefore have `damping=0.0`, not an invented compiler value. For provided tensors, validate positive definiteness with `eigenvalues = numpy.linalg.eigvalsh(inertia)`, require every eigenvalue positive, and require each principal moment no larger than the sum of the other two within `1e-12`. Accept a missing inertial only when the link's incoming joint is fixed and it has no visual/collision geometry; the real file has 14 such frames. Mark only `TCP_Link_L/R` as `source="fixed_point_mass"` with `inertia=None` when their positive mass accompanies an exact zero tensor and their direct fixed-joint parents are `Link7_L/R`; reject every other zero/non-physical tensor. Recognize and record exactly the 24 named `*_axis_[0-2]` cylinder visuals as omitted debug geometry; reject any other primitive visual and every primitive collision. Task 5 must aggregate and revalidate the two TCP point masses before MJCF emission.

- [ ] **Step 4: Derive deterministic topology and side/group ordering**

Use source XML joint order only as a sibling tie-breaker. Determine each arm as the revolute joints on the root-to-`l_wrist` or root-to-`r_wrist` path; determine each hand as revolute descendants of that wrist. Require exactly 7 arm + 20 hand joints per side and expose final order:

```text
left arm, left hand, right arm, right hand
```

`parent_child_exclusions` contains every direct parent-child body pair, including fixed links. Do not infer structure from existing MJCF or a hard-coded 54-name list.

- [ ] **Step 5: Run focused URDF tests**

Run: `cd PICO_tracker && pixi run python -m pytest src/spd_vr/test/test_urdf_model.py -q`

Expected: PASS, including real counts `80/79/54/14/62`, 66 inertial-bearing links, 14 massless fixed frames and exactly two fixed TCP point masses.

- [ ] **Step 6: Commit URDF model parser**

```bash
git add PICO_tracker/src/spd_vr/spd_vr/model_compiler PICO_tracker/src/spd_vr/test/test_urdf_model.py
git commit -m "feat: validate authoritative teleop urdf"
```

---

### Task 4: Generate deterministic multi-convex collision assets

**Files:**
- Create: `PICO_tracker/src/spd_vr/spd_vr/model_compiler/collision.py`
- Create: `PICO_tracker/src/spd_vr/test/test_collision_compiler.py`
- Create: `PICO_tracker/src/spd_vr/test/fixtures/concave_u.stl`

**Interfaces:**
- Consumes: `UrdfModel.source_meshes`.
- Produces: `compile_collision_assets(model, cache_dir) -> CollisionManifest`; each piece carries canonical vertices/faces, hash, volume and link name, ready for inline MJCF emission.

- [ ] **Step 1: Write cache, convexity and quality-gate tests**

The fixture is a watertight U-shaped mesh whose one-hull p95 error exceeds the hand threshold. Assert:

```python
config = CollisionConfig(seed=0, max_convex_hulls=16, max_hull_vertices=64)
first = compile_one(source, (1.0, 1.0, 1.0), "l_index_link", cache, config)
second = compile_one(source, (1.0, 1.0, 1.0), "l_index_link", cache, config)
assert first.cache_key == second.cache_key
assert first.output_hashes == second.output_hashes
assert 1 < len(first.pieces) <= 16
assert max(piece.vertex_count for piece in first.pieces) <= 64
assert first.p95_bidirectional_m <= 0.0015
```

Also assert cache invalidation for changed source bytes, scale, CoACD version or any parameter; interrupted staging never appears as a valid cache entry; decomposition/quality failure raises `CollisionBuildError` and produces no accepted result.

- [ ] **Step 2: Run collision tests and verify missing implementation**

Run: `cd PICO_tracker && pixi run python -m pytest src/spd_vr/test/test_collision_compiler.py -q`

Expected: FAIL because `collision.py` is absent.

- [ ] **Step 3: Implement exact decomposition parameters and cache key**

Use:

```python
scaled_vertices = np.asarray(vertices, dtype=np.float64) * np.asarray(scale)
parts = coacd.run_coacd(
    coacd.Mesh(scaled_vertices, faces),
    threshold=0.002,
    max_convex_hull=16,
    preprocess_mode="auto",
    max_ch_vertex=64,
    seed=0,
    real_metric=True,
)
```

The SHA-256 cache key serializes sorted JSON containing compiler schema version, source STL SHA-256, three URDF scale values, `importlib.metadata.version("coacd")`, threshold, max hulls, max vertices, preprocess mode, seed and real-metric flag. Cache canonical little-endian `float64` vertices and `int32` faces in `cache/<key>.tmp.<pid>/pieces.npz`, fsync file and directory, then `os.replace` to `cache/<key>/`; a concurrent writer uses the already-committed valid winner.

- [ ] **Step 4: Implement piece and surface-quality validation**

For every piece require finite vertices, integer triangular faces, `mesh.is_watertight`, `mesh.is_winding_consistent`, positive finite volume, at least four vertices/four faces and no more than 64 vertices. Canonicalize by deduplicating exact vertices, lexicographically sorting vertices and remapping indices, rotating each triangle so its smallest index is first while preserving outward winding, lexicographically sorting faces, serializing little-endian contiguous arrays, and sorting pieces by their SHA-256. Hash those canonical byte streams. Task 5 embeds these arrays in MJCF `<mesh vertex=\"…\" face=\"…\">`, so the final generated directory still contains exactly the five approved top-level artifacts and no undeclared collision sidecar dependency.

Sample 20,000 points from source and piece union with independent `numpy.random.Generator(PCG64(seed))` streams. Build `scipy.spatial.cKDTree` for both point clouds and define:

```python
p95 = max(
    float(np.percentile(source_to_union, 95)),
    float(np.percentile(union_to_source, 95)),
)
```

Use 0.0015 m for descendants of `l_wrist`/`r_wrist`, otherwise 0.003 m. Record both directional percentiles, sample count and random seed. Never catch the quality error to generate a fallback.

- [ ] **Step 5: Run focused collision tests twice**

Run:

```bash
cd PICO_tracker
OMP_NUM_THREADS=1 pixi run python -m pytest src/spd_vr/test/test_collision_compiler.py -q
OMP_NUM_THREADS=1 pixi run python -m pytest src/spd_vr/test/test_collision_compiler.py -q
```

Expected: both PASS; the second run reports a cache hit and produces identical hashes.

- [ ] **Step 6: Commit collision compiler**

```bash
git add PICO_tracker/src/spd_vr/spd_vr/model_compiler/collision.py PICO_tracker/src/spd_vr/test/test_collision_compiler.py PICO_tracker/src/spd_vr/test/fixtures/concave_u.stl
git commit -m "feat: compile deterministic convex collisions"
```

---

### Task 5: Generate and verify both MuJoCo models and manifests

**Files:**
- Create: `PICO_tracker/src/spd_vr/spd_vr/model_compiler/mjcf.py`
- Create: `PICO_tracker/src/spd_vr/spd_vr/model_compiler/artifacts.py`
- Create: `PICO_tracker/src/spd_vr/spd_vr/model_compiler/cli.py`
- Modify: `PICO_tracker/src/spd_vr/spd_vr/model_builder.py`
- Modify: `PICO_tracker/src/spd_vr/spd_vr/manifest.py`
- Create: `PICO_tracker/src/spd_vr/test/test_model_compiler.py`
- Modify: `PICO_tracker/src/spd_vr/test/test_model_builder.py`
- Modify: `PICO_tracker/src/spd_vr/setup.py`

**Interfaces:**
- Produces exactly: `unified_plant.xml`, `arm_ik.xml`, `model_manifest.yaml`, `collision_manifest.yaml`, `actuator_calibration.yaml`.
- Produces runtime validator: `verify_generated_artifacts(generated_dir, source_urdf) -> VerifiedArtifacts`.
- Preserves `model_builder.main()` only as the same new compiler CLI, not as a hybrid implementation or output alias.

- [ ] **Step 1: Write full-model and projection contract tests**

Load both generated files through `mujoco.MjModel.from_xml_path` and assert:

```python
assert plant.nq == plant.nv == plant.nu == 54
assert arm.nq == arm.nv == 14
assert mujoco.mj_name2id(arm, mujoco.mjtObj.mjOBJ_SITE, "l_wrist_target") >= 0
assert mujoco.mj_name2id(arm, mujoco.mjtObj.mjOBJ_SITE, "r_wrist_target") >= 0
assert manifest["source"]["urdf_sha256"] == sha256(AUTHORITATIVE_URDF)
assert len(manifest["joints"]) == 54
assert manifest["inertials"]["TCP_Link_L"]["source"] == "fixed_point_mass"
assert manifest["inertials"]["TCP_Link_L"]["destination"] == "Link7_L"
assert manifest["inertials"]["TCP_Link_R"]["source"] == "fixed_point_mass"
assert manifest["inertials"]["TCP_Link_R"]["destination"] == "Link7_R"
assert len(manifest["omitted_debug_visuals"]) == 24
assert manifest["joint_order"] == [item["joint"] for item in manifest["joints"]]
```

Verify every visual mesh hash matches its URDF source, all paths resolve relative to the output directory, exactly 24 debug-axis visual omissions are recorded, direct parent-child pairs have `<exclude>`, non-adjacent cross-arm and finger geoms keep compatible contact bits, and a one-byte URDF copy edit causes runtime hash rejection.

- [ ] **Step 2: Run model compiler tests and verify failure**

Run: `cd PICO_tracker && pixi run python -m pytest src/spd_vr/test/test_model_compiler.py -q`

Expected: FAIL because no URDF-first MJCF generator exists.

- [ ] **Step 3: Implement MJCF emission from the URDF graph only**

Emit `angle="radian"`, `timestep="0.00208333333333333"`, `integrator="implicitfast"`, `<size nuser_jnt="1">`, ground plane, fixed camera/light settings, validated URDF link inertials, and hinge joints with URDF axis/range, parsed damping (zero where omitted by this source), plus the URDF velocity limit in `user[0]`. Before emission, transform each TCP point mass location through its fixed joint into `Link7_L/R`, rotate the parent tensor from its inertial frame into link coordinates, and combine it with the exact `0.05 kg` point mass:

```python
combined_mass = parent_mass + point_mass
combined_com = (
    parent_mass * parent_com + point_mass * point_position
) / combined_mass
parent_delta = parent_com - combined_com
point_delta = point_position - combined_com
combined_inertia = (
    parent_inertia_link
    + parent_mass * (np.dot(parent_delta, parent_delta) * np.eye(3) - np.outer(parent_delta, parent_delta))
    + point_mass * (np.dot(point_delta, point_delta) * np.eye(3) - np.outer(point_delta, point_delta))
)
```

Re-run physical tensor checks, emit the combined inertial on `Link7_L/R`, and emit no separate TCP inertial; the 14 geometry-free fixed frames also remain without inertials. Omit only the 24 validated debug-axis cylinders. Emit every manufacturer visual mesh with exact URDF origin/scale/RGBA plus `contype="0" conaffinity="0" group="1"`, and collision geoms with `contype="1" conaffinity="1" group="3"`. Each convex piece is a named inline MJCF mesh whose `vertex` and `face` attributes come from Task 4 canonical arrays; no primitive, one-hull or sidecar-file fallback exists.


For the full plant, emit root body `Link_Base`, all 80 links and all 54 revolute joints. For arm IK, emit the union of root-to-wrist paths, the same URDF origins/inertials and only the 14 arm revolute joints; omit visual/collision geoms from this headless kinematic projection and attach zero-offset sites to `l_wrist`/`r_wrist`. Build body transforms from URDF joint origins, including every fixed mount body. Do not load or inspect `marvin_m6_qp_pico_fast.xml` or standalone hand MJCF files.

- [ ] **Step 4: Generate actuator mapping and deterministic calibration**

For each revolute joint emit one position actuator in manifest order. Use URDF effort as `forcerange`, URDF limits as `ctrlrange`, and existing deterministic critical-damping calibration logic to choose `kp/kv`; record candidate list, selected gain, home mass diagonal and test metrics in `actuator_calibration.yaml`. Require 54 complete mappings before commit.

- [ ] **Step 5: Implement artifact manifests and atomic final commit**


`model_manifest.yaml` records compiler schema/version, URDF hash, all source STL hashes, both XML hashes, model dimensions, every emitted/omitted inertial, both fixed point-mass inputs/destinations and combined tensors, all 24 omitted debug visuals, 54 ordered joint/actuator mappings, qpos/dof addresses, limits, fixed wrist transforms and sites. `collision_manifest.yaml` records cache inputs, output hashes and quality metrics. Hold an `fcntl.flock` on `<parent>/.<output-name>.lock`, build all files under `<output>.tmp.<pid>`, and validate both MuJoCo models plus every hash before publication. If an output exists, rename it to `<output>.backup`, rename staging to the final path, fsync the parent, then remove the backup; rollback on any failure, and recover a complete backup on the next invocation when final is absent. Consumers therefore observe an old complete set, a new complete set, or a fail-closed missing directory—never a partial set.


- [ ] **Step 6: Replace the old builder implementation with the new CLI boundary**

`model_builder.py` exports only:

```python
from .model_compiler.cli import build_models, main

__all__ = ["build_models", "main"]
```

Update `setup.py` data files to the five new artifact names. Keep old generated files until Task 11 migrates all consumers and deletes them.

- [ ] **Step 7: Build actual artifacts and run focused tests**

Run:

```bash
cd PICO_tracker
OMP_NUM_THREADS=1 pixi run python -m spd_vr.model_compiler.cli --urdf ../assets/tianji_wuji2/tianji_wuji2.urdf --output src/spd_vr/generated
OMP_NUM_THREADS=1 pixi run python -m pytest src/spd_vr/test/test_urdf_model.py src/spd_vr/test/test_collision_compiler.py src/spd_vr/test/test_model_compiler.py src/spd_vr/test/test_model_builder.py -q
```

Expected: five final artifacts; both MJCFs load; test output confirms 54 full DoFs, 14 arm DoFs and collision thresholds.

- [ ] **Step 8: Commit compiler and generated artifacts**

```bash
git add PICO_tracker/src/spd_vr/spd_vr/model_compiler PICO_tracker/src/spd_vr/spd_vr/model_builder.py PICO_tracker/src/spd_vr/spd_vr/manifest.py PICO_tracker/src/spd_vr/setup.py PICO_tracker/src/spd_vr/test PICO_tracker/src/spd_vr/generated
git commit -m "feat: generate urdf-first mujoco teleop models"
```

---

### Task 6: Implement the PXREA SDK to Zenoh tracking bridge

**Files:**
- Create: `TJ_arm_control/include/tianji_qp_ik/zenoh_session.hpp`
- Create: `TJ_arm_control/src/zenoh_session.cpp`
- Create: `TJ_arm_control/include/tianji_qp_ik/pico_bridge_core.hpp`
- Create: `TJ_arm_control/src/pico_bridge_core.cpp`
- Create: `TJ_arm_control/include/tianji_qp_ik/pico_bridge_runtime.hpp`
- Create: `TJ_arm_control/src/pico_bridge_runtime.cpp`
- Create: `TJ_arm_control/apps/pico_zenoh_bridge.cpp`
- Create: `TJ_arm_control/tests/test_pico_bridge_core.cpp`
- Modify: `TJ_arm_control/CMakeLists.txt`

**Interfaces:**
- Consumes: existing canonical `pico_bridge/pico_frame.hpp`, `pico_bridge/pico_hand_pairing.hpp`, `pico_bridge/tracking_epoch_store.hpp/.cpp` without copying their logic.
- Produces: `PicoBridgeCore::enqueueSdkCallback(PXREAClientCallbackType, int, const void*) noexcept`, `PicoBridgeCore::processNext() -> std::optional<TrackingFrame>`, `PicoBridgeRuntime::enqueueSdkCallback(PXREAClientCallbackType, int, const void*) noexcept`, `PicoBridgeRuntime::runOnce(std::int64_t)`, `ZenohSession::listenPeer(const std::string&)`, and `pico_zenoh_bridge` executable.

- [ ] **Step 1: Write bridge core tests using fake SDK events**

Cover exact 14-byte `0xAB` outer header, 733-byte hand payload, `0x05` head, `0x06` world reset, `0x38/0x39` hands, equal timestamp/epoch pairing, newer timestamp replacement, reconnect/reset epoch increment, explicit serial selection, a second custom-message serial causing ambiguity, malformed/oversize message rejection and drop-oldest queue bounds. Device lifecycle callbacks must not dereference undocumented `userData`.

```cpp
ASSERT_TRUE(core.enqueueSdkCallback(PXREADeviceFind, 0, nullptr));
EXPECT_FALSE(core.processNext().has_value());
ASSERT_TRUE(core.enqueueSdkCallback(
    PXREADeviceCustomMessage, 0, custom("SERIAL-A", leftFrame(100))));
EXPECT_FALSE(core.processNext().has_value());
ASSERT_TRUE(core.enqueueSdkCallback(
    PXREADeviceCustomMessage, 0, custom("SERIAL-A", rightFrame(100))));
auto tracking = core.processNext();
ASSERT_TRUE(tracking.has_value());
EXPECT_EQ(tracking->source_timestamp_ns, 100000000LL);
EXPECT_EQ(tracking->left_hand[1][6], 1.0F);
EXPECT_TRUE(core.enqueueSdkCallback(
    PXREADeviceCustomMessage, 0, custom("SERIAL-B", leftFrame(101))));
EXPECT_EQ(core.deviceState(), DeviceState::kAmbiguous);
```

- [ ] **Step 2: Run bridge tests and verify missing implementation**

Run: `cd PICO_tracker && pixi run cmake --build ../TJ_arm_control/build-teleop --target test_pico_bridge_core`

Expected: FAIL because the target and core are absent.

- [ ] **Step 3: Implement bounded callback handoff and device state**

For `PXREADeviceCustomMessage`, the SDK callback validates `dataSize` is between the 14-byte outer header and 2048, rejects null `dataPtr`, copies the fixed-capacity `devID[32]` with bounded NUL detection and copies exactly `dataSize` bytes into a capacity-128 drop-oldest queue, increments counters, and returns. The SDK header does not define `userData` types for Find/Connect/Missing, so those callbacks update only coarse lifecycle state and never dereference `userData`; Missing conservatively clears the observed-device set. `PicoBridgeRuntime` worker owns parsing, pairing, CRC encoding and Zenoh publication; production SDK and Task 12 fake callback source both drive this class. With `--serial`, ignore nonmatching custom messages. Without it, select the first observed custom-message serial and enter `AMBIGUOUS_DEVICE` if another appears; remain blocked until server reconnect, Missing or world reset clears selection. A head pose is attached only when valid in the current epoch and timestamp, and missing head data never gates a paired-hand publication.

Reserve epoch through the existing durable `reserve_tracking_epoch` at startup, server reconnect and world reset. Clear pending hands/head and reset sequence to zero on every new epoch. Preserve strictly positive, increasing epoch across process restarts.
In CMake, add `PICO_tracker/src/pico_bridge/include` as a private include directory and compile the existing `tracking_epoch_store.cpp` directly into the bridge core target; `pico_frame.hpp` and `pico_hand_pairing.hpp` remain their canonical header-only implementations. Do not copy them or link the ROS package.

- [ ] **Step 4: Implement zenoh-pico RAII transport**

`ZenohSession::listenPeer("tcp/127.0.0.1:7447")` creates default config, inserts `mode=peer`, inserts one listen endpoint, opens once, declares publishers, and drops publisher/session in reverse order. `connectPeer` inserts one explicit connect endpoint and disables multicast scouting. Tracking publishers use latest/drop congestion behavior; control publishers use reliable/block behavior. Subscriber callbacks copy sample payload to a bounded latest slot and never invoke IK.

- [ ] **Step 5: Implement strict SDK lifecycle executable**

`main` prints selected SDK header/library path from compile definitions, opens Zenoh before `PXREAInit`, calls:

```cpp
const unsigned mask = PXREAServerConnect | PXREAServerDisconnect |
                      PXREADeviceFind | PXREADeviceMissing |
                      PXREADeviceConnect | PXREADeviceCustomMessage;
const int init_status = PXREAInit(&application, &sdkCallback, mask);
```

It calls `PXREADeinit()` exactly once after the shared runtime worker stops, including SIGINT/SIGTERM and SHUTDOWN control paths. `PicoBridgeRuntime` publishes compact bridge status JSON at 1 Hz and immediately after control handling. Every status has `schema_version=1`, `component="bridge"`, process/publish monotonic timestamps, `last_control_sequence` and `session_state`, plus SDK/server/device/Zenoh state, callback rate, pair drops, queue drops, invalid frames, tracking sequence/epoch and source latency. A SHUTDOWN acknowledgement is published before teardown. Never call `PXREASendBytesToDevice` or any robot actuator API.
Add `pico_zenoh_bridge` to the `teleop_native` aggregate.


- [ ] **Step 6: Run focused bridge verification**

Run:

```bash
cd PICO_tracker
pixi run cmake --build ../TJ_arm_control/build-teleop --target test_pico_bridge_core pico_zenoh_bridge
pixi run ctest --test-dir ../TJ_arm_control/build-teleop --output-on-failure -R pico_bridge_core
```

Expected: PASS; linker resolves `/opt/apps/roboticsservice/SDK/x64/libPXREARobotSDK.so`; no ROS library appears in `ldd ../TJ_arm_control/build-teleop/pico_zenoh_bridge`.

- [ ] **Step 7: Commit the bridge**

```bash
git add TJ_arm_control/CMakeLists.txt TJ_arm_control/include/tianji_qp_ik TJ_arm_control/src TJ_arm_control/apps/pico_zenoh_bridge.cpp TJ_arm_control/tests/test_pico_bridge_core.cpp
git commit -m "feat: bridge pico tracking onto zenoh"
```

---

### Task 7: Implement the headless 200 Hz Zenoh QP IK process

**Files:**
- Create: `TJ_arm_control/include/tianji_qp_ik/generated_model_manifest.hpp`
- Create: `TJ_arm_control/src/generated_model_manifest.cpp`
- Create: `TJ_arm_control/include/tianji_qp_ik/zenoh_ik_core.hpp`
- Create: `TJ_arm_control/src/zenoh_ik_core.cpp`
- Create: `TJ_arm_control/apps/tianji_zenoh_ik.cpp`
- Create: `TJ_arm_control/tests/test_generated_model_manifest.cpp`
- Create: `TJ_arm_control/tests/test_zenoh_ik_core.cpp`
- Modify: `TJ_arm_control/include/tianji_qp_ik/pico_wrist_alignment.hpp`
- Modify: `TJ_arm_control/src/pico_wrist_alignment.cpp`
- Modify: `TJ_arm_control/tests/test_pico_wrist_alignment.cpp`
- Modify: `TJ_arm_control/include/tianji_qp_ik/mujoco_robot.hpp`
- Modify: `TJ_arm_control/src/mujoco_robot.cpp`
- Modify: `TJ_arm_control/tests/test_mujoco_robot.cpp`
- Modify: `TJ_arm_control/CMakeLists.txt`

**Interfaces:**
- Consumes: verified `arm_ik.xml`, `model_manifest.yaml`, tracking/control frames.
- Produces: `MocapTargetPolicy::{kRequired,kOptional}`, `ZenohIkCore::acceptTracking`, `ZenohIkCore::applyControl`, `ZenohIkCore::tick(now_ns) -> optional<ArmTargetFrame>` and `tianji_zenoh_ik`.

- [ ] **Step 1: Write manifest and independent-side state tests**

Assert startup rejects changed URDF/XML hash, wrong 14 DoFs, wrong joint order, missing velocity `user[0]`, and a wrist site attached to the wrong body. Add a `MujocoRobot` test proving its existing default still requires `target_L/target_R`, while `MocapTargetPolicy::kOptional` loads the generated headless IK model without either mocap body. State tests cover 10 stable frames, one-side inactivity, 0.02 m/0.15 rad jump rejection, 50 ms stale, solver failure, epoch change, PAUSE/RESUME/REALIGN/RESET and last-valid target retention.

```cpp
for (std::uint64_t sequence = 1; sequence <= 9; ++sequence) {
  core.acceptTracking(stableFrame(sequence, true, false), now(sequence));
  EXPECT_EQ(core.state(ArmSide::kLeft), AlignmentState::kStabilizing);
}
core.acceptTracking(stableFrame(10, true, false), now(10));
EXPECT_EQ(core.state(ArmSide::kLeft), AlignmentState::kAligned);
EXPECT_EQ(core.state(ArmSide::kRight), AlignmentState::kHoldInactive);
```

- [ ] **Step 2: Run IK tests and verify missing implementation**

Run: `cd PICO_tracker && pixi run cmake --build ../TJ_arm_control/build-teleop --target test_generated_model_manifest test_zenoh_ik_core`

Expected: FAIL because the targets do not exist.

- [ ] **Step 3: Implement generated artifact verification in C++**

Use yaml-cpp and the repository SHA-256 helper/OpenSSL to verify the manifest schema, source URDF hash, `arm_ik.xml` hash, 14 ordered arm joints, velocity `user[0]`, wrist sites and recorded dimensions before constructing `MujocoRobot`. Add `MocapTargetPolicy` to the robot constructor with `kRequired` as the compatibility-preserving default; only target-body lookup is skipped for `kOptional`. Construct the headless process with `EndEffectorSiteNames{"l_wrist_target", "r_wrist_target"}` and `MocapTargetPolicy::kOptional`. Refuse startup before opening Zenoh when verification fails.

- [ ] **Step 4: Adapt neutral alignment to atomic tracking frames**

Convert OpenXR joint index 1 from `xyz + quaternion_xyzw` to `Pose`; apply:

```text
T_robot_from_pico = T_urdf_wrist_neutral * inverse(T_pico_wrist_neutral)
T_robot_target = T_robot_from_pico * T_pico_wrist_current
```

Maintain side-local `DISCONNECTED`, `WAITING_INPUT`, `STABILIZING`, `ALIGNED`, `HOLD_STALE`, `HOLD_INACTIVE`, `HOLD_SOLVER`. In IDLE/PAUSED, accept frames into the latest slot but do not advance alignment. START, RESUME, REALIGN and RESET capture the current generation as a floor and clear both alignment windows; only later frames may contribute to the fresh 10-frame window. Clearing alignment must not clear `last_valid_q/qdot`. A side-specific active loss affects only that side; epoch change or timestamp rollback clears both alignment references.

- [ ] **Step 5: Implement the 200 Hz QP core**

Load `qp_ik_pico_teleop.yaml`, override controller rate to 200 Hz, initialize the optional-mocap `MujocoRobot` at manifest home posture and instantiate `DualArmController`. On each 5 ms tick:

1. reject input older than 50 ms;
2. update neutral alignment only from a newly accepted tracking frame;
3. call `controller.step` with each valid aligned wrist target;
4. update each accepted side from `controller.referenceState(side)`;
5. retain previous q/qdot for a failed/invalid side;
6. emit the corresponding valid bit and exact `ArmTargetHoldReason`.

While paused, return `std::nullopt` and do not publish QP targets. Resume clears both alignment windows. Do not instantiate GLFW, MuJoCo rendering, UDP sockets or hardware output.

- [ ] **Step 6: Implement the Zenoh IK application**

Connect to `tcp/127.0.0.1:7447`, subscribe to tracking/control, and publish arm targets/status using `zenoh_keys.hpp` only. The tracking callback only decodes and atomically replaces the latest frame. The 200 Hz loop uses monotonic absolute deadlines; overload skips missed deadlines rather than replaying ticks. Status at 1 Hz and immediately after control contains `schema_version=1`, `component="ik"`, process/publish monotonic timestamps, `last_control_sequence`, `session_state`, tracking/target rates, latency, sequence drops, side states/HOLD reasons, solver failure counters, solve p95/max and verified model hashes. Publish SHUTDOWN acknowledgement before exit.
Add `tianji_zenoh_ik` to the `teleop_native` aggregate.


- [ ] **Step 7: Run focused IK verification**

Run:

```bash
cd PICO_tracker
pixi run cmake --build ../TJ_arm_control/build-teleop --target test_generated_model_manifest test_mujoco_robot test_pico_wrist_alignment test_zenoh_ik_core tianji_zenoh_ik
pixi run ctest --test-dir ../TJ_arm_control/build-teleop --output-on-failure -R 'generated_model_manifest|mujoco_robot|pico_wrist_alignment|zenoh_ik_core'
```

Expected: PASS; per-side tests show no zero/neutral jump and pause emits no packet.

- [ ] **Step 8: Commit IK process**

```bash
git add TJ_arm_control/CMakeLists.txt TJ_arm_control/include/tianji_qp_ik TJ_arm_control/src TJ_arm_control/apps/tianji_zenoh_ik.cpp TJ_arm_control/tests
git commit -m "feat: solve pico wrist targets over zenoh"
```

---

### Task 8: Add Python Zenoh latest-only transport and session state

**Files:**
- Create: `PICO_tracker/src/spd_vr/spd_vr/zenoh_transport.py`
- Create: `PICO_tracker/src/spd_vr/spd_vr/session_state.py`
- Create: `PICO_tracker/src/spd_vr/test/test_zenoh_transport.py`
- Create: `PICO_tracker/src/spd_vr/test/test_session_state.py`

**Interfaces:**
- Produces: `ZenohTeleopClient`, `LatestSample[T]`, `DurableControlSequence`, `SessionStateMachine`, `SessionAction`.
- Later viewer calls `take_tracking()`, `take_arm_target()`, `publish_control(command)` and `publish_status(mapping)`.

- [ ] **Step 1: Write latest-only and state-transition tests**

Use an in-process real Zenoh peer for transport tests and deterministic fake clock/temp state path for session tests. Publish 1,000 tracking frames while consuming once and require sequence 1,000 with mailbox depth one. Verify wrong/duplicate/out-of-order frames increment counters and never replace the accepted sample. Open two `DurableControlSequence` instances on the same file, interleave reservations, reopen them, and require unique increasing values `1,2,3,4` with no reset.

```python
state = SessionStateMachine()
assert state.apply(ControlCommand.START).start_physics
assert state.apply(ControlCommand.PAUSE).freeze
assert state.apply(ControlCommand.RESUME).require_realign
assert state.apply(ControlCommand.RESET).reset_plant
assert state.apply(ControlCommand.SHUTDOWN).shutdown
```

- [ ] **Step 2: Run tests and verify missing modules**

Run: `cd PICO_tracker && pixi run python -m pytest src/spd_vr/test/test_zenoh_transport.py src/spd_vr/test/test_session_state.py -q`

Expected: FAIL because the modules are absent.

- [ ] **Step 3: Implement the latest-only mailbox**

`LatestSample.replace(sequence, epoch, value, arrival_ns)` keeps one immutable item under a lock and records accepted/rejected/dropped counts. `take_after(last_generation)` returns the item only when a local generation changed. Epoch increase resets sequence history; epoch rollback and same-epoch non-increase are rejected.

- [ ] **Step 4: Implement Zenoh peer configuration and QoS**

Create Python config exactly as:

```python
config = zenoh.Config()
config.insert_json5("mode", '"peer"')
config.insert_json5("connect/endpoints", '["tcp/127.0.0.1:7447"]')
config.insert_json5("scouting/multicast/enabled", "false")
```

Declare subscribers/publishers exclusively through `zenoh_keys.py`: tracking and arm-target subscribers plus control and viewer-status publishers. Subscriber callbacks call the protocol decoders and `LatestSample.replace` only; no status key is registered as control input. `publish_control` obtains each global sequence through `DurableControlSequence`, which uses `fcntl.flock`, strict uint64 parsing, temp-file fsync/replace and parent-directory fsync at default `~/.config/pico_tracker/control_sequence`; viewer keys and `control_cli` share this allocator, so publisher restarts cannot reuse IDs. Publication is reliable/ordered/blocking, and a failed publish raises a visible runtime error rather than queuing a local replay.

- [ ] **Step 5: Implement session transitions and resume generation floor**

States are `IDLE`, `RUNNING`, `PAUSED`, `SHUTTING_DOWN`. Space in IDLE sends START; Space in RUNNING sends PAUSE; Space in PAUSED sends RESUME. START, RESUME, REALIGN and RESET record current tracking/target generations as floors and require newer frames plus fresh 10-frame alignment; RESET additionally requests plant reset atomically while preserving RUNNING versus PAUSED state. Duplicate control sequence returns a no-op action.

- [ ] **Step 6: Run focused transport/state tests**

Run: `cd PICO_tracker && pixi run python -m pytest src/spd_vr/test/test_tracking_protocol.py src/spd_vr/test/test_control_protocol.py src/spd_vr/test/test_zenoh_transport.py src/spd_vr/test/test_session_state.py -q`

Expected: PASS; overload test reports only bounded replacement drops.

- [ ] **Step 7: Commit transport and session state**

```bash
git add PICO_tracker/src/spd_vr/spd_vr/zenoh_transport.py PICO_tracker/src/spd_vr/spd_vr/session_state.py PICO_tracker/src/spd_vr/test/test_zenoh_transport.py PICO_tracker/src/spd_vr/test/test_session_state.py
git commit -m "feat: add latest-only zenoh viewer transport"
```

---

### Task 9: Make Wuji retarget kinematics use the authoritative URDF

**Files:**
- Modify: `wuji-retargeting/wuji_retargeting/robot.py`
- Modify: `wuji-retargeting/wuji_retargeting/retarget.py`
- Modify: `wuji-retargeting/wuji_retargeting/opt/base.py`
- Modify: `PICO_tracker/src/spd_vr/config/wuji2_pico_left.yaml`
- Modify: `PICO_tracker/src/spd_vr/config/wuji2_pico_right.yaml`
- Modify: `PICO_tracker/src/spd_vr/spd_vr/retarget_pair.py`
- Modify: `PICO_tracker/src/spd_vr/test/test_retarget_pair.py`
- Create: `PICO_tracker/src/spd_vr/test/test_retarget_authoritative_urdf.py`

**Interfaces:**
- Consumes: `model_manifest.yaml` hand joint groups and the same integrated URDF used by Task 3.
- Produces: `RobotWrapper(urdf_path, hand_side=None, active_joint_names=None, reference_frame_name=None)`, `Retargeter.from_yaml(path, hand_side, active_joint_names=None)`, `WujiRetargetPair.from_manifest(left_config, right_config, manifest_path)`.

- [ ] **Step 1: Write failing reduced-hand and source-authority tests**

Resolve both SPD config paths and require:

```python
left = yaml.safe_load(LEFT_CONFIG.read_text(encoding="utf-8"))
right = yaml.safe_load(RIGHT_CONFIG.read_text(encoding="utf-8"))
assert "mjcf_path" not in left["optimizer"]
assert "mjcf_path" not in right["optimizer"]
assert (LEFT_CONFIG.parent / left["optimizer"]["urdf_path"]).resolve() == AUTHORITATIVE_URDF
assert (RIGHT_CONFIG.parent / right["optimizer"]["urdf_path"]).resolve() == AUTHORITATIVE_URDF

pair = WujiRetargetPair.from_manifest(LEFT_CONFIG, RIGHT_CONFIG, MODEL_MANIFEST)
assert pair.left_retargeter.optimizer.robot.model.nq == 20
assert pair.right_retargeter.optimizer.robot.model.nq == 20
assert set(pair.left_retargeter.optimizer.robot.dof_joint_names) == set(manifest_left_hand)
assert set(pair.right_retargeter.optimizer.robot.dof_joint_names) == set(manifest_right_hand)
assert pair.left_output_joint_names == manifest_left_hand
assert pair.right_output_joint_names == manifest_right_hand
```

Retarget one finite identity-hand frame and require two finite `(20,)` outputs within manifest URDF limits. Add malformed cases for unknown, duplicate, cross-side and non-single-DoF active joint names. At a nonzero finger qpos, independently compute Pinocchio world frame placements/Jacobians and require `compute_fk_batch`/`compute_all_jacobians_batch` to return `R_wrist.T @ (p_link - p_wrist)` and `R_wrist.T @ (J_link - J_wrist)`; the wrist-local result must not retain the locked arm's world rotation.

- [ ] **Step 2: Run the source-authority test and verify old standalone-model failure**

Run: `cd PICO_tracker && pixi run python -m pytest src/spd_vr/test/test_retarget_authoritative_urdf.py src/spd_vr/test/test_retarget_pair.py -q`

Expected: FAIL because current configs point to standalone Wuji URDF/MJCF and `RobotWrapper` exposes no active-joint reduction.

- [ ] **Step 3: Implement deterministic Pinocchio model reduction**

Load the full integrated model, validate the ordered active names, lock every other movable joint at `pin.neutral(full_model)`, and build the reduced model:

```python
full_model = pin.buildModelFromUrdf(str(Path(urdf_path).resolve()))
active = tuple(active_joint_names or ())
active_set = set(active)
locked_joint_ids = [
    joint_id
    for joint_id, name in enumerate(full_model.names)
    if joint_id > 0 and full_model.nqs[joint_id] > 0 and name not in active_set
]
model = (
    pin.buildReducedModel(full_model, locked_joint_ids, pin.neutral(full_model))
    if active
    else full_model
)
```

Before reduction, require exactly 20 unique requested joints, every name present, every requested joint `nq=nv=1`, every name starts with the requested side prefix, and every active joint descends from the requested wrist reference frame. After reduction, require `model.nq=model.nv=20` and the reduced `dof_joint_names` set equals the requested manifest set. Preserve Pinocchio's own deterministic qpos order inside the optimizer; do not assume it equals manifest order. When `reference_frame_name` is supplied, resolve it once and make batch FK/Jacobian results wrist-local using `R_ref.T @ (p - p_ref)` and `R_ref.T @ (J - J_ref)`; because validation excludes active ancestors of the reference, its rotation is constant with respect to optimizer qpos.


- [ ] **Step 4: Thread active joint names through the retarget API**

`Retargeter.from_yaml(..., active_joint_names=names)` loads a fresh config mapping, stores `optimizer.active_joint_names` as a list and never mutates a caller-owned mapping. Before constructing `RobotWrapper`, `BaseOptimizer` resolves `reference_frame_name` from `optimizer.link_naming.prefix + palm`, then passes active names, reference frame and `hand_side`. Preserve existing behavior only when `active_joint_names is None`, so unrelated standalone package users retain world-frame FK and their bundled model.

- [ ] **Step 5: Cut the SPD retarget path over to manifest-driven construction**

Change both SPD configs to:

```yaml
optimizer:
  type: "AdaptiveOptimizerAnalytical"
  urdf_path: "../../../../assets/tianji_wuji2/tianji_wuji2.urdf"
```
Preserve `link_naming` and retarget tuning values; remove `mjcf_path`. Implement `WujiRetargetPair.from_manifest` to load the 20 `left/hand` and 20 `right/hand` names, pass them as active joint names to each retargeter, then build an explicit permutation from each reduced robot's `dof_joint_names` to manifest destination order. Remove `mjcf_actuator_joint_names` and the implicit standalone-MJCF fallback from the SPD module.

- [ ] **Step 6: Run focused authoritative retarget tests**

Run:

```bash
cd PICO_tracker
pixi run python -m pytest src/spd_vr/test/test_retarget_authoritative_urdf.py src/spd_vr/test/test_retarget_pair.py -q
```

Expected: PASS; each optimizer reports 20 DoFs from the integrated URDF, both outputs are finite/in-range, and neither SPD config or runtime opens a standalone hand URDF/MJCF.

- [ ] **Step 7: Commit authoritative retarget kinematics**

```bash
git add wuji-retargeting/wuji_retargeting/robot.py wuji-retargeting/wuji_retargeting/retarget.py wuji-retargeting/wuji_retargeting/opt/base.py PICO_tracker/src/spd_vr/config PICO_tracker/src/spd_vr/spd_vr/retarget_pair.py PICO_tracker/src/spd_vr/test/test_retarget_pair.py PICO_tracker/src/spd_vr/test/test_retarget_authoritative_urdf.py
git commit -m "feat: retarget wuji hands from unified urdf"
```

---

### Task 10: Implement the single-owner MuJoCo viewer and controls

**Files:**
- Create: `PICO_tracker/src/spd_vr/spd_vr/viewer_window.py`
- Create: `PICO_tracker/src/spd_vr/spd_vr/viewer.py`
- Modify: `PICO_tracker/src/spd_vr/spd_vr/simulator.py`
- Create: `PICO_tracker/src/spd_vr/test/test_viewer_window.py`
- Create: `PICO_tracker/src/spd_vr/test/test_viewer.py`
- Modify: `PICO_tracker/src/spd_vr/test/test_simulator.py`
- Modify: `PICO_tracker/src/spd_vr/setup.py`

**Interfaces:**
- Consumes: verified full plant, Zenoh tracking/arm frames, `WujiRetargetPair`, session actions.
- Produces: `run_viewer(ViewerOptions) -> int`, CLI `spd_vr_viewer`, 480 Hz physics, 60 Hz GLFW/HUD, and optional atomic headless `--report PATH` evidence.

- [ ] **Step 1: Write key mapping, reset, freeze and side-isolation tests**

Use fake window/transport/clock and a real MuJoCo model. Assert Space emits START/PAUSE/RESUME by state, R emits REALIGN, N resets `qpos/qvel/ctrl/time`, Q/Escape emits SHUTDOWN. Publish left-only wrist/finger movement and require only left arm/hand actuator values change; repeat for right. Pause must preserve simulation time and callback counts. A headless report test requires final finite `qpos/qvel/ctrl`, rates/timing/render metrics, processed tracking/arm sequence numbers and bounded qpos snapshots keyed by each newly accepted tracking sequence.

- [ ] **Step 2: Run viewer tests and verify missing implementation**

Run: `cd PICO_tracker && pixi run python -m pytest src/spd_vr/test/test_viewer_window.py src/spd_vr/test/test_viewer.py -q`

Expected: FAIL because the viewer modules are absent.

- [ ] **Step 3: Add explicit empty-plant reset and remove render coupling**

Add `UnifiedSimulator.reset_plant()` that sets manifest home qpos, zeros qvel/act/ctrl, sets `data.time=0`, resets tick and invalidates all arm/hand snapshots. Keep `step()` as the only physics mutation boundary. Return per-step contacts and timing without calling any renderer.

- [ ] **Step 4: Implement tracking-to-Wuji conversion and once-per-frame retarget**

Construct the pair through `WujiRetargetPair.from_manifest`, convert protocol tuples to `PicoHandFrame` with `float32[26,7]`, side active flags, scale, epoch, sequence and source timestamp, then call `retarget` once only when tracking mailbox generation advances. Its 20-joint side outputs are already in `model_manifest.yaml` order; verify the lengths, clamp to recorded URDF ranges, call `simulator.on_pico_hands`, and leave the opposite side unchanged on a one-side failure.

- [ ] **Step 5: Implement the 480/60 deterministic loop**

Use an absolute monotonic deadline at `1/480 s`. At each physics boundary, consume at most one newest tracking and one newest arm target, apply valid sides, perform one `simulator.step()`, and record step duration. Every eighth physics tick render one frame; if render is late, never run extra physics or consume queued historical targets. Paused state continues status reception/render but performs no retarget, target application or physics step.

- [ ] **Step 6: Implement the GLFW MuJoCo window and HUD**

Own `MjvCamera`, `MjvOption`, `MjvScene`, `MjrContext` and GLFW window in the viewer thread. Support left-drag orbit, right-drag pan and wheel zoom. Render with `mujoco.mjv_updateScene` and `mujoco.mjr_render`; overlay two text columns with `mujoco.mjr_overlay` showing SDK/Zenoh state, rates, two latencies, drops/invalid frames, side alignment/HOLD, physics p95/max, contacts/collision warning and model hashes. Publish viewer status at 1 Hz and immediately after control with `schema_version=1`, `component="viewer"`, process/publish monotonic timestamps, `last_control_sequence`, `session_state`, latest accepted tracking/arm sequences, manifest-ordered finite `qpos` arrays for the four arm/hand groups and the same viewer metrics; publish SHUTDOWN acknowledgement before window teardown.

Map key callback exactly:

```python
KEY_COMMANDS = {
    glfw.KEY_R: ControlCommand.REALIGN,
    glfw.KEY_N: ControlCommand.RESET,
    glfw.KEY_Q: ControlCommand.SHUTDOWN,
    glfw.KEY_ESCAPE: ControlCommand.SHUTDOWN,
}
```

Space is state-dependent START/PAUSE/RESUME. Closing the GLFW window follows the same SHUTDOWN path as Q.

- [ ] **Step 7: Add headless/offscreen mode for integration**

`--headless --duration S` creates `mujoco.Renderer`, renders at least one manufacturer-mesh frame, computes its non-background pixel count, and exits only through the same cleanup path. With `--report PATH`, write one temp JSON document containing final state, counters, metrics and bounded per-new-tracking-sequence qpos snapshots, fsync and replace the requested path during cleanup. `test_viewer.py` opens a real in-process Zenoh listener before invoking this CLI path, while the viewer connects as a separate peer and steps the real full plant; it does not substitute mock transport/model/data.

- [ ] **Step 8: Run focused viewer verification**

Run:

```bash
cd PICO_tracker
MUJOCO_GL=egl pixi run python -m pytest src/spd_vr/test/test_simulator.py src/spd_vr/test/test_viewer_window.py src/spd_vr/test/test_viewer.py -q
```

Expected: PASS; the headless integration case establishes a real Zenoh peer connection, reaches viewer cleanup and reports at least one non-background manufacturer-mesh frame.

- [ ] **Step 9: Commit viewer**

```bash
git add PICO_tracker/src/spd_vr/spd_vr/viewer.py PICO_tracker/src/spd_vr/spd_vr/viewer_window.py PICO_tracker/src/spd_vr/spd_vr/simulator.py PICO_tracker/src/spd_vr/setup.py PICO_tracker/src/spd_vr/test
git commit -m "feat: add mujoco zenoh teleop viewer"
```

---

### Task 11: Add operator lifecycle and perform clean cutover

**Files:**
- Create: `PICO_tracker/src/spd_vr/spd_vr/preflight.py`
- Create: `PICO_tracker/src/spd_vr/spd_vr/control_cli.py`
- Create: `PICO_tracker/src/spd_vr/spd_vr/status_cli.py`
- Modify: `PICO_tracker/scripts/start_spd_vr.sh`
- Modify: `PICO_tracker/scripts/stop_spd_vr.sh`
- Modify: `PICO_tracker/pixi.toml`
- Modify: `PICO_tracker/src/spd_vr/setup.py`
- Modify: `PICO_tracker/src/spd_vr/package.xml`
- Modify: `PICO_tracker/src/spd_vr/spd_vr/runtime.py`
- Modify: `PICO_tracker/src/spd_vr/spd_vr/simulator.py`
- Delete: `PICO_tracker/src/spd_vr/spd_vr/ros_input.py`
- Delete: `PICO_tracker/src/spd_vr/generated/tianji_wuji2_spd.xml`
- Delete: `PICO_tracker/src/spd_vr/generated/joint_manifest.yaml`
- Delete: `PICO_tracker/src/spd_vr/generated/sim_actuator_calibration.yaml`
- Modify: `PICO_tracker/src/spd_vr/test/test_runtime.py`
- Modify: `PICO_tracker/src/spd_vr/test/test_simulator.py`
- Create: `PICO_tracker/src/spd_vr/test/test_preflight.py`
- Create: `PICO_tracker/src/spd_vr/test/test_lifecycle_scripts.py`
- Modify: `PICO_tracker/README.zh-CN.md`
- Modify: `PICO_tracker/README.md`

**Interfaces:**
- Produces Pixi commands: `spd-model`, `pico-adb`, `spd-teleop`, `spd-teleop-status`, `spd-teleop-stop`.
- Produces exactly one tmux session `spd-vr` with windows `pico_zenoh_bridge`, `tianji_zenoh_ik`, `spd_vr_viewer`.

- [ ] **Step 1: Write preflight and script contract tests**

Assert preflight failures for missing ADB reverse, SDK, display, artifact/hash mismatch, occupied 7447 and existing tmux session. Parse scripts as text and assert exact window set plus absence of `ros2`, `rclpy`, `DDS`, `Manus`, `smpl`, `optical_inner`, `pico_bridge_node`, M0 and physical command executables.

- [ ] **Step 2: Run lifecycle tests and capture old-script failure**

Run: `cd PICO_tracker && pixi run python -m pytest src/spd_vr/test/test_preflight.py src/spd_vr/test/test_lifecycle_scripts.py -q`

Expected: FAIL because current scripts start the old ROS/six-window chain.

- [ ] **Step 3: Implement preflight and diagnostic CLIs**

`preflight.py` checks `../adb.sh status`, RoboticsService availability, selected PICO, exact SDK header/library, importable CoACD/MuJoCo/Zenoh/display, verified generated hashes, free `127.0.0.1:7447` and absent `spd-vr` session. It outputs one JSON object and non-zero exit on any failed check.

`control_cli.py` connects, sends one versioned command and waits until all three status documents report `last_control_sequence >= sent_sequence` and the expected `session_state`, with a bounded timeout. `status_cli.py` validates schema/component identity and monotonic timestamp age, gathers the latest three status documents for one second and returns non-zero if any component is absent, malformed or stale. Status JSON is diagnostic acknowledgement only and is never decoded as a control command.

- [ ] **Step 4: Replace startup with exactly three windows**

`start_spd_vr.sh` uses `set -euo pipefail`, runs preflight, creates session/window commands with absolute paths, waits for bridge port and healthy statuses, and tears the session down if later startup fails. It never sources ROS setup. The three commands are:

```text
pico_zenoh_bridge --listen tcp/127.0.0.1:7447 --generated-dir <generated> [--serial <id>]
tianji_zenoh_ik --connect tcp/127.0.0.1:7447 --generated-dir <generated>
python -m spd_vr.viewer --connect tcp/127.0.0.1:7447 --generated-dir <generated>
```

- [ ] **Step 5: Replace shutdown with identity-checked graceful order**

`stop_spd_vr.sh` verifies session name and each pane command, publishes SHUTDOWN, waits viewer then IK then bridge with a five-second total graceful deadline, and only then signals still-matching tmux panes. It does not stop an ADB supervisor it did not start.

- [ ] **Step 6: Add exact Pixi operator tasks**

```toml
spd-model = "OMP_NUM_THREADS=1 python -m spd_vr.model_compiler.cli --urdf ../assets/tianji_wuji2/tianji_wuji2.urdf --output src/spd_vr/generated"
spd-teleop = "bash scripts/start_spd_vr.sh"
spd-teleop-status = "python -m spd_vr.status_cli --connect tcp/127.0.0.1:7447"
spd-teleop-stop = "bash scripts/stop_spd_vr.sh"
```

Keep `pico-adb = "bash ../adb.sh"`.

- [ ] **Step 7: Remove obsolete live paths and migrate remaining consumers**

Change simulator/runtime/test defaults to `unified_plant.xml` and `model_manifest.yaml`. Remove `socket`, UDP thread fields/methods and UDP tests from `simulator.py`. Remove `LiveInputMailbox`, ROS topic/arm bind CLI arguments and non-mock live branch from `runtime.py`; keep deterministic mock episode behavior. Delete `ros_input.py` and its tests. Delete the three old generated files only after repository search shows no remaining consumer.

- [ ] **Step 8: Update operator documentation**

Document SDK root, RoboticsService/ADB prerequisites, five Pixi commands, three windows, keys Space/R/N/Q/Escape, 10-frame neutral hold, 50 ms HOLD, no ROS/no hardware output, status fields and exact recovery for hash mismatch, occupied port, SDK disconnect, one-hand inactive and solver failure. Remove old SPD-VR startup instructions that name UDP or ROS components.

- [ ] **Step 9: Run focused clean-cutover checks**

Run:

```bash
cd PICO_tracker
pixi run python -m pytest src/spd_vr/test/test_preflight.py src/spd_vr/test/test_lifecycle_scripts.py src/spd_vr/test/test_runtime.py src/spd_vr/test/test_simulator.py -q
pixi run python -m spd_vr.preflight --offline --generated-dir src/spd_vr/generated
```

Expected: tests PASS; offline preflight validates dependencies/artifacts/port/session while explicitly marking hardware checks skipped; repository search has no new live import of `rclpy`, no `start_arm_udp`, and no old artifact filename.

- [ ] **Step 10: Commit clean cutover**

```bash
git add PICO_tracker/pixi.toml PICO_tracker/pixi.lock PICO_tracker/scripts PICO_tracker/src/spd_vr PICO_tracker/README.md PICO_tracker/README.zh-CN.md
git commit -m "feat: cut over spd teleop lifecycle to zenoh"
```

---

### Task 12: Verify hardware-free end to end, collision and performance

**Files:**
- Create: `TJ_arm_control/apps/spd_fake_pxrea_bridge.cpp`
- Create: `PICO_tracker/scripts/test_spd_teleop_e2e.py`
- Create: `PICO_tracker/src/spd_vr/test/test_teleop_performance.py`
- Create: `PICO_tracker/src/spd_vr/test/test_collision_runtime.py`
- Modify: `TJ_arm_control/CMakeLists.txt`
- Modify: `PICO_tracker/pixi.toml`

**Interfaces:**
- Produces: `spd_fake_pxrea_bridge --frames N --rate HZ --motion MODE`, including deterministic `full-e2e`; `pixi run test-spd-teleop-e2e`.
- Verifies fake PXREA callbacks, real bridge decode/pairing, real Zenoh transport, real generated models, real QP core and real MuJoCo plant without PXREA hardware.

- [ ] **Step 1: Write the end-to-end process assertions**

The integration script starts `spd_fake_pxrea_bridge --motion full-e2e` as the listening peer, then `tianji_zenoh_ik` and `spd_vr_viewer --headless --report <temp>/viewer.json`. It waits for all status schemas, sends START through the durable control publisher, and the fake bridge emits 12 stable paired `0x38/0x39` custom callbacks followed by isolated left wrist, right wrist and per-side finger phases with recorded sequence ranges. The script reads the viewer report after graceful SHUTDOWN and asserts:

```python
assert np.linalg.norm(left_arm_after - left_arm_before) > 1e-5
assert np.allclose(right_arm_during_left, right_arm_before, atol=1e-8)
assert np.linalg.norm(right_hand_after - right_hand_before) > 1e-5
assert np.isfinite(final_qpos).all()
assert np.isfinite(final_qvel).all()
assert np.isfinite(final_ctrl).all()
```

Also assert stale/occlusion HOLD, reset, pause time freeze, resume re-alignment, bounded overload drops, rendered non-background pixels and no surviving process/tmux/port.

- [ ] **Step 2: Implement fake PXREA callbacks through the real bridge core**

The C++ app listens on the requested Zenoh endpoint, constructs exact `0xAB` outer frames containing 733-byte left/right payloads, wraps them in `PXREADevCustomMessage`, and invokes `PicoBridgeRuntime::enqueueSdkCallback` for lifecycle and custom-message events. It drives `runOnce()` through the same core decoder, pairing, tracking encoder, publisher and status path as the production bridge. It supports deterministic motion modes `stable`, `left-wrist`, `right-wrist`, `left-fingers`, `right-fingers`, `left-inactive`, `right-inactive`, `overload`, plus `full-e2e`, which runs the documented phases and publishes their sequence ranges in bridge status. It never calls `PXREAInit`, bypasses pairing with a pre-encoded tracking packet, or publishes arm targets.
Add `spd_fake_pxrea_bridge` to the `teleop_native` aggregate.


- [ ] **Step 3: Implement performance and collision runtime tests**

Run 10,000 headless physics steps after warmup and require p95 below 2.083 ms on the target workstation. Render 600 logical frames and verify exactly one render per eight physics steps. Exercise ground, cross-arm, palm and finger contacts from finite seeded poses; require finite contact forces/energy and no NaN after 2,400 steps. Confirm generated collision hashes remain identical across a no-source-change rebuild.

- [ ] **Step 4: Run complete hardware-free acceptance**

Run:

```bash
cd PICO_tracker
pixi run spd-model
pixi run build-teleop-native
MUJOCO_GL=egl pixi run python scripts/test_spd_teleop_e2e.py --build-dir ../TJ_arm_control/build-teleop --generated-dir src/spd_vr/generated
MUJOCO_GL=egl pixi run python -m pytest src/spd_vr/test/test_teleop_performance.py src/spd_vr/test/test_collision_runtime.py -q
```

Expected: end-to-end PASS, 54 finite states, side isolation PASS, clean shutdown PASS, physics p95 `< 2.083 ms`, render ratio `1:8`, collision hash/quality/contact checks PASS.

- [ ] **Step 5: Run affected protocol/model/viewer suites together**

Run:

```bash
cd PICO_tracker
pixi run test-teleop-native
OMP_NUM_THREADS=1 MUJOCO_GL=egl pixi run python -m pytest src/spd_vr/test/test_urdf_model.py src/spd_vr/test/test_collision_compiler.py src/spd_vr/test/test_model_compiler.py src/spd_vr/test/test_tracking_protocol.py src/spd_vr/test/test_control_protocol.py src/spd_vr/test/test_arm_target_protocol.py src/spd_vr/test/test_zenoh_transport.py src/spd_vr/test/test_session_state.py src/spd_vr/test/test_simulator.py src/spd_vr/test/test_viewer.py src/spd_vr/test/test_preflight.py src/spd_vr/test/test_lifecycle_scripts.py -q
```

Expected: all selected tests PASS.

- [ ] **Step 6: Commit hardware-free acceptance**

```bash
git add TJ_arm_control/apps/spd_fake_pxrea_bridge.cpp TJ_arm_control/CMakeLists.txt PICO_tracker/scripts/test_spd_teleop_e2e.py PICO_tracker/src/spd_vr/test/test_teleop_performance.py PICO_tracker/src/spd_vr/test/test_collision_runtime.py PICO_tracker/pixi.toml PICO_tracker/pixi.lock
git commit -m "test: verify zenoh teleop end to end"
```

---

### Task 13: Run and record real PICO acceptance

**Files:**
- Create: `PICO_tracker/scripts/accept_real_pico.py`
- Modify: `PICO_tracker/README.zh-CN.md`
- Modify: `PICO_tracker/README.md`
- Runtime artifact: `PICO_tracker/data/spd_vr/acceptance/<timestamp>/acceptance.json`

**Interfaces:**
- Consumes: running RoboticsService, ADB reverse, XR app and the supported `spd-teleop` session.
- Produces: one immutable acceptance JSON containing environment, model hashes, rates, commands, side-state transitions and operator-confirmed movement results.

- [ ] **Step 1: Implement the acceptance recorder**

The script subscribes to three status keys and tracking/arm-target metadata for a bounded session. It automatically verifies continuous paired frames, 10-frame alignment, no ROS process in the three tmux panes, no undefined reference to `PXREASendBytesToDevice`/`PXREADeviceControlJson` in the new executables, one-side HOLD behavior and command acknowledgements. It presents exact operator prompts for left wrist, right wrist, both five-finger hands and one-hand occlusion, recording yes/no plus measured deltas from the viewer status group qpos arrays. At the end it requires operator Q/SHUTDOWN, observes each final acknowledgement and process exit, then writes the artifact.

Required JSON fields:

```json
{
  "result": "pass",
  "urdf_sha256": "<64 lowercase hex>",
  "unified_plant_sha256": "<64 lowercase hex>",
  "arm_ik_sha256": "<64 lowercase hex>",
  "tracking_rate_hz": 60.0,
  "arm_target_rate_hz": 200.0,
  "left_aligned": true,
  "right_aligned": true,
  "space_freeze_verified": true,
  "resume_realign_verified": true,
  "realign_verified": true,
  "reset_verified": true,
  "clean_shutdown_verified": true,
  "ros_free_verified": true,
  "physical_output_absent": true
}
```

The script writes `result=fail` with exact failed checks; it never turns a failed/missing hardware check into a skip.

- [ ] **Step 2: Run real-PICO preflight and start the supported session**

Run:

```bash
cd PICO_tracker
pixi run pico-adb -- --offline
pixi run spd-model
pixi run spd-teleop
pixi run spd-teleop-status
```

Expected: ADB reverse and RoboticsService/PICO checks healthy; exactly three named tmux windows; bridge receives paired optical hands.

- [ ] **Step 3: Exercise and record real PICO behavior**

Run:

```bash
cd PICO_tracker
pixi run python scripts/accept_real_pico.py --connect tcp/127.0.0.1:7447 --output data/spd_vr/acceptance
```

Perform the prompted neutral hold, left/right wrist motions, five-finger articulations, one-hand occlusion, Space pause/resume, R realign and N reset. Expected: generated `acceptance.json` has `result=pass`; left/right movements are isolated; occlusion holds only the corresponding side; no jump occurs at alignment/resume.

- [ ] **Step 4: Verify clean shutdown and update measured acceptance notes**

Run:

```bash
cd PICO_tracker
pixi run spd-teleop-stop
pixi run spd-teleop-status
```

Expected: stop exits zero; the following status command exits non-zero with all three components absent; port 7447 and tmux session are gone. Add only measured workstation/PICO rates and the acceptance artifact path to both READMEs.

- [ ] **Step 5: Commit acceptance tooling and measured documentation**

```bash
git add PICO_tracker/scripts/accept_real_pico.py PICO_tracker/README.md PICO_tracker/README.zh-CN.md
git commit -m "test: add real pico teleop acceptance"
```

Do not commit timestamped runtime acceptance data unless the repository’s existing data policy explicitly tracks hardware run artifacts.

---

## Final Self-Review Checklist

- [ ] Every approved spec section maps to at least one task: URDF/compiler Tasks 3–5; protocols Task 2; bridge Task 6; IK/alignment Task 7; transport Task 8; authoritative Wuji retarget Task 9; viewer/control Task 10; lifecycle/cutover Task 11; hardware-free verification Task 12; real PICO Task 13.
- [ ] C++ and Python names/types agree: `TrackingFrame`, `ControlFrame`, `ArmTargetFrame`, command enum values, HOLD enum values, byte sizes and Zenoh keys.
- [ ] No implementation task introduces ROS, UDP arm transport, `zenohd`, physical output, primitive collision fallback or a second model source.
- [ ] Old generated filenames and live consumers are deleted only after all new consumers migrate in Task 11.
- [ ] All permanent observable contracts have focused tests; end-to-end, collision and performance checks exercise the actual generated model and real Zenoh sessions.
- [ ] Real hardware acceptance is a measured final gate, not a claim inferred from mocks.
