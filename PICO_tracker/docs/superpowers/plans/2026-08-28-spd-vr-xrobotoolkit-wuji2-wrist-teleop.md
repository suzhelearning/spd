# SPD-VR XRoboToolkit PICO Wrist Teleop Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 XRoboToolkit PICO 4 Ultra 的官方 26 点双手追踪接入 SPD-VR，使 PICO Wrist 分别驱动组合 MuJoCo 模型的 `l_wrist`/`r_wrist`，并让同一原子帧驱动 Wuji2 双手 20-DoF 重定向。

**Architecture:** 主机侧新增 XRoboToolkit PC-Service C++ SDK 回调桥；每次 SDK 状态回调一次性解析左右手、active、scale、timestamp、epoch、sequence，并发布 `PicoHands`。独立的 wrist relay 从同一 ROS `PicoHands` 取 index 1，发送带语义标志的 TJVR UDP 帧；Tianji QP IK 使用 `l_wrist`/`r_wrist` 固定根 link/body 作为末端，按侧完成稳定窗口后的 SE(3) 启动相对对齐。Python `UnifiedSimulator` 同时接收 7-DoF arm UDP 和 `PicoHands`，在 480 Hz 推进单一 MuJoCo plant，在 pause 时冻结物理、记录和两侧耦合。

**Tech Stack:** C++17, ROS 2 Humble/rclcpp, XRoboToolkit `PXREARobotSDK.h`, nlohmann-json, Eigen, MuJoCo, Tianji QP IK, Python 3, NumPy, pytest/GoogleTest, Pixi, tmux。

**Spec:** `PICO_tracker/docs/superpowers/specs/2026-08-28-spd-vr-xrobotoolkit-wuji2-wrist-teleop-design.md`

## Global Constraints

- XRoboToolkit SDK 回调是唯一 PICO 输入源；不得通过左右手轮询接口拼帧，也不得重新启用 Manus、VR controller、Palm/SMPL、optical-hand 输入。
- PICO index 1 固定映射到 `l_wrist`/`r_wrist`；不得使用 `tcp_L`/`tcp_R`、Palm、Controller、SMPL Wrist 或 Manus frame 作为 IK 末端。
- `PicoHands` 的左右 26x7、active、scale、timestamp 必须来自同一个 SDK 回调；发布失败、SDK 断开、JSON 解析失败都必须有结构化状态，禁止把零姿态当有效输入。
- MuJoCo physics 固定 480 Hz；PICO 原子帧、Wrist target、Wuji2 hand retargeting 固定 60 Hz；不得伪造观测 timestamp。
- 左右侧分别维护 freshness、sequence/epoch、alignment、retarget、arm validity 和 HOLD reason；一侧失败不得冻结另一侧，SDK 全局断开才同时 HOLD。
- `pause` 同时冻结 MuJoCo time/physics 和 episode recorder，清除两侧相对对齐基准；resume 后每侧重新经过 stable window 才能恢复，未恢复侧继续 HOLD。
- `checkpoint` 在手-物接触时仍拒绝；`revert`、`skip` 和原 episode 状态机语义不改变。
- 组合模型保持 54 DoF；Wuji2 retargeting 只写每侧 20 个手指 actuator，不覆盖 `l_wrist`/`r_wrist` 6-DoF frame。
- 本阶段不冻结相机位置或外参；控制闭环不依赖 PICO 相机、头显 mesh、视频流或 body-transform streaming，现有录制相机接口只按既有 provisional 配置维持兼容。
- SDK 缺失时只能让官方 bridge 显式不可构建/不可启动；不得添加 mock SDK、零姿态 fallback 或静默降级。`--mock` 只保留给无硬件模拟路径。
- 每个任务跳过格式化器、lint 和项目级全量测试；任务内只运行列出的窄测试，最终任务统一做完整验证。

---

## 文件与职责映射

### 新增

- `PICO_tracker/src/pico_bridge/include/pico_bridge/xrobotoolkit_hand_decoder.hpp`：无 ROS/SDK 状态 JSON 解码接口和 26 点固定数据结构。
- `PICO_tracker/src/pico_bridge/src/xrobotoolkit_hand_decoder.cpp`：解析官方外层 `value` JSON 与内层 `Hand.leftHand/rightHand`。
- `PICO_tracker/src/pico_bridge/src/xrobotoolkit_pico_node.cpp`：调用 `PXREAInit`/`PXREADeinit`，将 SDK callback 原子转换为 ROS `PicoHands`。
- `PICO_tracker/src/pico_bridge/include/pico_bridge/xrobotoolkit_wrist_relay_core.hpp`：从官方快照提取 index 1、处理 pause/重对齐 generation 的纯核心。
- `PICO_tracker/src/pico_bridge/src/xrobotoolkit_wrist_relay_core.cpp`：wrist relay 核心实现。
- `PICO_tracker/src/pico_bridge/src/xrobotoolkit_wrist_relay_node.cpp`：订阅 `/pico/hands` 与 `/spd_vr/pause`，发送 TJVR UDP。
- `PICO_tracker/src/pico_bridge/launch/start_xrobotoolkit_pico.launch.py`：只启动官方 SDK bridge 和 wrist relay。
- `PICO_tracker/src/spd_vr/spd_vr/ros_input.py`：live ROS `PicoHands`/pause 输入的 latest-only mailbox。
- `TJ_arm_control/include/tianji_qp_ik/pico_wrist_alignment.hpp`：每侧稳定窗口和 SE(3) 相对对齐接口。
- `TJ_arm_control/src/pico_wrist_alignment.cpp`：对齐状态机实现。
- `PICO_tracker/src/pico_bridge/test/fixtures/xrobotoolkit_hand_state.json`：官方 callback JSON fixture。
- `PICO_tracker/src/pico_bridge/test/test_xrobotoolkit_hand_decoder.cpp`：JSON 原子快照解析测试。
- `PICO_tracker/src/pico_bridge/test/test_xrobotoolkit_wrist_relay_core.cpp`：wrist index、pause 和 reset flag 测试。
- `TJ_arm_control/tests/test_pico_wrist_alignment.cpp`：稳定窗口、SE(3)、单侧和 reset 测试。
- `PICO_tracker/src/spd_vr/test/test_model_builder.py`：组合模型 wrist bodies/sites、manifest 与 54-actuator 合约测试。
- `PICO_tracker/src/spd_vr/test/test_retarget_pair.py`：固定 26→21 映射、双侧 20-DoF 输出与单侧失败隔离测试。

### 修改

- `PICO_tracker/src/pico_bridge/CMakeLists.txt`、`PICO_tracker/src/pico_bridge/package.xml`、`PICO_tracker/pixi.toml`：nlohmann-json、ROS 依赖、可显式开启的官方 SDK target、测试与安装。
- `PICO_tracker/src/pico_bridge/include/pico_bridge/tianji_teleop_protocol.hpp`、`PICO_tracker/src/pico_bridge/src/tianji_teleop_protocol.cpp`、`TJ_arm_control/include/tianji_qp_ik/pico_teleop_protocol.hpp`、`TJ_arm_control/src/pico_teleop_protocol.cpp`：wrist pose input、active、alignment reset wire flags。
- `TJ_arm_control/models/marvin_m6_qp_pico_fast.xml`、`TJ_arm_control/models/marvin_m6_qp_test.xml`：加入与 Tianji-Wuji2 URDF 固定链一致的 `l_wrist`/`r_wrist` frame 与 target site。
- `TJ_arm_control/include/tianji_qp_ik/mujoco_robot.hpp`、`TJ_arm_control/src/mujoco_robot.cpp` 及全部引用方：将硬编码 `tcp*` 末端 API 迁移为通用 `endEffector*`/`end_effector_*` API；SPD-VR 显式选择 wrist sites，其他工作流保留 TCP 默认。
- `TJ_arm_control/apps/run_qp_ik_viewer.cpp`：消费 wrist raw pose、执行每侧 alignment、生成独立 arm target validity/HOLD。
- `TJ_arm_control/include/tianji_qp_ik/arm_target_protocol.hpp`、`TJ_arm_control/src/arm_target_protocol.cpp`、`PICO_tracker/src/spd_vr/spd_vr/arm_target_protocol.py`、`PICO_tracker/src/spd_vr/test/test_arm_target_protocol.py`：arm target packet 改为左右独立 HOLD reason。
- `PICO_tracker/src/spd_vr/spd_vr/model_builder.py`、`PICO_tracker/src/spd_vr/generated/tianji_wuji2_spd.xml`、`PICO_tracker/src/spd_vr/generated/joint_manifest.yaml`：生成/验证 `l_wrist`、`r_wrist` target site 和 54-DoF manifest 元数据。
- `PICO_tracker/src/spd_vr/spd_vr/simulator.py`：接收新 arm frame、单侧 validity、pause freeze、输入 gate reset。
- `PICO_tracker/src/spd_vr/spd_vr/episode.py`、`PICO_tracker/src/spd_vr/spd_vr/runtime.py`：pause 与 simulator 状态同步，live ROS input，暂停时不推进 tick/recorder。
- `PICO_tracker/src/spd_vr/package.xml`、`PICO_tracker/scripts/start_spd_vr.sh`、`PICO_tracker/README.md`、`PICO_tracker/README.zh-CN.md`：live 启动、SDK 前置条件、旧输入链路清除。
- `TJ_arm_control/CMakeLists.txt` 及受影响测试/benchmark：加入 alignment 源和测试，更新 wrist API 调用。

---

### Task 1: Implement Official XRoboToolkit State Decoder

**Files:**
- Create: `PICO_tracker/src/pico_bridge/include/pico_bridge/xrobotoolkit_hand_decoder.hpp`
- Create: `PICO_tracker/src/pico_bridge/src/xrobotoolkit_hand_decoder.cpp`
- Create: `PICO_tracker/src/pico_bridge/test/fixtures/xrobotoolkit_hand_state.json`
- Create: `PICO_tracker/src/pico_bridge/test/test_xrobotoolkit_hand_decoder.cpp`
- Modify: `PICO_tracker/src/pico_bridge/CMakeLists.txt`
- Modify: `PICO_tracker/pixi.toml`

**Interfaces:**
- Produces `pico_bridge::XrJointPose = std::array<double, 7>`, `pico_bridge::XrHandSideSnapshot`, `pico_bridge::XrHandSnapshot`, `pico_bridge::XrHandDecodeResult`, and `decode_xrobotoolkit_state_json(std::string_view)`.
- `XrHandSideSnapshot` contains `std::array<XrJointPose, 26> joints`, `bool active`, `double scale`, `bool structurally_valid`, and `std::string rejection_reason`.
- `XrHandSnapshot` contains one `std::int64_t source_timestamp_ns` plus `left/right` side snapshots. `XrHandDecodeResult` contains fatal callback-level `bool valid`, `std::string rejection_reason`, and the snapshot; no ROS, SDK, or network types appear in this header.

- [ ] **Step 1: Add failing fixture tests for one atomic callback**

Use a fixture whose outer object has a string `value`, whose nested object has `timeStampNs`, `Hand.leftHand`, and `Hand.rightHand`; each hand has `scale`, `isActive`, and exactly 26 `HandJointLocations` entries with `p` strings in `x y z qx qy qz qw` order. Assert one decode result carries one timestamp, both active flags, both scales, and index 1 values without reordering. Add a mutated fixture with an invalid left quaternion and assert the left side becomes structurally invalid/inactive while the right side remains valid and unchanged. Register this test in `CMakeLists.txt` in the same step, before adding the missing decoder implementation.

```cpp
const auto result = decode_xrobotoolkit_state_json(readFixture());
ASSERT_TRUE(result.valid);
EXPECT_EQ(result.snapshot.source_timestamp_ns, 1724846400000000000LL);
EXPECT_EQ(result.snapshot.left.joints[1][0], 0.101);
EXPECT_EQ(result.snapshot.right.joints[1][6], 1.0);
EXPECT_TRUE(result.snapshot.left.active);
EXPECT_FALSE(result.snapshot.right.active);
EXPECT_DOUBLE_EQ(result.snapshot.left.scale, 0.98);

const auto one_bad_side = decode_xrobotoolkit_state_json(invalidLeftFixture());
ASSERT_TRUE(one_bad_side.valid);
EXPECT_FALSE(one_bad_side.snapshot.left.structurally_valid);
EXPECT_FALSE(one_bad_side.snapshot.left.active);
EXPECT_TRUE(one_bad_side.snapshot.right.structurally_valid);
```

- [ ] **Step 2: Run only the decoder test and verify the expected missing-symbol failure**

Run: `colcon build --packages-select pico_bridge --cmake-clean-cache`

Expected: FAIL while compiling `test_xrobotoolkit_hand_decoder` because the decoder header/target does not exist yet; do not run the whole workspace.

- [ ] **Step 3: Implement strict JSON parsing**

Parse the outer JSON, require `value` to be a JSON string, parse that string, require positive `timeStampNs`, and require a `Hand` object with at least one named side. Parse each side independently. A missing or malformed side gets 26 identity-quaternion poses, `active=false`, `scale=1.0`, `structurally_valid=false`, and a side rejection reason; it does not discard a valid opposite side. For every structurally present side require 26 joints, finite position/quaternion values, non-zero finite quaternion norm, and finite positive scale; normalize each quaternion before storing. Fatal callback reasons are `outer_json_malformed`, `nested_value_invalid`, `timestamp_invalid`, or `hand_missing`; side reasons are `hand_joint_count_invalid`, `hand_pose_non_finite`, `hand_quaternion_invalid`, `hand_scale_invalid`, or `hand_missing`.

```cpp
XrHandDecodeResult decode_xrobotoolkit_state_json(std::string_view json_text);
```

The parser must finish both side snapshots before returning. `result.valid` describes the callback envelope/timestamp; `side.structurally_valid` describes each hand. A caller can publish one valid side without ever exposing partially updated state or treating an identity placeholder as active.

- [ ] **Step 4: Add nlohmann-json to the Pixi environment and decoder-only CMake target**

Add the `nlohmann_json` dependency to `PICO_tracker/pixi.toml`; expose `xrobotoolkit_hand_decoder` as a static library linked to `nlohmann_json::nlohmann_json`; complete the already-added `test_xrobotoolkit_hand_decoder` linkage and include the fixture file through the existing test install rule.

- [ ] **Step 5: Build and run the focused decoder test**

Run: `colcon build --packages-select pico_bridge && colcon test --packages-select pico_bridge --ctest-args -R test_xrobotoolkit_hand_decoder --event-handlers console_direct+`

Expected: PASS for valid atomic data, missing-side inactive data, malformed outer/nested JSON, wrong joint count, non-finite pose, invalid quaternion, invalid scale, no-hand rejection, and one-side-invalid/opposite-side-valid preservation.

- [ ] **Step 6: Commit the decoder unit**

```bash
git add PICO_tracker/pixi.toml PICO_tracker/src/pico_bridge/CMakeLists.txt PICO_tracker/src/pico_bridge/include/pico_bridge/xrobotoolkit_hand_decoder.hpp PICO_tracker/src/pico_bridge/src/xrobotoolkit_hand_decoder.cpp PICO_tracker/src/pico_bridge/test/fixtures/xrobotoolkit_hand_state.json PICO_tracker/src/pico_bridge/test/test_xrobotoolkit_hand_decoder.cpp
git commit -m "feat: decode atomic XRoboToolkit hand state"
```

---

### Task 2: Add the XRoboToolkit SDK ROS Bridge

**Files:**
- Create: `PICO_tracker/src/pico_bridge/src/xrobotoolkit_pico_node.cpp`
- Modify: `PICO_tracker/src/pico_bridge/CMakeLists.txt`
- Modify: `PICO_tracker/src/pico_bridge/package.xml`
- Modify: `PICO_tracker/src/pico_bridge/launch/start_xrobotoolkit_pico.launch.py`
- Modify: `PICO_tracker/src/pico_bridge/test/test_launch_integration.py`

**Interfaces:**
- Produces executable `xrobotoolkit_pico_bridge` and ROS topic `/pico/hands` with existing `pico_bridge/msg/PicoHands`.
- Parameters: `hands_topic` default `/pico/hands`, `tracking_epoch_topic` default `/pico/tracking_epoch`, `tracking_epoch_status_topic` default `/pico/tracking_epoch/status`, `tracking_epoch_state_file` default `~/.config/pico_tracker/tracking_epoch`, and `frame_id` default `pico`.
- Consumes official C ABI `PXREAInit(void*, pfPXREAClientCallback, unsigned)` and `PXREADeinit()` from `PXREARobotSDK.h`; the callback mask includes `PXREAServerConnect`, `PXREAServerDisconnect`, and `PXREADeviceStateJson`.

- [ ] **Step 1: Add launch/build contract tests before the node**

Extend the launch integration test to assert `start_xrobotoolkit_pico.launch.py` names only `xrobotoolkit_pico_bridge` and `xrobotoolkit_wrist_relay`, passes `/pico/hands` and `/spd_vr/pause`, and contains no `pico_bridge_node`, `start_pico_driver`, `start_pico_m0`, `smpl`, `optical`, or `Manus` executable. Add a CMake configure assertion that `PICO_BUILD_XROBOTOOLKIT_BRIDGE=ON` requires `PXREARobotSDK.h` and a library instead of silently omitting the target.

- [ ] **Step 2: Run the launch test and capture the expected target-missing failure**

Run: `pytest -q PICO_tracker/src/pico_bridge/test/test_launch_integration.py -k xrobotoolkit`

Expected: FAIL because the new launch file and executable declaration do not exist yet.

- [ ] **Step 3: Implement SDK discovery with an explicit build gate**

In `CMakeLists.txt`, add `option(PICO_BUILD_XROBOTOOLKIT_BRIDGE "Build the official XRoboToolkit PC-Service bridge" OFF)`. When enabled, resolve `XRTOOLKIT_SDK_ROOT` (CMake cache or environment), `find_path(PXREA_INCLUDE_DIR PXREARobotSDK.h ...)`, and `find_library(PXREA_LIBRARY NAMES PXREARobotSDK ...)`; issue `message(FATAL_ERROR, ...)` when either is absent. Link the executable only to the discovered official library and the decoder/typesupport targets. Keep the decoder and relay core buildable without the proprietary SDK.

- [ ] **Step 4: Implement the SDK callback lifecycle and epoch publishing**

Create a node that reserves a durable epoch with `reserve_tracking_epoch`, initializes the SDK once, handles callback status on the SDK thread without polling, and deinitializes cleanly in the destructor. On each server reconnect reserve a new non-zero epoch and publish it transient-local; on disconnect publish a structured status and stop treating frames as live. On `PXREADeviceStateJson`, cast payload to `PXREADevStateJson*`, copy at most `sizeof(stateJson)` bytes using bounded length, call the pure decoder, and publish one ROS message only after the complete two-side snapshot exists.

```cpp
void onSdkCallback(void* context,
                   PXREAClientCallbackType type,
                   int error_code,
                   void* payload);
void publishSnapshot(const XrHandSnapshot& snapshot);
void publishStatus(std::string_view state, std::string_view reason,
                   int error_code);
```

Map `source_timestamp_ns` to `Header.stamp`, set `Header.frame_id` to `frame_id`, copy each side’s 26 `[x,y,z,qx,qy,qz,qw]` values, scales, `tracking_epoch`, and one monotonically increasing `sequence_id`. Publish `left_active/right_active` only when both the SDK `isActive` bit and that side’s `structurally_valid` bit are true. A fatal callback parser rejection publishes no frame; a single malformed side is published inactive with its side reason in diagnostics while the valid opposite side remains live. Identity placeholders are therefore never active control input.

- [ ] **Step 5: Register dependencies, installation, and executable**

Add `pico_bridge`’s generated interface dependency to the node, link `xrobotoolkit_hand_decoder`, `pico_tracking_epoch_store`, and the C++ ROS typesupport target, install the executable under `lib/pico_bridge`, and install the new launch file. Add `rclcpp`, `std_msgs`, `sensor_msgs`, `geometry_msgs`, and `builtin_interfaces` declarations already used by the package plus any missing `nlohmann_json` build declaration.

- [ ] **Step 6: Run the launch contract test without enabling the proprietary target**

Run: `pytest -q PICO_tracker/src/pico_bridge/test/test_launch_integration.py -k xrobotoolkit`

Expected: PASS for launch argument/node contract. Separately run configure with `-DPICO_BUILD_XROBOTOOLKIT_BRIDGE=ON` only when `PXREARobotSDK.h` and the official library are installed; expected result is either a real target or the documented fatal missing-SDK error, never a fake fallback.

- [ ] **Step 7: Commit the official bridge**

```bash
git add PICO_tracker/src/pico_bridge/CMakeLists.txt PICO_tracker/src/pico_bridge/package.xml PICO_tracker/src/pico_bridge/src/xrobotoolkit_pico_node.cpp PICO_tracker/src/pico_bridge/launch/start_xrobotoolkit_pico.launch.py PICO_tracker/src/pico_bridge/test/test_launch_integration.py
git commit -m "feat: add XRoboToolkit SDK ROS bridge"
```

---

### Task 3: Relay PICO Wrist Frames and Pause Reset Flags

**Files:**
- Create: `PICO_tracker/src/pico_bridge/include/pico_bridge/xrobotoolkit_wrist_relay_core.hpp`
- Create: `PICO_tracker/src/pico_bridge/src/xrobotoolkit_wrist_relay_core.cpp`
- Create: `PICO_tracker/src/pico_bridge/src/xrobotoolkit_wrist_relay_node.cpp`
- Create: `PICO_tracker/src/pico_bridge/test/test_xrobotoolkit_wrist_relay_core.cpp`
- Modify: `PICO_tracker/src/pico_bridge/include/pico_bridge/tianji_teleop_protocol.hpp`
- Modify: `PICO_tracker/src/pico_bridge/src/tianji_teleop_protocol.cpp`
- Modify: `TJ_arm_control/include/tianji_qp_ik/pico_teleop_protocol.hpp`
- Modify: `TJ_arm_control/src/pico_teleop_protocol.cpp`
- Modify: `PICO_tracker/src/pico_bridge/CMakeLists.txt`
- Modify: `PICO_tracker/src/pico_bridge/launch/start_xrobotoolkit_pico.launch.py`

**Interfaces:**
- `XrWristRelayInput` contains `std::uint64_t tracking_epoch`, `std::uint64_t sequence_id`, `std::int64_t source_timestamp_ns`, `Eigen::Isometry3d left_wrist/right_wrist`, and `bool left_active/right_active`; it is constructed from one ROS `PicoHands` message after index-1 extraction.
- `XrWristRelayCore::setPaused(bool)` suppresses output while paused and sets a one-shot post-resume reset bit. The core also tracks the last emitted epoch; its first eligible frame and every epoch change carry the same reset bit.
- `XrWristRelayCore::ingest(const XrWristRelayInput&)` returns `std::optional<TianjiTeleopWireFrame>` carrying the source epoch/sequence/timestamp, raw Wrist poses, active bits, `wrist_pose_input=true`, and `wrist_alignment_reset=true` only when no baseline exists, the epoch changed, or the first eligible frame arrives after resume.
- New TJVR flags: `kTianjiTeleopWristPoseInputFlag = 1U << 9`, `kTianjiTeleopWristAlignmentResetFlag = 1U << 10`, `kTianjiTeleopLeftWristActiveFlag = 1U << 11`, `kTianjiTeleopRightWristActiveFlag = 1U << 12`.
- `TianjiTeleopWireFrame` and decoded `PicoTeleopFrame` both gain `bool wrist_pose_input`, `bool wrist_alignment_reset`, `bool left_wrist_active`, and `bool right_wrist_active`.

- [ ] **Step 1: Add wire round-trip and relay-core failing tests**

Construct a valid `XrWristRelayInput` with different left/right poses and assert relay output carries those exact poses, source metadata, and both active bits. Assert the first eligible frame carries reset; paused input returns no frame; the first resumed frame has reset; later same-epoch frames do not; an epoch change sets reset again; left inactive does not invalidate right. Extend the existing TJVR protocol test to encode/decode all four new flags and reject an invalid wrist packet with no active side. Register the new relay-core test in `PICO_tracker/src/pico_bridge/CMakeLists.txt` in this step so the pre-implementation build compiles the failing contract.

```cpp
XrWristRelayInput input;
input.tracking_epoch = 7;
input.sequence_id = 11;
input.source_timestamp_ns = 1724846400000000000LL;
input.left_active = true;
input.right_active = true;
input.left_wrist.translation().x() = 0.101;
input.right_wrist.translation().z() = -0.202;
core.setPaused(false);
const auto first = core.ingest(input);
ASSERT_TRUE(first.has_value());
EXPECT_TRUE(first->wrist_pose_input);
EXPECT_TRUE(first->wrist_alignment_reset);
EXPECT_NEAR(first->left_target.translation().x(), 0.101, 1e-12);
EXPECT_NEAR(first->right_target.translation().z(), -0.202, 1e-12);
```

- [ ] **Step 2: Run the two focused C++ tests to verify missing interfaces**

Run: `colcon build --packages-select pico_bridge`

Expected: FAIL while compiling the registered relay test because the relay core and new protocol fields are absent.

- [ ] **Step 3: Extend the TJVR semantic flags without changing packet size**

Keep version 4 and 656 bytes, use the existing flags word at offset 40, and extend the known-flag mask from bits 0–7 to `0x1eff` (bits 0–7 plus 9–12; bit 8 remains reserved and rejected). Serialize/deserialize wrist input/reset/left-active/right-active. Preserve old non-wrist v4 frames with `wrist_pose_input=false`; new relay frames must set `wrist_pose_input=true` and at least one active bit. Modify `PicoTeleopStreamGate` jump checks so inactive sides are excluded from position/orientation jump comparisons.

- [ ] **Step 4: Implement the pure wrist relay core**

Validate positive timestamp, epoch and sequence plus each active Wrist pose’s finite position and non-zero finite quaternion. Build `TianjiTeleopWireFrame` with `left_target`/`right_target` containing raw PICO Wrist poses, source sequence unchanged, `left_wrist_active/right_wrist_active`, and no shoulder/skeleton/direction claims. Reject a frame with neither side active. Start with `reset_pending_=true`; set it again on an epoch change and on the true-to-false pause transition. While paused return no frame; on the next emitted frame set `wrist_alignment_reset`, update `last_emitted_epoch_`, and clear the pending bit.

- [ ] **Step 5: Implement the ROS UDP relay node**

Subscribe to `pico_bridge::msg::PicoHands` at `/pico/hands` with SensorDataQoS and `std_msgs::msg::Bool` at `/spd_vr/pause`. Convert only `left_joints[1]` and `right_joints[1]`, copy the message epoch/sequence/header stamp, and send exactly one encoded packet per accepted ROS message to `destination_address`/`destination_port` (defaults `127.0.0.1`/`15000`). Publish JSON diagnostics with received, paused, emitted, reset, malformed, and UDP send failure counts.

- [ ] **Step 6: Wire launch and register focused tests**

Add `xrobotoolkit_wrist_relay` to CMake, link the pure relay/protocol libraries and ROS typesupport, install it, and make `start_xrobotoolkit_pico.launch.py` start both nodes. The launch must expose `hands_topic`, `pause_topic`, `destination_address`, and `destination_port`; it must not start the old skeleton/status bridge.

- [ ] **Step 7: Run focused relay/protocol tests**

Run: `colcon test --packages-select pico_bridge --ctest-args -R 'test_xrobotoolkit_wrist_relay_core|test_tianji_teleop_protocol' --event-handlers console_direct+`

Expected: PASS for raw index-1 mapping, source metadata, active-side independence, pause suppression, one-shot reset, CRC round-trip, unknown-flag rejection, and old non-wrist frame compatibility.

- [ ] **Step 8: Commit the wrist relay**

```bash
git add PICO_tracker/src/pico_bridge/include/pico_bridge/xrobotoolkit_wrist_relay_core.hpp PICO_tracker/src/pico_bridge/src/xrobotoolkit_wrist_relay_core.cpp PICO_tracker/src/pico_bridge/src/xrobotoolkit_wrist_relay_node.cpp PICO_tracker/src/pico_bridge/test/test_xrobotoolkit_wrist_relay_core.cpp PICO_tracker/src/pico_bridge/include/pico_bridge/tianji_teleop_protocol.hpp PICO_tracker/src/pico_bridge/src/tianji_teleop_protocol.cpp PICO_tracker/src/pico_bridge/CMakeLists.txt PICO_tracker/src/pico_bridge/launch/start_xrobotoolkit_pico.launch.py TJ_arm_control/include/tianji_qp_ik/pico_teleop_protocol.hpp TJ_arm_control/src/pico_teleop_protocol.cpp
git commit -m "feat: relay XRoboToolkit wrist poses"
```

---

### Task 4: Make Tianji IK Endpoints Explicitly Configurable as `l_wrist`/`r_wrist`

**Files:**
- Modify: `TJ_arm_control/models/marvin_m6_qp_pico_fast.xml`
- Modify: `TJ_arm_control/models/marvin_m6_qp_test.xml`
- Modify: `PICO_tracker/src/spd_vr/spd_vr/model_builder.py`
- Create: `PICO_tracker/src/spd_vr/test/test_model_builder.py`
- Modify: `TJ_arm_control/include/tianji_qp_ik/mujoco_robot.hpp`
- Modify: `TJ_arm_control/src/mujoco_robot.cpp`
- Modify: `TJ_arm_control/apps/benchmark_cartesian_frf.cpp`
- Modify: `TJ_arm_control/apps/benchmark_cartesian_otg.cpp`
- Modify: `TJ_arm_control/apps/benchmark_hierarchical_ik.cpp`
- Modify: `TJ_arm_control/apps/benchmark_solver.cpp`
- Modify: `TJ_arm_control/apps/inspect_model.cpp`
- Modify: `TJ_arm_control/apps/run_qp_ik_viewer.cpp`
- Modify: `TJ_arm_control/src/acceleration_controller.cpp`
- Modify: `TJ_arm_control/src/benchmark_dataset.cpp`
- Modify: `TJ_arm_control/src/controller.cpp`
- Modify: `TJ_arm_control/src/iterative_pose_dls.cpp`
- Modify: `TJ_arm_control/src/pinocchio_arm_kinematics.cpp`
- Modify: `TJ_arm_control/src/spark_guidance.cpp`
- Modify: `TJ_arm_control/src/spark_upper_qpoases_ik.cpp`
- Modify: `TJ_arm_control/tests/test_acceleration_controller.cpp`
- Modify: `TJ_arm_control/tests/test_benchmark_dataset.cpp`
- Modify: `TJ_arm_control/tests/test_controller.cpp`
- Modify: `TJ_arm_control/tests/test_hierarchical_controller.cpp`
- Modify: `TJ_arm_control/tests/test_iterative_pose_dls.cpp`
- Modify: `TJ_arm_control/tests/test_jacobian.cpp`
- Modify: `TJ_arm_control/tests/test_jdot_qdot.cpp`
- Modify: `TJ_arm_control/tests/test_mujoco_robot.cpp`
- Modify: `TJ_arm_control/tests/test_otg_controller.cpp`
- Modify: `TJ_arm_control/tests/test_pinocchio_arm_kinematics.cpp`
- Modify: `TJ_arm_control/tests/test_spark_guidance.cpp`
- Modify: `TJ_arm_control/tests/test_spark_upper_qpoases_ik.cpp`
- Modify: `TJ_arm_control/tests/test_trajectory_regression.cpp`
- Modify: `PICO_tracker/src/spd_vr/generated/tianji_wuji2_spd.xml`
- Modify: `PICO_tracker/src/spd_vr/generated/joint_manifest.yaml`
- Modify: `TJ_arm_control/CMakeLists.txt`

**Interfaces:**
- `EndEffectorSiteNames { std::string left{"tcp_L"}; std::string right{"tcp_R"}; }` is an explicit `MujocoRobot` constructor argument; legacy callers keep their existing TCP sites, while SPD-VR passes `{"l_wrist_target", "r_wrist_target"}`.
- `MujocoRobot::endEffectorPose(ArmSide)`, `endEffectorJacobianWorld(ArmSide)`, and `endEffectorJacobianDotTimesVelocityWorld(ArmSide, const Vec7&, const Vec7&)` replace endpoint APIs whose names hard-code TCP semantics.
- `ArmMapping` exposes `end_effector_site_name`, `end_effector_site_id`, and `end_effector_body_id`; `ArmKinematicSample` exposes `end_effector_pose` and `end_effector_jacobian`.
- The arm model contains fixed bodies named `l_wrist` and `r_wrist`, with sites `l_wrist_target` and `r_wrist_target` at each body origin. The SPD-VR viewer must pass those exact site names; it may not rely on the constructor’s legacy TCP defaults.

- [ ] **Step 1: Use LSP references before renaming exported endpoint symbols**

Run symbol-aware references for each exported symbol from `mujoco_robot.hpp` and migrate every caller listed under **Files**. Do not use repository-wide text replacement for cross-file symbols. Rename only the generic kinematics API; retain `tcp_L`/`tcp_R` model sites for non-SPD tools, but require the SPD launch/viewer command to select `l_wrist_target`/`r_wrist_target` explicitly.

- [ ] **Step 2: Add failing model mapping assertions**

Extend `test_mujoco_robot.cpp` to load `models/marvin_m6_qp_test.xml` twice: once with default TCP names to preserve the existing model contract, and once with `EndEffectorSiteNames{"l_wrist_target", "r_wrist_target"}`. For the wrist instance assert `nq==nv==14`, resolve body IDs `l_wrist`/`r_wrist`, resolve sites `l_wrist_target`/`r_wrist_target`, require each site’s `site_bodyid` to equal the matching wrist body, and compare `endEffectorPose` at the home state with the fixed URDF chain transform within `1e-8 m`/`1e-8 rad`. In new `test_model_builder.py`, build into a temporary directory and assert the generated model has both wrist bodies/sites, the manifest names all four, and the model has exactly 54 actuators.

- [ ] **Step 3: Run the focused model tests and verify the missing wrist-frame failure**

Run: `ctest --test-dir TJ_arm_control/build -R test_mujoco_robot --output-on-failure && pytest -q PICO_tracker/src/spd_vr/test/test_model_builder.py`

Expected: FAIL because the arm-only models do not yet expose `l_wrist_target`/`r_wrist_target` and the generic endpoint API does not exist.

- [ ] **Step 4: Add URDF-derived fixed wrist frames to both arm-only XML models**

Under `Link7_L`, add:

```xml
<body name="l_wrist" pos="0.003000180668 -0.131499815111 0.000300120254" quat="0.707106781182 -0.707106781187 -0.000002597349 0">
  <site name="l_wrist_target" pos="0 0 0" quat="1 0 0 0" size="0.008" rgba="1 0.2 0.2 1"/>
</body>
```

Under `Link7_R`, add:

```xml
<body name="r_wrist" pos="-0.002999866847 -0.131500011019 0.000250148981" quat="-0.000005797922 0.000003200574 0.707106781184 0.707106781158">
  <site name="r_wrist_target" pos="0 0 0" quat="1 0 0 0" size="0.008" rgba="0.2 0.4 1 1"/>
</body>
```

Retain target mocap bodies and legacy `tcp_L`/`tcp_R` sites for other repository workflows. SPD-VR acceptance is based only on the explicit wrist site selection.

- [ ] **Step 5: Generalize the MuJoCo endpoint API and migrate all callers**

Apply LSP-guided renames:

```text
tcpPose                          -> endEffectorPose
tcpJacobianWorld                 -> endEffectorJacobianWorld
tcpJacobianDotTimesVelocityWorld -> endEffectorJacobianDotTimesVelocityWorld
tcp_pose                         -> end_effector_pose
tcp_jacobian                     -> end_effector_jacobian
tcp_site_id                      -> end_effector_site_id
tcp_body_id                      -> end_effector_body_id
```

Resolve sites from the constructor’s `EndEffectorSiteNames`, reject missing sites, and keep the same Jacobian column extraction over each arm’s 7 DOFs. Generic controller, solver, benchmark, Pinocchio and test code must consume `end_effector_*` fields; Pinocchio keeps its existing fixed TCP frame as its default end effector because the new SPD command forces `hierarchical_qp` and does not instantiate SPARK/Pinocchio guidance. Update new SPD diagnostics to say `wrist_actual`; retain existing telemetry column spelling only where changing it would break an existing recorded schema.

- [ ] **Step 6: Make the generated combined model expose target sites at each hand root**

In `_append_hand`, after cloning the hand root body and applying its URDF-derived mount transform, insert one identity site named `l_wrist_target` or `r_wrist_target` if absent. In `_manifest_from_model`, add `wrist_targets: {left_body: l_wrist, left_site: l_wrist_target, right_body: r_wrist, right_site: r_wrist_target}` and validate all four names. Regenerate XML/manifest so generated artifacts, source hashes, and 54-DoF checks remain synchronized.

Run: `PYTHONPATH=PICO_tracker/src/spd_vr pixi run python -m spd_vr.model_builder --output-dir PICO_tracker/src/spd_vr/generated`

- [ ] **Step 7: Run focused model, Jacobian, and endpoint tests**

Run: `ctest --test-dir TJ_arm_control/build -R 'test_mujoco_robot|test_jacobian|test_jdot_qdot|test_pinocchio_arm_kinematics' --output-on-failure && pytest -q PICO_tracker/src/spd_vr/test/test_model_builder.py`

Expected: PASS for legacy TCP selection and explicit `l_wrist`/`r_wrist` selection; wrist Jacobians use the hand-root body-origin sites and the arm-only model remains 14 DoF.

- [ ] **Step 8: Commit the endpoint selection**

```bash
git add TJ_arm_control/models/marvin_m6_qp_pico_fast.xml TJ_arm_control/models/marvin_m6_qp_test.xml TJ_arm_control/include/tianji_qp_ik/mujoco_robot.hpp TJ_arm_control/src/mujoco_robot.cpp TJ_arm_control/apps TJ_arm_control/src TJ_arm_control/tests TJ_arm_control/CMakeLists.txt PICO_tracker/src/spd_vr/spd_vr/model_builder.py PICO_tracker/src/spd_vr/test/test_model_builder.py PICO_tracker/src/spd_vr/generated/tianji_wuji2_spd.xml PICO_tracker/src/spd_vr/generated/joint_manifest.yaml
git commit -m "refactor: select Tianji end-effector frames explicitly"
```

---

### Task 5: Add Per-Side Wrist Alignment to the QP Viewer

**Files:**
- Create: `TJ_arm_control/include/tianji_qp_ik/pico_wrist_alignment.hpp`
- Create: `TJ_arm_control/src/pico_wrist_alignment.cpp`
- Create: `TJ_arm_control/tests/test_pico_wrist_alignment.cpp`
- Modify: `TJ_arm_control/apps/run_qp_ik_viewer.cpp`
- Modify: `TJ_arm_control/include/tianji_qp_ik/pico_teleop_session.hpp`
- Modify: `TJ_arm_control/src/pico_teleop_session.cpp`
- Modify: `TJ_arm_control/CMakeLists.txt`

**Interfaces:**
- `WristAlignmentConfig { std::size_t stable_frames{10}; double max_stable_position_step_m{0.02}; double max_stable_orientation_step_rad{0.15}; double position_scale{1.0}; }`.
- `WristAlignmentSideResult { bool valid; bool aligned; Pose target; std::string hold_reason; }`.
- `WristAlignmentUpdate { WristAlignmentSideResult left; WristAlignmentSideResult right; }`.
- `PicoWristAlignment::update(const PicoTeleopFrame&, const Pose& current_left, const Pose& current_right)`, `reset()`, and `resetSide(ArmSide)`.
- Viewer CLI adds `--left-end-effector-site` (default `tcp_L`), `--right-end-effector-site` (default `tcp_R`), `--pico-wrist-input`, `--wrist-stable-frames` (default `10`), `--wrist-max-position-step` (default `0.02`), `--wrist-max-orientation-step` (default `0.15`), and `--wrist-position-scale` (default `1.0`). `--pico-wrist-input` requires the two wrist site names to be exactly `l_wrist_target` and `r_wrist_target`.

- [ ] **Step 1: Add failing alignment tests**

Cover: first active frame remains pending, the tenth stable frame establishes a reference with target equal to the current robot wrist, a later translation scales only the relative translation, a later rotation composes through SE(3), inactive/invalid left clears only left without invalidating right, a global alignment reset or epoch discontinuity clears both references, and post-pause reset cannot reuse the old baseline. Register `test_pico_wrist_alignment` in `TJ_arm_control/CMakeLists.txt` in this step so the next build compiles the failing contract.

```cpp
PicoWristAlignment alignment({10, 0.02, 0.15, 0.5});
for (std::size_t i = 0; i < 9; ++i)
  EXPECT_FALSE(alignment.update(frameAt(i), robot_left, robot_right).left.valid);
const auto established = alignment.update(frameAt(9), robot_left, robot_right);
EXPECT_TRUE(established.left.valid);
EXPECT_NEAR(established.left.target.position.x(), robot_left.position.x(), 1e-12);
```

- [ ] **Step 2: Run the focused alignment test to verify missing-symbol failure**

Run: `cmake --build TJ_arm_control/build --target test_pico_wrist_alignment -j2`

Expected: FAIL while compiling the registered test because the class and source are not present.

- [ ] **Step 3: Implement per-side stable-window and SE(3) math**

For each side require active, finite position, proper normalized quaternion, and continuity from the previous candidate. Increment a side-local count only when the position/orientation step is within the config thresholds; otherwise reset that side’s candidate count and reference. At establishment save `pico_reference` and the current `robot_reference` from `endEffectorPose` on a `MujocoRobot` configured with wrist sites. For later frames compute:

```text
delta = inverse(T_pico_reference) * T_pico_current
delta.translation *= position_scale
target = T_robot_reference * delta
```

Return `valid=false` with `stable_window`, `inactive`, `invalid_pose`, or `alignment_reset` while pending; never return a home/zero target. `reset()` clears both sides; `resetSide()` clears only one.

- [ ] **Step 4: Integrate wrist raw frames into the viewer control loop**

Add the CLI fields above, validate finite positive thresholds/scale and non-zero stable frame count, and construct `MujocoRobot` with the selected `EndEffectorSiteNames`. When `PicoTeleopFrame.wrist_pose_input` is true, call `PicoWristAlignment` with `robot.endEffectorPose(kLeft/kRight)`. In `--pico-wrist-input` mode reject legacy non-wrist TJVR frames; on `wrist_alignment_reset`, reset both sides before processing. Keep `PicoTeleopSession` freshness/epoch/sequence gates; commit a structurally valid frame even while the stable window is pending so the next sequence can be consumed. Update `TargetManager` per side with `setManualTarget`; do not call the bilateral setter when one side is pending/inactive. On a side HOLD, retain its last target and mark only that side stale/invalid.

- [ ] **Step 5: Emit independent arm validity and reset-aware diagnostics**

Compute `left_commit_accepted` and `right_commit_accepted` independently. Emit `ArmTargetFrame` with each side’s q/qdot and validity; use `kPaused`, `kInputStale`, or `kSolverFailure` per side. Record alignment count, aligned flag, and hold reason in viewer telemetry. `l_wrist`/`r_wrist` actual pose/error fields must use `robot.endEffectorPose` and `armKinematicsAt(...).end_effector_pose`.

- [ ] **Step 6: Build the viewer and run alignment plus existing viewer unit tests**

Run: `cmake --build TJ_arm_control/build --target tianji_qp_ik_viewer test_pico_wrist_alignment -j2`

Run: `ctest --test-dir TJ_arm_control/build -R 'test_pico_wrist_alignment|test_pico_teleop_session|test_pico_udp_receiver' --output-on-failure`

Expected: PASS for stable per-side alignment, no jump at baseline, post-reset reacquisition, and freshness/sequence gates.

- [ ] **Step 7: Commit wrist alignment integration**

```bash
git add TJ_arm_control/include/tianji_qp_ik/pico_wrist_alignment.hpp TJ_arm_control/src/pico_wrist_alignment.cpp TJ_arm_control/tests/test_pico_wrist_alignment.cpp TJ_arm_control/apps/run_qp_ik_viewer.cpp TJ_arm_control/include/tianji_qp_ik/pico_teleop_session.hpp TJ_arm_control/src/pico_teleop_session.cpp TJ_arm_control/CMakeLists.txt
git commit -m "feat: align PICO wrist poses per arm"
```

---

### Task 6: Version Arm Target Wire Contract and Simulator HOLD Semantics

**Files:**
- Modify: `TJ_arm_control/include/tianji_qp_ik/arm_target_protocol.hpp`
- Modify: `TJ_arm_control/src/arm_target_protocol.cpp`
- Modify: `TJ_arm_control/tests/test_arm_target_protocol.cpp`
- Modify: `PICO_tracker/src/spd_vr/spd_vr/arm_target_protocol.py`
- Modify: `PICO_tracker/src/spd_vr/test/test_arm_target_protocol.py`
- Modify: `PICO_tracker/src/spd_vr/spd_vr/simulator.py`
- Modify: `PICO_tracker/src/spd_vr/test/test_simulator.py`
- Modify: `TJ_arm_control/apps/run_qp_ik_viewer.cpp`

**Interfaces:**
- `ArmTargetFrame` stores `left_hold_reason` and `right_hold_reason`; packet size remains 272 bytes, version changes from 1 to 2, reason bytes are at offsets 41 and 42, and reserved byte 43 must remain zero.
- `ArmSnapshot` stores `left_hold_reason` and `right_hold_reason`; `UnifiedSimulator.set_paused(bool)` freezes/unfreezes the plant and resets input gates.
- Validity invariant: a valid side has reason `NONE`; an invalid side has a non-`NONE` reason. Each side can be valid while the other is HOLD.

- [ ] **Step 1: Add failing cross-language protocol tests**

Update the Python and C++ fixtures to encode left-valid/right-stale, left-solver-failure/right-valid, paused-both, CRC failure, wrong version, and non-zero reserved cases. Assert C++-encoded bytes decode to the same per-side reasons in Python and Python-encoded bytes decode to the same per-side reasons in C++.

```python
frame = ArmTargetFrame(
    sequence=2, tracking_epoch=3, source_timestamp_ns=4,
    control_timestamp_ns=5, valid_mask=1,
    left_hold_reason=ArmTargetHoldReason.NONE,
    right_hold_reason=ArmTargetHoldReason.INPUT_STALE,
    left_q=(0.0,) * 7, right_q=(0.0,) * 7,
    left_qdot=(0.0,) * 7, right_qdot=(0.0,) * 7,
)
assert decode_packet(encode_packet(frame)).right_hold_reason is ArmTargetHoldReason.INPUT_STALE
```

- [ ] **Step 2: Run focused protocol tests and verify old-field failure**

Run: `pytest -q PICO_tracker/src/spd_vr/test/test_arm_target_protocol.py`

Expected: FAIL because the current frame has one shared `hold_reason` and version 1 layout.

- [ ] **Step 3: Implement v2 packet encode/decode in both languages**

Change the packet version to 2, parse two reason bytes, reject `NONE` on an invalid side and non-`NONE` on a valid side, keep all q/qdot offsets and CRC rules unchanged, and remove the Python `hold_reason` field rather than adding a compatibility alias. Update `ArmTargetUdpOutput::send` to accept left/right reasons and populate both bytes.

- [ ] **Step 4: Implement simulator per-side application and pause gate**

In `UnifiedSimulator.on_arm_target`, validate and apply each side independently; a bad left vector marks only left invalid and retains the previous left q, while a valid right q is accepted. Extend `on_pico_hands(frame, *, now_ns: int | None = None)` to store receiver-local monotonic arrival time. At every `step()`, make arm and hand validity stale after 50 ms without a new accepted frame while retaining the last q target; do not keep reporting an old target as live. In `on_pico_hands`, reject new control frames while `paused` but preserve callback counters; call `reset_filter` and reset both arm/hand sequence gates in `set_paused(True)`. Make `step()` return the current tick/time without incrementing, applying targets, enqueueing recorder/camera work, or calling `mj_step` while paused. Set both snapshots to `PAUSED` on entry and `INPUT_STALE` on resume. Maintain `_resume_gate_mask=0b11`; clear one bit only after that side receives a valid post-resume arm target from the re-aligned viewer, and suppress that side’s Wuji2 hand application/validity until its bit clears.

- [ ] **Step 5: Add simulator pause and one-sided behavior tests**

Load `PICO_tracker/src/spd_vr/generated/tianji_wuji2_spd.xml` and its manifest in the simulator test. Assert a left-invalid/right-valid arm packet changes only right q; assert arm and hand validity become stale after 50 ms without new input while q targets hold; assert `set_paused(True)` keeps `tick`, `data.time`, qpos, recorder submissions, and camera queue unchanged across repeated `step()` calls. Send an out-of-manifest-range hand result and assert only that side becomes `invalid` without clipping or changing its previous 20 actuator targets. Send a valid left-hand-only result and assert only the manifest’s 20 left-hand actuators change; no arm actuator or fixed `l_wrist`/`r_wrist` root mapping is overwritten. After resume, send hand frames before arm re-alignment and assert neither hand applies; then send a right-valid/left-invalid arm target and assert only right hand/arm resume while left remains HOLD.

- [ ] **Step 6: Run focused Python/C++ protocol and simulator tests**

Run: `pytest -q PICO_tracker/src/spd_vr/test/test_arm_target_protocol.py PICO_tracker/src/spd_vr/test/test_simulator.py`

Run: `ctest --test-dir TJ_arm_control/build -R test_arm_target_protocol --output-on-failure`

Expected: PASS with the v2 packet and independent HOLD behavior.

- [ ] **Step 7: Commit the simulator contract**

```bash
git add TJ_arm_control/include/tianji_qp_ik/arm_target_protocol.hpp TJ_arm_control/src/arm_target_protocol.cpp TJ_arm_control/tests/test_arm_target_protocol.cpp TJ_arm_control/apps/run_qp_ik_viewer.cpp PICO_tracker/src/spd_vr/spd_vr/arm_target_protocol.py PICO_tracker/src/spd_vr/test/test_arm_target_protocol.py PICO_tracker/src/spd_vr/spd_vr/simulator.py PICO_tracker/src/spd_vr/test/test_simulator.py
git commit -m "feat: preserve arm validity per side"
```

---

### Task 7: Wire Live ROS Input, Episode Pause, and 480 Hz Runtime

**Files:**
- Create: `PICO_tracker/src/spd_vr/spd_vr/ros_input.py`
- Create: `PICO_tracker/src/spd_vr/test/test_retarget_pair.py`
- Modify: `PICO_tracker/src/spd_vr/spd_vr/runtime.py`
- Modify: `PICO_tracker/src/spd_vr/spd_vr/episode.py`
- Modify: `PICO_tracker/src/spd_vr/test/test_runtime.py`
- Modify: `PICO_tracker/src/spd_vr/test/test_episode.py`
- Modify: `PICO_tracker/src/spd_vr/package.xml`
- Modify: `PICO_tracker/src/spd_vr/setup.py`

**Interfaces:**
- `LiveInputMailbox(hands_topic: str, pause_topic: str)` creates a lazy `rclpy` node; `spin_once(timeout_sec: float)`, `take_latest_hands()`, `take_episode_commands()`, and `close()` are the only runtime-facing methods.
- `run_runtime(..., mock: bool = False, hands_topic: str = "/pico/hands", pause_topic: str = "/spd_vr/pause", arm_bind_host: str = "127.0.0.1", arm_bind_port: int = 15100, wrist_position_scale: float = 1.0, wrist_stable_frames: int = 10, wrist_max_position_step_m: float = 0.02, wrist_max_orientation_step_rad: float = 0.15)` uses live ROS input unless `mock=True`.
- `EpisodeController(..., run_metadata: Mapping[str, Any] | None = None)` deep-copies run metadata into `task_manifest["teleop"]`; `_pause/_resume` calls `simulator.set_paused(True/False)` and preserves existing command return strings and checkpoint/revert/skip semantics.

- [ ] **Step 1: Add failing tests for live gating, retargeting, and pause freeze**

Test the mailbox with a fake ROS message object and latest-only replacement; test a `std_msgs/Bool` transition enqueues exactly one pause/resume command per edge, clears the control mailbox on both edges, and ignores hands received while paused except for health counters. Test `run_runtime(mock=True)` still produces a valid episode. Test `EpisodeController` with a simulator spy: after pause, `step()` is never called by the loop, recorder robot/camera appends stop, and after resume the controller returns `resumed`. Retain explicit assertions that checkpoint succeeds without hand-object contact, is rejected during contact, and that revert/skip behavior is unchanged. Assert the episode manifest stores `wrist_position_scale`, stable-window count/thresholds, and `left_site/right_site`.

In `test_retarget_pair.py`, inject two deterministic fake retargeters. Verify `PICO_TO_MEDIAPIPE` selects `[1,2,3,4,5,7,8,9,10,12,13,14,15,17,18,19,20,22,23,24,25]`, each active side emits exactly 20 outputs, and an inactive/failed left side does not block a valid right output.

```python
controller.enqueue(EpisodeCommandType.PAUSE)
assert controller.process_one() == "paused"
assert simulator.paused is True
assert simulator.tick == paused_tick
```

- [ ] **Step 2: Run focused runtime/episode tests to verify missing live interfaces**

Run: `pytest -q PICO_tracker/src/spd_vr/test/test_runtime.py PICO_tracker/src/spd_vr/test/test_episode.py PICO_tracker/src/spd_vr/test/test_retarget_pair.py`

Expected: FAIL because `ros_input.py`, `set_paused`, the live runtime path, and the new test contracts are not implemented.

- [ ] **Step 3: Implement the ROS latest-only mailbox**

Import `rclpy` and generated `pico_bridge.msg.PicoHands` only when live mode is requested. Subscribe to `/pico/hands` with SensorDataQoS and copy the latest message under a lock only when the mailbox is not paused; still count callbacks for health diagnostics while paused. Subscribe to `/spd_vr/pause` with reliable QoS. The first pause sample seeds state without generating a command, so an initial `false` never creates a spurious resume; later boolean edges enqueue exactly one `PAUSE`/`RESUME`. Clear the latest control frame on both real edges so a frame captured during pause cannot be consumed after resume. `spin_once()` is called by the runtime loop; no callback performs MuJoCo mutation. If `rclpy` or `pico_bridge` is unavailable, raise an explicit live-mode error naming the missing dependency.

- [ ] **Step 4: Connect EpisodeController pause to UnifiedSimulator**

In `_pause`, call `simulator.set_paused(True)` before transitioning to `PAUSED` and incrementing `state_epoch`. In `_resume`, call `simulator.set_paused(False)`, transition to `RECORDING`, increment `state_epoch`, and return `resumed`. Extend the constructor with `run_metadata`; in `_start`, merge a deep copy under `self._manifest["teleop"]` after `scene_result.manifest()`. Do not finish/discard a paused episode implicitly; preserve `_finish`, `_skip`, contact-free checkpoint, and restore behavior.

- [ ] **Step 5: Replace the live runtime rejection with ROS/arm-UDP wiring**

For `mock=False`, construct `WujiRetargetPair(config/wuji2_pico_left.yaml, config/wuji2_pico_right.yaml)`, pass it to `UnifiedSimulator`, instantiate `LiveInputMailbox`, and start `simulator.start_arm_udp(arm_bind_host, arm_bind_port)`. Consume at most the latest ROS message per loop, convert it once through `PicoHandsInput`, call `simulator.on_pico_hands(frame, now_ns=time.monotonic_ns())`, and call `recorder.append_hands` exactly once with the frame’s original SDK `timestamp_ns`, sequence, epoch, active flags, scales, and both 26x7 arrays.

Call `spin_once(0.0)` every loop and process mailbox commands before input. While `controller.state` is `PAUSED`, sleep for 1 ms and do not call hand retargeting, `simulator.step`, recorder append, or camera drain. Replace the fixed `for tick in range(...)` loop with `while simulator.sim_time_ns < duration_s * 1e9`, so paused wall time does not consume requested simulation duration. When recording, run 480 physics ticks per simulated second and leave arm cadence to the 200 Hz UDP snapshot gate. Keep the existing synthetic camera provider only for recorder schema compatibility; it is not an input to Wrist alignment.

Pass this exact metadata to `EpisodeController`:

```python
{
    "input": "xrobotoolkit_pico_hands",
    "left_site": "l_wrist_target",
    "right_site": "r_wrist_target",
    "wrist_position_scale": wrist_position_scale,
    "wrist_stable_frames": wrist_stable_frames,
    "wrist_max_position_step_m": wrist_max_position_step_m,
    "wrist_max_orientation_step_rad": wrist_max_orientation_step_rad,
}
```

- [ ] **Step 6: Keep mock mode deterministic and update CLI**

Retain `_mock_hand_frame` and `_arm_frame` only behind `--mock`; set live mode as the default path and add `--hands-topic`, `--pause-topic`, `--arm-bind-host`, `--arm-bind-port`, `--wrist-position-scale`, `--wrist-stable-frames`, `--wrist-max-position-step`, and `--wrist-max-orientation-step`. Validate all scale/threshold values as finite positive and stable frames as non-zero. The live path must never call `_mock_hand_frame`, `_arm_frame`, or the old TCP PICO node.

- [ ] **Step 7: Declare Python package dependencies and run focused tests**

Add `pico_bridge`, `rclpy`, `std_msgs`, `geometry_msgs`, and `mujoco` runtime dependencies where missing; retain NumPy/h5py/yaml. Run:

```bash
pytest -q PICO_tracker/src/spd_vr/test/test_runtime.py PICO_tracker/src/spd_vr/test/test_episode.py PICO_tracker/src/spd_vr/test/test_retarget_pair.py PICO_tracker/src/spd_vr/test/test_arm_target_protocol.py
```

Expected: PASS for deterministic mock episode, live dependency error wording, latest-only ROS input, exact 26-to-21/20-DoF retargeting, pause freeze, resume reacquisition hook, metadata, and recorder state transitions.

- [ ] **Step 8: Commit live runtime wiring**

```bash
git add PICO_tracker/src/spd_vr/spd_vr/ros_input.py PICO_tracker/src/spd_vr/spd_vr/runtime.py PICO_tracker/src/spd_vr/spd_vr/episode.py PICO_tracker/src/spd_vr/test/test_runtime.py PICO_tracker/src/spd_vr/test/test_episode.py PICO_tracker/src/spd_vr/test/test_retarget_pair.py PICO_tracker/src/spd_vr/package.xml PICO_tracker/src/spd_vr/setup.py
git commit -m "feat: run SPD VR from live PICO hands"
```

---

### Task 8: Replace the Startup Path and Document the Real Prerequisites

**Files:**
- Modify: `PICO_tracker/scripts/start_spd_vr.sh`
- Modify: `PICO_tracker/scripts/stop_spd_vr.sh`
- Modify: `PICO_tracker/README.md`
- Modify: `PICO_tracker/README.zh-CN.md`
- Modify: `PICO_tracker/src/pico_bridge/launch/start_xrobotoolkit_pico.launch.py`
- Modify: `PICO_tracker/src/pico_bridge/test/test_cleanup_tianji_pico_processes.py`

**Interfaces:**
- `start_spd_vr.sh` live mode starts exactly: XRoboToolkit ROS launch, Tianji QP IK viewer with wrist input, and Python `spd_vr.runtime`; it does not start `start_pico_driver.sh`, `start_pico_m0.sh`, old SMPL/optical bridge, or Manus.
- `start_spd_vr.sh --mock` remains hardware-free and does not require the official SDK, ROS, or Tianji viewer.
- Documented environment variables: `XRTOOLKIT_SDK_ROOT`, `ROS_DOMAIN_ID=120`, and `ROS_LOCALHOST_ONLY=1`; documented CMake build option: `-DPICO_BUILD_XROBOTOOLKIT_BRIDGE=ON`.
- `start_spd_vr.sh` accepts `--wrist-position-scale` (default `1.0`) and passes the same value to the QP viewer and Python runtime; stable-window defaults are explicitly passed as `10`, `0.02 m`, and `0.15 rad`. It selects `config/qp_ik_pico_teleop.yaml`, whose `controller.rate_hz` remains `200.0`.

- [ ] **Step 1: Add startup script assertions before cutover**

Extend the script test to assert live command strings contain `start_xrobotoolkit_pico.launch.py`, `--config config/qp_ik_pico_teleop.yaml`, `--model models/marvin_m6_qp_pico_fast.xml`, `--pico-teleop`, `--pico-wrist-input`, `--left-end-effector-site l_wrist_target`, `--right-end-effector-site r_wrist_target`, `--algorithm hierarchical_qp`, `--arm-angle-mode default_down`, `--wrist-position-scale`, and `--arm-target-port 15100`; assert the same scale is passed to the runtime and assert the selected YAML keeps `controller.rate_hz: 200.0`. Assert no live command contains the old driver, M0, optical, SMPL, Manus, or undefined `optical_inner` command. Assert mock mode still launches only the mock server and mock runtime.

- [ ] **Step 2: Run the focused script test and verify the old-path failure**

Run: `pytest -q PICO_tracker/src/pico_bridge/test/test_cleanup_tianji_pico_processes.py -k spd_vr`

Expected: FAIL against the current six-window startup, which still references old input services and an undefined `optical_inner` variable.

- [ ] **Step 3: Replace live tmux windows with the approved process graph**

Use three live windows named `xrobotoolkit`, `arm_controller`, and `simulator`. Before creating the session, require the installed `xrobotoolkit_pico_bridge` executable and the existing Tianji viewer binary; emit a precise rebuild hint with `-DPICO_BUILD_XROBOTOOLKIT_BRIDGE=ON` when the SDK target is absent. The first window sources `install/local_setup.bash` and launches `start_xrobotoolkit_pico.launch.py`. The second runs `tianji_qp_ik_viewer` with `--config config/qp_ik_pico_teleop.yaml --pico-teleop --pico-wrist-input --left-end-effector-site l_wrist_target --right-end-effector-site r_wrist_target --algorithm hierarchical_qp --arm-angle-mode default_down`, the configured scale/stable thresholds, UDP input 15000, and arm output 15100. The YAML supplies the required 200 Hz QP loop. The third runs `python -m spd_vr.runtime` with `/pico/hands`, `/spd_vr/pause`, arm bind 15100, and the same scale/stable thresholds. Update `stop_spd_vr.sh` to stop these three windows in consumer-to-producer order. Keep domain/environment setup and duplicate-session checks. Keep `--mock` as the existing hardware-free two-process path and bypass all SDK/Tianji preflight checks there.

- [ ] **Step 4: Add SDK installation/build/run documentation**

In both READMEs, document that the user must install and start the official XRoboToolkit PC-Service, install its `PXREARobotSDK.h`/`libPXREARobotSDK.so`, export `XRTOOLKIT_SDK_ROOT`, build with `PICO_BUILD_XROBOTOOLKIT_BRIDGE=ON`, start the XR-Robotics app on PICO 4 Ultra, then run `./scripts/start_spd_vr.sh`. Document `/pico/hands`, `/pico/tracking_epoch`, `/spd_vr/pause`, UDP ports 15000/15100, index 1 Wrist mapping, explicit wrist site selection, 10-frame stable startup, scale metadata, and that pause freezes physics/recording and forces re-alignment. State that the hardware-specific foot-pedal driver is an external publisher of `/spd_vr/pause`; this plan implements the pause contract but cannot bind an unknown USB/HID event code. State explicitly that camera positions/extrinsics and headset streaming are deferred and not prerequisites for Wrist control.

- [ ] **Step 5: Run focused script/document checks**

Run: `pytest -q PICO_tracker/src/pico_bridge/test/test_cleanup_tianji_pico_processes.py -k spd_vr`

Run: `bash -n PICO_tracker/scripts/start_spd_vr.sh`

Expected: PASS; live command graph contains no old input process and mock command remains hardware-free.

- [ ] **Step 6: Commit the startup cutover**

```bash
git add PICO_tracker/scripts/start_spd_vr.sh PICO_tracker/scripts/stop_spd_vr.sh PICO_tracker/README.md PICO_tracker/README.zh-CN.md PICO_tracker/src/pico_bridge/launch/start_xrobotoolkit_pico.launch.py PICO_tracker/src/pico_bridge/test/test_cleanup_tianji_pico_processes.py
git commit -m "feat: start SPD VR from XRoboToolkit"
```

---

### Task 9: End-to-End Verification and Required Cleanup

**Files:**
- Modify only files exposed by verification failures; do not add a second input protocol, compatibility alias, camera calibration file, or real hardware adapter.
- Verify: `PICO_tracker/src/spd_vr/generated/tianji_wuji2_spd.xml`, `PICO_tracker/src/spd_vr/generated/joint_manifest.yaml`, and all changed tests/configs.

**Interfaces:**
- End-to-end observable path: official fixture/SDK callback -> atomic `/pico/hands` -> wrist relay TJVR -> QP viewer `l_wrist`/`r_wrist` -> 272-byte v2 arm target -> Python unified 54-DoF plant; the same `PicoHands` frame -> Wuji2 20-DoF hand target.
- Verification evidence must distinguish SDK-unavailable environment from source/test failures; no claim of real PICO operation is allowed without an installed SDK and running PC-Service.

- [ ] **Step 1: Run focused pure-core tests after all merges**

Run:

```bash
colcon test --packages-select pico_bridge --ctest-args -R 'test_xrobotoolkit_hand_decoder|test_xrobotoolkit_wrist_relay_core|test_tianji_teleop_protocol' --event-handlers console_direct+
ctest --test-dir TJ_arm_control/build -R 'test_mujoco_robot|test_pico_wrist_alignment|test_arm_target_protocol|test_pico_teleop_session|test_pico_udp_receiver' --output-on-failure
pytest -q PICO_tracker/src/spd_vr/test/test_model_builder.py PICO_tracker/src/spd_vr/test/test_retarget_pair.py PICO_tracker/src/spd_vr/test/test_arm_target_protocol.py PICO_tracker/src/spd_vr/test/test_simulator.py PICO_tracker/src/spd_vr/test/test_episode.py PICO_tracker/src/spd_vr/test/test_runtime.py
```

Expected: all focused tests pass.

- [ ] **Step 2: Run the real model/plant smoke scenario**

Run:

```bash
PYTHONPATH=PICO_tracker/src/spd_vr pixi run python -m spd_vr.runtime --mock --headless --duration 0.25 --scene jenga --task handover_lr --seed 0 --output /tmp/spd-vr-smoke
```

Expected: an episode directory is created, generated model loads with `nq==nv==54`, robot/hand samples are finite, hand target validity is side-specific, and existing recorder validation succeeds. Exercise `EpisodeController` pause/resume in the test harness and verify `sim_time_ns` and recorder lengths do not change during pause.

- [ ] **Step 3: Run a packet-path smoke without real hardware**

Start the built relay/viewer/simulator components with the checked-in JSON fixture converted to one `PicoHands` message, send a sequence of at least 12 frames with a known left translation/right rotation, and inspect arm target/telemetry output. Expected: left-only input changes left arm q target and `l_wrist` pose; right-only input changes right arm q target and `r_wrist` pose; the opposite side remains at its last valid target. The first stable frame has no target jump; a pause/resume cycle emits no control while paused and requires another stable window.

- [ ] **Step 4: Run official SDK configuration/build smoke when the SDK is installed**

Run:

```bash
colcon build --packages-select pico_bridge --cmake-args -DPICO_BUILD_XROBOTOOLKIT_BRIDGE=ON
ros2 launch pico_bridge start_xrobotoolkit_pico.launch.py
```

Expected: the SDK bridge reports initialization/connection state, publishes atomic `/pico/hands`, and the relay reports one output frame per valid callback. If the SDK is not installed, record the exact CMake fatal prerequisite and do not substitute mock SDK behavior.

- [ ] **Step 5: Remove obsolete code/config only after smoke passes**

Delete only the newly obsoleted live references in `start_spd_vr.sh`, launch files, parameter names, and comments; retain legacy nodes used by other non-SPD workflows until their callers are proven absent. Remove any temporary fixture conversion helper, debug print, or unused compatibility alias introduced during implementation. Do not delete the existing recorder/camera interface or old non-SPD calibration tools.

- [ ] **Step 6: Run final narrow checks and inspect the diff**

Run the exact focused commands from Steps 1–4 that are available in the environment, plus:

```bash
bash -n PICO_tracker/scripts/start_spd_vr.sh
python3 -m compileall -q PICO_tracker/src/spd_vr/spd_vr
```

Expected: no syntax errors, no stale old live startup command, no SPD-VR viewer/launch selection of `tcp_L`/`tcp_R`, no untracked zero-pose fallback, and no camera geometry change.

- [ ] **Step 7: Commit final verification cleanup**

```bash
git add -A
git commit -m "test: verify XRoboToolkit SPD VR control path"
```
