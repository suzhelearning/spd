# URDF-First Python Zenoh Scene Teleoperation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 以完整 `tianji_wuji2.urdf` 为唯一模型来源，用三个 Python 进程实现真实 PICO 光学手追踪到 PC MuJoCo 完整天际双臂与双 Wuji2 手的 ROS-free、simulation-only 场景遥操作链。

**Architecture:** 确定性 Python 编译器从同一 URDF 生成 54-DoF `unified_plant.xml` 与 14-DoF `arm_ik.xml`；视觉保留厂家 STL，碰撞使用 CoACD 多凸分解。运行时由 Python `spd_vr.pxrea_bridge`、`spd_vr.arm_ik`、`spd_vr.viewer` 三个进程组成；bridge 通过 `ctypes` 加载 PXREARobotSDK 并监听本机 Zenoh peer，IK 用 MuJoCo Jacobian + OSQP，viewer 是完整 plant 的唯一所有者。

**Tech Stack:** Python 3.11、ctypes、NumPy、SciPy sparse、OSQP 1.x、MuJoCo Python、PyYAML、trimesh 4.x、CoACD 1.0.14、eclipse-zenoh 1.10.0、Pinocchio Python、NLopt、pytest、Pixi、tmux。

**Spec:** `PICO_tracker/docs/superpowers/specs/2026-08-30-urdf-zenoh-scene-teleop-design.md`

## Global Constraints

- 唯一结构与物理模型来源是 `/home/current/syz/spd/assets/tianji_wuji2/tianji_wuji2.urdf`；现有 Tianji/Wuji MJCF 不得作为编译输入。
- 新增和维护的遥操作业务代码必须全部是 Python；不得新增 C/C++ source、CMake target、zenoh-pico 或跨语言 fixture。
- 允许调用 PXREARobotSDK、MuJoCo、OSQP、CoACD、Pinocchio、NLopt 的原生共享库或 Python extension。
- 完整 plant 必须恰好 54 个 revolute DoF；arm projection 必须恰好 14 个 revolute DoF，并包含 `l_wrist_target`、`r_wrist_target`。
- 厂家 STL visual 不降采样；碰撞固定 seed 0、每 link 最多 16 piece、每 piece 最多 64 vertex、`OMP_NUM_THREADS=1`，失败不得回退。
- 双手碰撞代理 p95 双向表面距离不超过 1.5 mm；臂/基座不超过 3 mm。
- tracking wire 固定 1,540-byte little-endian v1；arm target 固定现有 272-byte v2；control 固定 40-byte little-endian v1。
- Zenoh key 固定为 `spd/vr/v1/tracking`、`spd/vr/v1/arm_targets`、`spd/vr/v1/control`、`spd/vr/v1/status/{bridge,ik,viewer}`。
- Bridge peer 监听 `tcp/127.0.0.1:7447`；IK/viewer peer 显式连接；不得启动 `zenohd`。
- 对齐窗口 10 帧；最大相邻平移 0.02 m、旋转 0.15 rad；stale 阈值 50 ms。
- IK 200 Hz、physics 480 Hz、render 60 Hz；render 不能作为 physics clock。
- 新 live path 不得 import/start ROS 或发送任何物理 Tianji/Wuji2 命令。
- 首场景只含完整机器人、灯光、相机和 ground plane。
- Callback 只做有界复制和 latest-only 投递；不得执行 IK、retarget、MuJoCo step 或阻塞等待。
- 每个任务只运行聚焦测试；完整硬件-free、性能和真实 PICO 检查留到 Task 12、Task 13。

---

## File Structure

### 新建 Python 文件

- `PICO_tracker/src/spd_vr/spd_vr/wire/{__init__,crc,tracking,control,keys}.py`：唯一 Python wire codec 和 sequence gate。
- `PICO_tracker/src/spd_vr/spd_vr/pico_frames.py`：PICO 14-byte 内层流、hand/head/reset 解码与左右配对。
- `PICO_tracker/src/spd_vr/spd_vr/zenoh_transport.py`：peer 配置、latest-only mailbox、publisher/subscriber ownership。
- `PICO_tracker/src/spd_vr/spd_vr/pxrea_sdk.py`：PXREARobotSDK ctypes ABI 和 lifecycle。
- `PICO_tracker/src/spd_vr/spd_vr/pxrea_bridge.py`：SDK queue worker、frame pair、tracking/status publication。
- `PICO_tracker/src/spd_vr/spd_vr/model_compiler/{__init__,urdf_model,collision,mjcf,artifacts,cli}.py`：权威 URDF 编译链。
- `PICO_tracker/src/spd_vr/spd_vr/alignment.py`：左右独立中立姿态状态机。
- `PICO_tracker/src/spd_vr/spd_vr/qp_arm.py`：单臂 MuJoCo Jacobian + persistent OSQP workspace。
- `PICO_tracker/src/spd_vr/spd_vr/arm_ik.py`：200 Hz 双臂进程。
- `PICO_tracker/src/spd_vr/spd_vr/session_state.py`：会话命令状态机。
- `PICO_tracker/src/spd_vr/spd_vr/viewer_window.py`：MuJoCo passive viewer、键盘与 HUD adapter。
- `PICO_tracker/src/spd_vr/spd_vr/viewer.py`：480 Hz plant owner、Wuji retarget、60 Hz render。
- `PICO_tracker/src/spd_vr/spd_vr/preflight.py`、`control_cli.py`、`status_cli.py`：operator 工具。
- `PICO_tracker/scripts/test_spd_teleop_e2e.py`：真实 Zenoh、fake SDK、headless plant 验收。
- `PICO_tracker/scripts/accept_real_pico.py`：真实 PICO 可审计验收。

### 修改或删除

- `PICO_tracker/pixi.toml`、`pixi.lock`：OSQP 和 operator tasks。
- `PICO_tracker/src/spd_vr/setup.py`、`package.xml`：安装 generated artifacts 与 Python console scripts；删除新 live path 的 ROS runtime dependency。
- `PICO_tracker/src/spd_vr/spd_vr/arm_target_protocol.py`：迁到 canonical wire package或变为内部 import，扩展 HOLD enum，不改变 wire layout。
- `wuji-retargeting/wuji_retargeting/robot.py`、`opt/base.py`：从完整 URDF 构造 manifest 指定的 20-DoF reduced hand model。
- `PICO_tracker/src/spd_vr/config/wuji2_pico_{left,right}.yaml`、`spd_vr/retarget_pair.py`：使用 authoritative URDF 和 manifest joint order。
- `PICO_tracker/src/spd_vr/spd_vr/{model_builder,manifest,simulator,runtime}.py`：迁移到新 artifacts，删除 ROS live/UDP live 依赖。
- `PICO_tracker/scripts/start_spd_vr.sh`、`stop_spd_vr.sh`：严格三个 Python window。
- 删除 `PICO_tracker/src/spd_vr/spd_vr/ros_input.py` 及其 live tests、旧 generated `tianji_wuji2_spd.xml`、`joint_manifest.yaml`、`sim_actuator_calibration.yaml`。

---

### Task 1: Pin Python runtime dependencies

**Files:**
- Modify: `PICO_tracker/pixi.toml`
- Modify: `PICO_tracker/pixi.lock`
- Modify: `PICO_tracker/src/spd_vr/setup.py`
- Test: `PICO_tracker/src/spd_vr/test/test_dependency_contract.py`

**Interfaces:**
- Consumes: 已存在的 `eclipse-zenoh ==1.10.0`、`coacd ==1.0.14`、`trimesh >=4.8,<5` 和 editable `spd-vr`/`wuji-retargeting`。
- Produces: Python import `osqp` 1.x；保证没有 `qpoases`、`zenoh-pico`、`build-teleop-native`、`TIANJI_BUILD_ZENOH_TELEOP`。

- [ ] **Step 1: 扩展依赖契约测试**

```python
def test_python_teleop_dependencies_are_pinned():
    text = Path("pixi.toml").read_text(encoding="utf-8")
    assert 'osqp = ">=1,<2"' in text
    assert 'eclipse-zenoh = "==1.10.0"' in text
    assert 'coacd = "==1.0.14"' in text
    assert 'trimesh = ">=4.8,<5"' in text
    for forbidden in ("qpoases", "zenoh-pico", "build-teleop-native", "TIANJI_BUILD_ZENOH_TELEOP"):
        assert forbidden not in text
```

- [ ] **Step 2: 运行失败测试**

Run: `cd PICO_tracker && pixi run python -m pytest src/spd_vr/test/test_dependency_contract.py -q`

Expected: FAIL，仅因 `osqp >=1,<2` 尚未声明。

- [ ] **Step 3: 添加依赖并刷新锁**

在 `[dependencies]` 添加 `osqp = ">=1,<2"`；在 `setup.py` 的 `install_requires` 添加 `scipy`、`osqp`、`eclipse-zenoh`、`trimesh`、`coacd`。运行 `pixi install`。

- [ ] **Step 4: 验证 import 和契约**

Run: `cd PICO_tracker && pixi run python -c "import osqp, scipy.sparse, zenoh, coacd, trimesh" && pixi run python -m pytest src/spd_vr/test/test_dependency_contract.py -q`

Expected: 两个命令均退出 0，测试 PASS。

- [ ] **Step 5: 提交**

```bash
git add PICO_tracker/pixi.toml PICO_tracker/pixi.lock PICO_tracker/src/spd_vr/setup.py PICO_tracker/src/spd_vr/test/test_dependency_contract.py
git commit -m "build: pin Python teleoperation dependencies"
```

---

### Task 2: Implement canonical Python wire and PICO frame codecs

**Files:**
- Create: `PICO_tracker/src/spd_vr/spd_vr/wire/__init__.py`
- Create: `PICO_tracker/src/spd_vr/spd_vr/wire/crc.py`
- Create: `PICO_tracker/src/spd_vr/spd_vr/wire/tracking.py`
- Create: `PICO_tracker/src/spd_vr/spd_vr/wire/control.py`
- Create: `PICO_tracker/src/spd_vr/spd_vr/wire/keys.py`
- Create: `PICO_tracker/src/spd_vr/spd_vr/pico_frames.py`
- Modify: `PICO_tracker/src/spd_vr/spd_vr/arm_target_protocol.py`
- Test: `PICO_tracker/src/spd_vr/test/test_wire_protocols.py`
- Test: `PICO_tracker/src/spd_vr/test/test_pico_frames.py`

**Interfaces:**
- Produces: `TrackingFrame`, `encode_tracking`, `decode_tracking`, `TrackingStreamGate`; `ControlFrame`, `ControlCommand`, `encode_control`, `decode_control`, `ControlSequenceGate`; `ArmTargetFrame`, `ArmTargetHoldReason`; `PicoStreamDecoder.feed(data) -> list[PicoFrame]`; `HandPairer.accept(frame, epoch) -> PairedHands | None`。
- Constants: `TRACKING_PACKET_SIZE=1540`、`CONTROL_PACKET_SIZE=40`、`ARM_TARGET_PACKET_SIZE=272`、`MAX_INNER_FRAME_BYTES=747`。

- [ ] **Step 1: 写 wire 失败测试**

测试必须断言三种固定 size、magic/version/CRC/reserved、NaN/Inf、active quaternion norm、sequence/epoch rollback、control duplicate idempotence，以及这些 key 的精确字符串。最小 happy-path 断言：

```python
packet = encode_tracking(identity_tracking_frame(sequence=1, epoch=1))
assert len(packet) == 1540
assert packet[:4] == b"SVT1"
assert decode_tracking(packet).left_hand.shape == (26, 7)
assert encode_control(ControlFrame(1, 10, ControlCommand.START))[:4] == b"SVC1"
assert ARM_TARGET_PACKET_SIZE == 272
```

- [ ] **Step 2: 写 PICO 流失败测试**

用 `struct.pack("<BBqI", 0xAB, frame_type, ts_ms, len(payload))` 构造 hand frame，覆盖一次半帧、一次多帧、733-byte 严格长度、bad magic reset、超长声明、左右相同 timestamp 配对、新 timestamp 替换旧 incomplete pair、world reset epoch。

- [ ] **Step 3: 运行并确认缺失模块失败**

Run: `cd PICO_tracker && pixi run python -m pytest src/spd_vr/test/test_wire_protocols.py src/spd_vr/test/test_pico_frames.py -q`

Expected: FAIL with module/import errors。

- [ ] **Step 4: 实现 codec**

使用 frozen dataclass、NumPy shape validation 和预编译 `struct.Struct`。Tracking header 必须为 `Struct("<IHHIIQQqqff")`，CRC 覆盖 `[16,1540)`；control header/layout 总长 40；所有 reserved byte 必须为零。`PicoStreamDecoder` 持有 `bytearray`，只在完整 header+payload 时消费；payload 上限 733。

`ArmTargetHoldReason` 设置：

```python
class ArmTargetHoldReason(IntEnum):
    NONE = 0
    INPUT_STALE = 1
    SOLVER_FAILURE = 2
    PAUSED = 3
    INACTIVE = 4
    ALIGNING = 5
    DISCONNECTED = 6
    EPOCH_CHANGE = 7
```

- [ ] **Step 5: 运行聚焦测试并提交**

Run: `cd PICO_tracker && pixi run python -m pytest src/spd_vr/test/test_wire_protocols.py src/spd_vr/test/test_pico_frames.py src/spd_vr/test/test_arm_target_protocol.py -q`

Expected: PASS。

```bash
git add PICO_tracker/src/spd_vr/spd_vr/wire PICO_tracker/src/spd_vr/spd_vr/pico_frames.py PICO_tracker/src/spd_vr/spd_vr/arm_target_protocol.py PICO_tracker/src/spd_vr/test/test_wire_protocols.py PICO_tracker/src/spd_vr/test/test_pico_frames.py PICO_tracker/src/spd_vr/test/test_arm_target_protocol.py
git commit -m "feat: add Python teleoperation wire codecs"
```

---

### Task 3: Implement Python Zenoh transport primitives

**Files:**
- Create: `PICO_tracker/src/spd_vr/spd_vr/zenoh_transport.py`
- Test: `PICO_tracker/src/spd_vr/test/test_zenoh_transport.py`

**Interfaces:**
- Produces: `peer_config(*, listen: bool, endpoint: str) -> zenoh.Config`；`LatestSample[T].put(value)`、`.take_new(last_generation) -> tuple[int,T] | None`、`.invalidate()`；`ZenohNode` context manager；`declare_latest_subscriber(key, decoder, mailbox)`。
- Contract: callback 只 `bytes(sample.payload)`、decode 和覆盖单槽；mailbox 容量恒定为一；session close 幂等。

- [ ] **Step 1: 写 mailbox 与 config 失败测试**

```python
box = LatestSample[int]()
for value in range(1000):
    box.put(value)
generation, value = box.take_new(-1)
assert value == 999
assert box.storage_size == 1
assert peer_config(listen=True, endpoint="tcp/127.0.0.1:7447").to_json5().count("7447") == 1
```

使用两个真实 in-process Zenoh peer 验证 publish 后 subscriber 只留下最新 payload，关闭后不再接受 sample。

- [ ] **Step 2: 运行失败测试**

Run: `cd PICO_tracker && pixi run python -m pytest src/spd_vr/test/test_zenoh_transport.py -q`

Expected: FAIL because module is absent。

- [ ] **Step 3: 实现 transport**

`peer_config` 关闭 multicast scouting，listen 端设置 `listen/endpoints`，connect 端设置 `connect/endpoints`。`LatestSample` 用 `threading.Lock`、单一 value 和单调 generation，不使用 `queue.Queue`。`ZenohNode.close()` 按 subscriber、publisher、session 逆序关闭。

- [ ] **Step 4: 运行测试并提交**

Run: `cd PICO_tracker && pixi run python -m pytest src/spd_vr/test/test_zenoh_transport.py -q`

Expected: PASS，无遗留 peer process。

```bash
git add PICO_tracker/src/spd_vr/spd_vr/zenoh_transport.py PICO_tracker/src/spd_vr/test/test_zenoh_transport.py
git commit -m "feat: add Python Zenoh transport"
```

---

### Task 4: Implement ctypes PXREA adapter and bridge

**Files:**
- Create: `PICO_tracker/src/spd_vr/spd_vr/pxrea_sdk.py`
- Create: `PICO_tracker/src/spd_vr/spd_vr/pxrea_bridge.py`
- Test: `PICO_tracker/src/spd_vr/test/test_pxrea_sdk.py`
- Test: `PICO_tracker/src/spd_vr/test/test_pxrea_bridge.py`

**Interfaces:**
- Consumes: Task 2 `PicoStreamDecoder`/`HandPairer`/`encode_tracking`，Task 3 `ZenohNode`。
- Produces: `PXREADevCustomMessage(ctypes.Structure)`；`PXREAClient` context manager；`BoundedCallbackQueue(max_items=64,max_bytes=2048)`；`BridgeCore.accept_event(event) -> list[bytes]`；CLI `main(argv=None) -> int`。

- [ ] **Step 1: 写 ABI/lifecycle 失败测试**

用 fake Python object 模拟 `CDLL` 函数，断言 struct offsets 等于本机 ABI：`devID.offset==0`、`dataSize.offset==32`、`dataPtr.offset==40`、`sizeof==48`。断言 `PXREAInit` 非零时抛 `PXREAError` 且不调用 deinit；成功 context 恰好调用一次 deinit；callback 对 2,049 bytes 拒绝复制；callback object 在 client close 前保持强引用。

- [ ] **Step 2: 写 bridge core 失败测试**

直接构造 fake SDK custom callbacks，覆盖 64-slot overflow 丢最旧、左右配对 publication、device reconnect/reset epoch、device selection ambiguity、invalid payload counter 和 status JSON。断言 bridge 从不访问任何 SDK send symbol。

- [ ] **Step 3: 运行失败测试**

Run: `cd PICO_tracker && pixi run python -m pytest src/spd_vr/test/test_pxrea_sdk.py src/spd_vr/test/test_pxrea_bridge.py -q`

Expected: FAIL with missing modules。

- [ ] **Step 4: 实现 ctypes 和 bridge**

```python
class PXREADevCustomMessage(ctypes.Structure):
    _fields_ = [("devID", ctypes.c_char * 32),
                ("dataSize", ctypes.c_uint64),
                ("dataPtr", ctypes.POINTER(ctypes.c_char))]

CALLBACK = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_void_p)
```

固定 `PXREAInit.argtypes=[c_void_p,CALLBACK,c_uint]`、`restype=c_int`，`PXREADeinit.argtypes=[]`、`restype=c_int`。callback 用 `ctypes.string_at(message.dataPtr,size)` 完成唯一复制。Worker thread 完成解码、pair、tracking publication/status；SIGINT/SIGTERM 设置 event 后有序 close。

- [ ] **Step 5: 运行测试、真实库装载 smoke 并提交**

Run:

```bash
cd PICO_tracker
pixi run python -m pytest src/spd_vr/test/test_pxrea_sdk.py src/spd_vr/test/test_pxrea_bridge.py -q
pixi run python -c "from spd_vr.pxrea_sdk import PXREAClient; PXREAClient.load_library('/opt/apps/roboticsservice/SDK/x64/libPXREARobotSDK.so')"
```

Expected: PASS，第二条只装载和解析 symbol，不调用 `PXREAInit`。

```bash
git add PICO_tracker/src/spd_vr/spd_vr/pxrea_sdk.py PICO_tracker/src/spd_vr/spd_vr/pxrea_bridge.py PICO_tracker/src/spd_vr/test/test_pxrea_sdk.py PICO_tracker/src/spd_vr/test/test_pxrea_bridge.py
git commit -m "feat: bridge PXREA tracking from Python"
```

---

### Task 5: Parse and validate authoritative URDF

**Files:**
- Create: `PICO_tracker/src/spd_vr/spd_vr/model_compiler/__init__.py`
- Create: `PICO_tracker/src/spd_vr/spd_vr/model_compiler/urdf_model.py`
- Test: `PICO_tracker/src/spd_vr/test/test_urdf_model.py`

**Interfaces:**
- Produces immutable `UrdfModel`、`UrdfLink`、`UrdfJoint`、`Inertial`、`MeshGeometry`；`load_urdf(path) -> UrdfModel`；properties `root`, `revolute_joints`, `arm_joint_names`, `hand_joint_names(side)`, `fixed_transform(parent,child)`；`aggregate_fixed_point_masses(model) -> UrdfModel`。
- Joint order: topological child order stabilized by source XML index；manifest records source index。

- [ ] **Step 1: 写 fixture 和失败测试**

测试 malformed root、duplicate、disconnected、bad limit、missing mesh、non-finite、invalid inertia。对 authoritative URDF 断言：

```python
model = load_urdf(AUTHORITATIVE_URDF)
assert model.root == "Link_Base"
assert len(model.links) == 80
assert len(model.joints) == 79
assert len(model.revolute_joints) == 54
assert len(model.arm_joint_names) == 14
assert len(model.hand_joint_names("left")) == 20
assert len(model.hand_joint_names("right")) == 20
```

点质量测试计算 `Link7_L/R` 聚合后 mass 增加 0.05，COM/tensor 与平行轴公式一致，TCP 不再带 inertial；14 个空 fixed frame 被接受，其他 missing inertial 被拒绝。

- [ ] **Step 2: 运行失败测试**

Run: `cd PICO_tracker && pixi run python -m pytest src/spd_vr/test/test_urdf_model.py -q`

Expected: FAIL with missing module。

- [ ] **Step 3: 实现 parser 和验证**

用 `xml.etree.ElementTree`，所有 vector 解析后用 `np.isfinite` 验证。构图用 parent map + DFS cycle/connected check。惯量验证为对称矩阵正特征值，并验证 triangle inequalities；仅明确命名 TCP point mass 和 geometry-free fixed frame 走窄例外。

- [ ] **Step 4: 运行测试并提交**

Run: `cd PICO_tracker && pixi run python -m pytest src/spd_vr/test/test_urdf_model.py -q`

Expected: PASS。

```bash
git add PICO_tracker/src/spd_vr/spd_vr/model_compiler PICO_tracker/src/spd_vr/test/test_urdf_model.py
git commit -m "feat: validate authoritative Tianji Wuji URDF"
```

---

### Task 6: Build deterministic CoACD collision cache

**Files:**
- Create: `PICO_tracker/src/spd_vr/spd_vr/model_compiler/collision.py`
- Test: `PICO_tracker/src/spd_vr/test/test_collision_compiler.py`

**Interfaces:**
- Consumes: Task 5 `MeshGeometry`。
- Produces: frozen `CollisionSettings(seed=0,max_pieces=16,max_vertices=64)`；`CollisionArtifact`；`decompose_mesh(mesh,settings,cache_root) -> CollisionArtifact`；`bidirectional_surface_p95(source,pieces,samples) -> float`。

- [ ] **Step 1: 写 deterministic/cache/quality 失败测试**

用仓库中最小 hand STL，设置 `OMP_NUM_THREADS=1`，连续两次构建断言 cache key、piece 文件顺序和 SHA-256 完全一致。覆盖 corrupt cache 重建、>16 pieces、>64 vertices、zero volume、NaN、CoACD exception 不回退，以及手 1.5 mm/臂 3 mm 门。

- [ ] **Step 2: 运行失败测试**

Run: `cd PICO_tracker && OMP_NUM_THREADS=1 pixi run python -m pytest src/spd_vr/test/test_collision_compiler.py -q`

Expected: FAIL with missing collision module。

- [ ] **Step 3: 实现 cache 和质量门**

Cache key 为 canonical JSON 的 SHA-256，字段包括 source mesh hash、scale、`coacd.__version__`、seed、piece/vertex limits 和全部传给 CoACD 的参数。输出 piece 按每个 mesh 的 canonical vertex/face byte hash 排序后命名。写入 sibling temp directory，fsync manifest 后 `os.replace`。surface sampling 使用固定 NumPy RNG seed 0 和固定样本数。

- [ ] **Step 4: 运行测试并提交**

Run: `cd PICO_tracker && OMP_NUM_THREADS=1 pixi run python -m pytest src/spd_vr/test/test_collision_compiler.py -q`

Expected: PASS，第二次命中 cache。

```bash
git add PICO_tracker/src/spd_vr/spd_vr/model_compiler/collision.py PICO_tracker/src/spd_vr/test/test_collision_compiler.py
git commit -m "feat: compile deterministic convex collision proxies"
```

---

### Task 7: Generate and verify MuJoCo artifacts

**Files:**
- Create: `PICO_tracker/src/spd_vr/spd_vr/model_compiler/mjcf.py`
- Create: `PICO_tracker/src/spd_vr/spd_vr/model_compiler/artifacts.py`
- Create: `PICO_tracker/src/spd_vr/spd_vr/model_compiler/cli.py`
- Modify: `PICO_tracker/src/spd_vr/spd_vr/model_builder.py`
- Modify: `PICO_tracker/src/spd_vr/spd_vr/manifest.py`
- Modify: `PICO_tracker/src/spd_vr/setup.py`
- Test: `PICO_tracker/src/spd_vr/test/test_generated_models.py`

**Interfaces:**
- Consumes: Tasks 5–6 validated model/collision artifacts。
- Produces: `compile_models(urdf_path,output_dir,cache_dir) -> ModelManifest`；`verify_artifacts(manifest_path,urdf_path) -> VerifiedArtifacts`；CLI `spd-model`；five exact generated files from spec。

- [ ] **Step 1: 写 generation 失败测试**

断言 full/arm XML 都能 `mujoco.MjModel.from_xml_path`；`full.nq==54`、`arm.nq==14`；两个 wrist site 存在；visual mesh source hashes 等于 URDF；24 axis visuals 被 manifest 记录且不存在于 scene；joint ranges/order、actuator order、adjacent collision excludes 完整；相对路径在复制到另一个 workspace root 后仍可加载；修改 source/output byte 后 `verify_artifacts` 拒绝。

- [ ] **Step 2: 运行失败测试**

Run: `cd PICO_tracker && OMP_NUM_THREADS=1 pixi run python -m pytest src/spd_vr/test/test_generated_models.py -q`

Expected: FAIL with missing compiler modules。

- [ ] **Step 3: 实现 MJCF 与 atomic artifacts**

生成器递归复现 URDF body/joint transform；quaternion 写为 MuJoCo `wxyz`；visual/collision geom 分离 contype/conaffinity；actuator 采用 manifest joint order。Full model 添加 ground、light、camera、position actuator；arm model 仅保留 base/fixed arm chain、14 joints、两个 wrist sites。所有 YAML 用 `yaml.safe_dump(sort_keys=True)`，output hash 在 final bytes 上计算，整个 output directory 通过 temp sibling atomic replace。

- [ ] **Step 4: 生成真实 artifacts 并验证**

Run:

```bash
cd PICO_tracker
OMP_NUM_THREADS=1 pixi run python -m spd_vr.model_compiler.cli --urdf ../assets/tianji_wuji2/tianji_wuji2.urdf --output src/spd_vr/generated
pixi run python -m pytest src/spd_vr/test/test_generated_models.py src/spd_vr/test/test_model_builder.py -q
```

Expected: 五个产物生成，所有测试 PASS。

- [ ] **Step 5: 删除旧 hybrid 产物并提交**

删除 `generated/tianji_wuji2_spd.xml`、`joint_manifest.yaml`、`sim_actuator_calibration.yaml`，同步 `setup.py` data files；`model_builder.py` 只转发新 compiler CLI，不保留 hybrid path。

```bash
git add PICO_tracker/src/spd_vr/spd_vr/model_compiler PICO_tracker/src/spd_vr/spd_vr/model_builder.py PICO_tracker/src/spd_vr/spd_vr/manifest.py PICO_tracker/src/spd_vr/generated PICO_tracker/src/spd_vr/setup.py PICO_tracker/src/spd_vr/test/test_generated_models.py PICO_tracker/src/spd_vr/test/test_model_builder.py
git commit -m "feat: generate URDF-first MuJoCo models"
```

---

### Task 8: Implement independent alignment and Python OSQP arm IK

**Files:**
- Create: `PICO_tracker/src/spd_vr/spd_vr/alignment.py`
- Create: `PICO_tracker/src/spd_vr/spd_vr/qp_arm.py`
- Create: `PICO_tracker/src/spd_vr/spd_vr/arm_ik.py`
- Test: `PICO_tracker/src/spd_vr/test/test_alignment.py`
- Test: `PICO_tracker/src/spd_vr/test/test_qp_arm.py`
- Test: `PICO_tracker/src/spd_vr/test/test_arm_ik.py`

**Interfaces:**
- Consumes: tracking/control codec、Zenoh transport、`arm_ik.xml`/manifest。
- Produces: `SideAlignment.accept(wrist_pose,active,epoch,timestamp_ns) -> AlignedPose`；`ArmQPSolver.solve(q,target_pose,dt) -> ArmSolveResult`；`DualArmController.tick(now_ns) -> ArmTargetFrame`；CLI `main(argv=None)`。

- [ ] **Step 1: 写 alignment 失败测试**

覆盖 10 个稳定 frame 才 aligned、0.0201 m/0.1501 rad jump 重置窗口、左右独立、inactive 单侧 HOLD、epoch/rollback/realign/reset、50 ms stale、HOLD 保留 last target。用显式 4x4 pose 验证 `T_robot_from_pico = T_neutral_robot @ inv(T_neutral_pico)`。

- [ ] **Step 2: 写 QP 失败测试**

在真实 `arm_ik.xml` 左右 home pose 上断言零误差 `dq≈0`；+x 可达 target 减小 Cartesian error；velocity limit 和一步 position limit 都不被突破；不可达/NaN target 返回 failure 且不改变 last q；左右 solver failure 隔离。Spy `osqp.OSQP.setup` 断言每侧构造时只调用一次，连续 tick 只调用 `update/warm_start/solve`。

- [ ] **Step 3: 运行失败测试**

Run: `cd PICO_tracker && pixi run python -m pytest src/spd_vr/test/test_alignment.py src/spd_vr/test/test_qp_arm.py src/spd_vr/test/test_arm_ik.py -q`

Expected: FAIL with missing modules。

- [ ] **Step 4: 实现 persistent QP 和 200 Hz controller**

用 `mujoco.mj_forward`、`mujoco.mj_jacSite`、SO(3) log。每侧 workspace 固定 7 variables/7 box constraints；更新 `P=J.T@W@J+(lambda_damp+lambda_home)I`、`q=-J.T@W@v_des-lambda_home*dq_home`、`l/u`。接受 `solved` 和 `solved inaccurate`，随后验证 finite 和 `l-1e-7 <= dq <= u+1e-7`。Controller 用 `time.monotonic_ns()` 的绝对 deadline 运行 5 ms tick，missed deadline 不通过循环补算。

- [ ] **Step 5: 运行测试和 200 Hz smoke 并提交**

Run:

```bash
cd PICO_tracker
pixi run python -m pytest src/spd_vr/test/test_alignment.py src/spd_vr/test/test_qp_arm.py src/spd_vr/test/test_arm_ik.py -q
pixi run python -m spd_vr.arm_ik --self-test --ticks 400
```

Expected: PASS；self-test 报告 400 finite ticks、无 solver failure、频率统计。

```bash
git add PICO_tracker/src/spd_vr/spd_vr/alignment.py PICO_tracker/src/spd_vr/spd_vr/qp_arm.py PICO_tracker/src/spd_vr/spd_vr/arm_ik.py PICO_tracker/src/spd_vr/test/test_alignment.py PICO_tracker/src/spd_vr/test/test_qp_arm.py PICO_tracker/src/spd_vr/test/test_arm_ik.py
git commit -m "feat: solve dual-arm IK with Python OSQP"
```

---

### Task 9: Retarget both hands from the authoritative URDF

**Files:**
- Modify: `wuji-retargeting/wuji_retargeting/robot.py`
- Modify: `wuji-retargeting/wuji_retargeting/opt/base.py`
- Modify: `PICO_tracker/src/spd_vr/config/wuji2_pico_left.yaml`
- Modify: `PICO_tracker/src/spd_vr/config/wuji2_pico_right.yaml`
- Modify: `PICO_tracker/src/spd_vr/spd_vr/retarget_pair.py`
- Test: `wuji-retargeting/tests/test_reduced_robot.py`
- Test: `PICO_tracker/src/spd_vr/test/test_retarget_pair.py`

**Interfaces:**
- Consumes: authoritative URDF path and `model_manifest.yaml` left/right 20-joint lists。
- Produces: `RobotWrapper(urdf_path, hand_side, active_joint_names)` building a Pinocchio reduced model；`WujiRetargetPair.from_manifest(left_config,right_config,manifest_path,urdf_path)`；outputs exactly manifest actuator order。

- [ ] **Step 1: 写 reduced-model 失败测试**

用 full URDF 的左/右 20-joint list 构造两个 wrapper，断言 `nq==nv==20`、`dof_joint_names` 精确等于输入顺序集合并具有 manifest permutation、所需 palm/PIP/DIP/tip frame 可解析、未选 arm/另一手 joint 不在 model。

- [ ] **Step 2: 写 pair 失败测试**

对 26-joint OpenXR identity hand，验证到现有 21-keypoint mapping；左右输出 shape `(20,)`、finite、limit clamp、严格 manifest reorder；left inactive/invalid 不阻止 right；epoch 变化 reset 两个 low-pass filter。

- [ ] **Step 3: 运行失败测试**

Run: `cd PICO_tracker && pixi run python -m pytest ../wuji-retargeting/tests/test_reduced_robot.py src/spd_vr/test/test_retarget_pair.py -q`

Expected: FAIL because active reduced model/manifest constructor is absent。

- [ ] **Step 4: 实现 Pinocchio reduced model**

在原 model 上验证 active name 唯一存在；以 `pin.neutral(model)` 为 lock pose，将 universe 和 active joint 之外的 joint id 传给 `pin.buildReducedModel`。`BaseOptimizer` 从 `optimizer.active_joint_names` 读取列表并传给 `RobotWrapper`。RetargetPair 读取 manifest 后覆写 config 的 absolute URDF 和 active list，不再依赖 standalone hand URDF/MJCF。

- [ ] **Step 5: 运行测试并提交**

Run: `cd PICO_tracker && pixi run python -m pytest ../wuji-retargeting/tests/test_reduced_robot.py src/spd_vr/test/test_retarget_pair.py -q`

Expected: PASS。

```bash
git add wuji-retargeting/wuji_retargeting/robot.py wuji-retargeting/wuji_retargeting/opt/base.py wuji-retargeting/tests/test_reduced_robot.py PICO_tracker/src/spd_vr/config/wuji2_pico_left.yaml PICO_tracker/src/spd_vr/config/wuji2_pico_right.yaml PICO_tracker/src/spd_vr/spd_vr/retarget_pair.py PICO_tracker/src/spd_vr/test/test_retarget_pair.py
git commit -m "feat: retarget Wuji hands from authoritative URDF"
```

---

### Task 10: Implement the Python plant owner and operator viewer

**Files:**
- Create: `PICO_tracker/src/spd_vr/spd_vr/session_state.py`
- Create: `PICO_tracker/src/spd_vr/spd_vr/viewer_window.py`
- Create: `PICO_tracker/src/spd_vr/spd_vr/viewer.py`
- Modify: `PICO_tracker/src/spd_vr/spd_vr/simulator.py`
- Modify: `PICO_tracker/src/spd_vr/spd_vr/runtime.py`
- Test: `PICO_tracker/src/spd_vr/test/test_session_state.py`
- Test: `PICO_tracker/src/spd_vr/test/test_viewer.py`

**Interfaces:**
- Consumes: full model/manifests、wire、Zenoh、WujiRetargetPair。
- Produces: `SessionController.apply(ControlFrame) -> SessionSnapshot`；`PlantController.physics_tick(now_ns)`；`ViewerRuntime.run()`；headless CLI flags `--headless --ticks N`。

- [ ] **Step 1: 写 state 与 plant 失败测试**

覆盖 START/PAUSE/RESUME/REALIGN/RESET/SHUTDOWN、duplicate sequence 幂等、PAUSE 冻结 `data.time/qpos/qvel/ctrl`、RESET 精确清零并恢复 home、RESUME 要求 fresh alignment。覆盖 left-only arm/hand target、right-only、50 ms stale HOLD、manifest reorder、54 qpos/qvel/ctrl finite。

- [ ] **Step 2: 写 render-clock 失败测试**

注入 fake clock/window，执行 960 physics ticks，断言 physics schedule 为 480 Hz、render 调用约 120 次且 render delay 不改变 physics `dt`；Q/Escape 发送一次 SHUTDOWN。Headless 模式仍执行完整 plant tick，不创建 GLFW window。

- [ ] **Step 3: 运行失败测试**

Run: `cd PICO_tracker && pixi run python -m pytest src/spd_vr/test/test_session_state.py src/spd_vr/test/test_viewer.py -q`

Expected: FAIL with missing modules。

- [ ] **Step 4: 实现 viewer**

PlantController 是唯一 full `MjData` owner；最新 arm q 送 arm position actuator，hand q 送 manifest hand actuator；每 tick clamp ctrl 后 `mujoco.mj_step`。Window 使用 `mujoco.viewer.launch_passive`，按 60 Hz deadline `sync()`，通过 user scene/overlay 显示 HUD。Runtime callback 只更新 mailbox，retarget 只在新 tracking generation 时调用。

- [ ] **Step 5: 删除 ROS/UDP live branch 并 smoke**

`runtime.py` 保留 hardware-free episode/mock，删除 `LiveInputMailbox`、`rclpy` live branch 和 UDP arm 参数；`simulator.py` 默认新 artifacts。运行：

```bash
cd PICO_tracker
pixi run python -m pytest src/spd_vr/test/test_session_state.py src/spd_vr/test/test_viewer.py src/spd_vr/test/test_simulator.py src/spd_vr/test/test_runtime.py -q
pixi run python -m spd_vr.viewer --headless --ticks 480 --auto-start
```

Expected: PASS；headless smoke 完成 1 simulated second，全部 state finite。

- [ ] **Step 6: 提交**

```bash
git add PICO_tracker/src/spd_vr/spd_vr/session_state.py PICO_tracker/src/spd_vr/spd_vr/viewer_window.py PICO_tracker/src/spd_vr/spd_vr/viewer.py PICO_tracker/src/spd_vr/spd_vr/simulator.py PICO_tracker/src/spd_vr/spd_vr/runtime.py PICO_tracker/src/spd_vr/test/test_session_state.py PICO_tracker/src/spd_vr/test/test_viewer.py PICO_tracker/src/spd_vr/test/test_simulator.py PICO_tracker/src/spd_vr/test/test_runtime.py
git commit -m "feat: add Python MuJoCo teleoperation viewer"
```

---

### Task 11: Cut over lifecycle and operator commands

**Files:**
- Create: `PICO_tracker/src/spd_vr/spd_vr/preflight.py`
- Create: `PICO_tracker/src/spd_vr/spd_vr/control_cli.py`
- Create: `PICO_tracker/src/spd_vr/spd_vr/status_cli.py`
- Modify: `PICO_tracker/src/spd_vr/setup.py`
- Modify: `PICO_tracker/src/spd_vr/package.xml`
- Modify: `PICO_tracker/scripts/start_spd_vr.sh`
- Modify: `PICO_tracker/scripts/stop_spd_vr.sh`
- Modify: `PICO_tracker/pixi.toml`
- Delete: `PICO_tracker/src/spd_vr/spd_vr/ros_input.py`
- Test: `PICO_tracker/src/spd_vr/test/test_preflight.py`
- Test: `PICO_tracker/src/spd_vr/test/test_lifecycle_scripts.py`

**Interfaces:**
- Produces console scripts `spd-model`、`spd-pxrea-bridge`、`spd-arm-ik`、`spd-viewer`、`spd-preflight`、`spd-control`、`spd-status`；Pixi tasks `spd-model`、`spd-teleop`、`spd-teleop-status`、`spd-teleop-stop`。
- Session name fixed `spd-teleop`；window names fixed `pxrea_bridge`、`arm_ik`、`viewer`。

- [ ] **Step 1: 写 preflight/lifecycle 失败测试**

Fake filesystem/socket/subprocess 检查 ADB reverse、SDK load、display、hash、port 7447、已有 session。脚本文本和 dry-run output 必须恰好三个 `python -m spd_vr.*` runtime module，不含 `ros2`、`rclpy`、`dds`、`zenohd`、C++ binary、旧 `optical_inner`。Stop 必须先 control SHUTDOWN，再 viewer/IK/bridge wait，并校验 pane PID command。

- [ ] **Step 2: 运行失败测试**

Run: `cd PICO_tracker && pixi run python -m pytest src/spd_vr/test/test_preflight.py src/spd_vr/test/test_lifecycle_scripts.py -q`

Expected: FAIL because old scripts do not satisfy three-Python-process contract。

- [ ] **Step 3: 实现 operator path**

`preflight.py` 返回结构化 `CheckResult(name,ok,detail)` 并在任一 required check 失败时非零退出。Start script 先运行 preflight，再用 `tmux new-session/new-window` 启动 bridge→IK→viewer，并记录 session-specific metadata。Stop 通过 `spd-control shutdown`，有界等待后只终止命令行与记录 metadata 匹配的 pane。

- [ ] **Step 4: clean cutover package metadata**

删除 `ros_input.py` 及对应 test；`package.xml` 移除 `rclpy`、`geometry_msgs`、`std_msgs`、`pico_bridge` 作为 `spd_vr` live exec dependency；保留仓库其他 ROS package 不变。Setup 注册所有 console script 和五个 generated data file。

- [ ] **Step 5: 运行 dry-run、测试并提交**

Run:

```bash
cd PICO_tracker
pixi run python -m pytest src/spd_vr/test/test_preflight.py src/spd_vr/test/test_lifecycle_scripts.py -q
pixi run bash scripts/start_spd_vr.sh --dry-run
pixi run bash scripts/stop_spd_vr.sh --dry-run
```

Expected: PASS；start dry-run 只列出三个 Python runtime window。

```bash
git add PICO_tracker/src/spd_vr/spd_vr PICO_tracker/src/spd_vr/test PICO_tracker/src/spd_vr/setup.py PICO_tracker/src/spd_vr/package.xml PICO_tracker/scripts/start_spd_vr.sh PICO_tracker/scripts/stop_spd_vr.sh PICO_tracker/pixi.toml PICO_tracker/pixi.lock
git commit -m "feat: cut over Python teleoperation lifecycle"
```

---

### Task 12: Add hardware-free end-to-end and performance gates

**Files:**
- Create: `PICO_tracker/scripts/test_spd_teleop_e2e.py`
- Create: `PICO_tracker/src/spd_vr/test/test_performance.py`
- Create: `PICO_tracker/src/spd_vr/test/test_collision_dynamics.py`
- Modify: `PICO_tracker/pixi.toml`

**Interfaces:**
- Consumes: 完整三个 Python process、真实 Zenoh peer、fake PXREA event source、generated models。
- Produces Pixi task `spd-teleop-e2e`；JSON evidence 含 rates、latency、drops、solver/physics percentiles、finite/contact/shutdown checks。

- [ ] **Step 1: 写真实进程 E2E harness**

Harness 创建临时 endpoint/metadata，启动 `python -m spd_vr.pxrea_bridge --fake-source`、`python -m spd_vr.arm_ik`、`python -m spd_vr.viewer --headless`，等待三个 status ready，发布 START 和至少 12 组稳定 paired hand frame。随后分别施加 left wrist translation、right wrist rotation、left finger flex、right finger flex，比较前后 target/plant snapshot，最后 SHUTDOWN 并等待三个 exit。

- [ ] **Step 2: 添加 E2E assertions**

必须断言左右隔离、finger 隔离、54 state finite、stale/inactive HOLD、epoch 后重新对齐、pause 冻结、reset 清零、无 ROS import/process、无 physical send API、无残留 tmux/child/Zenoh endpoint。失败时输出最近三份 status 和子进程 stderr。

- [ ] **Step 3: 添加性能与碰撞测试**

在目标机运行 2,000 IK tick 和 4,800 physics tick；记录 `np.percentile(samples,95)`。断言 OSQP solve p95 `<5.0 ms`、`mj_step` p95 `<2.083 ms`、队列/LatestSample storage 恒定、ground/cross-arm/palm/finger contact force 和 state finite。对所有 collision manifest 重算误差门和 hash。

- [ ] **Step 4: 运行完整硬件-free 验收**

Run:

```bash
cd PICO_tracker
OMP_NUM_THREADS=1 pixi run python scripts/test_spd_teleop_e2e.py --json /tmp/spd-teleop-e2e.json
OMP_NUM_THREADS=1 pixi run python -m pytest src/spd_vr/test/test_performance.py src/spd_vr/test/test_collision_dynamics.py -q
```

Expected: 两条命令 PASS；JSON 标记三个进程、行为、性能、cleanup 全部 `true`。

- [ ] **Step 5: 提交**

```bash
git add PICO_tracker/scripts/test_spd_teleop_e2e.py PICO_tracker/src/spd_vr/test/test_performance.py PICO_tracker/src/spd_vr/test/test_collision_dynamics.py PICO_tracker/pixi.toml PICO_tracker/pixi.lock
git commit -m "test: verify Python teleoperation end to end"
```

---

### Task 13: Add and run real PICO acceptance

**Files:**
- Create: `PICO_tracker/scripts/accept_real_pico.py`
- Modify: `PICO_tracker/pixi.toml`

**Interfaces:**
- Consumes: running RoboticsService、ADB reverse、XR app、三个 Python process status/control。
- Produces Pixi task `spd-teleop-accept-pico`；append-only JSON acceptance record with environment, model hashes, measured rates, operator checkpoints and failure reason。

- [ ] **Step 1: 实现非伪造验收器**

脚本先验证真实 SDK library、至少一个 online device、非 fake source、连续 2 秒 paired tracking。自动检查 tracking rate、epoch monotonic、10-frame alignment、无 NaN、Space/R/N/Q command acknowledgements；交互提示用户完成左腕、右腕、左右五指和单手遮挡动作，每项要求 status/target 数值变化满足对应侧隔离后才能通过。`--non-interactive` 在需要动作时明确返回 exit 2，不得写 PASS。

- [ ] **Step 2: 添加 Pixi task 并执行可达检查**

Run:

```bash
cd PICO_tracker
pixi run python scripts/accept_real_pico.py --preflight-only
pixi run python scripts/accept_real_pico.py --output /tmp/spd-real-pico-acceptance.json
```

Expected: preflight 在设备就绪时 PASS；完整命令只有真实动作全部观察到才 PASS。设备/服务不可用时记录精确 external prerequisite 并非零退出，不使用 mock 替代。

- [ ] **Step 3: 提交验收工具**

```bash
git add PICO_tracker/scripts/accept_real_pico.py PICO_tracker/pixi.toml PICO_tracker/pixi.lock
git commit -m "test: add real PICO teleoperation acceptance"
```

- [ ] **Step 4: 最终 clean-cutover 检查**

Run:

```bash
cd PICO_tracker
pixi run python -m pytest src/spd_vr/test -q
python -c "from pathlib import Path; text='\n'.join(p.read_text(errors='ignore') for p in Path('src/spd_vr/spd_vr').glob('*.py')); assert 'rclpy' not in text; assert 'socket.recvfrom' not in text"
pixi run bash scripts/start_spd_vr.sh --dry-run
```

Expected: 全部 PASS；dry-run 只有 `spd_vr.pxrea_bridge`、`spd_vr.arm_ik`、`spd_vr.viewer`。

---

## Plan Self-Review

- Spec coverage：Task 2–4 覆盖 SDK、PICO frame、wire、Zenoh；Task 5–7 覆盖 URDF、point mass、visual、CoACD、MJCF/manifests；Task 8 覆盖 alignment/QP/HOLD；Task 9 覆盖 authoritative Wuji retarget；Task 10–11 覆盖 plant/viewer/control/lifecycle/clean cutover；Task 12–13 覆盖 hardware-free、性能、碰撞和真实 PICO。
- 类型一致性：tracking/control/arm target 都由 Task 2 产生；Task 3 只传递 bytes 和 decoded value；Task 4/8/10 分别消费同一 wire types；manifest joint lists 在 Task 7 产生并由 Task 8–10 消费。
- 语言边界：没有新增 C/C++ file、CMake target、zenoh-pico 或跨语言验证；ctypes、MuJoCo、OSQP、CoACD、Pinocchio/NLopt 仅作为 Python 可调用依赖。
- Clean cutover：Task 7 删除 hybrid artifacts；Task 10 删除 ROS/UDP live branch；Task 11 删除 `ros_input.py` 和旧 lifecycle；无 alias/shim。
