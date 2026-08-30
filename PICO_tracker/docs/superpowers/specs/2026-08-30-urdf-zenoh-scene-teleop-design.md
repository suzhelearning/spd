# URDF-First Python Zenoh Scene Teleoperation Design

**Date:** 2026-08-30

**Status:** Approved direction; supersedes the mixed C++/Python design

## 目标

使用 `/home/current/syz/spd/assets/tianji_wuji2/tianji_wuji2.urdf` 作为唯一结构与物理模型来源，实现真实 PICO 光学手追踪到 PC MuJoCo 完整天际双臂和双 Wuji2 手的 ROS-free、simulation-only 场景遥操作链。

所有项目业务代码、进程入口、协议、IK、模型编译和生命周期管理统一使用 Python 3.11。允许加载供应商提供的 PXREARobotSDK `.so`，以及 MuJoCo、OSQP、CoACD 等 Python 包内部的原生扩展；项目不新增或维护 C/C++ 遥操作代码、CMake 目标、zenoh-pico 或跨语言协议夹具。

首个场景为空场景：完整机器人、ground plane、灯光和操作相机。PC 键盘控制会话。启动和显式重对齐时，用户保持中立姿态建立 PICO 到机器人腕部的左右独立映射。

## 已确认事实

权威 URDF 包含：

- 80 个 link、79 个 joint，其中 54 个 revolute joint；
- 双臂各 7 个 revolute joint，双手各 20 个 revolute joint；
- 62 个唯一厂家 STL，visual 与 collision 都引用这些网格；
- 14 个无 geometry、无 inertial 的 fixed frame；
- `TCP_Link_L/R` 各声明 `0.05 kg` 质量和全零惯量，通过 fixed joint 连接 `Link7_L/R`；
- 24 个命名为 `*_axis_[0-2]` 的坐标轴调试 cylinder visual；
- 没有 joint `<dynamics>`，因此生成模型的默认 damping 必须由编译器显式记录。

PXREARobotSDK ABI 为：

```c
int PXREAInit(void* context, pfPXREAClientCallback callback, unsigned mask);
int PXREADeinit(void);
typedef void (*pfPXREAClientCallback)(
    void* context, PXREAClientCallbackType type, int status, void* userData);
typedef struct {
    char devID[32];
    uint64_t dataSize;
    const char* dataPtr;
} PXREADevCustomMessage;
```

PICO 内层帧使用 14-byte little-endian header，magic `0xAB`；光学左手为 `0x38`，右手为 `0x39`，hand payload 固定 733 bytes：`active:uint8`、`scale:float32`、26 组 `xyz + quaternion_xyzw`。

## 范围

### 包含

- 确定性 URDF-to-MuJoCo 编译器；
- 厂家 STL 原样 visual；
- 基于 URDF collision mesh 的 CoACD 多凸碰撞代理；
- 同源生成 54-DoF `unified_plant.xml` 和 14-DoF `arm_ik.xml`；
- Python `ctypes` PXREARobotSDK bridge；
- Python Zenoh peer 通信和固定二进制协议；
- Python MuJoCo Jacobian + OSQP 双臂 QP IK；
- Python Wuji retarget、MuJoCo physics、operator viewer；
- 独立左右侧对齐、有效性、stale HOLD、solver HOLD；
- PC 键盘 START/PAUSE/RESUME/REALIGN/RESET/SHUTDOWN；
- Pixi 管理的构建、启动、状态、停止和验收命令；
- mock 和真实 PICO 端到端验收。

### 不包含

- 物理 Tianji 或 Wuji2 输出；
- 新 live path 中的 ROS、DDS、ROS message 或 ROS launch；
- PICO 头显渲染或立体画面回传；
- 任务物体、评分、录制和六场景/17 任务注册表；
- 手势会话控制；
- 碰撞分解失败时的 primitive 或单 hull 静默回退；
- C/C++ 遥操作实现、跨语言兼容层或第二套 live 协议。

仓库中与新 `spd-teleop` 无关的旧 ROS 工具不在本次删除范围内，但新入口不得 import 或启动它们。

## 总体架构

模型构建链：

```text
assets/tianji_wuji2/tianji_wuji2.urdf
        |
        +-- unified_plant.xml   54-DoF 双臂 + 双手 plant
        `-- arm_ik.xml          14-DoF 双臂 projection + wrist sites
```

运行时固定三个 Python 进程，不启动单独的 Zenoh router：

```text
PICO XR App
    |
    | ADB reverse + RoboticsService + PXREARobotSDK custom bytes
    v
python -m spd_vr.pxrea_bridge
    | listens tcp/127.0.0.1:7447
    | spd/vr/v1/tracking
    +---------------------------+
    v                           v
python -m spd_vr.arm_ik     python -m spd_vr.viewer
    |                           |
    | spd/vr/v1/arm_targets     | Wuji retarget
    +-------------------------->| 单一 MjModel/MjData owner
    ^                           |
    +---- spd/vr/v1/control ----+
```

Bridge 以 Zenoh peer 模式监听 `tcp/127.0.0.1:7447`；IK 和 viewer 显式连接。不存在 `zenohd`、ROS daemon 或 DDS discovery。断连立即使下游 target 无效；重连不回放旧 target，必须重新对齐。

## Python 进程边界

### `spd_vr.pxrea_bridge`

职责：

- 通过 `ctypes.CDLL` 加载 `${PXREA_SDK_ROOT:-/opt/apps/roboticsservice/SDK}/x64/libPXREARobotSDK.so`；
- 用 `ctypes.Structure` 精确声明 `PXREADevCustomMessage`，用 `ctypes.CFUNCTYPE` 声明回调；
- 固定 `argtypes/restype`，在进程生命周期内强引用 callback；
- `PXREAInit` 成功后恰好调用一次 `PXREADeinit`；
- 维护设备在线、连接、丢包和 epoch 状态；
- 解码 PICO 内层帧并配对左右手；
- 发布 tracking/status，不发送任何 SDK 控制、机器人或手部命令。

SDK callback 只允许：验证 `userData` 和 `dataSize`、从 SDK-owned pointer 复制最多 2,048 bytes、把不可变 `bytes` 非阻塞放入有界队列、更新 drop counter 并返回。callback 不做 struct 解码、Zenoh publication、IK、retarget、日志格式化或阻塞等待。队列满时丢弃最旧项，内存上限固定。

内层流解析器支持一个 callback 中的半帧、单帧和多帧；最大声明 payload 为 733 bytes。错误 magic、超长 payload 或无法恢复的流错误清空 buffer，并记录原因。World reset、SDK reconnect 和设备切换增加 epoch，并清空未配对数据。

只有 source timestamp 和 epoch 都一致的左右手才组成 tracking frame。新 timestamp 到达时丢弃旧的不完整 pair。无歧义时自动选择唯一在线设备；多设备时要求 `--device` 明确指定。

### `spd_vr.arm_ik`

职责：

- 加载并验证 `arm_ik.xml` 和 manifest hash；
- 订阅 tracking/control；
- 从 OpenXR hand joint index 1 读取 Wrist；
- 独立维护左右侧中立姿态对齐和 HOLD 状态；
- 以 200 Hz 运行 MuJoCo kinematics/Jacobian + OSQP QP；
- 发布 272-byte arm target 和 status；
- 不打开 GUI、不发送物理设备命令。

IK 进程有自己的 14-DoF projection `MjModel/MjData`，只用于运动学，不拥有完整 plant。左右臂分别求解，一个侧失败不阻塞另一侧。

### `spd_vr.viewer`

职责：

- 加载并验证 `unified_plant.xml` 与 manifests；
- 拥有 live path 中唯一完整 plant 的 `MjModel/MjData`；
- 订阅 tracking、arm target 和 control；
- 每个新 hand frame 对左右手各运行一次 authoritative Wuji retarget；
- 在 480 Hz physics tick 边界应用最新有效 target；
- 在 60 Hz 渲染 PC operator window，render 不作为 physics clock；
- 发布 control/status，提供键盘和 HUD；
- 不发送物理硬件命令。

## URDF 编译器

### 唯一来源和预检

URDF 对 topology、fixed mount、inertial、joint axis/origin/limit、visual/collision identity 和 transform、wrist frame、joint ordering 都是唯一权威来源。现有 Tianji/Wuji MJCF 不能作为结构输入。

生成前必须拒绝：

- 零个或多个 root；
- 重复 link/joint、缺失 parent/child、断图或环；
- 缷失 mesh、非有限 transform/scale；
- revolute limit 缺失、非有限或倒置；
- 缺失 `l_wrist`/`r_wrist` 链；
- full revolute count 不是 54，或 arm projection count 不是 14；
- 除下述窄策略外缺失/非物理 inertial。

14 个无 geometry 的 fixed frame 保持无质量 frame。仅 `TCP_Link_L/R` 被解释为 source-declared fixed point mass：将质量经 fixed transform 聚合到 `Link7_L/R`，用平行轴定理更新父 link 的 mass、COM 和 inertia，重新验证正定性，不生成独立 TCP inertial。manifest 记录原始质量、transform、目标 link 和聚合结果。其他非物理 inertia 直接失败。

URDF 中 24 个 `*_axis_[0-2]` 调试 cylinder visual 从 operator scene 中省略并写入 manifest；其他 primitive visual/collision 不允许静默替代。

### Visual 和 collision

Visual 使用 URDF 引用的厂家 STL，不降采样。生成 XML 的 mesh 路径相对输出目录，拷贝或链接策略由 artifact writer 统一管理。

Collision 使用固定参数 CoACD：

- seed `0`；
- 每 link 最多 16 个 convex piece；
- 每 piece 最多 64 个 hull vertex；
- `OMP_NUM_THREADS=1`；
- cache key 包含 STL SHA-256、URDF scale、CoACD 版本和全部参数；
- 临时目录完成后 atomic rename；
- 失败不回退 box/capsule/single hull；
- 相邻 parent-child link 排除 self-collision；
- 非相邻双臂、跨臂、掌部和手指保持可碰撞。

每个 piece 必须 finite、non-empty、正体积且能被 MuJoCo 编译。固定 surface sampling 验证双向距离：双手 p95 不超过 1.5 mm；臂/基座不超过 3 mm。

### 产物

```text
PICO_tracker/src/spd_vr/generated/
├── unified_plant.xml
├── arm_ik.xml
├── model_manifest.yaml
├── collision_manifest.yaml
└── actuator_calibration.yaml
```

`unified_plant.xml` 恰好 54 revolute DoF。`arm_ik.xml` 恰好 14 revolute DoF，并在 URDF `l_wrist`、`r_wrist` body origin 设置 `l_wrist_target`、`r_wrist_target` site。

Manifest 记录 compiler version、URDF hash、全部 STL hash、输出 hash、joint/link map、source transform、CoACD 参数和质量指标、actuator map、wrist site、model dimensions、默认 damping。所有运行进程在启动时验证 source/output hash；不匹配时拒绝运行。

## Python wire contract

固定 key：

```text
spd/vr/v1/tracking
spd/vr/v1/arm_targets
spd/vr/v1/control
spd/vr/v1/status/bridge
spd/vr/v1/status/ik
spd/vr/v1/status/viewer
```

只有一个 canonical Python codec。使用 `struct.Struct` 显式 little-endian 编解码，不用 ctypes struct 直接作为 wire，不依赖 pickle，不保留 C++ fixture。

### Tracking v1

`spd/vr/v1/tracking` 固定 1,540 bytes：

```text
uint32 magic                 // "SVT1"
uint16 version               // 1
uint16 flags                 // left_active, right_active, head_valid
uint32 payload_size          // 1540
uint32 crc32                 // bytes [16, 1540)
uint64 sequence
uint64 tracking_epoch
int64  source_timestamp_ns
int64  bridge_monotonic_ns
float32 left_scale
float32 right_scale
float32 head_pose[7]
float32 left_hand[26][7]
float32 right_hand[26][7]
```

Decoder 拒绝错误 magic/version/size/CRC、non-finite、epoch/sequence 非正、同 epoch 时间或 sequence 回退、scale 非正、active pose quaternion 与单位长度偏差超过 `1e-3`。head 无效不阻止有效双手 frame。

### Arm target v2

沿用现有 272-byte `SPDA` v2 layout：sequence、tracking epoch、source/control timestamp、左右 q/qdot、valid mask、左右 HOLD reason、reserved 和 CRC。新增 HOLD enum 只占用现有 1-byte reason，不改变 layout：`NONE`、`INPUT_STALE`、`SOLVER_FAILURE`、`PAUSED`、`INACTIVE`、`ALIGNING`、`DISCONNECTED`、`EPOCH_CHANGE`。

### Control v1

`spd/vr/v1/control` 固定 40 bytes：magic `SVC1`、version、command、declared size、CRC、sequence、monotonic timestamp、8-byte zero reserved。命令为 START、PAUSE、RESUME、REALIGN、RESET、SHUTDOWN。sequence 重复幂等；回退拒绝。

Status 使用 JSON，只用于诊断，不作为控制输入。

Zenoh tracking 和 arm target 使用 latest-only mailbox；callback 只复制 sample payload 并覆盖单槽。control 使用 ordered/reliable publication，并由接收状态中的最后 sequence 形成可见 acknowledgement。

## 对齐、QP 和 HOLD

每侧独立状态：

```text
DISCONNECTED -> WAITING_INPUT -> STABILIZING -> ALIGNED
ALIGNED -> HOLD_STALE | HOLD_INACTIVE | HOLD_SOLVER | HOLD_PAUSED
HOLD_* -> STABILIZING -> ALIGNED
```

STABILIZING 需要连续 10 个 Wrist frame。相邻接受 frame 的 translation 变化不得超过 0.02 m，orientation geodesic 变化不得超过 0.15 rad。

对齐变换：

```text
T_robot_from_pico,s = T_urdf_wrist,s,neutral @ inverse(T_pico_wrist,s,neutral)
```

默认 position scale 为 1.0，并在 status 中报告。左右独立对齐。epoch 变化、timestamp rollback、REALIGN、RESET、PAUSE 后 RESUME 使两侧重新进入稳定窗口；单手 inactive 只影响对应侧。

Tracking 和 arm target 超过 50 ms 没有新有效 frame 即 stale。HOLD 保留最后有效 target，不跳零、不跳中立位、不应用失败求解结果。

每侧 QP 在固定 5 ms tick 求解速度 `dq`：

```text
min  0.5 * ||J(q) dq - v_des||^2_W
   + 0.5 * lambda_damp * ||dq||^2
   + 0.5 * lambda_home * ||dq - dq_home||^2
s.t. lower(q, dt) <= dq <= upper(q, dt)
```

其中：

```text
lower = max(-velocity_limit, (q_min - q) / dt)
upper = min(+velocity_limit, (q_max - q) / dt)
```

`v_des` 由 position error 和 SO(3) log orientation error 除以 `dt` 得到，并分别限幅。`J` 使用 MuJoCo Python `mj_jacSite`。QP 用 Python `osqp` API 和 SciPy sparse matrix；每侧预建 workspace，tick 中只 update 数值并 warm start，禁止每 tick 重建 solver 或分配与 DoF 成比例的大对象。只有 OSQP status `solved`/`solved inaccurate`、解 finite 且满足 bound tolerance 时才积分并发布。失败时该侧 HOLD_SOLVER。

Wuji retarget 读取 26-joint OpenXR hand，按 generated manifest 重排到各 20-DoF hand actuator，并 clamp URDF limit；一手失败不影响另一手。

## Viewer 与会话控制

键位：

- Space：START、PAUSE 或 RESUME；
- R：左右 REALIGN；
- N：清空 q、qvel、ctrl、simulation time 和两侧 alignment；
- Q 或 Escape：SHUTDOWN。

PAUSE 冻结 physics、retarget、IK target publication 和 target application；status reception 可继续。RESUME 必须重新经过 10-frame alignment，旧 frame 不重放。

HUD 显示：SDK/Zenoh 状态、tracking/target rate、source/bridge latency、drop/invalid counter、左右 alignment/HOLD、physics p95/max、active contacts、source/generated hash 状态。

## 启停与前置检查

支持的 Pixi 接口：

```bash
pixi run spd-model
pixi run pico-adb -- --offline
pixi run spd-teleop
pixi run spd-teleop-status
pixi run spd-teleop-stop
```

`spd-teleop` 在启动三个进程前验证：ADB reverse、RoboticsService/PICO、SDK `.so` 可加载、MuJoCo/OSQP/CoACD/Zenoh/display、source/generated hash、端口 7447 空闲、无已运行 SPD-VR session。

启动器创建恰好三个 tmux window：`pxrea_bridge`、`arm_ik`、`viewer`。不启动或提及旧 PICO ROS driver、M0、optical/SMPL bridge、Manus、DDS 或 ROS 环境变量。

停止先发布 SHUTDOWN，再按 viewer、IK、bridge 顺序等待。超时后只对已验证身份的本 session 进程升级信号。不得停止并非本次启动的 ADB supervisor。

## 失败语义

- SDK disconnect：bridge 保持运行并报告 disconnected；下游 HOLD；
- Zenoh disconnect：立即使 target 无效；重连需新 sequence 和 alignment；
- 单手 inactive：仅对应侧 HOLD_INACTIVE；
- tracking/target stale：50 ms 后对应侧 HOLD_STALE；
- QP failure：仅对应侧 HOLD_SOLVER；
- epoch/reset：清空 pending pair、latest frame 和两侧 alignment；
- model/manifest/hash mismatch：在 viewer/SDK 初始化前拒绝启动；
- CoACD/质量门失败：`spd-model` 失败，不生成替代碰撞；
- viewer close 或 Q/Escape：有序 shutdown；不存在物理输出 drain。

## 验证与验收

### 单元与模型契约

- PXREA ctypes layout、callback lifetime、最大复制、queue overflow；
- PICO 半帧/单帧/多帧、错误 header、左右配对和 epoch；
- 三种协议的 size、CRC、reserved、NaN/Inf、quaternion、sequence/epoch；
- URDF topology、54/14 DoF、fixed wrist transform、point-mass 聚合；
- visual mesh identity、relative path、deterministic collision cache/hash、p95 距离门；
- 两个 MJCF 可加载且 joint/site/actuator map 完整；
- 独立对齐、jump reject、stale/inactive/solver HOLD、pause/reset/latest-only；
- QP position/velocity boundary、不可达 target、单侧 solver failure。

### Hardware-free 端到端

Fake PXREA callback 通过真实 Python Zenoh session 发送至少 12 组 paired frames，验证：

- 三个入口均为 Python module，进程命令不含 C++ binary；
- 左 Wrist motion 只改变左臂 target；
- 右 Wrist rotation 只改变右臂 target；
- finger motion 只改变对应 Wuji2 joint；
- 54 个 plant q/qvel/ctrl 全部 finite；
- headless/offscreen viewer 渲染厂家 mesh；
- shutdown 后无 Zenoh peer、tmux session 或 child process。

### 性能与碰撞

- IK 200 Hz，单侧 OSQP solve p95 小于 5 ms；
- physics 480 Hz，step p95 小于 2.083 ms；
- render 60 Hz 且不控制 physics clock；
- tracking overload 只产生有界 drop，不增长内存；
- ground、cross-arm、palm、finger contact finite，无 NaN 或爆炸能量；
- collision proxy 在固定环境下输出 hash 稳定并满足几何误差门。

### 真实 PICO

在 RoboticsService、ADB reverse、XR app 运行时验证：连续 paired hands、10-frame 无跳变对齐、左右腕独立驱动、双手五指 retarget、遮挡单手只 HOLD 该侧、Space/R/N/Q 行为、无 ROS 进程、无物理机器人输出。

## Clean cutover

迁移所有新 SPD-VR live caller 到 Python 三进程、generated manifests 和唯一 Python wire codec。删除旧 hybrid `tianji_wuji2_spd.xml` consumer、live UDP arm wiring、ROS live branch 和旧六窗口启动逻辑；不保留 compatibility alias。与新 live path 无关的旧 ROS 应用可以继续存在，但不是本功能依赖。
