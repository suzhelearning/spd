# SPD 数据流水线

本文按当前源码说明 SPD 的输入、控制、物理、采集、轨迹校验与网页回放数据流。

**默认 Quest／PICO 裸手采集不经过 ROS 话题。** TCP 负责头显输入，管道负责原生求解子进程通信，内存快照负责控制交接，HDF5 负责持久化。可选外部 DDS 模式才启用下述两个 ROS 话题。

## 1. 总体数据流

```mermaid
flowchart TD
    H[Quest / PICO 裸手应用] -->|TCP 10002 / ADB 转发| T[TeleopSession]
    T <-->|stdin / stdout 二进制管道| A[DLS / Ruckig 双臂求解子进程]
    T <-->|stdin / stdout 二进制管道| F[左右 Hand2 求解子进程]
    T -->|不可变 TeleopSnapshot| C[CollectionControl]
    K[终端 / GLFW 按键] -->|进程内队列| C
    E[外部命令发布器：可选模式] -->|/spd/tianji_wuji2/v1/joint_command| M[原生命令邮箱]
    C -->|本地目标 / 授权| M
    M --> P[唯一物理线程 / MuJoCo]
    P -->|状态采样 / 接触累计| S[CollectionSession / Recorder]
    P --> V[在线 Viewer]
    S -->|外部 DDS 模式才实际发布| Q[/spd/collection/status]
    S --> R[完整物理轨迹 HDF5]
    R --> B[spd-web 浏览器回放]
    R --> X[validate_episode / replay_episode]
```

本地裸手与外部 DDS 是互斥输入模式，不是同时驱动机器人的两个命令源。图中的“一个采集进程”指主控／物理／采集归属同一个主进程；双臂和 Hand2 数值求解仍有独立子进程。

## 2. 启动入口与模式选择

| 阶段 | 入口 | 实际去向 |
| --- | --- | --- |
| 完整构建 | `pixi run spd-teleop-build` | `bash/build_teleop.sh` 构建求解 worker，再调用 `bash/build_native.sh` 构建 ROS 接口及原生执行器 |
| Quest 采集 | `pixi run spd-quest-teleop --height-m 1.75` | `bash/run_quest_hand_sim.sh` → `bash/run_pico_hand_sim.sh` |
| PICO 采集 | `pixi run spd-pico-teleop --height-m 1.75` | `bash/run_pico_hand_sim.sh` |
| 共用启动 | `bash/start_spd_sim.sh` | ROS Jazzy 环境 → `.ros/install/lib/spd_native/spd_executor` → `simulation.ros_viewer` |
| 本地裸手模式 | `spd-sim --height-m HEIGHT` | 创建 `TeleopSession`，原生执行器 `subscribe=false` |
| 外部 DDS 模式 | `spd-sim` 不传 `--height-m` | 不创建本地 TeleopSession，原生执行器 `subscribe=true` |

模式开关在 [ros_viewer.py](src/simulation/ros_viewer.py)。外部模式的命令发布器由外部系统提供，本项目不自动启动它。

## 3. 头显到求解器：不是 ROS 话题

| 通道 | 发送端 → 接收端 | 数据 | 实现 |
| --- | --- | --- | --- |
| TCP，默认 `127.0.0.1:10002` | Quest／PICO 应用 → TeleopSession | 头部、手腕及双手 OpenXR 跟踪数据 | `src/pico2_hands/collection_session.py` |
| ADB 转发 | 主机 `tcp:10002` → 头显同端口 | TCP 字节流，不改变输入语义 | `src/pico2_hands/input_transport.py` |
| 双臂 stdin/stdout 管道 | TeleopSession ↔ DLS worker | 双侧关节 seed、目标位姿、时间、sequence／epoch；返回关节解、误差与求解状态 | `src/pico2_hands/dls_worker.py` |
| 左右手 stdin/stdout 管道 | TeleopSession ↔ 各侧 Hand2 worker | 每手 21×3 重定向关键点、sequence／timestamp；返回该侧 20 维关节目标 | `src/pico2_hands/hand_worker.py` |
| 加锁的不可变内存快照 | TeleopSession worker → CollectionControl | `TeleopSnapshot`，不发布 ROS 消息 | `src/pico2_hands/collection_session.py` |

头显共用协议：小端 `<BBqI>` 帧头，magic `0xAB`、type `0x40`、version `1`，载荷 1968 字节；双手各 26 个 OpenXR 关节。坐标 FLU，四元数 xyzw。26 个输入关节不等同于 Hand2 重定向使用的 21 个关键点，也不等同于每手 20 个机器人关节。

双臂管道请求／响应 magic 为 `P2IQ`／`P2IR`；Hand2 为 `TJWI`／`TJHR`。两者使用请求身份校验，不是 DDS topic，也不对外提供第二套控制入口。

`TeleopSnapshot` 包含：

- `generation`、`sequence`：拒绝旧绑定和过期求解结果。
- `position_rad`：54 维机器人目标。
- `generated_ns`、`input_ns`：生成与输入时间。
- `can_bind`、`arms_valid`、`needs_rebind`：接手条件。
- `control_flags`、`finger_modes`、`mode`、`fault`：质量、每手状态和故障。

头显输入和求解器不能直接修改 MuJoCo 状态。重新绑定由物理线程先冻结场景，再向 TeleopSession 发起；生成的新目标经统一控制器进入原生执行器。

## 4. ROS 话题清单

当前生产相关的 ROS 发布／订阅只有以下两项，均由外部 DDS 模式创建。

| 话题 | 消息类型 | 发布者 → 订阅者 | QoS | 用途 |
| --- | --- | --- | --- | --- |
| `/spd/tianji_wuji2/v1/joint_command` | `tianji_spd_interfaces/msg/JointCommand` | 外部命令源 → 原生 `RosJointCommandExecutor` | BEST_EFFORT、KEEP_LAST(1)、VOLATILE | 54 维位置目标输入 |
| `/spd/collection/status` | `std_msgs/msg/String`，内容为 JSON | 原生执行器 → `data_collector.trigger` 或外部观察器 | RELIABLE、KEEP_LAST(1)、TRANSIENT_LOCAL | 采集生命周期和状态观察 |

原生节点名为 `spd_joint_command_executor_<序号>`；观察客户端为 `spd_collection_trigger_<pid>`。Pixi ROS 环境默认 `ROS_DOMAIN_ID=120`、`rmw_fastrtps_cpp`、仅本机自动发现，具体配置见 [pixi.toml](pixi.toml)。

### 4.1 命令话题字段与处理

消息定义：[JointCommand.msg](src/interfaces/tianji_spd_interfaces/msg/JointCommand.msg)。

| 字段 | 类型 | 语义 |
| --- | --- | --- |
| `schema_version` | uint16 | 必须为 1 |
| `robot_config` | string | `tianji_wuji2_v1` |
| `session_id` | string | 命令源会话身份 |
| `sequence` | uint64 | 命令序列 |
| `stamp` | builtin_interfaces/Time | 命令生成的 UTC 时间，不是仿真时间或接收时间 |
| `ready_mask` | uint8 | 1=双臂、2=右手、4=左手；可按位组合 |
| `joint_names` | string[54] | 固定关节名称及顺序 |
| `position_rad` | float64[54] | 关节目标，单位 rad |

54 维顺序：左臂 `[0:7]`、右臂 `[7:14]`、左手 `[14:34]`、右手 `[34:54]`。

订阅回调只校验并更新最新值邮箱，不在回调线程推进物理。原生执行器检查版本、配置、关节名、有限数值、序列和时间合法性；命令超过 100 ms 或比当前时间超前超过 5 ms 时拒绝。应用目标时再次检查新鲜度，接收到目标不代表已经授权运动。

KEEP_LAST(1)／BEST_EFFORT 是最新目标通道，不保证每次控制发送都被保留。当前轨迹录制实际物理状态，不将此 topic 当作 action 日志。

### 4.2 状态话题字段与发布时机

[ros_control.py](src/data_collector/ros_control.py) 将 `CollectionSession.snapshot()` 编码为 JSON，再交给原生 publisher。

| 字段 | 含义 |
| --- | --- |
| `collector_id`、`operation_id` | 采集器和操作身份 |
| `operation`、`state`、`message`、`error` | 当前操作、状态、提示和错误 |
| `episode_path`、`last_saved_path` | 当前及最近保存路径 |
| `state_frames`、`elapsed_s`、`max_frames` | 帧数、会话经过时间和上限 |
| `physics_paused` | 物理是否暂停 |
| `checkpoint_frames`、`auto_checkpoint_frames` | 人工检查点和失跟踪现场帧位置；无点时为 null |
| `last_outcome` | 最近完成结果 |

状态转换时尝试发布，心跳调用间隔达到 250 ms 时再次发布；不是硬实时频率保证。

**本地模式不会实际发布此话题。** [executor.cpp](src/spd_native/src/executor.cpp) 在 `subscribe=false` 时提前返回，既不创建命令 subscription，也不创建 status publisher；`publish_status()` 在没有 publisher 时不发送。本地 Viewer 通过进程内 `snapshot()` 获取状态，不依赖 `/spd/collection/status`。

### 4.3 不应误认为已提供的服务和话题

`ros_control.py` 中还列有以下服务名称，`trigger.py` 也保留对应 `std_srvs/srv/Trigger` 客户端：

```text
/spd/collection/start
/spd/collection/save
/spd/collection/discard
/spd/collection/checkpoint
/spd/collection/pause
/spd/collection/resume
/spd/collection/revert
/spd/collection/skip
```

**这些是服务名称，不是 topic；当前生产入口没有创建对应服务端。** 不能把客户端对象存在解释为接口可调用。正常采集只能通过主程序终端／窗口按键操作，状态观察器不是第二个控制终端。

本项目主流程也不发布 `/tianji/arm/{side}/state`、`/tianji/arm/{side}/action`、`/wuji/hand/{side}/state`、`/wuji/hand/{side}/action` 或相机图像话题。这些是外部统一录制规范的来源约定，不是 SPD 已有话题。未来 HDF5 的 `state/...`、`action/...`、`images/...` 是文件路径，不能当作 ROS topic。

`src/teleop_native/config/input_provenance/` 中的冻结参考脚本含有 ROS 声明，但不参与当前启动流程，不列为运行时话题。

## 5. 控制、物理与在线采集

| 数据流 | 处理位置 | 输出／约束 |
| --- | --- | --- |
| 终端与 GLFW 单键 → 内存队列 | `src/simulation/ros_viewer.py` | 携带场景 generation，旧场景按键被拒绝 |
| 队列 → 统一控制状态机 | `src/simulation/collection_control.py` | `r/s/d/q` 根据状态解释；无 ROS 按键 topic |
| 控制器 → 原生执行器／Physics | `src/spd_native/` | 唯一物理 owner 应用目标、冻结和恢复 |
| MuJoCo → 轨迹快照 | `src/data_collector/trajectory.py` | 物理 480 Hz，每 8 步采一帧，轨迹 60 Hz |
| 快照 → 文件协调与有界写队列 | `src/data_collector/session.py`、`recorder.py` | 整帧写入 HDF5；溢出／漏 tick 明确失败 |
| 物理／状态 → 在线窗口 | `src/simulation/viewer_window.py` 等 | 操作反馈，不是训练 RGB 数据流 |

名义物理及采样频率是仿真时间契约，不保证同等墙钟吞吐。暂停冻结物理和采样，不补帧；回退裁掉失败后缀，再从检查点继续。保存或丢弃完成后生成下一场景，Home 等待重新接手。

在线产物默认位于：

```text
data/episodes/YYYYMMDD/
├── dataset_config.json
├── episode_<UUID>.partial.h5
└── episode_<UUID>.h5
```

主要内容：

- `model/mjb`、`model/metadata`：模型快照、资源及元数据。
- `trajectory/tick`、`sim_time`、`monotonic_ns`：物理和主机时钟。
- `trajectory/qpos`、`qvel`、`robot_qpos`、`robot_qvel`：全场景和机器人实际状态。
- `trajectory/object_pose`、`hand_contact`、`hand_object`：物体位姿与手物接触。
- `collection_events/recovery_transition`、`control_flags`、可选 `rewind`：恢复、质量与回退记录。

完整字段见 [物理轨迹 Schema v2](docs/schema-v2.md)。在线不保存训练 RGB、执行器 ctrl 或 action；模型中存在时的 `trajectory/act` 是 MuJoCo 执行器激活状态，不是控制命令。

## 6. 采后校验和可视化

| 入口 | 输入 → 输出 | 是否使用 ROS topic |
| --- | --- | --- |
| `pixi run validate_episode FILE` | 完整 HDF5 → 格式／一致性校验报告 | 否 |
| `pixi run replay_episode FILE` | 内嵌模型与轨迹 → 逐帧恢复及误差报告；不推进物理、不渲染 | 否 |
| `pixi run spd-web --directory data/episodes` | 数据目录 → HTTP 服务 → 浏览器回放 | 否 |

浏览器入口为 [src/web_replay/cli.py](src/web_replay/cli.py)，默认 `127.0.0.1:8765`。它直接读取原始轨迹，不要求先离线渲染；HTTP 不是 ROS 通信。服务无身份认证，只用于本机或可信网络。