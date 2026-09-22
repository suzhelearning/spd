# 数据采集 HDF5 Schema

## 1. 范围

本文定义 SPD 的 **state-only schema-v1**：采集的是仿真环境中机器人的实际关节状态，而不是遥操作发布的 cmd。旧 `observations/commands` 扩展已移除。

一个 episode 对应一个 HDF5 文件，**只保存 MuJoCo 实际关节位置、仿真 RGB、各自时间戳和必要元数据**，不保存参考动作、执行器目标、插值后的下发命令、深度、IMU、速度、电流或训练副本。

ROS JointCommand 仅驱动 MuJoCo 执行器；完成物理步后，采集器按机器人关节名称读取 `data.qpos`。命令位置与实际位置可以因动力学、重力或接触而不同，采集器不以 `ctrl` 或源端目标代替 state，也不接入 RealSense 或硬件反馈。

训练端自行构造 state/action 配对；采集器不把未来实测位置声明为实际下发动作。文件仍没有 `actions` 组或 `action_type` 属性。当前实现位于 `src/spd/spd_vr/data_collector/`，相机配置为 `config/sim_cameras.yaml`，启动器默认输出到 `/data/TianjiSim/`。

## 2. 频率与维度

| 数据 | 原始采集频率 | 内容 |
|---|---:|---|
| 双臂实际关节位置 | 120 Hz 目标 | 左 7 + 右 7，共 14 维 |
| 双手实际关节位置 | 120 Hz 目标 | 左 20 + 右 20，共 40 维 |
| RGB | 每路 30 fps | 1280 × 720，源格式 RGB8，存储为 JPEG |
| 建议训练时间网格 | 30 Hz | 训练端自行按时间戳对齐 |

所有关节角使用 rad。120 Hz 是默认实际状态读取和记录目标，可通过采集配置调整；不修改仿真物理步长，也不承诺运行时达到该频率。
每条样本使用实际时间戳，不假设间隔严格等于 1/120 秒；实际频率必须由时间戳统计。

## 3. 文件结构

```text
/data/TianjiSim/
├── 20260922/
│   ├── dataset_config.json
│   ├── episode_<UUID>.h5
│   └── episode_<UUID>.partial.h5
└── 20260923/
    ├── dataset_config.json
    └── episode_<UUID>.h5
```

日期使用本机本地时间，在每次接受开始录制时确定；后台准备或录制期间跨日不会移动该段，下一段才使用新的日期目录。配置和命令行指定的是根目录，日期子目录自动追加。已有文件不移动。

```text
episode_<UUID>.h5
│
├── @schema_version = 1
├── @task                         # 任务标识，UTF-8
├── @success                      # bool，操作者确认的任务结果
├── @robot_config                 # 如 tianji_wuji2_v1
│
├── observations/
│   ├── arms/
│   │   ├── timestamp_ns          int64 [Na]
│   │   └── qpos                  float32 [Na,14]
│   └── hands/
│       ├── timestamp_ns          int64 [Nh]
│       └── qpos                  float32 [Nh,40]
│
└── images/
    ├── top/
    │   ├── timestamp_ns          int64 [Ft]
    │   └── jpeg                  vlen uint8 [Ft]
    └── left_wrist/
        ├── timestamp_ns          int64 [Fl]
        └── jpeg                  vlen uint8 [Fl]
```

`Na`、`Nh` 和各相机帧数独立，不要求相等。同一组内时间戳数量必须与数据行数一致。
上图展示原始观测契约的双相机示例，不是当前 SPD 相机数量的硬编码要求。只创建实际使用的相机组，不创建空流或全零图片。
当前 `config/sim_cameras.yaml` 启用 `top`、`left_wrist`、`right_wrist` 三路仿真视角，各自保存 `images/NAME/{timestamp_ns,jpeg}`；相机列表必须与数据集配置一致，变更配置时使用匹配的数据集。

每个 `jpeg[i]` 是一帧完整 JPEG 的一维 uint8 编码字节数组，不是解码后的像素矩阵。

## 4. 真实关节状态

拼接后的状态顺序固定为：

| 列范围，左闭右开 | 内容 |
|---|---|
| `[0:7]` | 左臂 |
| `[7:14]` | 右臂 |
| `[14:34]` | 左手 |
| `[34:54]` | 右手 |

数据集配置给出按此顺序排列的 54 个真实关节名称。
关节零位、正方向或机器人配置改变时，使用新的 `robot_config`，不得无标记混合。

`observations/{arms,hands}/qpos` 必须来自实际反馈，不使用目标角度代替。
SPD 按模型名称映射读取 MuJoCo 状态，分别形成双臂 14 维和双手 40 维快照；场景 free joints 不属于机器人状态数组。
原始硬件契约中 SDK 新反馈／有效性、双手均有新反馈才形成下一条快照的要求，不应被描述成 SPD 已提供的硬件采集功能。

## 5. 时间戳

- 使用同一采集主机的单调时钟，以本 episode 录制起点为零，单位 ns，存为 int64。
- 状态时间戳：完整状态快照在公共输入缓冲区可用的时间。
- 图像时间戳：完整 RGB 帧在公共输入缓冲区可用的时间，在 JPEG 编码之前记录。
- 录制开始前的缓存不补写为零时刻，也不重打时间戳冒充新样本。
- 各流时间戳单调非递减；同一时间戳有多个样本时，按存储顺序取最后一条。

观测契约不保存设备源时间戳、命令 UTC、曝光时间或多机时钟映射。采样不等待新的 ROS cmd；每次读取的是当时已推进的仿真状态。
观测时间支持按在线可用性对齐，不用于精确分析传感器、传输和执行延迟。多主机各自单调时钟不可直接混用。

## 6. 数据集级配置

每个日期目录的 `dataset_config.json` 保存该日共享配置，不在采集根目录新建共享配置，也不在每个 episode 重复完整内容；单日目录可独立校验和读取。

| 字段 | 要求 |
|---|---|
| `schema_version` | `1` |
| `robot_config` | 与 episode 属性一致 |
| `joint_names` | 按固定顺序排列的 54 个真实关节名 |
| `joint_unit` | `rad` |
| `policy_rate_hz` | `30`，训练端可选的名义网格，不是采样间隔保证 |
| `camera_names` | 实际使用的有序相机名称列表 |
| `image_width` / `image_height` | `1280` / `720` |
| `image_encoding` | `jpeg` |
| `decoded_color_order` | `RGB` |
| `jpeg_quality` | 采集采用的固定编码质量，当前为 `90` |

同一日期目录保持上述契约一致。配置不匹配时拒绝追加，不覆盖旧配置；新日期单独创建并校验自己的配置。
SPD 的运行采集配置由 `config/collect_sim.yaml` 及显式覆盖确定，实际配置逐段保存于 task_manifest；不将不同段的帧数上限当作数据集 schema。数据集配置只列实际启用相机。
相机内外参不是本版训练字段，不纳入此 schema；SPD 的运行视角定义在 `config/sim_cameras.yaml`，当前仍标为 provisional。

## 7. 操作员录制生命周期

SPD 运动授权与录制相互独立。启动订阅器或显式启用控制不自动开始 episode：

| 按键 | 作用 |
|---|---|
| `r` | 已启用且至少一组新鲜 ready、未 hold 时准备新 episode，随后按配置读取实际仿真 state／RGB |
| `s` | 停止当前段接收数据，后台保存；表示操作者确认成功，`success=true` |
| `d` | 停止并丢弃当前录制段，只删除该段未完成文件，不删除以前已保存的 episode |

Viewer 按键不需要回车，控制终端输入需要回车。录制按键不触发 HOME 或修改运动授权。
准备、保存或丢弃尚未完成时，不重复开始另一段；完成后可以再次按 `r`。
`e` 显式启用／禁用控制，`c` 清除候选与授权；二者都不会把当前物理模型重置到 HOME。

授权撤销、退出或已知采集失效时，当前未保存段不自动标记成功；保留 `.partial.h5`。分组 hold 与全部授权撤销是不同状态，不能仅凭 ready/hold 的变化推断录制已经停止。
采集失败只报告并结束数据段，不修改机器人控制授权；控制侧的会话、新鲜度与保持门独立生效。

独立触发器使用同样的 `r/s/d` 语义，无需回车；`q` 或 Ctrl+C 只关闭客户端，不结束录制。`/spd/collection/{start,save,discard}` 服务只管理采集，不授予运动权限；响应是接受确认，最终结果看携带相同 collector／operation ID 的 `/spd/collection/status`。

`config/collect_sim.yaml` 配置独立的状态／RGB 调度、输出路径、有界写入队列及 `max_frames`。`max_frames=0` 不限，正数按实际状态样本计数；达到上限结束并发布校验通过的 `.h5`，但 `success=false`，不能视作完成任务。只有操作者显式保存标记成功。每段采用 UUID 文件名，实际生效的采集配置与来源保存在已有 task_manifest JSON 中，不增加或重命名核心数据集字段。

## 8. 写入与最小校验

- 使用单个后台 HDF5 写入者；JPEG 编码、写盘和保存校验不在控制回调同步等待。仿真相机创建与同步渲染仍可能阻塞主循环，不保证录制时的实时频率。
- 数据集首维可追加；队列有界，溢出必须明确报告并结束当前数据段，不静默丢帧。
- 正在录制的文件以 `.partial.h5` 结尾，收到显式保存或达到帧数上限后关闭并校验，通过后发布为 `.h5`；前者 success=true，后者 success=false。
- 验证字段长度、关节维数、有限数值、时间戳顺序、配置匹配及 JPEG 可解码性。
- 校验失败的文件保持 `.partial.h5`，不会覆盖已有完整 episode。
- 未按 `s` 的退出不是成功确认；按 `d` 是显式丢弃，不保留该段训练文件。

## 9. 训练端职责与排除项

训练端自行选择时间偏移、时间网格、窗口长度、state/action 配对以及图像预处理。
不得把未记录的控制命令当作已知标签，也不得将未来实际 qpos 伪称原始下发动作。不要跨 episode 或已知断流区间构造样本。

不在采集文件中重复保存 30 Hz 副本、动作块、缩放图像、归一化数据，也不保存深度、IMU、速度、电流或力矩。排除参考动作、执行器目标、命令序号／session／ready／hold 和设备源时间戳。

核心观测保持为：**实际关节状态 + RGB + 各自时间戳**。

## 10. 元数据与旧文件边界

episode 保留 `task_manifest` JSON 属性，以及可用的 `scene`、`seed` 属性，用于描述实际采样场景、模型和有效采集配置；不改变核心 qpos 与图像字段。ROS 接口定义位于 `src/interfaces/tianji_spd_interfaces/msg/JointCommand.msg`，仅供执行使用，机器人模型与 manifest 位于 `src/description/tianji_wuji2/generated/`。

当前校验器要求 `observations` 恰好包含 `arms`、`hands`，并拒绝 `actions`。此前含 `observations/commands` 的扩展文件不是本契约，不会因为同为 `schema_version=1` 就被静默接受；已有文件不自动删除、覆盖或转换，使用前应显式识别并处理。

`pixi run validate_episode PATH` 校验 state-only 文件，`pixi run replay_episode PATH` 只读输出统计。它们不启动发布器、不驱动物理模型，也不凭观测恢复未记录的原始控制命令。
