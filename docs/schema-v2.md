# 完整场景物理轨迹 HDF5 Schema v2

本规范定义 SPD 在线采集的原始 HDF5 轨迹：每段保存可独立重建的完整场景物理状态。它不保存 ROS 命令、执行器目标 `ctrl`、actions 或 RGB。控制操作见 [Pipeline.md](../Pipeline.md)。

## 兼容性

- schema-v1 的机器人 qpos＋JPEG 文件不转换、覆盖或删除。
- `recovery_transition=4` 和可选 `control_flags` 是 schema-v2 的 `collection_events` 扩展，不改变 trajectory 或每日配置。读取器必须接受值 `4`，不能将其判为损坏或改写为 `0`。
- 旧文件缺少恢复字段时报告 `recovery_annotated=false`；缺少质量字段时报告 `control_flags_annotated=false`，不能据此断言旧输入正常。

## 采样与时钟

- MuJoCo 物理步长为 1/480 秒；每 8 个物理步采一帧，仿真时间为 60 Hz。首帧是后台准备完成后的第一个物理步，不补写此前缓存。
- `tick` 是会话物理步编号，`sim_time` 是 MuJoCo 绝对仿真秒，`monotonic_ns` 是采集主机绝对单调纳秒。在线回退会恢复检查点的 tick/仿真时间并删去失败后缀；已保留帧的三条时间轴仍严格递增，主机单调时钟从不回退。
- 相邻帧 tick 必须相差 8，sim_time 必须相差 1/60 秒（允许浮点累积误差），`monotonic_ns` 必须严格递增。墙钟频率只能从 `monotonic_ns` 统计，不得伪造固定间隔。
- 会话逐步检测漏步；writer 检测采样间隔和非有限状态。重复或缺失帧报错并保留 partial，不覆盖、补零或静默跳过。
- 采样不等待新输入；短缺口的有界制动仍记录实际状态及质量标志。人工暂停、回退和持续失跟踪冻结整个物理世界与采样，恢复后延续采样相位且不补帧，主机时钟保留真实等待。源或求解 worker 故障 fail-closed，未完成段保留 partial。

## 文件和数据集布局

```text
<data_dir>/YYYYMMDD/
├── dataset_config.json
├── episode_<UUID>.partial.h5
└── episode_<UUID>.h5
```

日期在接受 start 时按本机日期固定，跨午夜不拆段；下一段使用新日期。单实例采集，输出目录不共享写入。当天 `dataset_config.json` 仅包含 `schema_version=2`、`robot_config=tianji_wuji2_v1`、`robot_joint_names`、`joint_unit=rad`、`physics_hz=480`、`state_rate_hz=60`；同日配置冲突时拒绝追加。模型、物体数和 qpos 维度可按段不同，以该段模型元数据为准。

```text
episode_<UUID>.h5
├── @schema_version = 2
├── @robot_config = "tianji_wuji2_v1"
├── @task                          UTF-8 任务名称
├── @success                       bool，操作者确认任务成功
├── @complete                      bool，完整关闭并通过校验
├── @abort_reason                  仅异常 partial 可出现
├── model/
│   ├── @model_sha256              MJB 原始字节哈希
│   ├── @metadata_sha256           metadata JSON 原始 UTF-8 字节哈希
│   ├── mjb                        uint8[B]，完整编译模型
│   └── metadata                   scalar UTF-8 JSON
├── collection_events/             新文件包含恢复与控制质量标签
│   ├── recovery_transition        uint8[N]，0 正常／1 开段／2 人工恢复／3 人工点回退恢复／4 失跟踪重新接手
│   ├── control_flags              可选 uint8[N]，区间累计输入／左右手质量位
│   └── rewind                     可选 compound[R]: frame_count int64, monotonic_ns int64
└── trajectory/
    ├── tick                       int64[N]
    ├── monotonic_ns               int64[N]
    ├── sim_time                   float64[N]，秒
    ├── qpos                       float64[N,nq]
    ├── qvel                       float64[N,nv]
    ├── robot_qpos                 float64[N,54]
    ├── robot_qvel                 float64[N,54]
    ├── hand_contact               bool[N,2]
    ├── object_pose                float64[N,O,7]，仅 O>0
    ├── hand_object                bool[N,2,O]，仅 O>0
    ├── act                        float64[N,na]，仅 na>0
    ├── mocap_pos                  float64[N,nmocap,3]，仅 nmocap>0
    ├── mocap_quat                 float64[N,nmocap,4]，仅 nmocap>0
    └── eq_active                  bool[N,neq]，仅 neq>0
```

所有 trajectory 数据集和逐帧事件字段的首维均为同一非零 `N`；可选字段只在模型存在相应维度时写入，不伪造零宽度数据集。`qpos/qvel` 包含机器人和全部动态场景自由度，采用 MuJoCo 原生坐标：free joint 为 7 个位置坐标、6 个速度自由度，不能以 qpos 地址索引 qvel。抽屉的有界 slide joint 各占一个位置和速度坐标，qpos 是从关闭位置向外打开的米数，不占机器人 54 维命令。

`robot_qpos/robot_qvel` 从完整状态按固定名称投影，顺序为左臂 7、右臂 7、左手 20、右手 20，单位分别为 rad/rad/s，且必须逐元素等于完整状态的对应投影，不是目标值。`object_pose` 是任务物体根 body 的世界坐标 `[x,y,z,qw,qx,qy,qz]`，包含动态物体和固定任务支架；具体顺序由 `metadata.object_names/object_body_ids` 定义，并在独立 `MjData` 上由当前 qpos 重新正运动学计算，不修改在线模拟状态。

## 恢复和控制质量字段

`recovery_transition` 为无额外属性的 `uint8[N]`，合法值为 0..4：`0` 正常、`1` 开段绑定过渡、`2` 人工暂停恢复、`3` 手动保存点回退恢复、`4` 持续失跟踪后的重新绑定接手。它不是动作命令或任务成功标签，也不承诺过渡持续一秒。采样区间内出现的非零阶段保留至该帧；多个非零值取最后一个，落在下一个采样点前的过渡也不得漏标。旧文件无此字段时不推断阶段。

`control_flags` 是新录制文件写入的可选、无额外属性 `uint8[N]` 位掩码，合法值为 0..31，高三位必须为零：

| 位值 | 含义 |
|---|---|
| `1` | 双臂／头腕输入退化 |
| `2` | 右手手指保持／等待有效输入，包含短缺口 |
| `4` | 左手手指保持／等待有效输入，包含短缺口 |
| `8` | 右手手指平滑重新接入 |
| `16` | 左手手指平滑重新接入 |

flags 是上次采样后所有物理步的按位 OR（首帧仅首步），同侧保持和重入可同时为 1。零只表示已标注区间未报告这些状态，不证明抓取成功、跟踪精度或辅助策略状态；显示用 `finger_modes` 不构成额外 HDF5 数据。读取器为旧文件零占位时必须保留 `control_flags_annotated=false`，不得冒充实测全零。

## 手–物接触语义

- 手索引固定为 left、right，根 body 分别为 `l_wrist`、`r_wrist`，各自包含全部子树几何；物体来自 scene manifest 的 objects 列表并包含物体子树几何，二者子树不得重叠。
- 每个物理步在 `mj_step` 后观察 solver-active（`efc_address` 非负）接触，并累计自上次 capture 以来的结果；首帧只覆盖首个物理步，之后通常覆盖 8 步。
- `hand_object[n,s,o]` 表示区间内手 `s` 曾与物体 `o` 有有效接触，`hand_contact[n,s]` 是该物体维的逻辑 OR。没有任务物体时 `hand_contact` 恒 false，且不创建 `hand_object` 或 `object_pose`。
- 不计机器人自碰撞和手与桌面／地面的接触。这些布尔值不是完整接触力，也不表示仅采样瞬间的接触。开段的 0 号手动保存点和后续手动点均允许接触。

## 模型快照与元数据

`model/mjb` 保存完整编译模型及其中的网格、纹理、相机和物理配置，恢复不依赖重新查找源 XML 或资产。`metadata` 至少包含：

- `snapshot_format`、精确 `mujoco_version`、`model_sha256`、`physics_hz`、`state_rate_hz`，以及模型 dimensions 和各 trajectory 字段的 dtype/shape；
- `robot_joint_names`、joint IDs、qpos/qvel 地址和完整 joint/body 映射；左右手根及几何映射、任务物体名称和 body/geom IDs；
- `task_manifest`、`scene_manifest`、实际采样布局、物理参数、seed、任务和有效采集配置，以及 `object_pose_convention`、`contact_convention`；
- 已编译 camera 数组和相机配置。零相机模型也可记录，不必构造 Renderer。version-2 相机配置保存 parent、以米为单位的局部 position、以度为单位的 `rpy_deg`，旋转为 `Rz(yaw) Ry(pitch) Rx(roll)`；历史内嵌 version-1 `look_at` 配置仍按原模型校验，不迁移或改写；
- 可访问的源模型、URDF、manifest、配置、网格和纹理路径及 SHA-256，仅作溯源，不是恢复依赖。

新场景的 `scene_manifest.physical_materials` 和对象 `surface_materials` 保存批准的有效滑动摩擦矩阵、材料/区域标签、来源和表面近似边界；对象 `friction/friction_range` 仍是原 geom 系数及采样范围，不是各接触点的最终有效值。MJB 的 `spd_material_friction` numeric 为 `[2, ...64 个 row-major 系数]`，`geom_user` 字段 0/1 为实例/类别、字段 2/3 为材料/区域 ID。材料数值、标签顺序和区域分类见 [architecture.md 的材料与接触](architecture.md#材料与接触)；旧文件无策略时不得推断新材料或用当前规则重写旧 MJB。

metadata JSON 使用排序键、紧凑分隔符、UTF-8、不转义非 ASCII 且拒绝 NaN。MJB 和 metadata 原始字节分别以 SHA-256 校验；哈希用于一致性检查，不是来源认证或安全签名。

恢复 MJB 必须使用记录时相同版本的 MuJoCo，并重新核验字段、维度、关节/物体地址和相机配置。因未记录插件状态，插件模型明确拒绝。采集期间不得修改模型、物体物理参数、相机或资产；变更后必须重建会话，不能混用旧模型快照。

带材料策略的新快照在物理推进、续跑或材料求解力诊断时，必须使用匹配版本的 MuJoCo 与材料感知 `_spd_native`：`material_step(model, data)` 推进，`material_forward(model, data)` 重建材料约束并计算力，原生 `Physics` 使用同一策略。此类刚体模型仅支持 Euler、implicit、implicitfast；RK4 或启用 EFM 明确拒绝。MJB 自带矩阵和区域标签而不需要源场景/资产，但裸 `mj_step/mj_forward` 不解释这些字段，不能等价推进。只读回放仅恢复记录姿态，使用普通 `mj_forward`，不推进物理也不提供新策略的求解力；不含策略的历史模型仍使用普通 MuJoCo step/forward，保持历史行为。

## 发布、partial 与失败

有界队列每次传递一整帧自有快照；`append_frame` 后不得再修改该帧数组。trajectory、恢复标签和 flags 必须同队列写入或裁剪，失败写入不得留下半行，最终文件只保留选中的前缀及其续采，不保留失败分支为示范。

只有关闭文件并校验行数、标签、类型、有限值、时钟、模型、metadata 和状态投影后，才标记 `complete=true` 并发布 `.h5`。中断、错误、漏 tick、队列溢出以及源/求解故障保留 `.partial.h5`；`max_frames=0` 表示不限，达到正数上限时可发布 `complete=true, success=false`，只有操作者显式保存才标记 `success=true`。公开校验器拒绝 `.partial.h5` 与 `complete=false`；内部 `allow_partial=True` 仅用于发布前验证，`valid=true` 不表示完整或成功。意外终止留下的未关闭 HDF5 是失败数据，不能改扩展名绕过检查。

## 在线检查点与回退边界

在线手动检查点是进程内完整 `MjData` 副本，同时保存 tick、保留目标、采样相位、未结束的接触累计和恢复/flags 累计。失跟踪现场快照只用于等待提示，不是首次重新接手的恢复来源；两者都不落盘，进程重启后不可恢复，且不等同于文件的 state-only 重建。

手动回退暂停物理和命令应用，保留检查点已接受的前 `N` 帧（`N=0` 合法），同步裁剪标签和 flags、恢复写入时钟并刷盘，再恢复完整状态和内部重绑定。稳定输入后自动续采，不补等待区间；采样仍严格每 8 tick 一帧，generation 丢弃旧异步结果。失跟踪后的首次重新接手只绑定当前现场，不恢复快照、不产生 `rewind`，也不创建或覆盖手动点。检查点保存尚未结束采样区间的恢复/flags 累计；回退恢复这些累计，不带回失败后缀。

可选 `/collection_events/rewind` 是一维可扩展 compound 数据集，字段顺序为 `frame_count: <i8`、`monotonic_ns: <i8`。每行标识零基 `frame_count` **之前**的分支边界和实际回退主机时间：`0 <= frame_count <= N`，`frame_count=N` 表示尚未追加续采帧。更早回退会删除大于新保留帧数的边界、保留等值边界；帧数非递减，事件时间严格递增。它不是完整失败尝试审计日志，旧的无回退 schema-v2 文件仍可读取。

最终轨迹不含从失败现场跳回检查点的状态；训练窗口不得跨 `rewind` 边界或任何新的非零恢复阶段起点，即使跳帧略过该点。文件只记录最终保留轨迹，不是所有尝试的审计日志。

## 独立状态重建

回放先校验完整文件，再从内嵌 MJB 加载独立模型。每帧重置 `MjData`，赋值 qpos/qvel 与存在的 act/mocap/equality 状态，设置 `data.time` 后调用普通 `mj_forward`；不推进物理、不恢复或下发命令、不创建相机渲染器，也不引入 ROS 原生依赖。它核对机器人状态投影和任务物体世界位姿，报告最大误差；四元数 `q` 与 `-q` 视作同一方向。

这只是独立状态重建，不是控制循环检查点：文件不保存 ctrl、外加力、求解器 warmstart 等全部推进历史。消费者必须从记录的实际状态开始，不能重跑目标控制来猜测物体轨迹。
