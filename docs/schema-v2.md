# 完整场景物理轨迹 HDF5 Schema v2

## 范围与迁移

在线仿真记录可独立恢复的完整场景物理轨迹，不记录 ROS cmd、执行器目标 `ctrl`、actions 或 RGB。输入接入、标定与 IK 属于外部 `tianji_teleop`；`src/offline_rendering/` 独立读取这些文件并生成渲染结果，绝不改写原轨迹。

本版是不兼容的 schema-v2：旧的机器人 qpos＋JPEG 文件不自动迁移、覆盖或删除。校验器拒绝旧 schema，同日 `dataset_config.json` 冲突时拒绝追加。升级应使用新的输出根目录，例如 `/data/TianjiSim-trajectories`，并将自定义采集配置更新为 version 2、state_rate_hz 60，删除 camera_rate_hz。

## 采样与时钟

- MuJoCo 物理步长严格为 1/480 秒；完整轨迹每 8 个物理步采一帧，仿真时间 60 Hz。
- 首帧来自后台准备完成后的第一个物理步，不补写开始前的缓存。
- `tick` 是会话物理步编号，`sim_time` 是 MuJoCo 绝对仿真秒数，`monotonic_ns` 是采集主机绝对单调纳秒。在线回退恢复检查点 tick／仿真时间并删除失败后缀；保留文件内的三条时间轴仍严格递增，主机单调时钟从不回退。
- 相邻帧 tick 差必须恰好为 8，sim_time 差为 1/60 秒（允许浮点累积误差），monotonic_ns 严格递增。实际墙钟频率由 monotonic_ns 统计；不得伪造固定墙钟间隔。
- 会话逐步检测漏步，writer 检测采样间隔与非有限状态；重复或缺失帧报错保留 partial，不覆盖、不补零、不静默跳过。
- 采样不等待新 JointCommand；没有新命令时仍记录实际物理状态。非暂停造成的授权撤销中止录制，分组 hold 不冻结物理。显式暂停／回退冻结物理和采样，恢复后延续原采样相位；墙钟间隔可包含操作等待。

## 文件和数据集布局

```text
<data_dir>/YYYYMMDD/
├── dataset_config.json
├── episode_<UUID>.partial.h5
└── episode_<UUID>.h5
```

日期在接受 start 时以本机本地日期固定；跨午夜不拆段，下一段进入新日期。单实例采集，不共享输出目录写入。每天的数据集配置只包含：

`schema_version=2`、`robot_config=tianji_wuji2_v1`、`robot_joint_names`、`joint_unit=rad`、`physics_hz=480`、`state_rate_hz=60`。

不同场景的模型、物体数和 qpos 维度可以不同，由每段模型元数据定义。`--output` 优先于 `SPD_EPISODE_OUTPUT`，再使用采集配置的 data_dir；配置内相对路径相对配置文件解析。

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
├── collection_events/             可选，仅发生回退时创建
│   └── rewind                     compound[R]: frame_count int64, monotonic_ns int64
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

所有轨迹数据集首维必须相同且非空。可选字段按模型存在与否确定，不写虚构的零宽度数据集。`qpos/qvel` 包含机器人和所有动态场景自由度；nq 与 nv 不一定相等，free joint 是 7 个位置坐标、6 个速度自由度，不能拿 qpos 地址索引 qvel。

`robot_qpos/robot_qvel` 按固定名称从完整状态提取，顺序为左臂 7、右臂 7、左手 20、右手 20，单位 rad／rad/s。它们必须与完整状态对应投影逐元素相等，不是目标值。

`object_pose` 为任务物体根 body 的世界坐标 `[x,y,z,qw,qx,qy,qz]`，包含动态物体和固定任务支架；具体顺序在 metadata.object_names／object_body_ids 中定义。采样在独立 MjData 上从当前 qpos 重新计算正运动学，不使用 mj_step 后滞留的步前派生位置，不改变在线模拟状态。

## 手–物接触语义

- 两个手索引固定为 left、right，根 body 分别为 `l_wrist`、`r_wrist`，包含其全部子树几何。
- 物体来自 scene manifest 的 objects 列表，包含物体子树几何；手／物体子树必须互不重叠。
- 每个物理步在 mj_step 后观察 solver-active 接触（efc_address 非负），累计从上次 capture 至今的接触。第一帧只覆盖首个物理步，之后通常覆盖 8 步。
- `hand_object[n,s,o]` 表示该区间手 s 曾与物体 o 有有效接触；`hand_contact[n,s]` 是对应物体维的逻辑 OR。
- 没有任务物体时 hand_contact 恒 false，不创建 hand_object 或 object_pose。
- 不计机器人自碰撞、手与桌面／地面的接触。此布尔标签不等于完整接触力，也不声称只表示采样时刻的瞬时接触。
- 在线检查点另外在独立 scratch MjData 刷新当前接触并使用同一手／物体映射；任一手的当前有效接触都会拒绝存档。它不使用区间累计值，也不修改在线积分历史。超过 10 秒的无接触裁剪仍未实现。

## 模型快照与元数据

MJB 保存编译模型及其中的网格、纹理、相机和物理配置，不需要恢复时重新查找源 XML 或网格文件。metadata 包含：

- snapshot_format、精确 mujoco_version、model_sha256、physics_hz、state_rate_hz。
- 模型 dimensions 和各 trajectory 字段的 dtype／shape。
- robot_joint_names、joint IDs、qpos／qvel 地址、完整 joint／body 映射。
- 左右手根及几何映射，任务物体名称、body／geom IDs。
- task_manifest、scene_manifest，实际采样的布局、物理参数、seed、任务和有效采集配置。
- 已编译的 camera 数组与相机配置文档；零相机模型也可记录，无须构造 Renderer。
- 可访问的源模型／URDF／manifest／配置／网格／纹理路径与 SHA-256 溯源；这些路径仅用于溯源，不是恢复依赖。
- object_pose_convention 和 contact_convention。

元数据 JSON 使用排序键、紧凑分隔符、UTF-8、不转义非 ASCII 字符且拒绝 NaN。文件字节与元数据分别校验 SHA-256；哈希用于一致性检查，不是来源认证或安全签名。

MJB 恢复严格要求记录时相同的 MuJoCo 版本，并重新核验字段、维度、关节／物体地址和相机配置。插件模型因未记录插件状态被明确拒绝。采集期间不得修改模型、物体物理参数、相机或资产；更改后重建会话，不能混用旧模型快照。

## 生命周期与错误

本地和 ROS 触发共用 CollectionSession。`r` 开始，`s` 保存并标记任务成功，`d/n` 丢弃／跳过当前段。左踏／`k` 创建最新无接触检查点，中踏／SPD 本地 `p` 切换暂停与恢复，右踏短按／`b` 回退并保持暂停，右踏长按至少 1 秒后松开跳过。暂停状态的中踏是本地显式授权请求：检查新鲜目标及对齐门限后才授权并恢复；失败保持暂停。ROS `resume` 和独立 `u` 仍要求已有本地授权，不能借远程操作绕过。不会启动或重新标定上游。检查点在保存／丢弃／新段开始后失效。

单个有界队列每次传递一整帧自有快照；调用 append_frame 后不得再次修改该帧数组。后台 HDF5 线程写入，保存／丢弃由协调 worker 执行，不在 ROS 控制回调同步等待。

只有关闭文件、校验行数／类型／有限值／时钟／模型／元数据／状态投影后，才标记 complete 并发布 `.h5`。中断、错误、漏 tick、队列溢出保留 partial，不静默丢帧。`max_frames=0` 不限，达到正数上限发布 `complete=true, success=false`；只有显式 `s/save` 标记 success=true。任务成功不是自动评分。

公开校验器拒绝 `.partial.h5` 和 complete=false 的文件。内部 `allow_partial=True` 用于最终发布前验证，其 valid=true 不表示 complete=true，更不表示采集成功。意外进程终止可能留下尚未关闭的 HDF5，应作为失败数据处理，不能通过改扩展名绕过 complete 检查。

### 在线检查点与回退边界

在线检查点是进程内完整 `MjData` 副本，另存 tick、保留关节目标、采样相位和未结束的接触累计。它与下述磁盘 state-only 恢复不是同一契约。检查点不落盘，不能在进程重启后恢复。

回退期间物理和命令应用冻结，授权与候选清除。唯一写线程处理 FIFO 裁剪事件，保留检查点时已接受的前 N 帧，删除其余所有轨迹行，恢复写入时钟并刷盘后确认；物理线程才恢复完整状态并进入 paused。出错只保留 partial，不发布完成文件；重新采样仍严格每 8 tick 一帧。

可选 `/collection_events/rewind` 是一维可扩展 compound 数据集，字段顺序为 `frame_count: <i8`、`monotonic_ns: <i8`。每行标识零基帧索引 `frame_count` **之前**的分支边界及实际回退主机时间；`0 <= frame_count <= N`，等于 N 表示尚未追加续采帧。后续更早回退删除大于新保留帧数的边界，等值边界保留；帧数非递减，事件时间严格递增。此记录不是完整失败尝试审计日志。没有回退的旧 schema-v2 文件仍可读取。

最终轨迹不包含失败现场到恢复状态的瞬间跳变；后续训练窗口仍不得跨 `/collection_events/rewind` 标识的边界。暂停／回退期间的主机时间差真实保留，不用补帧掩盖。

## 独立恢复与当前边界

```bash
pixi run validate_episode /path/to/episode_<UUID>.h5
pixi run replay_episode /path/to/episode_<UUID>.h5
pixi run replay_episode /path/to/episode_<UUID>.h5 --expected-model-sha256 <SHA256>
```

replay_episode 先校验完整文件，再从内嵌 MJB 加载独立模型。每帧重置 MjData、赋值 qpos/qvel 与存在的 act/mocap/equality 状态、设置 data.time，然后调用 mj_forward；不调用 mj_step，不恢复或下发命令，不创建相机渲染器。它核对机器人状态投影和所有任务物体世界位姿，报告最大误差；四元数 q 与 -q 视作相同方向。

这是独立状态重建，不是恢复原控制循环的检查点：文件不保存 ctrl、外加力、求解器 warmstart 等全部推进历史。后续离线渲染应从记录的实际状态逐帧渲染，而不是重跑目标控制来猜物体轨迹。

本版实现进程内检查点／回退，但不实现采后接触裁剪、30 Hz 重采样、动作标签或数据增强。旧 align_30hz／filter_contacts 入口已移除；离线渲染逐行保留源帧，不代替时间网格处理。未来样本构建不得跨 episode、回退或裁剪边界，也不得把未来实测 qpos 伪称未记录的原始控制命令。

## 离线渲染伴随文件（render schema 1）

渲染产物为独立的 `episode_<ID>.render.h5`，不是采集 schema-v2 的新字段。源完整轨迹及同目录 dataset_config.json 必须可用；服务器只需匹配 MuJoCo 精确版本，无须采集主机上的源 XML、纹理目录或 ROS。

相机逻辑名固定为 top、left_wrist、right_wrist，位置和投影参数完全来自内嵌模型。位置尚未定稿；模型 camera_config.calibration_revision 缺失／空白／含 provisional 时默认拒绝。只有显式诊断开关允许预览，结果标为 diagnostic_only，不把临时相机升级为正式标定。URDF 相机安装格式及转换尚待用户定稿，不在渲染配置中添加猜测外参。

```text
episode_<ID>.render.h5
├── @render_schema_version = 1
├── @complete
├── @source_sha256 / @model_sha256 / @source_metadata_sha256
├── @settings_sha256 / @metadata_sha256
├── metadata                       scalar canonical UTF-8 JSON
├── frames/
│   ├── source_index               int64[N]，逐行 0..N-1
│   ├── tick                       int64[N]，等于源轨迹
│   ├── sim_time                   float64[N]，等于源轨迹
│   └── monotonic_ns               int64[N]，等于源轨迹
└── cameras/{top,left_wrist,right_wrist}/
    ├── jpeg                       vlen uint8[N]，RGB JPEG
    ├── instance_id                int32[N,H,W]，无损 LZF
    ├── position                   float64[N,3]，相机世界位置
    └── rotation                   float64[N,3,3]，camera-to-world 旋转
```

默认 W=224、H=168、JPEG quality=90，subsampling=0；尺寸与编码设置进入 metadata 和 settings_sha256。所有 dataset 带逐行内容 SHA-256；JPEG 摘要包含每帧长度，固定宽度数组按连续 little-endian 字节累积。发布前和复用时流式验证实际 JPEG 解码、类型／维度、掩码 ID、时钟关联、变换矩阵与校验和，不一次读入整段图像。

metadata 包含源身份、原始编译相机定义与临时／已确认状态、渲染配置与引擎版本、实际 GL vendor／renderer／version、GPU EGL 设备号、实例映射、恢复误差摘要。相机世界变换从每帧 mj_forward 后的 cam_xpos／cam_xmat 获取，不用机器人腕部位置代替相机光学位姿。

正实例 ID 复用当前 episode 的 scene_manifest.objects.instance_id，所有属于同一物体子树的外观 mesh 共享同一个 ID。0=天空或无 GEOM，-1=机器人外观（非任务 group1），-2=非任务环境。盘架／杯架／箱体是任务物体，仍使用其正 ID。实例 ID 不意味着跨 episode 追踪同一个实物。底层 MuJoCo 分割返回 `(object_id, object_type)`，仅 GEOM 类型可索引几何映射；不能将 RGB 编码色号直接当实例 ID。

源文件只读，并在渲染／复用前后核验整文件 SHA-256。每帧恢复还会检查机器人状态和任务物体位姿；不调用 mj_step。每个 episode 的一个 EGL worker 顺序渲染三视角，帧间队列不传输图像。

独占 `.lock` 保护目标文件，先写 `.render.partial.h5`，完成并校验后通过同目录硬链接无覆盖发布，再移除 partial。失败不覆盖已有完整文件；已存在 partial／lock 需人工确认无活动进程后处理。中断不自动重试，完整结果仅在源和设置身份匹配、全部内容校验通过后复用。改变分辨率、相机／源模型或编码设置应使用新输出目录。

不同 GPU／驱动上下文的光栅化和 JPEG 不保证字节级一致，输出记录实际环境用于溯源。正常恢复误差容限为 1e-6，复用时相机变换比较为 1e-12；哈希保证已有输出未改变，不是跨硬件图像完全相同的承诺。
