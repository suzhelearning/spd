# 完整场景物理轨迹 HDF5 Schema v2

## 范围与迁移

在线仿真记录可独立恢复的完整场景物理轨迹，不记录 ROS cmd、执行器目标 `ctrl`、actions 或 RGB。输入接入、标定与 IK 属于外部 `tianji_teleop`；训练图像由后续离线渲染生成，`src/offline_rendering/` 当前仍仅预留。

本版是不兼容的 schema-v2：旧的机器人 qpos＋JPEG 文件不自动迁移、覆盖或删除。校验器拒绝旧 schema，同日 `dataset_config.json` 冲突时拒绝追加。升级应使用新的输出根目录，例如 `/data/TianjiSim-trajectories`，并将自定义采集配置更新为 version 2、state_rate_hz 60，删除 camera_rate_hz。

## 采样与时钟

- MuJoCo 物理步长严格为 1/480 秒；完整轨迹每 8 个物理步采一帧，仿真时间 60 Hz。
- 首帧来自后台准备完成后的第一个物理步，不补写开始前的缓存。
- `tick` 是会话物理步编号，`sim_time` 是 MuJoCo 绝对仿真秒数，`monotonic_ns` 是采集主机绝对单调纳秒，均不在 episode 内重置。
- 相邻帧 tick 差必须恰好为 8，sim_time 差为 1/60 秒（允许浮点累积误差），monotonic_ns 严格递增。实际墙钟频率由 monotonic_ns 统计；不得伪造固定墙钟间隔。
- 会话逐步检测漏步，writer 检测采样间隔与非有限状态；重复或缺失帧报错保留 partial，不覆盖、不补零、不静默跳过。
- 采样不等待新 JointCommand；没有新命令时仍记录实际物理状态。授权撤销中止录制，分组 hold 不冻结物理。

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
- 记录这些信息不等于已实现 10 秒无接触裁剪或检查点限制。

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

本地和 ROS 触发共用 CollectionSession。`r` 开始，`s` 保存并标记任务成功，`d` 丢弃当前段；不会替代本地 `e` 运动授权，也不会启动上游。

单个有界队列每次传递一整帧自有快照；调用 append_frame 后不得再次修改该帧数组。后台 HDF5 线程写入，保存／丢弃由协调 worker 执行，不在 ROS 控制回调同步等待。

只有关闭文件、校验行数／类型／有限值／时钟／模型／元数据／状态投影后，才标记 complete 并发布 `.h5`。中断、错误、漏 tick、队列溢出保留 partial，不静默丢帧。`max_frames=0` 不限，达到正数上限发布 `complete=true, success=false`；只有显式 `s/save` 标记 success=true。任务成功不是自动评分。

公开校验器拒绝 `.partial.h5` 和 complete=false 的文件。内部 `allow_partial=True` 用于最终发布前验证，其 valid=true 不表示 complete=true，更不表示采集成功。意外进程终止可能留下尚未关闭的 HDF5，应作为失败数据处理，不能通过改扩展名绕过 complete 检查。

## 独立恢复与当前边界

```bash
pixi run validate_episode /path/to/episode_<UUID>.h5
pixi run replay_episode /path/to/episode_<UUID>.h5
pixi run replay_episode /path/to/episode_<UUID>.h5 --expected-model-sha256 <SHA256>
```

replay_episode 先校验完整文件，再从内嵌 MJB 加载独立模型。每帧重置 MjData、赋值 qpos/qvel 与存在的 act/mocap/equality 状态、设置 data.time，然后调用 mj_forward；不调用 mj_step，不恢复或下发命令，不创建相机渲染器。它核对机器人状态投影和所有任务物体世界位姿，报告最大误差；四元数 q 与 -q 视作相同方向。

这是独立状态重建，不是恢复原控制循环的检查点：文件不保存 ctrl、外加力、求解器 warmstart 等全部推进历史。后续离线渲染应从记录的实际状态逐帧渲染，而不是重跑目标控制来猜物体轨迹。

本次不实现离线图像／分割生成、检查点／回退、接触裁剪、30 Hz 重采样、动作标签或数据增强。旧 align_30hz／filter_contacts 入口已移除；未来样本构建不得跨 episode、回退或裁剪边界，也不得把未来实测 qpos 伪称未记录的原始控制命令。
