# 完整场景物理轨迹 HDF5 Schema v2

## 范围与迁移

在线仿真记录可独立恢复的完整场景物理轨迹，不记录 ROS cmd、执行器目标 `ctrl`、actions 或 RGB。输入、相对绑定及 IK 由进程内 `pico2_hands.collection_session.TeleopSession` 和统一 `CollectionControl` 编排；可选外部 DDS 模式不是生产裸手流程，也没有本地重绑定保证。`src/offline_rendering/` 独立读取这些文件并生成渲染结果，绝不改写原轨迹。

旧机器人 qpos＋JPEG 文件不自动迁移、覆盖或删除；同日 `dataset_config.json` 冲突时拒绝追加。当前默认根目录 `/data/TianjiSim/trajectories`，采集配置 version 2、state_rate_hz 60。`recovery_transition=4` 与可选 `control_flags` 是 schema-v2 的兼容 `collection_events` 扩展，不改 trajectory 或每日配置。旧缺失恢复标签的文件报告 `recovery_annotated=false`；旧缺失质量标志报告 `control_flags_annotated=false`，不能据此断言旧输入正常。消费端需支持扩展值，不能将 4 误当损坏或改写成 0。

## 采样与时钟

- MuJoCo 物理步长严格为 1/480 秒；完整轨迹每 8 个物理步采一帧，仿真时间 60 Hz。
- 首帧来自后台准备完成后的第一个物理步，不补写开始前的缓存。
- `tick` 是会话物理步编号，`sim_time` 是 MuJoCo 绝对仿真秒数，`monotonic_ns` 是采集主机绝对单调纳秒。在线回退恢复检查点 tick／仿真时间并删除失败后缀；保留文件内的三条时间轴仍严格递增，主机单调时钟从不回退。
- 相邻帧 tick 差必须恰好为 8，sim_time 差为 1/60 秒（允许浮点累积误差），monotonic_ns 严格递增。实际墙钟频率由 monotonic_ns 统计；不得伪造固定墙钟间隔。
- 会话逐步检测漏步，writer 检测采样间隔与非有限状态；重复或缺失帧报错保留 partial，不覆盖、不补零、不静默跳过。
- 采样不等待新输入帧；短时缺口的有界制动仍记录实际状态及质量标志。人工暂停、回退或持续失跟踪自动暂停冻结整个物理世界和采样，恢复后延续原采样相位；不补写冻结区间，主机时钟保留真实等待。源／求解 worker 故障 fail-closed，未完成段保留 partial。

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

所有轨迹数据集首维必须相同且非空。可选字段按模型存在与否确定，不写虚构的零宽度数据集。`qpos/qvel` 包含机器人和所有动态场景自由度；nq 与 nv 不一定相等，free joint 是 7 个位置坐标、6 个速度自由度，不能拿 qpos 地址索引 qvel。抽屉的有界 slide joint 各占一个位置和一个速度坐标，qpos 表示从关闭位置向外打开的米数；不占用机器人 54 维命令。柜体、各抽屉和字母块使用不重叠的实例根。

`recovery_transition` 与轨迹逐行对应，值为 `0` 正常、`1` 开段绑定后的过渡、`2` 人工暂停恢复、`3` 手动保存点回退恢复、`4` 持续失跟踪后重新绑定接手（当前需跟踪稳定后首次按 `r`；旧数据可能来自自动恢复或快照回退流程）。它不是动作命令或任务成功标签，也不承诺所有恢复均持续一秒；本地绑定不叠加外部 DDS 的全关节一秒混合。每个物理步提交阶段，采样区间内出现的非零阶段保留到该区间输出；存在多个非零值时保留最后一个。阶段结束前发生但落在其后采样点的过渡也不会漏标。数据必须为 uint8[N]、值在 0..4、无额外属性；旧无此字段时不推断恢复阶段。

`control_flags` 是可选 uint8[N] 位掩码，新录制文件写入；长度与轨迹相同，无额外属性，合法值 0..31，高三位保留为零：

| 位值 | 含义 |
|---|---|
| `1` | 双臂／头腕输入退化 |
| `2` | 右手手指保持／等待有效输入，包含短缺口 |
| `4` | 左手手指保持／等待有效输入，包含短缺口 |
| `8` | 右手手指平滑重新接入 |
| `16` | 左手手指平滑重新接入 |

flags 是上次采样后所有物理步的按位 OR（首帧只含首步），不是仅采样瞬间的状态；因此同侧保持和重入位可同时为 1。位为零表示该已标注区间未报告这些状态，不代表成功抓取、跟踪精度合格或辅助策略未介入的通用证明。人工／自动检查点保存尚未结束区间的恢复／flags 累计，回退恢复该累计而不带回失败后缀。

短缺口期间即使保留 `live`／`blend` 接入状态并抑制新增虚影，实际因无效／过期输入保持目标的物理步仍置对应 `2`／`4` 位；不得因 HUD 尚未进入持续等待而漏标。显示状态通过进程内 `finger_modes` 单独传递，不修改 HDF5 位掩码或增加虚构的新鲜数据。

轨迹、恢复标签和 flags 作为整帧同队列写入／裁剪，失败写入不能留下半行；最终只保留选中前缀及其续采，失败分支不保留为示范。训练可过滤非零恢复或质量位，但不能把删除后的跨回退／重绑定段拼成连续动作。旧无 flags 文件仍可验证；读取器零占位的历史来源必须通过 `control_flags_annotated=false` 保留，绝不能冒充实测全零标志。

`robot_qpos/robot_qvel` 按固定名称从完整状态提取，顺序为左臂 7、右臂 7、左手 20、右手 20，单位 rad／rad/s。它们必须与完整状态对应投影逐元素相等，不是目标值。

`object_pose` 为任务物体根 body 的世界坐标 `[x,y,z,qw,qx,qy,qz]`，包含动态物体和固定任务支架；具体顺序在 metadata.object_names／object_body_ids 中定义。采样在独立 MjData 上从当前 qpos 重新计算正运动学，不使用 mj_step 后滞留的步前派生位置，不改变在线模拟状态。

## 手–物接触语义

- 两个手索引固定为 left、right，根 body 分别为 `l_wrist`、`r_wrist`，包含其全部子树几何。
- 物体来自 scene manifest 的 objects 列表，包含物体子树几何；手／物体子树必须互不重叠。
- 每个物理步在 mj_step 后观察 solver-active 接触（efc_address 非负），累计从上次 capture 至今的接触。第一帧只覆盖首个物理步，之后通常覆盖 8 步。
- `hand_object[n,s,o]` 表示该区间手 s 曾与物体 o 有有效接触；`hand_contact[n,s]` 是对应物体维的逻辑 OR。
- 没有任务物体时 hand_contact 恒 false，不创建 hand_object 或 object_pose。
- 不计机器人自碰撞、手与桌面／地面的接触。此布尔标签不等于完整接触力，也不声称只表示采样时刻的瞬时接触。
- 开段在运动前建立完整 0 号保存点，正常采集中 `r` 可更新；两者都允许接触。每次失跟踪后的首次 `r` 仅重新绑定、接手，不创建或覆盖手动保存点。

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

Quest／PICO 共用 `spd-quest-teleop`／`spd-pico-teleop --height-m HEIGHT`。按键由状态解释：待接手时 `r` 开始／重新接手；运动中 `r` 存检查点、`s` 人工暂停、`d` 回退并自动重新绑定续采；人工暂停中 `s` 重新绑定继续、`r` 保存整条、`d` 丢弃整条。`s` 继续和运动中 `d` 回退都无需额外 `r`。普通失跟踪仍保持现场、等待首次 `r`，不回退或覆盖检查点。手指平滑限速、输入新鲜度、非有限拒绝及有限越限饱和规则不变。默认仅开放只读状态，无远程 Trigger 控制。

终端／窗口不再识别组合键，空格和 `x` 不操作录制；只有 `r/s/d` 与退出键生效，单键按下即时派发。`q`／Ctrl+C／窗口 Esc 退出不代替保存，未完成段保留 partial。回退和内部重绑定期间暂停采样，但恢复条件满足后自动续采，不补写等待时段，不要求再次按键。

单个有界队列每次传递一整帧自有快照；调用 append_frame 后不得再次修改该帧数组。后台 HDF5 线程写入，保存／丢弃由协调 worker 执行，不在 ROS 控制回调同步等待。

只有关闭文件、校验行数／标签／类型／有限值／时钟／模型／元数据／状态投影后，才标记 complete 并发布 `.h5`。中断、错误、漏 tick、队列溢出保留 partial。`max_frames=0` 不限，达到正数上限发布 `complete=true, success=false`；只有人工暂停中的 `r` 显式保存标记 success=true，任务成功不是自动评分。人工暂停中的 `d` 删除当前整条，不影响已保存段；运动中的 `d` 仅裁剪检查点之后的失败后缀。

保存和丢弃完成后都直接生成全目录随机新任务、新 seed／布局／桌面，机器人初始化 Home、零速度，不继承旧状态、不进入准备区或执行回零运动，等待新的 `r` 绑定开段。首次显式任务／场景／桌距不锁定后续任务。新场景清空旧检查点及授权，不自动录制；保存或丢弃失败不更换当前场景。

公开校验器拒绝 `.partial.h5` 和 complete=false 的文件。内部 `allow_partial=True` 用于最终发布前验证，其 valid=true 不表示 complete=true，更不表示采集成功。意外进程终止可能留下尚未关闭的 HDF5，应作为失败数据处理，不能通过改扩展名绕过 complete 检查。

### 在线检查点与回退边界

在线手动检查点是进程内完整 `MjData` 副本，另存 tick、保留目标、采样相位、未结束的接触累计与恢复／flags 累计；另有失跟踪现场快照仅用于等待提示，不作为首次 `r` 的恢复来源。它们与磁盘 state-only 恢复不是同一契约，不落盘，不能在进程重启后恢复。

运动中 `d` 先暂停物理和命令应用、清除授权／候选，唯一写线程保留手动检查点时已接受的前 N 帧，同步裁剪标签及 flags、恢复写入时钟并刷盘，物理线程再恢复完整状态并发起内部重绑定。N=0 合法；稳定输入到来后自动续采，不停留在人工暂停，无需 `r/s`。人工暂停中的 `s` 同样直接发起重新绑定和续采授权。失跟踪后的首次 `r` 则仅绑定当前现场，不走快照恢复、不产生 `rewind` 或覆盖手动点。generation 拒绝旧结果；续采仍严格每 8 tick 一帧，等待区间不补帧。

可选 `/collection_events/rewind` 是一维可扩展 compound 数据集，字段顺序为 `frame_count: <i8`、`monotonic_ns: <i8`。每行标识零基帧索引 `frame_count` **之前**的分支边界及实际回退主机时间；`0 <= frame_count <= N`，等于 N 表示尚未追加续采帧。后续更早回退删除大于新保留帧数的边界，等值边界保留；帧数非递减，事件时间严格递增。此记录不是完整失败尝试审计日志。没有回退的旧 schema-v2 文件仍可读取。

最终轨迹不包含被裁掉的失败现场到检查点的跳变；训练窗口不得跨 `rewind` 边界或新的非零恢复阶段起点，即使通过跳帧略过它们。暂停／回退期间的主机时间差真实保留，不补帧。文件仅含最终保留轨迹，不是所有尝试的审计日志。

## 独立恢复与当前边界

```bash
pixi run validate_episode /path/to/episode_<UUID>.h5
pixi run replay_episode /path/to/episode_<UUID>.h5
pixi run replay_episode /path/to/episode_<UUID>.h5 --expected-model-sha256 <SHA256>
```

replay_episode 先校验完整文件，再从内嵌 MJB 加载独立模型。每帧重置 MjData、赋值 qpos/qvel 与存在的 act/mocap/equality 状态、设置 data.time，然后调用 mj_forward；不调用 mj_step，不恢复或下发命令，不创建相机渲染器。它核对机器人状态投影和所有任务物体世界位姿，报告最大误差；四元数 q 与 -q 视作相同方向。

这是独立状态重建，不是恢复原控制循环的检查点：文件不保存 ctrl、外加力、求解器 warmstart 等全部推进历史。后续离线渲染应从记录的实际状态逐帧渲染，而不是重跑目标控制来猜物体轨迹。

本版实现进程内检查点／回退及读取后的训练视觉增强，但不实现采后接触裁剪、30 Hz 重采样或动作标签。旧 align_30hz／filter_contacts 入口已移除；离线渲染逐行保留源帧，不代替时间网格处理。训练序列拒绝跨回退／重绑定边界，不得把未来实测 qpos 伪称原始命令。CONTROL-TYPE approach／alignment／grasp／insertion 辅助及其单独标注是**未来计划，尚未实现**；当前提示仅为任务／状态／检查点／手指等待，现有恢复／flags 不是这些辅助动作标签。

## 离线渲染伴随文件（render schema 2）

渲染产物为独立的 `episode_<ID>.render.h5`，不是采集 schema-v2 的新字段。源完整轨迹及同目录 dataset_config.json 必须可用；服务器只需匹配 MuJoCo 精确版本，无须采集主机上的源 XML、纹理目录或 ROS。

相机逻辑名固定为 top、left_wrist、right_wrist，位置和投影参数完全来自内嵌模型。位置尚未定稿；模型 camera_config.calibration_revision 缺失／空白／含 provisional 时默认拒绝。只有显式诊断开关允许预览，结果标为 diagnostic_only，不把临时相机升级为正式标定。URDF 相机安装格式及转换尚待用户定稿，不在渲染配置中添加猜测外参。

```text
episode_<ID>.render.h5
├── @render_schema_version = 2
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

正实例 ID 复用当前 episode 的 scene_manifest.objects.instance_id，所有属于同一物体子树的外观 mesh 共享同一个 ID。0=天空或无 GEOM，-1=机器人外观（非任务 group1），-2=其他非任务环境，-3=桌子（scene_table 及 scene_detail_table_* 外观，包括桌腿）。盘架／杯架／箱体／柜体／各抽屉仍使用正 ID。实例 ID 不意味着跨 episode 追踪同一个实物。底层 MuJoCo 分割返回 `(object_id, object_type)`，仅 GEOM 类型可索引几何映射；不能将 RGB 编码色号直接当实例 ID。schema 1 混合了桌子和环境，不兼容新的增强语义，必须从原轨迹重新渲染到新输出路径，禁止伪造新版本号。

源文件只读，并在渲染／复用前后核验整文件 SHA-256。每帧恢复还会检查机器人状态和任务物体位姿；不调用 mj_step。每个 episode 的一个 EGL worker 顺序渲染三视角，帧间队列不传输图像。

独占 `.lock` 保护目标文件，先写 `.render.partial.h5`，完成并校验后通过同目录硬链接无覆盖发布，再移除 partial。失败不覆盖已有完整文件；已存在 partial／lock 需人工确认无活动进程后处理。中断不自动重试，完整结果仅在源和设置身份匹配、全部内容校验通过后复用。改变分辨率、相机／源模型或编码设置应使用新输出目录。

不同 GPU／驱动上下文的光栅化和 JPEG 不保证字节级一致，输出记录实际环境用于溯源。正常恢复误差容限为 1e-6，复用时相机变换比较为 1e-12；哈希保证已有输出未改变，不是跨硬件图像完全相同的承诺。

## 读取后的训练视觉增强

`training_data.RenderedSequence(render, source_path=...)` 只读已完成的 render schema 2 和关联原轨迹，构造时校验源 SHA-256，读取前后检查文件身份／大小／时间签名。每次读取自行开关 HDF5 文件，不把句柄带入 fork worker；支持上下文管理器。元数据／选中帧校验不替代渲染器的完整像素校验和与重建验证。

`read(indices, augmentation=VisualAugmenter(...), seed=...)` 要求非空严格递增帧索引，不改变帧率、不补帧。边界集合是所有 `rewind.frame_count`，加上 `recovery_transition[i] != 0` 且与前一帧不同的每个索引（首帧前视为 0）；即每个非零恢复阶段的起点。若 `first_index < boundary <= last_index` 就拒绝读取，非连续选帧或过滤中间过渡帧不能绕过；从边界本身开始的同段序列允许。恢复回到 0 本身不新增边界，质量 flags 改变也不是额外分段规则。

返回 RGB `[T,C,H,W,3]`、整型实例掩码 `[T,C,H,W]`、source_index／原时钟、全部轨迹状态、恢复／回退／control_flags 标签和相机位姿。旧缺失恢复标签保持缺失，`reader.recovery_annotated=false`；旧缺失 flags 时 `events["control_flags"]` 为 uint8 零占位，且 `reader.control_flags_annotated=false`，必须保留此 provenance，不能当作经过标注的正常输入。没有动作监督输出。

`VisualAugmenter` 接受 uint8 `(...,H,W,3)` 和对应整型掩码。物体按正实例 ID 独立采样颜色；桌子 -3 与背景 -2／0 独立选择纹理；机器人 -1 不参与变换。一个 seed 的参数在时序和多相机间共享，明暗细节保留，输入数组／掩码不被就地修改。支持程序化纹理和用户 RGB 纹理库，计划记录 seed、各实例颜色、表面参数和纹理哈希。当前为 CPU NumPy 实现；纹理在归一化图像坐标中共享，不等同于几何投影纹理。CLI `pixi run spd-augment` 只创建新 PNG 预览及打印参数，不写回 HDF5。
