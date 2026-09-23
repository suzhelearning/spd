# 仿真采集架构

## 1. 边界与数据流

SPD 是 **ROS 关节命令订阅端与 MuJoCo 仿真数据采集端**。人体标定、共享根 DLS/Ruckig、Hand2 映射与 ROS 发布属于独立上游 `tianji_teleop-ros2`。上游普通入口 `bash bash/run_pico_hand_sim.sh --height-m HEIGHT` 使用 `r` 标定、`s` 显式启动。SPD 不请求或管理上游标定、跟随、暂停；本地暂停时上游继续发布目标。SPD 仅对收到的合法目标做一秒接入，不实现 IK 或实机控制。

```text
独立上游：输入 / 标定 / 求解 / ROS JointCommand 发布
                              ↓
SPD interfaces：校验 → 最新候选 → 会话与显式授权 → 分组 hold
                              ↓
SPD simulation：按名称映射目标 → position actuators → MuJoCo 物理积分
                              ↓
              完整场景状态 / 任务物体 / 手–物接触
                     ↙                    ↘
       data_collector：60 Hz 轨迹     Viewer：操作反馈与目标曲线
                     ↓
       模型快照 + .partial.h5 → 校验 → .h5
                     ↓
       replay_episode：独立逐帧恢复（不推进物理、不渲染）
                     ↓
       offline_rendering：按 episode 分配到 EGL GPU，生成 RGB／稳定实例掩码
```

正式入口为前台 `pixi run spd-sim`，以及模型、场景、数据检查命令。不存在独立停止命令、旧 PICO、HDF5 命令发布或 Zenoh 遥操作启动器和转发 shim。`replay_episode` 从文件内嵌模型恢复状态并验证，不把历史观测用作合成发布目标。

## 2. 源码与资源职责

| 路径 | 职责 |
|---|---|
| `src/interfaces/` | JointCommand wire 契约、校验、邮箱、订阅执行器与授权／保持门 |
| `src/simulation/` | 机器人 MuJoCo 物理执行、ROS Viewer、窗口与场景查看 |
| `src/cameras/` | 世界／腕部仿真相机与 RGB 获取 |
| `src/data_collector/` | 配置、采集状态机、ROS 控制、完整物理轨迹与模型快照、独立恢复验证 |
| `src/offline_rendering/` | spawn 多 GPU 调度、原生 EGL、逐帧恢复渲染与输出校验 |
| `src/description/` | manifest、模型编译与资源定位 |
| `src/environments/spd_envs/` | 独立环境包，任务注册、随机化、场景生成与重置检查 |
| `src/tianji_wuji2/tianji_wuji2/assets/` | 原始 URDF、网格和碰撞资产 |
| `src/tianji_wuji2/tianji_wuji2/generated/` | 编译后的可加载模型与 manifest |
| `src/interfaces/tianji_spd_interfaces/` | ROS 2 `JointCommand.msg` 与接口构建元数据 |
| `config/` | 采集、临时预览相机和 `render_server.yaml` 八卡渲染配置 |
| `bash/` | 前台订阅／采集启动入口与独立触发终端 |
| `data/` | 采集产物和已有样本，不随代码清理删除 |

运行时 Python 包直接位于 `src/`，按职责使用 `interfaces`、`simulation`、`cameras`、`data_collector`、`description`，不保留统一外层包或旧导入兼容层。根目录 `setup.py` 安装这些包，发行包名仍为 `spd`；依赖方向为 `spd → spd-envs`，独立环境包保持 `spd_envs`，只负责场景，不依赖 ROS 或遥操作算法，也不硬编码机器人路径。资源定位通过 `description/model_builder.py` 的 `workspace_root()`、`description_root()` 和 `config_root()`，不以调用者当前目录猜测资源位置。

`pixi.toml` / `pixi.lock` 是受维护运行环境；ROS 接口构建到 `.ros/{build,install}`，与原始源文件和机器人生成模型分离。`ros-build-interfaces` 使用 `src/interfaces/tianji_spd_interfaces`，SPD 不自动构建上游工作区。

## 3. 订阅契约与授权状态

接口类型 `tianji_spd_interfaces/msg/JointCommand`，话题 `/spd/tianji_wuji2/v1/joint_command`，`schema_version=1`，`robot_config=tianji_wuji2_v1`。54 维顺序是左臂 7、右臂 7、左手 20、右手 20，单位 rad。消息保留原 wire 字段，不因目录迁移改变。

订阅回调校验名称顺序、维度、有限值、ready 组限位、session、递增 sequence 与 UTC 新鲜度，完整通过后才原子替换最新候选。物理 tick 消费时再次校验年龄；按 manifest 名称预计算 qpos/actuator 地址，场景 free joints 不改变机器人索引。

SPD 统一使用本地状态化 r/s/d：待开始 r 开段，暂停 s 恢复，暂停 d 回退后自动恢复。这些操作调用受控的一秒目标接入，不直接将大目标差应用到执行器。计时从准备完成后的首个物理应用步开始，以主机单调时钟计算五次平滑权重；起点是实际 qpos，终点每步取最新合法目标。1 秒结束目标混合，不保证物理到位。普通 authorize API 的 0.15 rad 差值门仍保留，但不再提供 e/c 快捷键或该门限的交互选项。

物理软限位允许少量实测超调。接入时将实测位置投影为合法命令起点，不写回物理 qpos/qvel；上游目标仍严格做范围验证，不对输入静默裁剪。回退清空邮箱后可在100ms新鲜度窗口内等待同session的新包再接入；超时或换session保持暂停，需人工操作。

ready/hold 三组分别为双臂（bit 0）、右手（bit 1）、左手（bit 2）。未 ready 组保持目标；每组超过 100 ms 没有新鲜 ready 目标后锁存 hold，其他新鲜组仍可执行。数据恢复不能自动恢复该组运动，须再次显式禁用／启用。目标保持不是冻结 qpos：物理积分、接触和跟随误差继续存在。

Viewer 用 `F8/F9` 选择关节，展示实际应用目标与实际 qpos；HUD 展示接收／有效／拒绝计数、候选年龄、session、ready/hold 与跟踪误差。进程 ready、DDS 对端发现、有效候选和运动授权是四种不同状态。

暂停／回退期间，Viewer 在 user_scn 中显示检查点目标的 Wuji2 双手虚影；接入期间切换为实时目标虚影，正常采集隐藏。使用只读目标、独立 MjData 正运动学和仅左右 wrist 子树的视觉几何；不修改实际 model/data、不复制机身／机械臂／物体、不参与碰撞。中文任务条标明“检查点目标／实时目标”。

## 4. 进程与网络边界

`bash/start_spd_sim.sh` 在 ROS Pixi 环境中以 `exec` 启动前台订阅进程，直接使用当前终端，不创建 tmux 会话、不后台运行。`pixi run spd-sim` 直接选择 `ros-jazzy` 环境，避免嵌套任务启动；脚本从裸 shell 调用时自行进入同一环境。当前终端 `Ctrl+C` 或 Viewer 退出只结束 SPD，不停止上游或硬件控制器。启动不代替运动授权；一次只运行一个采集进程，不共享输出目录。

Jazzy/Fast DDS 使用 domain 120，QoS 为 `BEST_EFFORT / KEEP_LAST(1) / VOLATILE`。在 Pixi 激活和接口 overlay 加载后显式设置 `ROS_DOMAIN_ID=120`、`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`、`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`、`ROS_STATIC_PEERS=''`；无桥接进程。

启动器固定同机发现。跨主机需另外配置发现范围／静态 peers 或发现服务、网络接口、防火墙和 UTC 同步。domain 不是安全边界；DDS 直连也不是延迟、丢包或实时性能保证。

## 5. 物理场景、随机化与模型限制

环境注册 18 个任务。spd-sim 未指定 --task 时启动随机选择，每段完成后重建下一场景并清除授权；显式 --task 固定任务，--scene 可限制随机范围，--seed 可复现选择序列。跨模型只保留机器人实际位置、速度及保留目标，新物体重新初始化，旧检查点不继承；新任务等待 r 开段。

先验证基础机器人资产，再组合任务模型，不覆盖基础 MJCF。桌高 0.75 m；固定盘架、杯架和箱体，任务物体通过 free joints、重力、摩擦与真实接触运动。场景不包含自动策略或成功评分。物理状态不是命令 qpos 回放，RGB 来自该状态渲染，不是录制画面的替代粘贴。

精细场景采用 `SceneBuildResult.assets + worldbody`：`abc_assets.py` 读取环境包自带的六种 ABC 瓶子网格、贴图及分块碰撞，`visual_details.py` 生成其余物体的圆角／旋转曲面、材质和细节。局部资产路径从环境包位置解析，wheel 包含所需 OBJ／PNG／JSON，运行时不依赖原 ABC 目录。ABC 源资产与变换、导出文件哈希写入 provenance；公开再分发权利尚未确认。

场景外观 geom 为 group 2、mass=0、contype=conaffinity=0；独立碰撞代理为 group 3，实际承担接触和质量分配。隐藏 group 3 不等于关闭碰撞。杯／杯柄／箱子的空腔通过真实分块碰撞保留，盘子使用中心浅盘和倾斜环状盘沿。桌腿和装饰仅为外观，不扩大可交互物理范围。木纹、釉面与 A–Z 贴图为原创程序生成，字母任务不再用大量小 box 拼字。

种子同时选择瓶子型号／尺寸、材质和纹理变体；`geometry_revision=detailed-scenes-v1`、对象资产 ID、原始／导出哈希、外观选择、物理参数与字母分配进入场景 manifest。碰撞调试色与真实外观材质分开记录。桌距变化同步平移物体、桌面外观、灯光和相关元数据，不重新采样。

模型合并先解析原模型资源，再深拷贝场景资产和实体；重复资产名明确拒绝，不修改或消耗原 SceneBuildResult。重复合并输出一致。新模型的网格、纹理和相机继续由 schema-v2 的 MJB 快照完整携带，状态恢复与场景精细化解耦；离线渲染由独立模块消费这些快照。

带桌场景在创建窗口前解析 `simulation.scene.resolve_table_distance`；`--table-distance` 显式指定或交互询问，非交互必须显式提供。距离沿 +X 从底座原点到近侧桌沿，默认 0.10 m。桌子、物体和固定支架整体平移，不重新采样随机参数；实际距离、几何和 seed 保存到 scene/task manifest。

机器人保留 URDF 质量、质心和惯性；物体材质参数是工程默认值，不是实物标定结果。显示透明度不改变碰撞。模型保留 `Link5_L–Link7_L`、`Link5_R–Link7_R` 两对临时碰撞排除，记录在 `collision.temporary_excludes`；它们也会忽略真实碰撞，不得作为硬件安全保证。SPD 保留模型限位和控制门，不承担上游轨迹求解或避碰。

## 6. 完整场景轨迹采集与恢复

`config/collect_sim.yaml` 使用 version 2：物理 480 Hz、固定轨迹 60 Hz，每 8 个物理步记录一帧。三路相机定义仍由 `config/sim_cameras.yaml` 注入模型并保存，但在线采集不创建渲染器、不采 RGB。Viewer 是操作反馈，与后续训练渲染无关。相机配置仍为 provisional；名义仿真频率不是负载下墙钟性能保证。

SPD 本地 r 开段并在任何运动前创建 0 号检查点，采集中 r 更新检查点、s 冻结物理和采样。暂停 s 做一秒接入恢复；暂停 d 回退并自动接入，不需再次按 s；采集中 d 无效。暂停 r→r 确认成功保存，没有操作者丢弃分支。接入期间普通 r/s/d 丢弃，不在终点后重放；失鲜、非法数据或接入期间 ready 集合变化仍撤权暂停。开文件等 I/O 准备不计入一秒过渡。

`TrajectorySource` 在会话初始化时序列化完整 MuJoCo 模型，包含网格、纹理和相机。SHA-256、精确 MuJoCo 版本、物理设置、源资产溯源、任务随机参数及关节／物体地址映射随 episode 保存。原场景 XML 删除或搬迁不影响恢复；不支持不同 MuJoCo 版本之间直接加载二进制快照。

物理线程每步观察手–物接触，按左右手及任务物体累计到下一个轨迹样本。接触分类使用 `l_wrist/r_wrist` 子树和 task manifest 的物体子树，不把机器人自碰撞、手–桌接触算作手–物接触。每帧保存全场景 qpos/qvel、机器人实际 54 维状态、任务物体世界位姿，以及存在时的 act/mocap/equality 状态。派生位姿通过独立数据对象刷新，不修改在线物理状态。

每帧记录严格递增的物理 tick、绝对仿真秒数和主机单调纳秒。物理 tick 差必须为 8，仿真间隔必须为 1/60 秒；墙钟间隔单独保留，不伪造实时频率。首帧来自准备完成后的第一个物理步；接触首区间只覆盖该步，之后覆盖 8 个物理步。

`CollectionSession` 是本地按键和 ROS 请求共享的唯一录制状态机。单个协调 worker 负责准备／保存／丢弃；单个 HDF5 写线程接收有界整帧队列。队列满、数据不合法或漏 tick 立即报错，绝不覆盖旧样本或静默丢帧。完成后校验全部行、数据类型、时间轴、模型和元数据哈希，才发布 `.h5`；完整性 `complete` 与任务结果 `success` 分离。帧数上限为 `success=false`，仅显式保存为 true。

开段自动保存完整初始检查点，允许初始接触；采集中手动更新仍拒绝任一手与任务物体的当前 solver-active 接触。检查点保存 MjData、tick、保留目标、采样相位及接触累计。回退先冻结并清除授权，唯一写线程裁去失败后缀和标签后刷盘，再恢复完整状态。目标仍有效时自动一秒接入；否则保持暂停等待人工 s，不自动重试。检查点不跨 episode 或模型。

普通键盘和踏板均使用 r/s/d，终端 cbreak 输入无需回车；q／Ctrl+C 退出，F8/F9 保留曲线选择。输入回调只排队，接入期间收到的录制按键直接丢弃。无 evdev、长按或拔出检测；普通自动重复可能确认保存，操作者须点按。上游键位独立，不因 SPD 按键转发任何控制请求。

裁剪删除失败分支，恢复原仿真 tick／时间和采样相位；主机单调时钟不回退。collection_events/rewind 记录分支边界；新增 recovery_transition uint8[N] 标记正常、开段接入、暂停恢复、回退恢复（0/1/2/3）。接入期间保留真实采样和标签，不伪造连续人工动作。旧无标签文件仍能验证，但明确报告未标注；训练窗口不能跨回退或被排除的接入段。

每次接受 start 时按本机日期固定 `YYYYMMDD/`。跨午夜不拆段，下一段重新选日期；每日 `dataset_config.json` 仅约束模型无关的共享 schema。旧 schema-v1 目录拒绝追加，不修改历史数据。输出优先级保持 `--output`、`SPD_EPISODE_OUTPUT`、配置 `data_dir`。

`replay_episode` 加载内嵌模型，在独立 MjData 逐帧赋值并调用前向计算，验证机器人投影和物体位姿；不调用 `mj_step`、不发送目标、不渲染。记录不包含 ctrl 或全部积分器历史，不能当作恢复原控制运行的检查点。公开校验和恢复拒绝 partial／未完成文件，且拒绝 schema、模型、元数据和 MuJoCo 版本不匹配。详见 [schema-v2.md](schema-v2.md)。

默认 CollectionRosControl 只发布可靠 transient-local 的 /spd/collection/status；外部 Trigger 控制禁用，避免绕过本地确认和接入流程。状态保留 collector／operation、路径、帧数、physics_paused 和 checkpoint_frames，0 表示合法起始检查点。底层 CollectionSession/recorder 的管理接口仍服务测试和专用集成，不属于操作者按键流程。

采集侧已实现在线检查点／回退，不实现采后接触裁剪或 30 Hz 样本构建。旧 `align_30hz`、`filter_contacts` 依赖已废弃契约，已移除；离线渲染输出当前保留轨迹的所有源帧，不代替这些处理。

## 7. 八 GPU 离线渲染

`pixi run -e render spd-render` 使用独立无 ROS 的 render 环境。默认部署配置选择 8 个 EGL 设备，每卡一个 spawn worker；父进程不导入 MuJoCo／GL、不加载图像。worker 在任何 native import 前设置 `MUJOCO_GL=egl`、`MUJOCO_EGL_DEVICE_ID`、`PYOPENGL_PLATFORM=egl` 和数值库线程数，再用实际 GL vendor／renderer 检查 NVIDIA 硬件与目标型号。EGL 索引不等于 CUDA_VISIBLE_DEVICES 映射，服务器必须先运行 `--check-gpus`。

所有 worker 初始化成功才派发 episode，空闲进程从共享队列取下一段。每段独立加载内嵌模型，单个 Renderer 顺序产生三视角 RGB 和实例掩码，缓冲区有界，不把像素送到父进程。每卡显存独立；workers_per_gpu 可调但不承诺线性加速。当前是原生 EGL 而不是 Warp／Madrona，不需要 CUDA 训练框架。

相机位置由用户后续 URDF 定义，当前只固定逻辑名 top／left_wrist／right_wrist。渲染器不决定外参，只读模型中已有相机；缺失即报错。默认拒绝 provisional 或无 calibration_revision 的快照，显式诊断开关允许预览但输出标记 diagnostic_only。现有临时 YAML 的坐标不代表正式相机位置。URDF link/joint／相机扩展转换和历史轨迹换相机须在实际格式确定后单独接入，不隐式修改已记录模型。

渲染对原始轨迹只读，不调用 mj_step；逐帧赋值并 mj_forward，复用机器人／物体位姿一致性检查。隐藏 group0／3 碰撞代理；MuJoCo 分割的 GEOM ID 通过物体子树映射成源 instance_id，同一物体的多个网格共用一个 ID。保留 0 天空、-1 机器人、-2 非任务环境。相机世界位置、旋转矩阵及源 frame/tick/time 同行写出。

结果独立为 .render.h5；独占锁、partial、完整内容校验和同目录原子无覆盖发布防止混写。源文件／模型／元数据／设置哈希决定复用，已完成输出仍须逐流校验；不匹配、损坏或残留 partial/lock 明确失败，不自动重试／覆盖。进程硬退出或启动超时清理其余自有进程，报告未确认任务；重跑可跳过校验通过的已完成段。输出格式详见 [schema-v2.md](schema-v2.md)。

## 8. 研究与验证边界

论文参考为 [Pre-training Visual Dexterity in Simulation](papers/2608.15917v1.pdf) §3.1、附录 A.1。论文物理 480 Hz、控制／流传输／记录 60 Hz、训练网格 30 Hz 是不同阶段的契约，不等同于当前实现各流频率或机器性能保证。

上游发布契约已定稿；这里不宣称真实 PICO → 上游 → SPD 已完成端到端验收。GPU 渲染链路与八卡部署接口不代表已经在 8×5090 实测吞吐；相机最终标定、训练增强、完整示范质量及论文等价性仍须分别验证。实机控制、真实传感器融合、策略训练与部署不属于 SPD。
