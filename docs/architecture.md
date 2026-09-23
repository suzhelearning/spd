# 仿真采集架构

## 1. 边界与数据流

SPD 是 **ROS 关节命令订阅端与 MuJoCo 仿真数据采集端**。人体标定、共享根 DLS/Ruckig、Hand2 映射与 ROS 发布属于独立上游 `tianji_teleop-ros2`。上游普通入口 `bash bash/run_pico_hand_sim.sh --height-m HEIGHT` 使用 `r` 标定、`s` 显式启动。SPD 不请求或管理上游标定、跟随、暂停；本地暂停时上游继续发布目标。SPD 仅对收到的合法目标做一秒接入，不实现 IK 或实机控制。

```text
独立上游：输入 / 标定 / 求解 / ROS JointCommand 发布
                              ↓
SPD C++ executor：校验 → 最新候选 → 会话与显式授权 → 分组 hold
                              ↓
SPD C++ physics：按名称映射目标 → position actuators → MuJoCo 物理积分
                              ↓
              完整场景状态 / 任务物体 / 手–物接触
                     ↙                    ↘
       data_collector：60 Hz 轨迹     Viewer：任务、采集状态与双手虚影
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
| `src/spd_native/` | C++ ROS 订阅、邮箱与授权、三键状态机、物理调度／步进、检查点与接触累计 |
| `src/interfaces/` | Python wire 工具、终端键盘输入与 ROS 消息定义 |
| `src/simulation/` | Python 模型准备、原生执行器编排、Viewer 与场景查看 |
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

`pixi.toml` / `pixi.lock` 是受维护运行环境。`pixi run spd-native-build` 在 `ros-jazzy` 环境中构建 `tianji_spd_interfaces` 和 `spd_native` 到 `.ros/{build,install}`，与源文件和机器人模型分离；`ros-build-interfaces` 仅供单独构建消息。SPD 不自动构建上游工作区。

`spd_executor` 是内嵌 CPython 的 C++ 入口；`_spd_native` 暴露原生 ROS 执行器、三键状态机、MuJoCo 状态操作及接触累计。Python 保留模型／场景准备、Viewer、轨迹序列化和 HDF5 异步文件生命周期。原生 `run_loop` 持有调度循环，仍会调用这些 Python 生命周期回调，因此不宣称无 GIL、无 Python 热路径或硬实时。`mj_step` 期间释放 GIL，避免接触密集时阻塞终端与写入线程。

ROS 回调只校验并更新邮箱；控制应用、步进、恢复和接触观察顺序运行在唯一执行线程。MuJoCo Python 对象为 Viewer／序列化保留模型与数据存储，原生组件持有强引用，避免悬垂指针；原生目标数组不提供可写视图。检查点与接触区间带创建者身份，拒绝跨模型／来源恢复。旧 Python 订阅执行器及三键实现已删除，不提供回退路径。

## 3. 订阅契约与授权状态

接口类型 `tianji_spd_interfaces/msg/JointCommand`，话题 `/spd/tianji_wuji2/v1/joint_command`，`schema_version=1`，`robot_config=tianji_wuji2_v1`。54 维顺序是左臂 7、右臂 7、左手 20、右手 20，单位 rad。消息保留原 wire 字段，不因目录迁移改变。

订阅回调校验名称顺序、维度、有限值、ready 组限位、session、递增 sequence 与 UTC 新鲜度，完整通过后才原子替换最新候选。物理 tick 消费时再次校验年龄；按 manifest 名称预计算 qpos/actuator 地址，场景 free／有界 slide joints 不改变机器人索引。非机器人滑动关节必须有有限限位、独立 world 根和匹配名称，不能绕过机器人关节／执行器契约。

SPD 统一使用本地状态化 r/s/d：待开始 r 开段，暂停 s 恢复，暂停 d 回退后自动恢复。这些操作调用受控的一秒目标接入，不直接将大目标差应用到执行器。计时从准备完成后的首个物理应用步开始，以主机单调时钟计算五次平滑权重；起点是实际 qpos，终点每步取最新合法目标。1 秒结束目标混合，不保证物理到位。普通 authorize API 的 0.15 rad 差值门仍保留，但不再提供 e/c 快捷键或该门限的交互选项。

物理软限位允许少量实测超调。接入时将实测位置投影为合法命令起点，不写回物理 qpos/qvel；上游目标仍严格做范围验证，不对输入静默裁剪。回退清空邮箱后可在100ms新鲜度窗口内等待同session的新包再接入；超时或换session保持暂停，需人工操作。

ready/hold 三组分别为双臂（bit 0）、右手（bit 1）、左手（bit 2）。未 ready 组保持目标；每组超过 100 ms 没有新鲜 ready 目标后锁存 hold，其他新鲜组仍可执行。数据恢复不能自动恢复该组运动，须再次显式禁用／启用。目标保持不是冻结 qpos：物理积分、接触和跟随误差继续存在。

Viewer 顶部使用一张中文双列图像：左侧任务名称、目标及虚影说明，右侧状态／帧数／检查点摘要、当前操作提示及存在时的异常原因。顶部区域与下方场景视口互不遮挡；长文本独立换行，极长诊断用省略号截断以保留至少 60% 场景高度，完整信息仍保留在终端／采集状态。状态和常见控制提示在显示层中文化，未识别底层诊断原文保留，ROS／采集状态契约不变。

图形 `spd-sim` 使用一个 GLFW 窗口和 GL 上下文，下方两个真实 MuJoCo 透视视口：左侧自由旋转／平移／缩放，右侧固定头部观察。右视角位于双臂 Base_L／Base_R 中点上方 0.35m，沿 +X 前看并下倾 35°，垂直视场角 70°；不是垂直俯视，不改变采集用 top／腕部相机配置。鼠标命中检测在 framebuffer 坐标中进行，顶部／右侧输入不改变左侧相机；r/s/d/q/Esc 的键盘操作作用于整个窗口。独立 spd-scene 保留单自由视口。

`ViewerWindow` 只保存展示数据和投递输入；`SplitViewRenderer` 的渲染线程独占模型克隆、MjData、两组 MjvScene/MjvCamera 和 GLFW／GL 资源。物理 owner 捕获有所有权的完整状态快照，通过短锁交换单个待显示包，锁不跨 GPU 绘制；渲染线程只做状态恢复、运动学和绘制，从不 mj_step。两侧使用同一快照。暂停／接入的 HandGhost 分别追加在两个视口的物理几何后，不覆盖实体、不参与接触，正常采集隐藏。

旧的观察子进程、临时 MJB 和第二窗口已删除。退出／换场景先停止并回收渲染线程和 GL 资源；headless 不导入该渲染后端或创建窗口。渲染异常由 owner 观察并抛出，启动和关闭有界等待；不提供旧 passive viewer 回退路径。图像绘制前用顶部矩形初始化 2D 状态，再绘制连续 RGB 缓冲区，避免继承 3D 深度／光照状态。

## 4. 进程与网络边界

`bash/start_spd_sim.sh` 在 ROS Pixi 环境中以 `exec` 启动前台 `.ros/install/lib/spd_native/spd_executor`，直接使用当前终端，不创建 tmux 会话或观察子进程。`pixi run spd-sim` 直接选择 `ros-jazzy` 环境；脚本从裸 shell 调用时自行进入同一环境。终端 Ctrl+C 或窗口退出只结束 SPD 并回收渲染线程，不停止上游或硬件控制器。启动不代替运动授权；一次只运行一个采集进程，不共享输出目录。

Jazzy/Fast DDS 使用 domain 120，QoS 为 `BEST_EFFORT / KEEP_LAST(1) / VOLATILE`。在 Pixi 激活和接口 overlay 加载后显式设置 `ROS_DOMAIN_ID=120`、`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`、`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`、`ROS_STATIC_PEERS=''`；无桥接进程。

启动器固定同机发现。跨主机需另外配置发现范围／静态 peers 或发现服务、网络接口、防火墙和 UTC 同步。domain 不是安全边界；DDS 直连也不是延迟、丢包或实时性能保证。

## 5. 物理场景、随机化与模型限制

环境注册 18 个任务。spd-sim 未指定 --task 时启动随机选择，每段完成后重建下一场景并清除授权；显式 --task 固定任务，--scene 可限制随机范围，--seed 可复现选择序列。跨模型只保留机器人实际位置、速度及保留目标，新物体重新初始化，旧检查点不继承；新任务等待 r 开段。

先验证基础机器人资产，再组合任务模型，不覆盖基础 MJCF。每次生成场景直接均匀采样桌高 0.70–0.80 m、近侧桌沿距离 0.10–0.30 m，生成后固定；物体和固定盘架、杯架、箱体、柜体同步定位，桌腿伸缩而脚垫仍落地。普通物体通过 free joints、重力、摩擦与真实接触运动，三个抽屉通过被动有界 slide joints 运动。场景不包含自动策略或成功评分。物理状态不是命令 qpos 回放，RGB 来自该状态渲染。

精细场景采用 `SceneBuildResult.assets + worldbody`：`abc_assets.py` 读取环境包自带的六种 ABC 瓶子网格、贴图及分块碰撞，`visual_details.py` 生成其余物体的圆角／旋转曲面、材质和细节。局部资产路径从环境包位置解析，wheel 包含所需 OBJ／PNG／JSON，运行时不依赖原 ABC 目录。ABC 源资产与变换、导出文件哈希写入 provenance；公开再分发权利尚未确认。

场景外观 geom 为 group 2、mass=0、contype=conaffinity=0；独立碰撞代理为 group 3，实际承担接触和质量分配。隐藏 group 3 不等于关闭碰撞。杯／杯柄／箱子的空腔通过真实分块碰撞保留，盘子使用中心浅盘和倾斜环状盘沿。桌腿和装饰仅为外观，不扩大可交互物理范围。木纹、釉面与 A–Z 贴图为原创程序生成，字母任务不再用大量小 box 拼字。

种子选择六种瓶子资产、三种杯／马克杯／盘／箱几何、完整配色和多种有效布局；`geometry_revision=paper-aligned-scenes-v2`、对象资产 ID、实际尺寸／质量、外观来源／哈希及字母分配进入 manifest。多米诺改为普通木块；字母使用八色字形／边框；塑料杯覆盖红绿蓝黄。拼词目标从八个词中选择，实际中英文目标进入 sampled_values 并驱动 Viewer／采集任务说明。碰撞调试色与真实外观色分开记录。桌距变化同步平移物体、桌面外观、灯光和相关元数据，不重新采样。

柜体及三个抽屉分别是独立实例根；抽屉是有真实底板、侧壁、前板和把手的开放托盘，沿柜体局部 -X 滑动 0–250mm，qpos 为相对关闭位置的实际开度。world-root 组织避免对象子树重叠，固定柜体保证导向基准不动；全量 qpos/qvel 和 MJB 已覆盖滑动 DOF，不扩展机器人命令。分拣任务从三层托盘内的 3／3／2 块字母开始。搁板与滑动底板保留 1mm 运行间隙，保留所有接触，避免受约束法向上的共面接触造成数值摩擦锁死。

初始碰撞检查不豁免套杯／柜体等组件；桌面包围盒净空覆盖抽屉完整行程和把手。`sampled_values.affordances` 描述盘架 50mm 槽、杯柄与切向挂杆、箱内空间、套叠间隙和抽屉内部坐标，可用于物理探针。此为任务功能近似与受控验证，不宣称论文 CAD 精确复刻或完整机器人操作成功率。

模型合并先解析原模型资源，再深拷贝场景资产和实体；重复资产名明确拒绝，不修改或消耗原 SceneBuildResult。重复合并输出一致。新模型的网格、纹理和相机继续由 schema-v2 的 MJB 快照完整携带，状态恢复与场景精细化解耦；离线渲染由独立模块消费这些快照。

带桌场景在 builder 中使用现有 seed 随机生成桌高与桌距，不再交互询问；`--table-distance` 仅在显式传入时覆盖随机桌距。距离沿 +X 从底座原点到近侧桌沿测量。实际高度、距离、工作区中心、采样范围和 seed 保存到 scene/task manifest。默认随机任务切换时使用新场景 seed 重采样，不把上一次的随机桌距当作用户固定值；显式覆盖则跨场景保持。暂停／回退不重采样。

机器人保留 URDF 质量、质心和惯性；物体材质参数是工程默认值，不是实物标定结果。显示透明度不改变碰撞。模型保留 `Link5_L–Link7_L`、`Link5_R–Link7_R` 两对临时碰撞排除，记录在 `collision.temporary_excludes`；它们也会忽略真实碰撞，不得作为硬件安全保证。SPD 保留模型限位和控制门，不承担上游轨迹求解或避碰。

## 6. 完整场景轨迹采集与恢复

`config/collect_sim.yaml` 使用 version 2：物理 480 Hz、固定轨迹 60 Hz，每 8 个物理步记录一帧。三路相机定义仍由 `config/sim_cameras.yaml` 注入模型并保存，但在线采集不创建渲染器、不采 RGB。Viewer 是操作反馈，与后续训练渲染无关。相机配置仍为 provisional；名义仿真频率不是负载下墙钟性能保证。

SPD 本地 r 开段并在任何运动前创建 0 号检查点，采集中 r 更新检查点、s 冻结物理和采样。暂停 s 做一秒接入恢复；暂停 d 回退并自动接入，不需再次按 s；采集中 d 无效。暂停 r→r 确认成功保存，没有操作者丢弃分支。接入期间普通 r/s/d 丢弃，不在终点后重放；失鲜、非法数据或接入期间 ready 集合变化仍撤权暂停。开文件等 I/O 准备不计入一秒过渡。

`TrajectorySource` 在会话初始化时序列化完整 MuJoCo 模型，包含网格、纹理和相机。SHA-256、精确 MuJoCo 版本、物理设置、源资产溯源、任务随机参数及关节／物体地址映射随 episode 保存。原场景 XML 删除或搬迁不影响恢复；不支持不同 MuJoCo 版本之间直接加载二进制快照。

原生 `ContactCollector` 每步观察手–物接触，按左右手及任务物体累计到下一个轨迹样本。接触分类使用 `l_wrist/r_wrist` 子树和 task manifest 的物体子树，不把机器人自碰撞、手–桌接触算作手–物接触；手动检查点在独立 scratch data 上刷新当前接触，不改变在线求解器历史。每帧保存全场景 qpos/qvel、机器人实际 54 维状态、任务物体世界位姿，以及存在时的 act/mocap/equality 状态。Python 轨迹层负责快照与序列化，派生位姿通过独立数据对象刷新。

每帧记录严格递增的物理 tick、绝对仿真秒数和主机单调纳秒。物理 tick 差必须为 8，仿真间隔必须为 1/60 秒；墙钟间隔单独保留，不伪造实时频率。首帧来自准备完成后的第一个物理步；接触首区间只覆盖该步，之后覆盖 8 个物理步。

`CollectionSession` 保留为 Python 文件生命周期协调器，由原生三键状态机驱动。单个协调 worker 负责准备／保存／丢弃；单个 HDF5 写线程接收有界整帧队列。队列满、数据不合法或漏 tick 立即报错，绝不覆盖旧样本或静默丢帧。完成后校验全部行、数据类型、时间轴、模型和元数据哈希，才发布 `.h5`；完整性 `complete` 与任务结果 `success` 分离。帧数上限为 `success=false`，仅显式保存为 true。

开段自动保存完整初始检查点，允许初始接触；采集中手动更新仍拒绝任一手与任务物体的当前 solver-active 接触。检查点保存 MjData、tick、保留目标、采样相位及接触累计。回退先冻结并清除授权，唯一写线程裁去失败后缀和标签后刷盘，再恢复完整状态。目标仍有效时自动一秒接入；否则保持暂停等待人工 s，不自动重试。检查点不跨 episode 或模型。

普通键盘和踏板均使用 r/s/d，终端 cbreak 输入无需回车；q／Ctrl+C 退出。输入回调只排队，接入期间收到的录制按键直接丢弃。无 evdev、长按或拔出检测；普通自动重复可能确认保存，操作者须点按。上游键位独立，不因 SPD 按键转发任何控制请求。

裁剪删除失败分支，恢复原仿真 tick／时间和采样相位；主机单调时钟不回退。collection_events/rewind 记录分支边界；新增 recovery_transition uint8[N] 标记正常、开段接入、暂停恢复、回退恢复（0/1/2/3）。接入期间保留真实采样和标签，不伪造连续人工动作。旧无标签文件仍能验证，但明确报告未标注；训练窗口不能跨回退或被排除的接入段。

每次接受 start 时按本机日期固定 `YYYYMMDD/`。跨午夜不拆段，下一段重新选日期；每日 `dataset_config.json` 仅约束模型无关的共享 schema。旧 schema-v1 目录拒绝追加，不修改历史数据。输出优先级保持 `--output`、`SPD_EPISODE_OUTPUT`、配置 `data_dir`。

`replay_episode` 加载内嵌模型，在独立 MjData 逐帧赋值并调用前向计算，验证机器人投影和物体位姿；不调用 `mj_step`、不发送目标、不渲染。记录不包含 ctrl 或全部积分器历史，不能当作恢复原控制运行的检查点。公开校验和恢复拒绝 partial／未完成文件，且拒绝 schema、模型、元数据和 MuJoCo 版本不匹配。详见 [schema-v2.md](schema-v2.md)。

`CollectionRosControl` 只负责 JSON 状态序列化与心跳，由原生 rclcpp publisher 向 `/spd/collection/status` 发布可靠 transient-local 消息；不创建外部 Trigger 控制服务，避免绕过本地确认和接入流程。状态保留 collector／operation、路径、帧数、physics_paused 和 checkpoint_frames，0 表示合法起始检查点。底层 CollectionSession/recorder 管理接口仍服务测试和专用集成，不属于操作者按键流程。

采集侧已实现在线检查点／回退，不实现采后接触裁剪或 30 Hz 样本构建。旧 `align_30hz`、`filter_contacts` 依赖已废弃契约，已移除；离线渲染输出当前保留轨迹的所有源帧，不代替这些处理。

## 7. 八 GPU 离线渲染

`pixi run -e render spd-render` 使用独立无 ROS 的 render 环境。默认部署配置选择 8 个 EGL 设备，每卡一个 spawn worker；父进程不导入 MuJoCo／GL、不加载图像。worker 在任何 native import 前设置 `MUJOCO_GL=egl`、`MUJOCO_EGL_DEVICE_ID`、`PYOPENGL_PLATFORM=egl` 和数值库线程数，再用实际 GL vendor／renderer 检查 NVIDIA 硬件与目标型号。EGL 索引不等于 CUDA_VISIBLE_DEVICES 映射，服务器必须先运行 `--check-gpus`。

所有 worker 初始化成功才派发 episode，空闲进程从共享队列取下一段。每段独立加载内嵌模型，单个 Renderer 顺序产生三视角 RGB 和实例掩码，缓冲区有界，不把像素送到父进程。每卡显存独立；workers_per_gpu 可调但不承诺线性加速。当前是原生 EGL 而不是 Warp／Madrona，不需要 CUDA 训练框架。

相机位置由用户后续 URDF 定义，当前只固定逻辑名 top／left_wrist／right_wrist。渲染器不决定外参，只读模型中已有相机；缺失即报错。默认拒绝 provisional 或无 calibration_revision 的快照，显式诊断开关允许预览但输出标记 diagnostic_only。现有临时 YAML 的坐标不代表正式相机位置。URDF link/joint／相机扩展转换和历史轨迹换相机须在实际格式确定后单独接入，不隐式修改已记录模型。

渲染对原始轨迹只读，不调用 mj_step；逐帧赋值并 mj_forward，复用机器人／物体位姿一致性检查。隐藏 group0／3 碰撞代理；MuJoCo 分割的 GEOM ID 通过物体子树映射成源 instance_id，同一物体的多个网格共用一个 ID。render schema 2 保留 0 天空、-1 机器人、-2 其他环境，并新增 -3 桌子；柜体、抽屉等任务物体仍用正 ID。相机世界位置、旋转矩阵及源 frame/tick/time 同行写出。

结果独立为 .render.h5；独占锁、partial、完整内容校验和同目录原子无覆盖发布防止混写。源文件／模型／元数据／设置哈希决定复用，已完成输出仍须逐流校验；不匹配、损坏或残留 partial/lock 明确失败，不自动重试／覆盖。进程硬退出或启动超时清理其余自有进程，报告未确认任务；重跑可跳过校验通过的已完成段。输出格式详见 [schema-v2.md](schema-v2.md)。

`training_data` 提供 NumPy CPU 读取后视觉增强：实例染色、独立桌子／背景纹理，时序与多视角共享计划，保留机器人像素、整数掩码、状态和恢复标签。读取器要求 render schema 2 与对应完整源轨迹，每次读取自行开关 HDF5，校验身份和选中帧，拒绝跨回退边界。增强不改原 HDF5；旧 render schema 1 必须重新渲染到新目录，不猜测其混合环境掩码中的桌面。纹理为图像空间变换，不宣称重建三维材质或 GPU 增强；`pixi run spd-augment` 可生成对照图与计划参数。完整图像内容校验继续由 renderer 负责。

## 8. 研究与验证边界

论文参考为 [Pre-training Visual Dexterity in Simulation](papers/2608.15917v1.pdf) §3.1、附录 A.1。论文物理 480 Hz、控制／流传输／记录 60 Hz、训练网格 30 Hz 是不同阶段的契约，不等同于当前实现各流频率或机器性能保证。

上游发布契约已定稿；这里不宣称真实 PICO → 上游 → SPD 已完成端到端验收。GPU 渲染链路与八卡部署接口不代表已经在 8×5090 实测吞吐；新增训练增强已验证语义隔离与时序一致性，不代表相机已最终标定、训练收益已测量或场景与论文等价。实机控制、真实传感器融合、策略训练与部署不属于 SPD。
