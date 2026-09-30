# 仿真采集架构

## 1. 边界与数据流

SPD 的生产流程是**一个仿真进程内的本地裸手后端、统一采集协调器和原生物理执行器**。Quest／PICO 入口 `pixi run spd-quest-teleop --height-m HEIGHT`／`spd-pico-teleop --height-m HEIGHT` 共用 `bash/run_pico_hand_sim.sh` → `bash/start_spd_sim.sh` → `spd_executor` → `simulation.ros_viewer`，透传相同任务／输出／headless 参数。`pico2_hands.collection_session.TeleopSession` 管理输入、相对绑定和 DLS/Ruckig＋Hand2；不启动独立 ROS 发布控制进程。省略 `--height-m` 可使用外部 DDS 订阅模式，但没有本地无运动重绑定和每手匹配保证。项目不控制实机。

```text
PICO／Quest TCP → 本地 TeleopSession worker：输入／相对参考／DLS+Hand2
                              ↓ 不可变快照 + generation
唯一物理线程：CollectionControl → C++ executor 校验／邮箱／授权
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

正式入口包括统一 Quest／PICO 裸手采集、`spd-sim --height-m HEIGHT`，以及模型、场景、数据检查命令。无第二套控制终端、旧双进程兼容入口、独立停止命令、HDF5 命令发布或 Zenoh 遥操入口。`replay_episode` 从内嵌模型恢复状态，不把历史观测作为合成发布目标。

## 2. 源码与资源职责

| 路径 | 职责 |
|---|---|
| `src/spd_native/` | C++ 目标校验／授权、可选 ROS 订阅、原生 run_loop／Physics、检查点与接触累计 |
| `src/pico2_hands/` | `collection_session.py` 本地 TCP／相对绑定／每手平滑重接入及 DLS／Hand2 worker 编排；内部模型工具在私有 `_simulation` |
| `src/teleop_native/` | 仿真用 DLS/Ruckig、辅助 Viewer、控制 profile 及其模型闭包 |
| `tools/wuji_hand_native/` | 独立固定 Hand2 环境、重定向桥接与模型 |
| `src/interfaces/` | Python wire 工具、终端键盘输入与 ROS 消息定义 |
| `src/simulation/` | 物理线程 `CollectionControl`、模型准备、原生编排、Viewer 与场景／Home 生命周期 |
| `src/cameras/` | 世界／腕部仿真相机与 RGB 获取 |
| `src/data_collector/` | 配置、采集状态机、ROS 控制、完整物理轨迹与模型快照、独立恢复验证 |
| `src/offline_rendering/` | spawn 多 GPU 调度、原生 EGL、逐帧恢复渲染与输出校验 |
| `src/description/` | manifest、模型编译与资源定位 |
| `src/environments/spd_envs/` | 独立环境包，任务注册、随机化、场景生成与重置检查 |
| `src/tianji_wuji2/tianji_wuji2/assets/` | 原始 URDF、网格和碰撞资产 |
| `src/tianji_wuji2/tianji_wuji2/generated/` | 编译后的可加载模型与 manifest |
| `src/interfaces/tianji_spd_interfaces/` | ROS 2 `JointCommand.msg` 与接口构建元数据 |
| `config/` | 采集、临时预览相机和 `render_server.yaml` 八卡渲染配置 |
| `bash/` | 统一裸手／仿真前台启动与只读状态查询入口 |
| `data/` | 采集产物和已有样本，不随代码清理删除 |

运行时 Python 包直接位于 `src/`，按职责使用 `interfaces`、`simulation`、`cameras`、`data_collector`、`description`，不保留统一外层包或旧导入兼容层。根目录 `setup.py` 安装这些包，发行包名仍为 `spd`；依赖方向为 `spd → spd-envs`，独立环境包保持 `spd_envs`，只负责场景，不依赖 ROS 或遥操作算法，也不硬编码机器人路径。资源定位通过 `description/model_builder.py` 的 `workspace_root()`、`description_root()` 和 `config_root()`，不以调用者当前目录猜测资源位置。

`pixi.toml` / `pixi.lock` 是受维护运行环境。`pixi run spd-native-build` 在 `ros-jazzy` 环境中构建 `tianji_spd_interfaces` 和 `spd_native` 到 `.ros/{build,install}`；`ros-build-interfaces` 仅供单独构建消息。`pixi run spd-teleop-build` 另将双臂／Viewer（`teleop-native`：MuJoCo 3.10、Pinocchio 3、Eigen 3）和独立 Hand2（`tools/wuji_hand_native`：Pinocchio 4）构建到 `.teleop/{build,install}`，最后复用前述 ROS 构建。不混用原生 ABI，不回退外部工作区。`config/input_provenance` 仅保存输入契约指纹校验用的冻结源文件，不参与 Python 导入或原生编译；模型指纹与数值检查仍保留。

`spd_executor` 是内嵌 CPython 的 C++ 入口；`_spd_native` 暴露目标执行器、MuJoCo 状态操作与接触累计，保留原生 `run_loop`／`Physics`。`CollectionControl` 在唯一物理线程处理按键、绑定、冻结、恢复、采集和场景转换；旧原生三键状态机已删除。`TeleopSession` 的单个 worker 线程拥有 TCP 输入和 DLS／Hand2 原生 worker 交互，只产出不可变最新快照，不改 MuJoCo。generation／sequence 屏障拒绝重绑定前结果，无第二个键盘权威。

Python 还负责场景准备、Viewer、序列化和 HDF5 生命周期；原生循环调用这些回调，因此不宣称无 GIL、无 Python 热路径或硬实时。`mj_step` 期间释放 GIL。可选 ROS 回调只校验并更新邮箱；控制应用、物理步进、恢复和接触观察均由物理 owner 执行。MuJoCo 对象保持强引用，目标不提供可写视图；检查点与接触区间检查模型／创建者身份。

## 3. 本地绑定、恢复与外部订阅边界

操作者选择稳定舒适的腰间准备姿势并让头／双腕保持可见，按 `r` 开始。冻结世界后，当前腕部位置和完整朝向相对机器人保留目标 FK 绑定，初始输出等于保留目标；绑定本身不移动机器人、不写 qpos，也不要求前伸标定。身高用于映射尺度；没有躯干／腰部传感或腰姿估计，“腰间”不是自动测得的坐标。

左右手独立跟随，不要求匹配握姿。`_FingerGate` 在输入无效或距最后有效目标超过 45 ms 时立即保持目标；距最后有效目标不足 120 ms 时保留 `live`／`blend` 状态，暂停混合时钟，不外推陈旧目标，恢复后继续原进度。达到 120 ms 才 reset 为 `waiting`，随后新鲜输入重新从保留目标进行 200 ms 混合，目标限速始终为 2 rad/s。重复无效样本不续期；显式重绑定／参考身份变更仍立即重置。`TeleopSnapshot.finger_modes`（左、右）供 HUD／虚影区分持续等待与短缺口；`control_flags` 仍即时记录实际保持，不能用显示去抖掩盖录制质量。短时头／腕缺口继续走有界制动，持续丢失超过初始 120 ms 预算则整段进入 `auto_paused`；首次 `r` 重建当前参考后续采，不回退、不裁剪、不更新保存点。正常 `r` 存点、`d` 回退不变。源／worker 故障、非有限命令仍 fail-closed；有限越限目标饱和，不触发异常冻结。时间预算不构成墙钟实时保证。

`NativeHandWorker` 的会话请求等待预算为 300 ms，从 submit 起累计且不续期；DLS 等待预算未随之修改。stdout 为非阻塞管道，读取先消耗已缓存字节，只有遇到 `BlockingIOError` 才检查截止时间并 select 等待剩余时间。这样调用方调度迟到不等于 worker 故障；残缺响应仍受原截止时间限制。返回结果保持请求关联与源时间戳，session 仍按 45 ms 判断输入／手指目标新鲜度，不因响应读取成功而重新授予陈旧目标运动权限。真正超时报告左右手与已接收／预期字节数。

以下 DDS wire 契约仅用于可选外部订阅模式；本地模式禁用 DDS 目标订阅，但复用原生目标校验／授权。

接口类型 `tianji_spd_interfaces/msg/JointCommand`，话题 `/spd/tianji_wuji2/v1/joint_command`，`schema_version=1`，`robot_config=tianji_wuji2_v1`。54 维顺序是左臂 7、右臂 7、左手 20、右手 20，单位 rad。消息保留原 wire 字段，不因目录迁移改变。

订阅回调校验名称顺序、维度、有限值、ready 掩码、session、递增 sequence 与 UTC 新鲜度，有限命令按 manifest／执行器限位交集饱和后原子替换最新候选。授权误差比较、过渡插值与实际执行均使用该饱和目标；调用方持有的输入快照不被修改。物理 tick 消费时再次校验年龄，未 ready／held 组保持。按 manifest 名称预计算 qpos/actuator 地址，场景自由度不改变机器人索引。已存 Home、检查点和继承目标仍要求合法，损坏状态不作为命令饱和处理。

所有操作由 `CollectionControl` 按状态解释原始单键。待接手（首次、新任务、失跟踪）仅 `r` 发起绑定；运动中 `r` 存点、`s` 人工暂停、`d` 回退并自动重新绑定续采；人工暂停中 `s` 重新绑定继续、`r` 保存整条、`d` 丢弃整条。`s` 继续和运动中 `d` 回退均无需额外 `r`。外部 DDS 模式不保证本地参考重置；外部失效进入待接手，发布端对齐后按 `r`，人工暂停恢复仍用 `s`。

外部开始／恢复使用一秒实时目标混合，起点为实际 qpos 的合法命令投影，终点为新鲜候选的饱和值；不写回实际 qpos/qvel。有限输入越限不再拒绝，非有限值和其他契约错误仍拒绝。一秒结束不保证实际到位。本地绑定已经以保留目标起步，不叠加此全关节一秒接入控制器。

外部 ready/hold 三组为双臂（bit 0）、右手（bit 1）、左手（bit 2）；未 ready 组保持目标，超过 100 ms 无新鲜 ready 目标锁存 hold。它不是本地每手平滑重接入逻辑。保持目标与冻结整个物理世界不同；协调器负责需要时的全世界冻结。

Viewer 顶部使用一张中文双列图像：左侧任务名称、目标及虚影说明，右侧状态／帧数／检查点摘要、当前操作提示及存在时的异常原因。顶部区域与下方场景视口互不遮挡；长文本独立换行，极长诊断用省略号截断以保留至少 60% 场景高度，完整信息仍保留在终端／采集状态。状态和常见控制提示在显示层中文化，未识别底层诊断原文保留，ROS／采集状态契约不变。

图形采集使用一个 GLFW 窗口和 GL 上下文，下方两个真实 MuJoCo 透视视口：左侧自由旋转／平移／缩放，右侧固定头部观察。右视角位于双臂 Base_L／Base_R 中点上方 0.35m，沿 +X 前看并下倾 35°；不是垂直俯视，不改变采集相机标定。鼠标只控制左侧相机；`r/s/d/q/Esc` 作用于整个窗口，空格、`x` 和旧组合键不再操作采集。独立 spd-scene 保留单自由视口。

`ViewerWindow` 只保存展示数据和投递输入；`SplitViewRenderer` 的渲染线程独占模型克隆、MjData、两组 MjvScene/MjvCamera 和 GLFW／GL 资源。物理 owner 捕获有所有权的完整状态快照，通过短锁交换单个待显示包，锁不跨 GPU 绘制；渲染线程只做状态恢复、运动学和绘制，从不 mj_step。两侧使用同一快照。暂停／接入的 HandGhost 分别追加在两个视口的物理几何后，不覆盖实体、不参与接触，正常采集隐藏。

旧的观察子进程、临时 MJB 和第二窗口已删除。退出／换场景先停止并回收渲染线程和 GL 资源；headless 不导入该渲染后端或创建窗口。渲染异常由 owner 观察并抛出，启动和关闭有界等待；不提供旧 passive viewer 回退路径。图像绘制前用顶部矩形初始化 2D 状态，再绘制连续 RGB 缓冲区，避免继承 3D 深度／光照状态。

## 4. 进程与网络边界

`bash/start_spd_sim.sh` 在 ROS Pixi 环境以 `exec` 启动前台 `.ros/install/lib/spd_native/spd_executor`，不创建 tmux 或观察子进程。Quest 包装转到 PICO 共用启动器，必须指定 `--height-m`；全部任务、输出与 headless 参数透传。Ctrl+C／退出回收自有输入／求解和渲染资源，不停止外部硬件控制器。启动不代替绑定／运动授权，一次只运行一个采集进程，不共享输出目录。

可选外部 DDS 模式采用 Jazzy/Fast DDS、默认 domain 120、`BEST_EFFORT / KEEP_LAST(1) / VOLATILE`。根 Pixi 的 `ros-jazzy` 激活环境提供 `ROS_DOMAIN_ID=120`、`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`、`ROS_STATIC_PEERS=''` 默认值；启动器加载本地 overlay 并使用 `rmw_fastrtps_cpp`，无桥接进程。本地模式不经 DDS 传输裸手目标。

默认同机发现。跨主机需另外配置发现范围／静态 peers 或发现服务、网络接口、防火墙和 UTC 同步。domain 不是安全边界；DDS 直连也不是延迟、丢包或实时性能保证。

## 5. 物理场景、随机化与模型限制

环境注册 18 个任务。首次选择遵守 `--task`／`--scene`，`--seed` 可复现序列。人工暂停中的保存／丢弃实际完成后，`next_task_after_episode` 从完整目录重抽任务与新 seed、布局及桌面；新模型直接 Home、零速度，不继承旧机器人状态，不经过准备区或 Home 运动。清空旧授权、检查点与排队输入，等待新的 `r` 开段。文件结束失败不切换，且每个完成事件只推进一次。GLFW 换场景先关闭并 join 旧 renderer，再打开新窗口，旧窗口回调由场景 generation 拒绝。

先验证基础机器人资产，再组合任务模型，不覆盖基础 MJCF。每次生成场景直接均匀采样桌高 0.70–0.80 m、近侧桌沿距离 0.10–0.30 m，生成后固定；物体和固定盘架、杯架、箱体、柜体同步定位，桌腿伸缩而脚垫仍落地。普通物体通过 free joints、重力、摩擦与真实接触运动，三个抽屉通过被动有界 slide joints 运动。场景不包含自动策略或成功评分。物理状态不是命令 qpos 回放，RGB 来自该状态渲染。

精细场景采用 `SceneBuildResult.assets + worldbody`：`abc_assets.py` 读取环境包自带的六种 ABC 瓶子网格、贴图及分块碰撞，`visual_details.py` 生成其余物体的圆角／旋转曲面、材质和细节。局部资产路径从环境包位置解析，wheel 包含所需 OBJ／PNG／JSON，运行时不依赖原 ABC 目录。ABC 源资产与变换、导出文件哈希写入 provenance；公开再分发权利尚未确认。

场景外观 geom 为 group 2、mass=0、contype=conaffinity=0；独立碰撞代理为 group 3，实际承担接触和质量分配。隐藏 group 3 不等于关闭碰撞。杯／杯柄／箱子的空腔通过真实分块碰撞保留，盘子使用中心浅盘和倾斜环状盘沿。桌腿和装饰仅为外观，不扩大可交互物理范围。木纹、釉面与 A–Z 贴图为原创程序生成，字母任务不再用大量小 box 拼字。

种子选择六种瓶子资产、三种杯／马克杯／盘／箱几何、完整配色和多种有效布局；`geometry_revision=paper-aligned-scenes-v2`、对象资产 ID、实际尺寸／质量、外观来源／哈希及字母分配进入 manifest。多米诺改为普通木块；字母使用八色字形／边框；塑料杯覆盖红绿蓝黄。拼词目标从八个词中选择，实际中英文目标进入 sampled_values 并驱动 Viewer／采集任务说明。碰撞调试色与真实外观色分开记录。桌距变化同步平移物体、桌面外观、灯光和相关元数据，不重新采样。

柜体及三个抽屉分别是独立实例根；抽屉是有真实底板、侧壁、前板和把手的开放托盘，沿柜体局部 -X 滑动 0–250mm，qpos 为相对关闭位置的实际开度。world-root 组织避免对象子树重叠，固定柜体保证导向基准不动；全量 qpos/qvel 和 MJB 已覆盖滑动 DOF，不扩展机器人命令。分拣任务从三层托盘内的 3／3／2 块字母开始。搁板与滑动底板保留 1mm 运行间隙，保留所有接触，避免受约束法向上的共面接触造成数值摩擦锁死。

初始碰撞检查不豁免套杯／柜体等组件；桌面包围盒净空覆盖抽屉完整行程和把手。`sampled_values.affordances` 描述盘架 50mm 槽、杯柄与切向挂杆、箱内空间、套叠间隙和抽屉内部坐标，可用于物理探针。此为任务功能近似与受控验证，不宣称论文 CAD 精确复刻或完整机器人操作成功率。

模型合并先解析原模型资源，再深拷贝场景资产和实体；重复资产名明确拒绝，不修改或消耗原 SceneBuildResult。重复合并输出一致。新模型的网格、纹理和相机继续由 schema-v2 的 MJB 快照完整携带，状态恢复与场景精细化解耦；离线渲染由独立模块消费这些快照。

带桌场景按 seed 随机生成桌高与桌距，不交互询问；在线 `--table-distance` 只覆盖首次场景，成功保存后的新任务不沿用覆盖值。独立 `spd-scene` 的显式桌距规则不变。距离沿 +X 从底座原点到近侧桌沿测量。实际高度、距离、工作区中心、采样范围和 seed 保存到 manifest。暂停／回退不重采样；丢弃后重做仍保持原任务、seed 和桌面。

机器人保留 URDF 质量、质心和惯性；物体材质参数是工程默认值，不是实物标定结果。显示透明度不改变碰撞。模型保留 `Link5_L–Link7_L`、`Link5_R–Link7_R` 两对临时碰撞排除，记录在 `collision.temporary_excludes`；它们也会忽略真实碰撞。项目包含本地 IK／轨迹生成，但这些排除、限位和控制门均不提供实机避碰或安全认证。

## 6. 完整场景轨迹采集与恢复

`config/collect_sim.yaml` 使用 version 2：物理 480 Hz、固定轨迹 60 Hz，每 8 个物理步记录一帧。三路相机定义仍由 `config/sim_cameras.yaml` 注入模型并保存，但在线采集不创建渲染器、不采 RGB。Viewer 是操作反馈，与后续训练渲染无关。相机配置仍为 provisional；名义仿真频率不是负载下墙钟性能保证。

`r/s/d` 根据状态分流：待接手的 `r` 绑定开段或恢复现场；运动中的 `r` 更新检查点，`s` 人工暂停，`d` 回退；人工暂停中的 `r` 保存整条、`d` 丢弃整条、`s` 恢复运动。人工恢复直接 `_begin_bind(2)`，回退事务完成后直接 `_begin_bind(3)`，都在有效稳定输入下自动授权续采，无额外按键；普通失跟踪仍需首次 `r`。绑定处理中 `s` 可取消／暂停，磁盘保存、丢弃和回退期间普通键不排队重放。`q` 退出保留 partial，不触发保存。

`TrajectorySource` 在会话初始化时序列化完整 MuJoCo 模型，包含网格、纹理和相机。SHA-256、精确 MuJoCo 版本、物理设置、源资产溯源、任务随机参数及关节／物体地址映射随 episode 保存。原场景 XML 删除或搬迁不影响恢复；不支持不同 MuJoCo 版本之间直接加载二进制快照。

原生 `ContactCollector` 每步观察手–物接触，按左右手及任务物体累计到下一个轨迹样本。接触分类使用 `l_wrist/r_wrist` 子树和 task manifest 的物体子树，不把机器人自碰撞、手–桌接触算作手–物接触；接触不再阻止人工检查点。每帧保存全场景 qpos/qvel、机器人实际 54 维状态、任务物体世界位姿，以及存在时的 act/mocap/equality 状态。Python 轨迹层负责快照与序列化，派生位姿通过独立数据对象刷新。

每帧记录严格递增的物理 tick、绝对仿真秒数和主机单调纳秒。物理 tick 差必须为 8，仿真间隔必须为 1/60 秒；墙钟间隔单独保留，不伪造实时频率。首帧来自准备完成后的第一个物理步；接触首区间只覆盖该步，之后覆盖 8 个物理步。

`CollectionSession` 是 Python 文件生命周期协调器，由物理线程 `CollectionControl` 驱动。单个协调 worker 负责文件准备／保存／丢弃，单个 HDF5 写线程接收有界整帧队列。队列满、数据不合法或漏 tick 报错，不覆盖旧样本或静默丢帧。校验全部行、类型、时钟、模型与状态投影后才发布 `.h5`；`complete` 与 `success` 分离，帧数上限为 `success=false`，人工暂停中的 `r` 显式保存才为 true。

手动保存点包含完整 MjData、tick、保留目标、采样相位和未结束区间的接触／质量标签。`d` 请求 `revert`，唯一写线程裁掉手动点后的后缀及标签并刷盘，再在物理线程恢复完整状态。失跟踪现场快照独立于手动点，仅供等待期间提示；首次 `r` 直接重绑定当前目标，不调用回退、状态恢复或存点接口，续采时清除现场快照。重新接手不产生 `rewind` 事件，不补写等待区间。检查点不跨 episode、模型或进程持久化。

终端和 GLFW 只派发原始 `r/s/d/q`，不在输入层映射业务动作。终端 cbreak 即时单字符、无需回车；窗口只处理 PRESS，忽略 RELEASE／REPEAT。旧组合缓冲、100 ms 等待窗、空格和双 `x` 入口移除，快速相邻字符分别执行；请避免长按。场景 generation 及 busy-state 入队检查拒绝旧窗口／磁盘操作期间按键，防止延迟输入在新状态被解释成保存或丢弃。低层状态观察客户端不提供普通操作按键入口。

HDF5 仅保留选中前缀和其后续采，主机单调时钟不回退。schema-v2 兼容扩展 `collection_events/recovery_transition` 为正常／开段／人工恢复／人工点回退恢复／失跟踪重新接手（0/1/2/3/4），`control_flags` 为可选 uint8[N]：1 输入退化、2 右手保持、4 左手保持、8 右手平滑重入、16 左手平滑重入，保留高位为零。flags 按采样区间逐物理步 OR，恢复区间标签也不因采样落在过渡结束后而漏掉；写入和裁剪与轨迹同事务。`rewind` 只标记实际回退边界，重新接手不产生此事件。训练窗口不能跨回退／重绑定边界，不能把排除恢复段后的样本拼成连续人工动作；旧无标注文件须保留 provenance。

每次接受 start 时按本机日期固定 `YYYYMMDD/`。跨午夜不拆段，下一段重新选日期；每日 `dataset_config.json` 仅约束模型无关的共享 schema。旧 schema-v1 目录拒绝追加，不修改历史数据。输出优先级保持 `--output`、`SPD_EPISODE_OUTPUT`、配置 `data_dir`。

`replay_episode` 加载内嵌模型，在独立 MjData 逐帧赋值并调用前向计算，验证机器人投影和物体位姿；不调用 `mj_step`、不发送目标、不渲染。记录不包含 ctrl 或全部积分器历史，不能当作恢复原控制运行的检查点。公开校验和恢复拒绝 partial／未完成文件，且拒绝 schema、模型、元数据和 MuJoCo 版本不匹配。详见 [schema-v2.md](schema-v2.md)。

`CollectionRosControl` 只负责 JSON 状态序列化与心跳，由原生 rclcpp publisher 向 `/spd/collection/status` 发布可靠 transient-local 消息；不开放外部 Trigger 控制。状态包含 collector／operation、路径、帧数、physics_paused、人工 checkpoint_frames、auto_checkpoint_frames 和完成结果；0 是合法起始人工点。底层 session／recorder 管理 API 服务专用集成，不构成第二套操作者流程。

采集侧已实现在线检查点／回退，不实现采后接触裁剪或 30 Hz 样本构建。旧 `align_30hz`、`filter_contacts` 依赖已废弃契约，已移除；离线渲染输出当前保留轨迹的所有源帧，不代替这些处理。

## 7. 八 GPU 离线渲染

`pixi run -e render spd-render` 使用独立无 ROS 的 render 环境。默认部署配置选择 8 个 EGL 设备，每卡一个 spawn worker；父进程不导入 MuJoCo／GL、不加载图像。worker 在任何 native import 前设置 `MUJOCO_GL=egl`、`MUJOCO_EGL_DEVICE_ID`、`PYOPENGL_PLATFORM=egl` 和数值库线程数，再用实际 GL vendor／renderer 检查 NVIDIA 硬件与目标型号。EGL 索引不等于 CUDA_VISIBLE_DEVICES 映射，服务器必须先运行 `--check-gpus`。

所有 worker 初始化成功才派发 episode，空闲进程从共享队列取下一段。每段独立加载内嵌模型，单个 Renderer 顺序产生三视角 RGB 和实例掩码，缓冲区有界，不把像素送到父进程。每卡显存独立；workers_per_gpu 可调但不承诺线性加速。当前是原生 EGL 而不是 Warp／Madrona，不需要 CUDA 训练框架。

相机位置由用户后续 URDF 定义，当前只固定逻辑名 top／left_wrist／right_wrist。渲染器不决定外参，只读模型中已有相机；缺失即报错。默认拒绝 provisional 或无 calibration_revision 的快照，显式诊断开关允许预览但输出标记 diagnostic_only。现有临时 YAML 的坐标不代表正式相机位置。URDF link/joint／相机扩展转换和历史轨迹换相机须在实际格式确定后单独接入，不隐式修改已记录模型。

渲染对原始轨迹只读，不调用 mj_step；逐帧赋值并 mj_forward，复用机器人／物体位姿一致性检查。隐藏 group0／3 碰撞代理；MuJoCo 分割的 GEOM ID 通过物体子树映射成源 instance_id，同一物体的多个网格共用一个 ID。render schema 2 保留 0 天空、-1 机器人、-2 其他环境，并新增 -3 桌子；柜体、抽屉等任务物体仍用正 ID。相机世界位置、旋转矩阵及源 frame/tick/time 同行写出。

结果独立为 .render.h5；独占锁、partial、完整内容校验和同目录原子无覆盖发布防止混写。源文件／模型／元数据／设置哈希决定复用，已完成输出仍须逐流校验；不匹配、损坏或残留 partial/lock 明确失败，不自动重试／覆盖。进程硬退出或启动超时清理其余自有进程，报告未确认任务；重跑可跳过校验通过的已完成段。输出格式详见 [schema-v2.md](schema-v2.md)。

`training_data` 提供 NumPy CPU 读取后视觉增强：实例染色、独立桌子／背景纹理，时序与多视角共享计划，保留机器人像素、整数掩码、状态与控制质量标签。读取器要求 render schema 2 和完整源轨迹，拒绝跨 rewind 边界或进入非零 recovery_transition 新阶段的边界，即使索引跳过中间帧。旧缺失 recovery 保持缺失；旧缺失 flags 返回零占位但 `control_flags_annotated=false`，不宣称已知正常。增强不改 HDF5；旧 render schema 1 须重新渲染，不猜测桌面掩码。纹理是图像空间变换，不宣称三维材质或 GPU 增强；`pixi run spd-augment` 可生成对照图。完整像素校验仍由 renderer 负责。

## 8. 研究与验证边界

论文参考为 [Pre-training Visual Dexterity in Simulation](papers/2608.15917v1.pdf) §3.1、附录 A.1。论文物理 480 Hz、控制／流传输／记录 60 Hz、训练网格 30 Hz 是不同阶段的契约，不等同于当前实现各流频率或机器性能保证。

原生构建和 41 项定向回归通过，后续 Home 暂停、场景交接和自动快照分支清理有补充验证。真实 TCP 合成输入流程保留 91 帧，恢复标签 0/1/3/4；实际图形 CLI 和原生循环完成终端 r/s/d 操作、244 帧成功文件、准备场景 Home、下一任务及正常退出。真实 EGL render/source 对验证了训练读取拒绝跨重绑定序列、兼容旧标签来源。硬件安全、真实头显跟踪、硬实时和八卡吞吐仍未验证；相机最终标定、训练收益与论文等价性不能由模块存在推断。PICO v1 无显式 tracking-origin epoch；连接／源时钟异常和大幅姿态跳变可撤销参考，但同连接小幅坐标重置无法可靠区分正常运动。

**未来计划：CONTROL-TYPE 辅助，尚未实现。** approach／alignment／grasp／insertion（接近／对齐／抓取／插入）辅助需作为独立控制类型设计和标注，不能与纯人工动作混标。当前 Viewer 只有任务、状态、检查点和手指等待提示，不含上述辅助策略。
