# Quest／PICO 统一裸手仿真采集

**一个命令、一个采集进程、一套按键。** 头显输入、相对参考绑定、DLS/Ruckig、Hand2、物理和采集由本项目统一编排；不再启动第二个 ROS 裸手发布／控制终端。只控制仿真，不控制实机。

## 安装与头显准备

```bash
# 首次使用，或依赖／原生源码更新后，在项目根目录执行
pixi install --locked
pixi run --locked spd-teleop-build

adb devices -l
# Quest：开启开发者模式并授权 USB 调试
adb install -r apps/quest/quest3s_hand_tracking.apk
# PICO：使用裸手跟踪 APK，不是手柄 APK
# adb install -r apps/pico/pico_hand_tracking_adb.apk
```

在头显中打开对应应用、开启手部跟踪并授予权限。停止旧 PICO／Quest／Manus／外骨骼会话，不要让诊断接收器和采集入口同时占用输入。多设备时设置 `ANDROID_SERIAL`；本地后端需要时建立 ADB 转发。

Quest 与 PICO 共用 TCP `10002`：小端 `<BBqI>` 帧头，magic `0xAB`、type `0x40`、version `1`，载荷 `1968` 字节，双手各 26 个 OpenXR 关节；坐标 FLU（前、左、上），四元数 `xyzw`，不额外翻轴或交换左右手。日志中的 PICO 是共用输入实现名称。

可选诊断 `pixi run spd-quest-receive --print`，或加 `--save-jsonl /tmp/quest-frames.jsonl`；诊断结束后 Ctrl+C，再启动采集。独立诊断 GUI 的依赖在 `tools/hand_tracking/requirements-visualize.txt`，入口为 `python tools/hand_tracking/quest_hand_tracking_receiver.py --visualize`，不是 MuJoCo 窗口。Quest APK SHA-256：`4206849228aa6be0c8c16c3ee48240afb142f053820dc970bcd47991687ac22c`。

## 一个命令开始

```bash
# Quest；1.75 换成操作者实际身高（米）
pixi run spd-quest-teleop --height-m 1.75

# PICO，二选一运行；任务、输出、无图形等参数与 spd-sim 相同
# pixi run spd-pico-teleop --height-m 1.75 --task mugs/hang_mug --seed 0
# pixi run spd-pico-teleop --height-m 1.75 --headless --output data/episodes
```

Quest 包装脚本转入 `bash/run_pico_hand_sim.sh`，再经 `bash/start_spd_sim.sh` → 原生 `spd_executor` → `simulation.ros_viewer --height-m ...`。也可直接 `pixi run spd-sim --height-m 1.75`。不再使用旧 `r` 前伸标定、`s` 跟随的独立发布入口，也不需要相邻源码工作区或 `TIANJI_TELEOP_ROOT`。

首次启动不传 `--task` 时从 18 个任务中选择；`--task mugs/hang_mug` 指定首个任务，`--scene cups` 限制首个任务范围，`--scene hardware_free` 首次为无任务场景。每次成功保存后，都从完整任务目录重新随机分配任务和新 seed，重新生成布局、桌高 `0.70–0.80 m` 与桌距 `0.10–0.30 m`；随机抽样允许任务重复。`--seed 0` 可复现整个选择序列，`--table-distance 0.2` 只覆盖首个场景桌距。输出默认 `/data/TianjiSim/trajectories/YYYYMMDD/episode_<UUID>.h5`，`--output` 优先于 `SPD_EPISODE_OUTPUT` 和配置。日期以开段时为准，跨午夜不拆当前段。

## 操作与恢复

操作者自行选择舒适、稳定的腰间准备姿势，面向前方并让头和双腕可跟踪，然后按 `r`。系统在冻结状态下将当前手腕位置／朝向绑定到机器人保留目标，**绑定本身不引起运动**，不要求双臂水平前伸。这里的“腰间”是操作者选择的姿势，**没有躯干或腰部跟踪器，也不估计腰部运动**。后续以绑定参考下的手腕相对运动控制双臂。

| 按键 | 当前统一含义 |
|---|---|
| `r` | 待开始时绑定并开段；每次失跟踪后首次按下只重新接手，不存点、不回退；正常采集中更新保存点 |
| `s` | 保存并校验当前已有样本的段；完成后直接随机新任务，机器人初始化 Home，等待重新按 `r` |
| `d` | 正常采集／人工暂停时回退当前保存点并裁掉失败后缀；失跟踪等待时不触发恢复，须先按 `r` |
| 空格 | 人工暂停／重新绑定后恢复；回 Home 期间也可暂停／继续；不解除失跟踪等待 |
| `x`、`x` | 第一次暂停并提示，第二次明确丢弃整条；其他操作取消确认 |
| `q` | 退出；不代替保存，未完成段保留 `.partial.h5` |

终端单键无需回车，Viewer 使用同一键位；Ctrl+C／窗口 Esc 也可退出。处理中普通操作可能被忽略，安全冻结仍有效。只点按，不长按；终端自动重复可能产生第二次 `x`。三键踏板仍可映射 `r/s/d`，暂停与整条丢弃需要空格／`x`，不需要 evdev、设备路径或专用权限。

**典型流程：** 稳定准备姿势 → `r` 开段 → `r` 存人工检查点 → 失误直接 `d` 回退并重新接手 → 成功 `s` 保存；整条不要则 `x`、`x`。任务成功由操作者判断，不是自动评分。

手指按左右手独立跟随，不要求匹配握姿。首次接入和持续失效后的恢复使用 200 ms 平滑混合，每关节目标限速 2 rad/s。任一侧输入无效，或目标超过 **45 ms** 不新鲜，立即保持当前目标，不外推、不继续执行旧目标；但距最后有效目标不足 **120 ms** 时保留原跟随／接入状态，暂停接入计时，恢复新鲜输入后接着原进度继续，不重走整套接入。达到 120 ms 才进入等待，恢复后重新平滑接入；连续无效帧不会延长预算。已正常跟随的手在短缺口时显示“短时保持”，不因此新增虚影；等待或平滑接入时才显示目标虚影。录制仍即时标记手指保持／退化，不把短缺口伪装成正常数据。另一手和双臂不被单侧手指缺口阻塞。

短缺口更新通过 14 项定向测试；真实 DLS／Hand2＋MuJoCo 窗口的合成输入验证包含反复 40 ms 单侧关键点失效、持续失效和恢复。短缺口期间保持 live 状态且不新增虚影，持续失效后显示等待虚影，另一手和录制继续；424 帧文件校验通过并保留手指保持／重入标记。真人遮挡和实际负载下的体验尚待验收。

短时输入缺口使用有界制动；可信输入及时回来可在制动途中从保留的 q/v/a 恢复，不必先完全停止。可信双侧头／腕候选持续丢失超过初始 `120 ms` 预算时，暂停整个世界和采样并记录现场快照用于提示，进入可恢复等待。操作者参照停住的机器人姿态大致摆好现实姿态，跟踪稳定后首次按 `r`，将当前现实手腕参考绑定到机器人当前保留目标，从断开处继续跟随。**此 `r` 不读取或恢复快照，不修改物理状态、不裁剪数据，也不覆盖原保存点。** 每次新的失跟踪都重新进入这个流程；平常 `r` 更新保存点，`d` 回退原保存点，语义不变。等待时 `d`／空格不重新接手，`s` 仍可保存已有样本。参考身份不连续也进入该流程；源／求解 worker 故障和非有限数仍 fail-closed。有限关节目标越限则饱和到 manifest／执行器限位交集，不再因此异常冻结；物理限位仍保留。120 ms 是控制预算，不是硬实时承诺。

Hand2 本地进程响应等待预算为 **300 ms**（原 100 ms），不是允许使用 300 ms 旧目标。读取时先取管道中已缓存的响应，再对缺失字节检查剩余预算，避免场景构建／线程调度推迟读取后将已完成求解误报为超时。预算不逐次重置，响应序号、原始观察时间戳和 45 ms 新鲜度规则不变，过期结果不驱动手指；真正缺少响应仍报错，并标明左右手及已收到字节数。诊断中合成输入的原生响应约 1.4 ms 已完整写出，场景构建约 1.3 s 后读取：旧版误报、新版成功取回且丢弃过期控制目标；人为延迟 worker 150 ms 时，100 ms 预算失败、300 ms 预算通过。16 项手指输送／接手定向测试通过；这些结果不代表已确认此前真人故障的唯一原因。

**成功保存后直接新任务，不进入安全准备区、不执行回 Home 运动。** 文件关闭、校验并发布 `.h5` 后，随机生成下一任务及布局，新机器人直接初始化为 Home、零速度，清空旧检查点和授权，等待重新按 `r` 绑定开段；不继承上条机器人的实际姿态或目标，也不自动录制。保存失败则保留原场景及 partial。**丢弃 `x`、`x` 的流程不变：** 进入无任务物体准备场景并平滑回 Home，再重建同任务、同初始 seed／布局；该路径携带实际机器人状态，Home 完成要求双臂接近目标、速度收敛及全关节位置稳定，不要求柔顺伺服下 qpos 精确相等。Home 失败不强行切场景。

新段保留成功前缀，人工回退会同步裁剪轨迹和标签。schema-v2 扩展 `/collection_events/recovery_transition`：`0` 正常、`1` 开段、`2` 人工暂停恢复、`3` 人工检查点回退恢复、`4` 失跟踪后重新接手；可选 `control_flags` 按采样区间累计输入退化和每手等待／重入。失跟踪重新接手本身不产生回退事件，但训练仍不能跨重新绑定边界拼接，旧无标签文件须保留来源不明的语义，详见 [数据契约](docs/schema-v2.md)。

`spd-teleop-build` 分别构建 `.teleop/{build,install}` 中的双臂 DLS/Ruckig（MuJoCo 3.10／Pinocchio 3／Eigen 3）和独立 Hand2（Pinocchio 4），再构建 `.ros/{build,install}` 中的消息与仿真执行器；不将前者 ABI 混入 MuJoCo 3.12／ROS Jazzy 仿真环境。求解器、模型或配置更新后重新构建。

---

# SPD Simulation Collection — Tianji + Wuji Hand 2

SPD 负责统一裸手仿真采集与离线渲染：本地后端接收 PICO／Quest 输入并生成 DLS/Ruckig＋Hand2 目标，唯一物理线程协调绑定、执行、录制和场景生命周期。在线记录 60 Hz 完整场景物理轨迹及接触，不创建采集相机渲染器、不保存 RGB；离线恢复状态生成 RGB 和实例分割。整个项目不控制实机。

`src/pico2_hands/collection_session.py` 是本地后端；`simulation.collection_control.CollectionControl` 是物理线程上的唯一流程权威，worker 线程负责输入与原生求解器交互，不写 MuJoCo 状态。省略 `--height-m` 的 `spd-sim` 仍可作为可选外部 DDS 订阅模式，使用相同按键，但不提供本地手腕重绑定／每手平滑重接入控制，发布端对齐由外部集成负责。

论文参考：[Pre-training Visual Dexterity in Simulation](docs/papers/2608.15917v1.pdf)。程序化场景、相机与数据流程不是论文结果的等价性证明。

## 源码与资源职责

```text
pixi.toml / pixi.lock                 受维护运行环境与命令
setup.py                            Python 安装配置与 CLI 入口
src/
  spd_native/                       C++ 可选 ROS 订阅、授权、run_loop／Physics 与接触采集
  pico2_hands/                       本地 TCP 输入、相对绑定与 DLS／Hand2 worker 编排
  teleop_native/                     DLS/Ruckig、辅助 Viewer、模型和配置
  interfaces/                       Python wire 工具、终端键盘与 ROS 消息定义
  simulation/                       CollectionControl、模型准备、原生编排、Viewer 与场景生命周期
  cameras/                          仿真多视角相机
  data_collector/                   完整场景轨迹、模型快照、ROS 采集控制及独立恢复检查
  offline_rendering/                EGL 多 GPU 调度、只读状态恢复、RGB／实例分割输出
  training_data/                    只读多视角序列读取、分割感知训练视觉增强与预览
  description/                      manifest、资源定位、机器人模型编译
  environments/spd_envs/             独立环境包：任务、随机重置、场景生成
  tianji_wuji2/tianji_wuji2/
    assets/                          原始 URDF、网格与碰撞资产
    generated/                       编译后的模型与 manifest
  interfaces/tianji_spd_interfaces/   ROS 2 JointCommand 消息包
config/                              采集、临时预览相机与 8×5090 渲染服务器配置
bash/                                统一裸手／仿真前台入口与只读状态查询
tools/wuji_hand_native/               独立 Hand2 环境、官方重定向桥接与模型
apps/pico/ / apps/quest/              两种头显的裸手跟踪 APK
data/                                采集输出和已有数据；不随代码清理删除
docs/                                架构、数据契约与论文
```

运行时 Python 包直接位于 `src/`，使用 `simulation.*`、`data_collector.*`、`cameras.*`、`interfaces.*`、`description.*` 导入，不保留统一外层包或旧导入兼容层。根目录 `setup.py` 安装这些包，发行包名仍为 `spd`；独立环境包保持 `spd_envs`，依赖方向为 `spd → spd-envs`，环境包不依赖 ROS、PICO 或机器人输入算法，也不硬编码 Tianji 模型路径。

## 安装与启动

在项目根目录操作，支持 Linux x86-64。环境固定 Python 3.12，保留 MuJoCo 3.12，并锁定兼容的 ROS 2 Jazzy / Fast DDS 依赖。图形查看需要可用显示环境；中文任务条使用系统 `fonts-noto-cjk` 的 Noto Sans CJK 字体（缺失时程序给出安装提示），无图形模式不加载字体。本地头显采集使用 ADB／TCP；可选外部 DDS 模式和独立场景查看不需要头显。在线仿真从本 checkout 读取资源；轨迹内嵌已编译模型和网格／纹理，可脱离原场景 XML 恢复，但必须使用记录时的精确 MuJoCo 版本。不支持将 Python wheel 脱离工作区作为完整在线仿真部署。

```bash
pixi install --locked
pixi run spd-teleop-build

# 本地统一模式，默认逐段随机任务、桌高和桌距
pixi run spd-sim --height-m 1.75

# 指定首个采集任务；成功保存后仍随机新任务
pixi run spd-sim --height-m 1.75 --task mugs/hang_mug --seed 0

# 无图形随机任务；此例只固定首个场景桌距
pixi run spd-sim --height-m 1.75 --headless --table-distance 0.2 --output data/headless-episodes

# 首个场景无任务物体；成功保存后进入随机任务
pixi run spd-sim --height-m 1.75 --scene hardware_free

# q／Ctrl+C 退出；先 s 并等待保存完成，退出本身不保存成功
```

`spd-native-build` 在 `ros-jazzy` Pixi 环境中构建 `tianji_spd_interfaces` 和 `spd_native`，生成 `.ros/install/lib/spd_native/spd_executor` 与 `_spd_native` Python 扩展。原生源码或依赖更新后重新构建；仅运行旧的 `ros-build-interfaces` 不会构建执行器。

在线入口为内嵌 CPython 的 C++ `spd_executor`；原生 `run_loop`／`Physics` 保留 480 Hz 调度、MuJoCo 步进和手–物接触循环，调用物理线程上的 Python `CollectionControl` 统一协调输入绑定、采集与 Home。本地 `TeleopSession` 的 worker 线程管理 TCP 与原生 DLS／Hand2；只通过不可变、带 generation 的快照交接，旧场景／重绑定前结果不能重新授权。原生执行器仍校验目标、管理邮箱和授权，本地模式禁用 DDS 目标订阅。步进释放 GIL；不宣称完全无 Python、无 GIL 或硬实时，也不保证消除 MuJoCo 接触求解瓶颈。

`pixi run spd-scene` 自动进入原生运行环境并加载 overlay；模型编译、独立场景生成及离线恢复／渲染仍可使用各自 Python 环境。回归入口为 `pixi run spd-test`，需先完成原生构建。旧独立双进程控制入口、原生三键状态机和 `spd-viewer` console 入口已移除，不提供旧流程兼容入口。

SPD 启动器不会自动重建接口。`config/collect_sim.yaml` 默认采集根目录为 `/data/TianjiSim/trajectories`，新 episode 自动存入开始当天的 `YYYYMMDD/` 子目录；显式 `--output PATH` 优先于 `SPD_EPISODE_OUTPUT`，两者均未提供时使用配置的 `data_dir`。

主进程在当前终端前台运行，不使用 tmux。终端 `q`／`Ctrl+C` 或窗口 `q`／`Esc` 退出并关闭自有后端，不停止外部硬件控制器；未完成段保留 partial。一次只启动一个采集进程，不共享输出目录。暂停／回退冻结物理，输入监控继续以便稳定重绑定。

### 可选外部 ROS 订阅模式

```text
外部 JointCommand 发布器（省略 --height-m 时）
    → Fast DDS / ROS domain 120
    → SPD 校验与最新目标邮箱
    → 本地显式授权 / 分组 hold / 名称映射
    → MuJoCo position actuators 与物理积分
    → 全场景 qpos/qvel、机器人状态、物体位姿、接触、时钟 → HDF5
```

消息定义：`src/interfaces/tianji_spd_interfaces/msg/JointCommand.msg`；类型 `tianji_spd_interfaces/msg/JointCommand`；话题 `/spd/tianji_wuji2/v1/joint_command`。版本为 `schema_version=1`，机器人配置为 `tianji_wuji2_v1`，QoS 为 `BEST_EFFORT / KEEP_LAST(1) / VOLATILE`。

启动器在 Pixi 激活、加载接口 overlay **之后**固定设置同机发现配置：

```bash
export ROS_DOMAIN_ID=120
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_STATIC_PEERS=''
```

上游需使用兼容配置。跨主机不能仅依赖此同机入口：必须另外显式配置 DDS 发现、网络接口、防火墙和 UTC 时钟同步；domain 不是认证或安全边界，应使用可信网络。没有 ROS/DDS 桥。

54 维顺序为左臂 7、右臂 7、左手 20、右手 20，单位 rad。订阅端校验固定关节名称契约、维度、有限值、ready 掩码、session、递增 sequence 和 UTC 新鲜度；有限目标按 manifest／执行器限位交集饱和后进入邮箱，授权误差比较、过渡插值与实际执行使用一致的饱和值，再按名称映射到 qpos/actuator。未 ready 或保持中的组不被新目标驱动。场景自由度不占这 54 维；收到目标不意味着授权运动。

该模式使用与本地采集相同的 `r/s/d/空格/x/q` 键位，只发布 `/spd/collection/status`，不开放外部 Trigger 控制服务。它不能要求外部发布端重置手腕参考、执行每手平滑重接入或遵守本地 generation；因此**没有本地无运动重绑定与失跟踪重新接手保证**。外部目标失效后冻结，外部重新对齐后由操作者按空格恢复。

外部开段／恢复使用受控的一秒目标混合，起点取实际 qpos 并投影到合法命令范围，终点为新鲜候选的饱和值，不写回物理状态；一秒结束不等于实际到位。有限越限命令不再报错冻结，NaN／Inf 等非法命令仍拒绝。ready／session／新鲜度校验与分组 hold 属于外部 wire 安全契约，不要将其当成本地每手平滑重接入逻辑。

应用输出 `ready` 只表示初始化完成，不代表输入已绑定或运动已授权。顶部中文任务／状态条显示任务目标、状态、帧数、人工／自动检查点和手指等待提示；异常保留诊断原因。双手虚影在暂停／重接入时辅助观察，正常采集隐藏。当前提示仅覆盖任务、状态、检查点和手指等待，尚无 CONTROL-TYPE 动作辅助。

### 单窗口双视角观察

有图形界面的 `pixi run spd-sim` 只打开一个 MuJoCo 渲染窗口：顶部整条中文任务／状态提示，下面左右等宽双视口。左侧保留原始自由视角，左键拖动旋转、右键拖动平移、中键拖动或滚轮缩放，Shift 可切换水平操作；鼠标操作只在左侧场景区域生效，右侧和顶部提示区域不移动相机。

右侧固定为头部第一人称视角：观察点位于 `Base_L`／`Base_R` 中点上方 `0.35 m`，当前模型约为世界坐标 `(0, 0, 1.471) m`，朝机器人前方 `+X` 看并下倾 `35°`，垂直视场角 `70°`。这不是垂直俯视，也不修改训练相机 `top` 的标定。两侧使用同一个状态快照，并同步显示暂停／接入时的双手虚影。

窗口内 `r/s/d/空格/x/q/Esc` 与终端含义相同；界面忽略长按重复，终端长按仍可能产生重复字符，建议点按。默认窗口 `1600×900`，最小 `960×600`；提示自动换行，过长诊断以省略号提示截断，完整信息保留在终端／采集状态。`spd-scene` 独立场景查看仍是单自由视口。

渲染线程独占一个 GLFW 窗口、GL 上下文及模型／数据副本，使用两个真实 MuJoCo 透视视口，不以裁剪单张画面伪造分屏。物理线程只提交有界最新快照，不等待 GPU 绘制；显示可跳过中间快照，采集不丢帧。渲染只更新运动学，不推进物理、不改变原模型训练相机，也不保存观察 RGB。旧的头部观察子进程／第二窗口已移除；`--headless` 不创建渲染线程或窗口。

### 检查点与失败回退

- `r` 开段前自动建立 0 号人工检查点；采集中 `r` 更新人工点，允许手–物接触，不再有无接触门。
- 检查点只属于当前 episode，保存完整 `MjData`、保留目标、tick、采样相位和区间标签／接触累计，不从 HDF5 猜测控制状态。
- 正常采集／人工暂停时，`d` 无需先暂停：冻结、裁掉当前保存点后的轨迹和标签，刷盘确认后恢复完整物理状态，再等待稳定输入重绑定。参考重置时机器人保持不动，旧 generation 结果被拒绝。
- 每次失跟踪后，首次 `r` 只从停住的现场重新绑定、接手；不会读取快照、回退物理状态、裁剪数据或替换手动保存点。正常采集中的 `r` 才更新保存点，`d` 回退最近手动点。首次接手前，稳定输入本身、`d` 或空格都不会启动跟随。现场快照只用于提示，并在续采时清除；保存点只在当前进程有效，不是成功完成的轨迹文件。
- `.h5` 仅包含最终保留前缀及其续采；失败分支被裁掉，`collection_events/rewind` 标明分支边界，不是完整失败审计日志。检查点不跨进程／模型持久化，主机单调时钟不回退。

踏板可继续输出左 `r`、中 `s`、右 `d`，但含义是存点／保存／直接回退；空格暂停和双击 `x` 丢弃由键盘提供。终端使用 cbreak 即时输入，退出恢复终端设置，不读取 `/dev/input`，不检测拔出或长按。

## 任务场景与模型

环境包管理 `jenga`、`spelling_blocks`、`mugs`、`dishes`、`cups`、`bottles` 六类场景，保留论文 Table 2 的 17 个任务和附录 A.4 的 `jenga/playing`。独立场景查看保持机器人 HOME 目标，不启动 ROS 或发布器：

在线采集的显式任务／场景限制只作用于首次启动。成功保存关闭并校验文件后，直接从全目录随机选择新任务和 seed、生成新布局，新机器人初始化 Home 并等待 `r`，不经过准备场景或回零运动。丢弃完成后仍通过准备场景平滑回 Home，再重建原任务及其初始 seed／布局。暂停、回退和保存失败不重建场景。每段 metadata 包含 `task_title_zh`、`task_goal_zh`、实际 `scene/task/seed`。

```bash
pixi run spd-scene --task dishes/rack_dishes --seed 0
pixi run spd-scene --task mugs/hang_mug --seed 0
pixi run spd-scene --task jenga/playing --seed 0
pixi run spd-scene --task cups/pyramid --seed 0
pixi run spd-scene --task bottles/toss_in_bin --seed 0
pixi run spd-scene --task spelling_blocks/spelling --seed 0
pixi run spd-scene --task spelling_blocks/sort_and_unload --seed 0

# 导出物理场景、随机参数、状态和截图
pixi run spd-scene --task cups/pyramid --seed 0 \
  --table-distance 0.10 --headless --duration 3 --output data/task_scenes
```

每次生成场景时直接均匀采样桌面上表面高度 `h ∈ [0.70, 0.80] m` 和近侧桌沿距离 `d ∈ [0.10, 0.30] m`，生成后固定。桌板尺寸 `0.80 × 1.10 × 0.05 m`，中心为 `(d + 0.40, 0, h - 0.025) m`；距离沿机器人前方 `+X` 从底座原点测量。桌上物体与固定支架同步定位，桌腿伸缩而脚垫保持落地。在线 `--table-distance` 只覆盖首次场景，成功保存后的新任务重新随机桌距；独立 `spd-scene` 仍接受显式桌距。相同 seed 可复现，暂停／恢复／回退不重采样。随机范围不构成全姿态可达或避碰保证。

种子可重现位置、质量、摩擦、资产型号、材质变体、拼写目标和字母分配；任务 manifest 记录实际参数、桌距和 `geometry_revision=paper-aligned-scenes-v2`。物体使用重力、碰撞与摩擦，不直接写 qpos 播放、不焊住自由物体、不用禁用接触或允许穿透伪造稳定。盘架、杯架、箱体和柜体固定；字母块等任务物体自由运动，抽屉通过被动有界滑动关节运动。场景不是自动策略、任务评分或论文原始 CAD 的精确复刻。

物理求解统一使用 `480 Hz`、`implicitfast`、`cone="elliptic"` 和 `noslip_iterations="1"`，对齐论文 A.1；机器人完整模型／双臂投影、独立场景及初始化接触检查均使用该设置。物体质量、摩擦和几何仍是工程设定，不代表与论文物理等价。接触密集场景不保证墙钟实时：六类 seed 0 场景各推进 960 步的本机无渲染检查中，`jenga/playing` 新设置平均约 `13.96 ms/步`（原设置约 `6.56 ms/步`），超过 480 Hz 的 `2.08 ms/步` 预算；其他五类新设置平均约 `0.18–0.99 ms/步`。这仅是 HOME 保持下的短时接触／耗时检查，不是抓取或完整遥操作验收。

场景按 ABC 的方式分开**外观资产与碰撞代理**：外观使用有纹理的网格、圆滑表面和细节零件，质量为零且不参与接触；碰撞几何独立承担质量、惯性与实际接触。场景碰撞组为 3，Viewer／相机默认隐藏这一组，只影响显示、不禁用物理。

| 场景 | 细粒度内容 |
|---|---|
| Jenga／多米诺 | 普通木块和三种木纹，无点数／棋子分隔线；多米诺为竖立的 `25×15×75 mm` 木块 |
| 字母积木／柜体 | A–Z 六面彩色字母和同色边框，8 种墨色；三层可开合抽屉和真实内腔，分拣任务从抽屉内部取块 |
| 马克杯 | 三种杯身几何、四种配色、开放椭圆杯柄；杯架切向挂杆穿过杯柄孔，避免径向支杆刺入杯沿 |
| 餐盘 | 三种浅盘尺寸；盘架三条净宽 `50 mm` 槽位，保留真实插入和支撑空间 |
| 杯子 | 三种可套叠几何，红／绿／蓝／黄四种鲜明配色全部可采样；中空杯壁和防卡支点与碰撞代理一致 |
| 瓶子／箱子 | 6 种 ABC 瓶子网格及贴图；三种箱体内腔尺寸，保留真实开口、底板和侧壁 |

桌面增加木纹、边框、桌腿和统一双光源；桌腿／边饰／背景地面为外观元素，桌面仍使用原尺寸静态碰撞体。物体开口、杯柄和箱体内腔不是贴图伪造。物理代理与细小外观倒角并非逐三角面完全相同。

拼词目标按 seed 从 `CAFE / IMAGE / ROBOTICS / ROBOT / TABLE / CUP / BLOCKS / VISION` 中选择；中文任务条和采集 metadata 显示实际目标，不固定显示 ROBOTICS。布局包含规则、错列、散布方案，松散物体可全周朝向随机；塔、套杯和柜体组件整体旋转／平移，保持装配关系。所有摆放通过实际碰撞检查，桌沿检查包含抽屉完整行程与把手。

柜体尺寸 `260×320×426 mm`，三个抽屉托盘尺寸 `238×292×85 mm`，被动行程 `0–250 mm`，开口朝机器人方向（局部 `-X`）。柜体正面可通过鼠标旋转观察；抽屉不是贴图或动画。柜体与各抽屉是独立实例根，避免接触／分割子树重叠；抽屉位置、速度进入完整 `qpos/qvel`，不增加机器人命令维度。导向搁板保留 `1 mm` 运行间隙，不用关闭接触消除卡滞。

`sampled_values.affordances` 按实例 ID 记录盘架槽位、杯柄／挂杆间隙、箱内尺寸、抽屉行程／内底／把手和套杯间隙；局部坐标需乘物体位姿。`mass_kg` 为实际采样质量，`reference_mass_kg` 仅是基准类别质量；几何变体按碰撞体积缩放，柜体／抽屉按木材密度估算，均不等于实物标定。

环境包本地资产在 `src/environments/spd_envs/spd_envs/assets/`：`abc_bottles/` 包含 63 个网格、7 张原始贴图及来源／变换／SHA-256 清单，运行时不读取外部 ABC 工作区；`detail_textures/` 包含 31 张原创木纹／釉面／字母贴图及可重建生成脚本。瓶子按同一变换规范化视觉和碰撞，底部 z=0，半径与高度按种子轻微变化；manifest 同时保留资产来源、外观参数和碰撞调试色。

**ABC 瓶子资产仅按已有本地文件适配；其公开再分发授权未确认。** 原始文件路径与哈希记录在 `abc_bottles/manifest.json`，复制到本仓库不代表获得发布授权，公开发布前必须确认权利。

机器人动力学保留 URDF 的质量、质心和惯性，不以外观网格体积重算装配惯性。木、陶瓷、塑料、涂层金属等接触参数仍是未实物标定的工程默认值；瓶子按轻质空瓶配置名义质量，不能把品牌贴图当作真实材质或质量测量。刚体仿真不模拟材料屈服、破碎或柔性。机器人外观透明度只影响显示，不改变接触和动力学。

模型现有临时碰撞例外为左侧 `Link5_L–Link7_L` 和右侧 `Link5_R–Link7_R`，记录在 `collision.temporary_excludes`。它们绕过原始凸包腕部干涉，也会忽略这两对连杆的真实碰撞，不是硬件安全保证。项目包含本地 IK／轨迹生成，但不提供硬件避碰或安全认证；限位和控制门也不能替代这些保证。

模型编译默认拒绝覆盖非空产物目录，检查新模型应使用新目录：

```bash
pixi run spd-model --output /tmp/spd-model-check
pixi run spd-envs-check
```

正式资源在 `src/tianji_wuji2/tianji_wuji2/{assets,generated}`。不要在采集会话中替换模型；更改资产后重新编译、验证再使用，不混合不同机器人配置的数据。

## 配置化采集与状态查看

`pixi run spd-collect` 与 `spd-sim` 使用同一仿真／录制入口，二选一运行，均使用首页统一按键；传入 `--height-m` 启用本地裸手后端，省略时仅为可选外部 DDS 模式。

当前交互入口只发布只读状态 `/spd/collection/status`；可使用 `pixi run spd-collect-trigger --command status` 查看。旧远程 start/save/discard/skip 等控制示例不再适用于当前入口，外部 Trigger 服务不开放。

采集配置 `config/collect_sim.yaml`：

```yaml
version: 2
data_dir: /data/TianjiSim/trajectories
state_rate_hz: 60
writer_queue_size: 256
max_frames: 0
```

- `data_dir` 是采集根目录；相对路径基于配置文件所在目录解析，可用 `--collection-config PATH` 指定配置。每次开始新段按本机本地日期选择 `YYYYMMDD/`，例如 `/data/TianjiSim/trajectories/20260922/`。跨午夜正在录制的段不拆分，仍保存到开始当天；下一段自动进入新日期，无需重启。`--output` 和环境变量覆盖的根目录也遵循此规则。
- 轨迹固定每 8 个 480 Hz 物理步采一帧，即仿真时间 60 Hz。记录物理步编号、仿真时间和主机单调时间；实际墙钟频率必须另行统计，不能以名义调度推断。旧配置中的 `camera_rate_hz` 已移除，配置版本改为 2。
- `writer_queue_size` 限制后台写入队列；溢出报错并保留 partial，不静默丢帧。
- `max_frames: 0` 表示不限；正数限制完整场景轨迹帧数，不按接收的命令数计数。可用 `--max-frames N` 覆盖。
- 到达正数上限自动结束、校验并保存为 **`success=false`**，完成后同样直接生成随机新任务、初始化 Home 并等待 `r`，不自动开启下一段；默认 `0` 不限帧数。只有操作者 `s` 显式保存才标记成功。
- 每段记录实际生效的配置与配置文件路径；采样直接读取当前物理状态，不等待新的 ROS cmd，也不补写录制前缓存。

默认仅发布 `/spd/collection/status`（`std_msgs/msg/String` JSON，可靠、transient-local），包含状态、帧数、路径、`physics_paused`、`checkpoint_frames`、`auto_checkpoint_frames` 和完成结果等。`checkpoint_frames=0` 是合法起始保存点；`auto_checkpoint_frames` 仅表示失跟踪现场快照的帧数，重新接手续采后清空，不转为 `checkpoint_frames`。底层采集类保留管理接口供专用集成，但 `spd-sim` 不开放外部 Trigger 控制；升级后须重启采集进程。

## 物理轨迹、场景恢复与离线渲染边界

**state-only 不等于只存机器人关节角。** schema-v2 每帧保存全场景 `qpos/qvel`、按名称排列的机器人 54 维实际位置／速度、任务物体世界位姿、左右手接触状态及接触对象。模型存在的执行器内部状态、mocap 状态和 equality 启用状态也随帧保存。没有 `ctrl`、ROS 命令、actions 或在线 RGB。

每段内嵌 MuJoCo 编译模型（含网格、纹理与相机）、版本与 SHA-256、任务和实际随机参数、关节／物体映射与相机元数据。采集开始后的样本直接来自完成物理积分的场景，不等新的 ROS 命令、不补录旧缓存。手–物接触在每个物理步观察，按采样区间累计，避免只看 60 Hz 瞬间漏掉短接触；不会把桌面接触和机器人自碰撞当作手–物接触。

`config/sim_cameras.yaml` 的三路相机当前只是 `provisional-v1` 预览定义，位置尚未定稿。离线渲染器只使用模型中已有的 `top`、`left_wrist`、`right_wrist` 命名相机，不硬编码外参，不新增或替代缺失相机。正式渲染默认拒绝临时或缺少标定确认的快照。最终位置将由用户提供的 URDF 相机安装定义转换到模型；标准 URDF 无原生相机标签，具体 link/joint 或 Gazebo 扩展转换待实际文件格式确定后接入，本次不猜测实现。

新段写入 `episode_<UUID>.partial.h5`。每个采样事件是一整帧，所有轨迹数据集严格同长；显式回退使用同一队列的有序裁剪事件。非回退造成的重复／缺失物理步、非递增时间戳、非有限状态、队列溢出或写盘失败都保留不完整段，不静默覆盖或丢帧。显式保存或达到帧数上限后，关闭并校验数据、模型和元数据，完整通过才发布 `.h5`。`complete` 表示数据完成，`success` 表示操作者确认任务成功，二者不同；帧数上限完成为 `complete=true, success=false`。

每天的目录独立保存 `dataset_config.json` 和当天的 HDF5。schema-v2 不与旧的机器人 qpos＋JPEG schema-v1 混写；同日契约不匹配会拒绝追加，不覆盖原配置。默认根目录为 `/data/TianjiSim/trajectories`；若该目录已有不兼容数据，请用 `--output` 指定新的目录。历史数据不迁移、不删除。

```bash
# 将路径替换为采集状态输出的实际文件路径
pixi run validate_episode '/data/TianjiSim/trajectories/YYYYMMDD/episode_<UUID>.h5'
pixi run replay_episode '/data/TianjiSim/trajectories/YYYYMMDD/episode_<UUID>.h5'
```

`replay_episode` 在独立 MuJoCo 模型中逐帧恢复记录状态，计算机器人状态及物体位姿的最大恢复误差；不发送控制目标，不推进物理，不渲染图像，不修改文件。拒绝不完整段、版本或模型校验不匹配。它是离线渲染前的重建验证，不是检查点继续仿真：文件没有保存重启原控制循环所需的命令和全部积分器内部历史。

数据契约见 [docs/schema-v2.md](docs/schema-v2.md)。在线人工／自动检查点、暂停与失败回退已实现，人工点允许接触；超过 10 秒无接触裁剪和 30 Hz 训练样本构建仍未实现，属于后续数据处理。离线渲染生成所有保留源帧的图像，不改变采样时间网格。旧 `align_30hz`、`filter_contacts` 入口已移除，不能从 state-only 文件恢复未记录的原始命令。

## 训练时视觉增强

`training_data` 在读取渲染结果之后执行物体染色、桌面纹理替换、背景纹理替换，不修改源轨迹或原始渲染文件。一个序列的各帧／各相机共享增强参数；机器人像素逐字节保持不变，实例掩码、关节状态、时钟和恢复标签不变。染色保留明暗细节；纹理采用程序化木纹／织纹／格纹，或用户提供的 RGB 纹理库。

**渲染伴随文件已升级为 render schema 2**，新增 `-3=桌子`，与 `-2=其他环境`、`-1=机器人`、`0=天空` 区分。旧 render schema 1 没有独立桌面掩码，必须从原轨迹重新渲染到新目录；不能仅改版本号。采集 HDF5 仍为 schema-v2，历史轨迹不被修改。

```bash
# 替换为实际已完成的渲染文件和原轨迹路径；输出 PNG 不覆盖已有文件
pixi run spd-augment episode.render.h5 --source episode.h5 \
  --frames 0,1,2 --seed 42 --output augmentation.png

# 仅替换桌子；可多次传入 --texture 建立纹理库
pixi run spd-augment episode.render.h5 --source episode.h5 \
  --frames 0,1,2 --seed 42 --texture texture.png \
  --no-object-tint --no-background --output table-preview.png
```

```python
from training_data import AugmentationConfig, RenderedSequence, VisualAugmenter

augment = VisualAugmenter(AugmentationConfig())
with RenderedSequence("episode.render.h5", source_path="episode.h5") as reader:
    sample = reader.read([0, 1, 2], augmentation=augment, seed=42)
    rgb = sample.rgb                 # uint8 [T, 3, H, W, 3]
    masks = sample.instance_id       # int32 [T, 3, H, W]
    state = sample.state            # 原记录状态，不是动作标签
    plan = sample.augmentation.as_dict()
```

当前是 NumPy CPU 增强接口，不是 CUDA／Torch 增强内核或完整策略训练器。纹理坐标共享于归一化图像空间，不宣称三维表面投影一致。读取器每次读完关闭 HDF5 句柄，检查源身份和选中帧关联，拒绝 partial、未知实例和跨回退／重绑定边界的序列；过滤非零恢复样本也不能绕过边界。旧缺失恢复标签保持缺失；旧缺失 `control_flags` 返回零占位并以 `control_flags_annotated=false` 标明，不视为已验证正常输入。完整图像校验仍由渲染器负责，增强不改变相机标定状态。

## 8×RTX 5090 离线渲染服务器

`config/render_server.yaml` 默认选择 EGL 设备 0–7、每卡 1 个独立进程、每进程 1 个数值库 CPU 线程；episode 动态分配给空闲 worker，图像不经进程间队列传输。同一段由单个进程完成，不把八张卡显存当作一个池。`--workers-per-gpu` 可调整并发，但应先测 CPU、编码、存储吞吐和显存，不能仅因显存空闲就增加进程。

服务器需要支持 RTX 5090 的 NVIDIA 驱动和 EGL/OpenGL 图形运行库，不需要显示器、X11、ROS 或 VR 设备。当前实现使用原生 MuJoCo EGL，不使用 CUDA/Warp/Madrona，不要求 Torch 的 Blackwell CUDA 构建。容器必须暴露 GPU 和图形驱动能力，例如 `--gpus all`、`NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute`；只提供 compute 库不能保证 EGL 可用。

```bash
# 使用同一份 pixi.lock，保证与轨迹内嵌 MJB 的 MuJoCo 精确版本一致
pixi install -e render --locked

# 不读取数据，只在每个 worker 中创建 EGL 上下文并报告真实 GPU
pixi run -e render spd-render --check-gpus

# 最终相机已写入模型并确认标定后，正式批量渲染
pixi run -e render spd-render \
  --input /data/TianjiSim/trajectories \
  --output /data/TianjiSim-rendered
```

**EGL 设备序号不保证等于 nvidia-smi 或 CUDA 序号。** `CUDA_VISIBLE_DEVICES` 不能替代 EGL 绑定。每个 spawn worker 在导入 MuJoCo／OpenGL 前设置 `MUJOCO_GL=egl`、`MUJOCO_EGL_DEVICE_ID`，然后校验实际 GL vendor／renderer；默认必须匹配 `RTX 5090`，不允许静默转 CPU。先运行预检，再按服务器枚举结果用 `--gpus 0,1,...` 调整。

本机或少量 GPU 可覆盖设备型号和并发，例如：

```bash
pixi run -e render spd-render --check-gpus --gpus 0 --expected-gpu-name "RTX 5060 Ti"

# 仅诊断临时视角；不是对相机位置的确认，不应混入正式训练图像
pixi run -e render spd-render \
  --gpus 0 --expected-gpu-name "RTX 5060 Ti" \
  --allow-provisional-cameras \
  --input /path/to/trajectories --output /path/to/diagnostic-renders
```

相机位置未定时可以完成 GPU／吞吐诊断，但默认正式命令会拒绝当前 provisional 数据。不要通过改 revision 名称冒充实测标定；正式 URDF 相机接入后必须重新检查视角及投影。既有轨迹中的相机不会随仓库 URDF 改动而自动改变；对旧轨迹注入新标定需要另行提供明确的转换流程，不能静默换模型。

将完整日期目录（包括 `dataset_config.json`）复制到服务器。内嵌模型包含网格／纹理，不需要原始场景 XML、原 ABC 工作区或采集主机路径。输入和输出目录必须分开；建议用本地 NVMe 暂存，完成后再归档到共享存储。输出需要支持 POSIX 独占创建、硬链接和 fsync 的文件系统。

默认输出 224×168、JPEG quality 90 RGB 和无损 int32 实例掩码。物体的多个外观网格统一映射到原始实例 ID；0 表示天空／无几何，-1 表示机器人，-2 表示非任务环境，正数为任务物体（含固定支架）。每帧保留源行号、物理 tick、仿真时间、单调时间及实际相机世界位置／旋转矩阵。相机姿态只来自源模型和记录状态。

输出路径镜像输入目录，文件名为 `episode_<ID>.render.h5`，与原轨迹分离。每段先写 partial，逐帧验证 JPEG、掩码、关联时钟、姿态、哈希和恢复误差后再发布；已有完整输出只在源文件／模型／元数据／设置均匹配且内容校验通过时跳过。配置变化或输出损坏直接失败，不覆盖、不自动重试。残留 partial／lock 必须先确认没有运行进程，再人工检查处理。

所有选择的 GPU 都须预检成功才开始作业。worker 异常退出或初始化超时会中止本批次，报告未确认完成的 episode 并清理自有进程；重跑时已完成且验证通过的文件可以跳过。跨 GPU／驱动的 JPEG 字节级一致性不作保证，原始状态和稳定实例 ID 不变，输出记录实际 GL 与 MuJoCo 版本。

## 能力与验证边界

正式入口包括 Quest／PICO 统一裸手采集、可选外部 DDS 仿真、本地统一按键、只读状态查询、模型／场景检查、轨迹恢复和 EGL 离线渲染。不提供旧双终端控制兼容入口、外部采集 Trigger、H5 命令发布、Zenoh tracking 或真机控制。

当前保存／限位更新已重建原生模块，并通过 25 项定向回归（命令过渡 13、统一按键 5、随机任务 7）。实际 MuJoCo 窗口验证了有限越限目标饱和但继续录制，连续保存 33／5 帧后直接生成 Home 新任务；真实本地 DLS／Hand2 配合合成跟踪输入连续保存 20／5 帧，验证换场景后旧目标不授权、需重新按 `r`。四个文件均完成校验；这不是真人头显或负载实时性验收。

以下为此前版本的历史验证，旧的成功保存后准备区／回 Home 流程已被上述直接切换取代：

原生构建及 41 项定向回归通过，之后补验了 Home 暂停／继续、场景切换及回退清除失败分支自动快照。真实 TCP 合成输入经 DLS＋Hand2、MuJoCo 和 HDF5 的流程 smoke 保留 **91 帧**、恢复标签 **0／1／3／4**，覆盖起步、局部手指失效、自动冻结、回退、保存和放弃。实际 `spd-quest-teleop` 图形入口经原生循环与终端按键完成采集、回退、保存 **244 帧**、回 Home、切下一任务和正常退出；已观察任务窗口与无任务物体准备窗口。真实 EGL 渲染后的训练读取验证了跨重绑定边界拒绝与旧文件标签来源标记。真实头显跟踪及负载实时性尚未验收；PICO v1 没有显式 tracking-origin epoch，同一连接内小幅坐标重置无法可靠区别于正常运动，不能把这些 smoke 当作实机或硬实时证明。

**未来计划，尚未实现：CONTROL-TYPE 辅助。** 计划研究 approach（接近）、alignment（对齐）、grasp（抓取）、insertion（插入）辅助，并对辅助阶段单独标注，不能混作纯人工演示。当前只有任务／状态／检查点／手指等待提示，没有上述动作策略或辅助标签。

架构与职责见 [docs/architecture.md](docs/architecture.md)。以下旧验收结果仅记录其当时版本，**旧键位、双进程发布和接触检查点策略均已被本页统一流程取代**，不是当前操作说明或当前最终测试数量。

### 服务器渲染适配验证（2026-09-22）

- 本机实际 NVIDIA RTX 5060 Ti、驱动 580.173.02、MuJoCo 3.12.0 的 EGL 预检通过；使用同一卡上的两个 spawn worker，实际渲染六个 episode、18 个源帧，产生 54 组 RGB／实例掩码，原轨迹哈希未变。此结果不是 8×5090 性能测试。
- 输出 JPEG 尺寸／RGB 模式、稳定实例映射、源时钟和相机世界变换通过流式校验；重跑六段全部校验后跳过。改设置、损坏有效范围内的掩码像素、残留 partial、错误 GPU 型号和不存在的 EGL 设备均明确失败。
- provisional 相机默认拒绝且不创建输出；仅显式诊断模式用于上述 GPU 检查。当前视角没有被确认为最终采集／训练相机。
- 独立重复渲染的实例掩码完全相同；本次解码 RGB 最大通道差为 2/255，不将不同原生 GL 上下文的 JPEG 字节一致性作为保证。禁止 mj_step 后实际离线渲染仍通过。
- 注入 worker 硬退出和启动超时后，父进程报告全部未完成任务并回收子进程。未验证服务器八卡吞吐、长时间运行、最终 URDF 相机转换或真实标定；没有新增永久测试目录。

### 精细场景验证（2026-09-22）

- 18 个任务 × seeds 0–2，共 54 个场景：重复生成的 manifest／完整 MJCF 一致，初始接触检查通过，每个独立场景推进 480 步后状态有限、物体根 body 高度均大于 0.70 m。确认动态物体质量与配置一致，固定支架无 free joint，外观层不参与碰撞。
- 六类代表场景分别组合真实 Tianji／Wuji 模型并渲染截图；重复合并不修改源场景。各记录 3 帧 schema-v2 轨迹，移除临时场景 XML 后内嵌模型恢复通过，物体位姿误差小于 1e-12。
- 实际球体落入杯腔／箱体并停在内底上；12 mm 探针可置于杯柄孔内，无接触穿透。字母 R 的实际渲染方向与原图一致，已修正贴图 V 方向；陶瓷盘沿与内凹面在渲染检查中可见。
- 环境包 wheel 包含 104 个运行资产文件；从临时解包安装位置加载瓶子／字母／马克杯场景通过，全部六种瓶子资产和导出哈希一致。`pixi run spd-envs-check --seed-count 3` 和实际 `spd-scene` 无图形截图入口通过。
- 这些是有限种子、短时物理与视觉检查，不代替真实头显操作者的抓取／挂杯／堆叠验收或实物参数标定。该阶段没有新增永久测试目录或离线渲染实现；后续独立渲染适配见上节。

### schema-v2 完整场景轨迹验证（2026-09-22）

- 真实 Fast DDS（隔离 domain 149）→ MuJoCo → 采集服务完成：未授权开始被拒绝；没有新命令时仍采状态；12 帧上限保存为 `complete=true, success=false`，显式保存为 `success=true`，丢弃不影响已保存段。
- 实际 `cups/pyramid` 场景包含 54 个机器人关节和 6 个任务物体；受控初始落体产生真实物体运动。记录 qpos/qvel 与采样时物理状态逐元素相等，tick 间隔恰好为 8。在线 Renderer 被替换为抛错探针，整个采集仍通过。
- 关闭采集并删除临时场景 XML 后，新 Python 进程仅加载内嵌模型恢复 12 帧，机器人位置／速度和物体位置／四元数最大误差均为 0；恢复进程禁止 mj_step 和 Renderer，两个公开校验／恢复 CLI 均通过。
- 漏物理步、中断、队列溢出均保留不完整 partial；模型字节、元数据哈希和 tick 损坏被拒绝。另一真实 MuJoCo 小场景验证左右手区间接触、非任务接触排除、act/mocap/equality 恢复、零物体／零相机及旧数据集／版本冲突拒绝。
- 实际无图形 RosViewerApp 和独立触发器状态查询在 domain 150 启动验证通过。未验证真实头显示范、桌面 Viewer、离线图像或长期采集实时性能；没有新增永久测试目录。

### spd-syz 重构验证（2026-09-21）

以下为旧 schema 的历史验收记录，不代表当前完整场景轨迹契约；旧回归目录与测试命令已移除，当前检查点回归见上节。

- `pixi install --all --locked`、`pixi run ros-build-interfaces` 成功；发布端与本仓库的 `JointCommand.msg` 已逐字节比较一致。
- 当时回归测试 49 项通过。无效碰撞输入回归触发一条 trimesh 数值警告，不影响通过结果。
- 新路径下 `cups/pyramid` 场景已完成无图形物理推进、manifest/状态导出和截图检查，机器人 54 维索引与场景自由关节共存。
- 独立 ROS domain 127 的真实 Fast DDS 测试发布进程驱动 MuJoCo：未授权保持、显式启用、关节物理响应、失鲜后锁存保持及新会话撤权通过。测试发布器不是产品入口，未复制上游遥操作算法。
- 实际无图形控制终端 `e/r/s/d` 完成保存和丢弃，中断保留未标记成功的 partial 文件。保存的示范包含双臂／双手各 2,202 帧、每路 RGB 551 帧，`validate_episode` 和 `replay_episode` 均通过。去除 tmux 后，前台 `spd-sim --headless` 启动、终端输入及 SIGINT 退出已重新验证，无残留订阅进程。
- 三路 1280×720 RGB 已实际渲染并检查；本次未验证桌面 Viewer 交互、真实头显到上游发布端的完整链路、相机标定或录制实时性能。帧数不是采集频率保证。

### 历史：protype 采集流程适配验证（2026-09-22）

- 完整保留测试共 56 项通过；首次采样状态提示修正后，相关采集／录制 8 项回归再次通过。
- 独立 ROS domain 128 的真实 Fast DDS、MuJoCo 和三路相机验证：未授权开始被拒绝；帧数上限 12 自动保存双臂／双手各 12 帧、各路 RGB 3 帧，`success=false`；显式保存段为状态各 57 帧、各路 RGB 15 帧，`success=true`，均通过文件校验。
- 实际独立终端 `r` 启动、`q` 退出后录制继续；另一客户端 `discard` 只删除当前段，已保存段保留。中断仿真后保留非成功 partial，未发布正式文件。
- 采集服务验收使用测试发布源，不覆盖真实头显、桌面 Viewer 操作、相机标定或录制实时性能。未停止用户现有发布端或采集进程。

### 历史：state-only 契约校正（2026-09-22，已由完整场景 schema-v2 取代）

- 移除文件中的命令目标、命令序号／会话等扩展，采样接口不再接收 AppliedCommand。此前验证记录属于校正前的输出，不能作为当前 state-only 文件契约的证明。
- 57 项测试通过；新增真实 MuJoCo 回归，验证没有新 cmd 时仍可采集实际 qpos，并与保留目标明确区分。
- 再次执行真实 DDS → MuJoCo → 采集服务：新文件仅有 `observations/{arms,hands}` 和 `images`，无 commands/actions；状态各 12 帧、每路 RGB 3 帧，校验通过。录制状态与本次测试源发送目标的最大差约 0.0633 rad，未将 cmd 当 state 写入。

### 历史：已被统一流程取代的控制迁移验收（2026-09-22～23）

此前远端控制、独立 R/S 发布、原生三键和单话题双进程阶段的键位、前伸标定、一秒恢复及无接触检查点规则均已废止，不保留其操作命令作为当前指南。

- 早期合成 PICO TCP 经原生 DLS／Hand2、ROS 和仿真保存过 19 帧；独立一秒恢复阶段保存过 200 帧，标签计数为 0=18、1=61、2=60、3=61，独立恢复误差为 0。
- 单话题双进程阶段在隔离 domain 173 保存 6,374 帧，完整性／成功和恢复标注校验通过，机器人 qpos/qvel 恢复误差为 0。
- 原生执行器迁移阶段保存 1,795 帧，schema-v2 校验及独立恢复通过，机器人和物体状态恢复误差为 0；当时修复内嵌 Python 解释器路径并保留步进时 GIL 释放。
- 旧中文任务条、手部虚影与随机任务检查证明的是各自当时的展示／场景行为，不证明当前统一控制、真实头显精度或负载实时性。当前控制证据见本节开头的统一流程 smoke。

### 历史：采集窗口精简验证（2026-09-23，旧保存确认界面已取代）

- 已删除关节曲线、对应历史缓存和 F8/F9 处理，并移除通信统计、关节误差和 operation ID；状态与操作提示以中文合并到顶部任务条右侧，不再绘制左上角独立状态框。
- 实际打开 MuJoCo Viewer 并检查待开始、接入、录制、暂停、保存确认、失鲜和异常截图。顶部任务条与核心状态保留；接入／暂停存在 42 个双手虚影几何，正常录制为 0；异常原因可见，窗口正常关闭。
- 中文双列任务条已在实际 Viewer 的 1280 与 640 像素宽度下验证，覆盖录制、暂停、接入、保存确认、失鲜和非法关节目标；窄窗口自动换行，异常使用醒目文字颜色。MuJoCo 3.12 图像叠加前通过空文本渲染调用初始化 2D 状态，空文本底板由任务条完全覆盖，不恢复旧 HUD。

### 场景对齐与视觉增强验证（2026-09-23）

- 18 个任务 × seeds 0–2 的 54 个机器人组合场景均通过生成、物体子树映射、HOME 初始净空和 96 步物理检查；同 seed 场景／manifest 可复现，观察到全部三种杯／马克杯／盘／箱几何和全部六种瓶子资产。
- seeds 0–2 的 9 个带载抽屉在 ±6 N 外力下完成关闭和拉开，3／3／2 块物体留在对应托盘内；软限位下端点误差小于 0.5 mm。盘子在槽内落稳、杯柄挂稳、瓶子落入箱内均通过真实重力／接触推进。修复抽屉搁板共面接触引起的卡滞，未关闭碰撞。
- `pixi run spd-test` 33 项通过；新增带载抽屉与语义增强隔离回归。实际渲染检查普通木块、彩色字母、鲜明杯色和可见抽屉内腔。
- 含实际抽屉运动的 4 帧轨迹完成三视角 EGL 渲染及独立恢复，最大恢复误差为 0；增强同 seed 重复一致、跨帧／视角参数一致，机器人像素、掩码和状态标签不变，源文件哈希不变。纹理库预览 CLI 通过。
- 以上为受控物理／渲染验证，不是完整遥操作成功率、实物参数标定、相机最终标定或论文原资产等价性证明。

### 历史：单窗口双视角验证（2026-09-23，控制键位已取代）

- 实际 GLFW/MuJoCo 窗口验证仅一个可见窗口、顶部中文提示、左侧自由视角、右侧固定头部视角；两侧均显示暂停双手虚影，1600×900 与 961×601 的绘制／换行通过。
- 真实鼠标输入验证左侧旋转、平移、缩放，右侧拖动／滚轮不改变视角；主仿真状态和训练相机未被观察操作改变。
- 当时图形录制／保存／退出与 headless 启停通过；旧暂停和保存键位已被首页统一键位取代。未验证真人遥操作或负载下 480 Hz 实时性。
