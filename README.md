# 简介 cmd

## Quest 3S 裸手 teleop → 本项目仿真

Quest 交付文件已移入本项目：

- `apps/quest/quest3s_hand_tracking.apk`：头显安装包。
- `tools/hand_tracking/quest_hand_tracking_receiver.py`：原始 TCP 接收、JSONL 记录和可视化工具。
- `tools/hand_tracking/requirements-visualize.txt`：独立诊断可视化依赖，仿真遥操不需要安装。

Quest 接收协议与现有上游裸手解码器一致：TCP `10002`，小端
`<BBqI>` 帧头，magic `0xAB`、type `0x40`、version `1`，载荷 `1968` 字节，
双手各 26 个 OpenXR 关节；坐标为 FLU（前、左、上），四元数为 `xyzw`。
不做额外轴翻转或左右手交换。`bash/run_quest_hand_sim.sh` 复用上游
`tianji_teleop-ros2` 的标定、DLS/Ruckig、Hand2 和 ROS 发布，不在 SPD 内复制 IK。
上游日志仍可能显示 PICO，这是共用输入实现的名称。

```bash
# 在本项目根目录执行；先在 Quest 开启开发者模式并授权 USB 调试
adb devices -l
adb install -r apps/quest/quest3s_hand_tracking.apk
# 在头显中打开已安装应用，开启手部跟踪并授予应用所需权限

# 可选诊断：确认头、双腕位置和 hands=L1/R1；结束后 Ctrl+C 再启动遥操
pixi run spd-quest-receive --print
# 可选记录原始解码数据：
# pixi run spd-quest-receive --print --save-jsonl /tmp/quest-frames.jsonl

# 终端 A：首次使用先按下文构建上游 spd 环境
pixi run spd-quest-teleop --height-m 1.75 --headless
# 终端 B：
pixi run spd-sim --table-distance 0.2
```

将身高替换为实测值。终端 A `r` 标定（双臂前伸、肩宽、掌心相对，稳定约一秒），
成功后 `s` 开始跟随；终端 B 按下文 SPD 的 `r/s/d` 流程采集。
不要同时运行独立接收诊断和遥操入口，也不要同时运行 PICO 和 Quest 发布器。
默认查找本项目相邻的 `tianji_teleop-ros2`；其他位置设置
`TIANJI_TELEOP_ROOT=/absolute/path/to/tianji_teleop-ros2`。
多设备遥操使用 `ANDROID_SERIAL`；诊断可加 `--adb-serial SERIAL`。
两者默认自动建立 ADB 转发；自定义遥操端口使用 `--port PORT`，
并事先手动执行 `adb forward tcp:PORT tcp:10002`。

诊断 GUI 可在独立 Python 环境安装
`pip install -r tools/hand_tracking/requirements-visualize.txt` 后，
运行 `python tools/hand_tracking/quest_hand_tracking_receiver.py --visualize`；
这不是 MuJoCo 仿真窗口。APK SHA-256：
`4206849228aa6be0c8c16c3ee48240afb142f053820dc970bcd47991687ac22c`。
真实 Quest 跟踪精度、腕部朝向、失跟踪恢复及 USB 稳定性仍需连接头显现场验收；
协议兼容不代表实机端到端验收完成。

适配验证：已通过合成 Quest 帧的 TCP 分片接收、头／双腕／52 关节与上游解码结果逐项对比，
以及右手失跟踪有效位检查。`pixi run --locked spd-quest-teleop --height-m 1.75
--headless --self-test --duration-s 8` 已通过标定、跟随、重新标定和退出，
使用隔离 DDS domain `121`；这不是连接 Quest 或 SPD 采集端的端到端测试，
也未通过实时性认证。若出现 `No module named 'tianji_runtime.hand2'`，
请在上游工作区重新执行 `pixi run --locked -e spd build` 更新安装产物。

## PICO 裸手 teleop → 本项目仿真

准备：头显运行**裸手跟踪 APK（不是手柄 APK）**，USB 连接电脑并授权调试；停止旧 PICO／Manus／外骨骼会话。本流程仅控制仿真，张手、握拳、捏合等识别标签**不会自动启停遥操**。

```bash
# 终端 A：上游独立操作 r 标定 → s 开始，随后持续生成双臂／双手目标
cd /home/current/syz/tianji_teleop-ros2
bash bash/run_pico_hand_sim.sh --height-m 1.75 --headless

# 终端 B：SPD 本地 r 开始采集；默认每段随机任务
cd /home/current/syz/spd-syz
pixi run spd-sim --table-distance 0.2
```

`1.75` 换成操作者实际身高（米）；先聚焦终端 A 按 `r` 标定：面向前方、双臂水平前伸、双手约肩宽、掌心相对，稳定保持约 1 秒。标定成功后按上游 `s`，上游才开始跟随并持续发布目标。SPD 不再向上游发送标定、跟随或暂停请求；上游暂停、退出等管理操作仍在上游执行。

再聚焦 SPD 窗口或终端 B，按 `r` 开始本条数据。SPD **不传 `--task` 就从 18 个任务中随机选择**；成功保存后自动选择下一任务，等待再次按 `r` 开段。暂停、恢复、检查点回退和保存失败不换任务；机器人实际姿态、速度和保留目标延续，旧检查点不跨任务。

窗口顶部显示中文任务名称和目标。每次生成场景时，桌高随机 `0.70–0.80 m`、近侧桌沿距基座随机 `0.10–0.30 m`，不再询问输入；`--table-distance 0.2` 可将桌距固定为 0.2 米。默认目录 `/data/TianjiSim/trajectories`，可用 `--output` 覆盖。两端独立加载环境，关节目标直接通过本机 ROS domain `120` 的 `JointCommand` 传递，不需要控制 socket 或远程控制开关。DDS 只用于可信环境，不是认证或实机安全边界。

轨迹按每段**开始当天的本机日期**分文件夹保存：`/data/TianjiSim/trajectories/YYYYMMDD/episode_<UUID>.h5`。跨午夜的当前段不拆分，下一段自动进入新的日期目录。

可选：`--task mugs/hang_mug` 固定任务，不逐段切换；`--scene cups` 只在杯子任务中随机；`--seed 0` 复现随机任务／场景序列，不指定则每次启动重新随机。随机抽取允许重复任务；每段记录实际任务和场景种子。固定任务未给种子时仍使用 `0`。`--scene hardware_free` 保留无任务场景。

**按键按窗口和状态解释，普通单键即时生效，无需回车：**

| 位置／状态 | `r` | `s` | `d` |
|---|---|---|---|
| 上游 PICO | 标定／重新标定 | 标定成功后启动跟随 | — |
| SPD 待开始 | 开段＋0 号检查点＋1 秒接入 | 无操作 | 无操作 |
| SPD 正常采集 | 更新最近检查点；手接触任务物体时拒绝 | 冻结场景和采集，上游继续 | 无操作，必须先暂停 |
| SPD 已暂停 | 第一次提示保存，第二次确认成功保存 | 不回退，1 秒接入后继续 | 回退最近检查点、裁掉失败分支，再1秒接入并自动继续 |
| SPD 接入／准备／回退等待／保存／中止处理中 | 无效 | 无效 | 无效 |

暂停显示**检查点双手目标虚影**；接入期间切换为**实时上游双手目标虚影**；正常采集隐藏。只画 Wuji2 双手，不画双臂／机身，虚影无碰撞且不写物理状态。

一秒接入从实体当前关节位置出发，对所有 ready 的双臂／双手关节做平滑混合，目标持续跟随上游更新；一秒以主机单调时钟计，从首个实际执行步开始计时。时间到即结束目标混合，不保证实体实际到位，不瞬移 qpos。失鲜、session 改变或接入期间 ready 组改变会撤销授权并保持暂停，即使接入期间按键无效，安全保护仍有效。

MuJoCo 软限位可能让实测关节略微超出命令范围；这时只将接入的**命令起点**投影到合法限位，不改实际 qpos/qvel。上游越限目标仍严格拒绝，不做隐式裁剪。

**一条数据的流程：** 上游 `r → s` 一次准备 → SPD `r` 开段 → 采集中 `r` 存点 → 失误时 `s → d` 回退并自动继续 → 成功后 `s → r → r` 保存 → 等中文任务更新 → SPD `r` 开下一条。没有丢弃按键，失败通过回退重做；“成功”由操作者确认，不是自动评分。保存提示出现后，`s/d` 取消确认并执行各自的恢复／回退动作。

开段自动保存运动前的完整 0 号检查点；接入过程仍记录真实物理状态，并用 `/collection_events/recovery_transition` 标注：`0` 正常，`1` 开段接入，`2` 暂停恢复，`3` 回退恢复。训练可排除非零样本，但不得把跨接入／回退的状态拼成连续人工动作。

保存后两端分别 `Ctrl+C` 退出；SPD 退出不会代替成功保存，也不会停止上游。只点按、不长按，系统自动重复可能触发保存确认；不需要设备路径或输入设备权限。PICO 仍自动检查／补建 ADB 转发，多设备时设置 `ANDROID_SERIAL`。

<details>
<summary>首次使用／依赖或原生代码更新后：先安装与构建</summary>

```bash
# 上游：构建双臂控制器、ROS 消息及独立 Hand2 重定向程序
cd /home/current/syz/tianji_teleop-ros2
pixi install --locked -e spd
pixi run --locked -e spd build

# 本项目：安装仿真环境并构建 ROS 消息和 C++ 执行器
cd /home/current/syz/spd-syz
pixi install --locked
pixi run spd-native-build
```

</details>

---

# SPD Simulation Collection — Tianji + Wuji Hand 2

SPD **负责仿真轨迹采集与离线渲染**：订阅外部 ROS 2 `JointCommand`，在线记录 60 Hz 完整场景物理轨迹及接触信息；离线恢复状态生成 RGB 和实例分割。在线采集不创建相机渲染器、不保存 RGB。SPD 不接收 PICO 原始输入，不计算人体标定、IK 或手部重定向，不提供关节命令发布器，也不控制实机；仅在显式开始／恢复时对收到的合法目标做一秒接入。

上游 `tianji_teleop-ros2` 已定稿：负责 PICO 标定、共享根坐标下的 Franka DLS + Ruckig、Hand2 映射及默认 ROS 关节命令发布。其入口是在上游工作区运行 `bash bash/run_pico_hand_sim.sh --height-m 1.75`。SPD 启动器不会启动或管理上游进程；上游功能定稿不替代本工作区的真实头显端到端验收。

论文参考：[Pre-training Visual Dexterity in Simulation](docs/papers/2608.15917v1.pdf)。程序化场景、相机与数据流程不是论文结果的等价性证明。

## 源码与资源职责

```text
pixi.toml / pixi.lock                 受维护运行环境与命令
setup.py                            Python 安装配置与 CLI 入口
src/
  spd_native/                       C++ ROS 订阅、控制状态机、物理步进与接触采集
  interfaces/                       Python wire 工具、终端键盘与 ROS 消息定义
  simulation/                       Python 模型准备、原生执行器编排、Viewer 与场景查看
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
bash/                                仿真／采集前台入口与独立采集触发器
data/                                采集输出和已有数据；不随代码清理删除
docs/                                架构、数据契约与论文
```

运行时 Python 包直接位于 `src/`，使用 `simulation.*`、`data_collector.*`、`cameras.*`、`interfaces.*`、`description.*` 导入，不保留统一外层包或旧导入兼容层。根目录 `setup.py` 安装这些包，发行包名仍为 `spd`；独立环境包保持 `spd_envs`，依赖方向为 `spd → spd-envs`，环境包不依赖 ROS、PICO 或机器人输入算法，也不硬编码 Tianji 模型路径。

## 安装与启动

在项目根目录操作，支持 Linux x86-64。环境固定 Python 3.12，保留 MuJoCo 3.12，并锁定兼容的 ROS 2 Jazzy / Fast DDS 依赖。图形查看需要可用显示环境；中文任务条使用系统 `fonts-noto-cjk` 的 Noto Sans CJK 字体（缺失时程序给出安装提示），无图形模式不加载字体。SPD 本身不需要 ADB 或头显。在线仿真从本 checkout 读取资源；轨迹内嵌已编译模型和网格／纹理，可脱离原场景 XML 恢复，但必须使用记录时的精确 MuJoCo 版本。不支持将 Python wheel 脱离工作区作为完整在线仿真部署。

```bash
pixi install --locked
pixi run spd-native-build

# 默认逐段随机任务、桌高和桌距；发布端在上游工作区单独启动
pixi run spd-sim

# 固定采集任务；桌高和桌距由场景 seed 决定，不询问输入
pixi run spd-sim --task mugs/hang_mug --seed 0

# 无图形随机任务；此例显式固定桌距，省略则随机
pixi run spd-sim --headless --table-distance 0.2 --output data/headless-episodes

# 无任务场景，不随机选择
pixi run spd-sim --scene hardware_free

# 退出：在运行 SPD 的终端按 Ctrl+C，不停止上游发布器
```

`spd-native-build` 在 `ros-jazzy` Pixi 环境中构建 `tianji_spd_interfaces` 和 `spd_native`，生成 `.ros/install/lib/spd_native/spd_executor` 与 `_spd_native` Python 扩展。原生源码或依赖更新后重新构建；仅运行旧的 `ros-build-interfaces` 不会构建执行器。

在线入口为 C++ 可执行程序，内嵌 Python 复用场景生成、Viewer 和 HDF5 文件生命周期；ROS 订阅、命令校验／授权／失鲜保持、一秒接入、三键状态机、480 Hz 调度、MuJoCo 步进及手–物接触循环在 C++ 中。ROS 回调只更新邮箱，执行线程独占物理变更；步进释放 GIL，使终端输入和后台写入继续运行。这不是完全无 Python 或硬实时执行器，也不保证消除 MuJoCo 接触求解瓶颈。

`pixi run spd-scene` 自动进入原生运行环境并加载 overlay；模型编译、独立场景生成及离线恢复／渲染仍可使用各自 Python 环境。回归入口为 `pixi run spd-test`，需先完成原生构建。旧 Python 订阅／三键状态机实现和 `spd-viewer` console 入口已移除，不提供回退实现。

发布侧须使用相同消息定义并加载对应 ROS 接口 overlay。SPD 启动器不会自动重建接口。`config/collect_sim.yaml` 默认采集根目录为 `/data/TianjiSim/trajectories`，新 episode 自动存入开始当天的 `YYYYMMDD/` 子目录；显式 `--output PATH` 优先于 `SPD_EPISODE_OUTPUT`，两者均未提供时使用配置的 `data_dir`。

SPD 主进程在当前终端前台运行，不使用 tmux。终端 `q`／`Ctrl+C` 或窗口 `q`／`Esc` 退出 SPD，不停止上游。先在窗口／终端 `s → r → r` 成功保存并等待完成再退出；中断不是成功确认，未完成段保留 partial。一次只启动一个采集进程。

上游始终独立执行 `r` 标定、`s` 跟随。SPD 暂停／回退期间，上游继续计算和发布当前目标；SPD 只控制本地物理执行和采集，不让虚影改变物理场景。

### ROS 连接与授权

```text
上游 JointCommand 发布器（独立工作区）
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

54 维顺序为左臂 7、右臂 7、左手 20、右手 20，单位 rad。订阅端校验固定关节名称契约、维度、有限值、ready 组关节限位、session、递增 sequence 和 UTC 新鲜度，随后按 manifest 名称映射到机器人 qpos/actuator 地址。场景 free／有界 slide joints 不占用这 54 维；收到目标不意味着授权运动。

所有启动方式统一使用首页的本地 `r/s/d` 状态机，不再提供 `e/c/g/f` 运动／录制快捷键。关节曲线及其 `F8/F9` 切换已移除，`q`／`Ctrl+C` 保留退出。默认只发布 `/spd/collection/status`，不开放外部 Trigger 控制服务，避免绕过本地确认和一秒接入流程。

`r` 开段、暂停后的 `s` 和 `d` 是显式运动授权操作。它们检查新鲜合法候选、ready 和 session，并从实际 qpos 做一秒混合；不要求先把大目标差人工压到 `0.15 rad`，也不直接跳到该目标。普通执行器 `authorize()` 的旧目标差门仍保留在底层 API，交互入口只使用受控接入。接入期间 ready 集合改变、非法输入、新 session 或失鲜会撤销授权；恢复必须再次显式操作，不能自动续控。

应用输出 `ready` 仅表示初始化完成，不代表发现了发布端、已收到有效候选或已授权。窗口顶部为统一中文任务／状态条：左侧任务名称、目标和虚影说明；右侧状态、帧数、检查点、当前操作提示和存在时的异常原因。原左上角独立 HUD 已移除；两列按窗口宽度自动换行。常见操作与控制异常用中文显示，未识别的底层诊断保留原文以免丢失原因。通信详细统计、关节误差、operation ID 和关节曲线不再显示。双手虚影仅在暂停／回退和接入时显示，正常采集隐藏；精简显示不改变控制、采集或物理行为。

### 单窗口双视角观察

有图形界面的 `pixi run spd-sim` 只打开一个 MuJoCo 渲染窗口：顶部整条中文任务／状态提示，下面左右等宽双视口。左侧保留原始自由视角，左键拖动旋转、右键拖动平移、中键拖动或滚轮缩放，Shift 可切换水平操作；鼠标操作只在左侧场景区域生效，右侧和顶部提示区域不移动相机。

右侧固定为头部第一人称视角：观察点位于 `Base_L`／`Base_R` 中点上方 `0.35 m`，当前模型约为世界坐标 `(0, 0, 1.471) m`，朝机器人前方 `+X` 看并下倾 `35°`，垂直视场角 `70°`。这不是垂直俯视，也不修改训练相机 `top` 的标定。两侧使用同一个状态快照，并同步显示暂停／接入时的双手虚影。

窗口内 `r/s/d/q/Esc` 保留原有控制含义，界面键盘长按重复不用于确认保存；终端长按仍可能产生重复字符，建议点按。默认窗口 `1600×900`，最小 `960×600`；提示自动换行，过长诊断以省略号提示截断，完整信息保留在终端／采集状态。`spd-scene` 独立场景查看仍是单自由视口。

渲染线程独占一个 GLFW 窗口、GL 上下文及模型／数据副本，使用两个真实 MuJoCo 透视视口，不以裁剪单张画面伪造分屏。物理线程只提交有界最新快照，不等待 GPU 绘制；显示可跳过中间快照，采集不丢帧。渲染只更新运动学，不推进物理、不改变原模型训练相机，也不保存观察 RGB。旧的头部观察子进程／第二窗口已移除；`--headless` 不创建渲染线程或窗口。

### 论文 A.1 检查点与失败回退

上游 `r → s`；SPD `r` 开段并自动建立 0 号检查点 → 采集中 `r` 更新检查点 → `s` 暂停 → `d` 回退并自动一秒接入 → 成功后 `s → r → r` 保存。

- 检查点仅属于当前 episode，保存完整 `MjData`、保留目标、tick、采样相位和接触累计；不是从 HDF5 反推控制状态。
- 0 号检查点在任何运动前自动建立，允许保留初始接触。采集中手动 `r` 仍要求双手没有与任务物体的有效接触；拒绝时旧点保留。
- 回退先冻结物理并裁掉检查点后的全部数据和恢复标签，刷盘后恢复完整状态。邮箱清空后最多等待一个新鲜度窗口（100 ms）接收同 session 的新目标，再自动接入，避免把正常 60 Hz 包间隔误判为失败；超时、换 session 或目标无效则保持暂停，需人工 `s` 重试。
- 每段始终有 0 号或更新后的检查点，`d` 不再表示丢弃；失败应回退重做。正常成功保存后随机换任务，新任务等待 `r`，不自动录制。
- 检查点不跨进程持久化。最终 HDF5 只保留选择后的轨迹和 `/collection_events/rewind` 分支边界；失败片段不作为正常演示保留。暂停期间仿真时钟不走，真实单调时钟继续走。

#### 脚踏板直接作为键盘

踏板保持左 `r`、中 `s`、右 `d`，无需重编程、设备路径、读取权限或专用驱动；程序不区分普通键盘与踏板。所有启动方式统一使用本地三键状态机：

```bash
pixi run spd-sim --task cups/pyramid --table-distance 0.10
```

把焦点放到 SPD 控制终端或 Viewer 后直接点按。终端使用 cbreak 单键输入，保留 Ctrl+C；关闭、EOF 或启动失败时恢复原终端设置。原 evdev 模块、设备参数和长按逻辑已经删除，不再读取或占用 `/dev/input`，无需执行设备 ACL 配置。

**只点按，不长按。** 普通终端没有可靠松开事件；自动重复可能确认保存或在不同状态产生新的操作。接入期间收到的 `r/s/d` 丢弃，不在结束后执行；程序不修改系统键盘设置，也不检测设备拔出。

## 任务场景与模型

环境包管理 `jenga`、`spelling_blocks`、`mugs`、`dishes`、`cups`、`bottles` 六类场景，保留论文 Table 2 的 17 个任务和附录 A.4 的 `jenga/playing`。独立场景查看保持机器人 HOME 目标，不启动 ROS 或发布器：

在线 `spd-sim` 不指定 `--task` 时按 episode 随机选择；指定 `--task` 时固定。选择范围与任务中文名称／目标统一由注册表维护，任务 ID 不变。任务切换发生在成功保存完成后，不在暂停或回退中重建场景；新场景保留机器人状态并清除授权，物体重新生成，等待本地 `r`。每段 metadata 包含 `task_title_zh`、`task_goal_zh`、实际 `scene/task/seed`。

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

每次生成场景时直接均匀采样桌面上表面高度 `h ∈ [0.70, 0.80] m` 和近侧桌沿距离 `d ∈ [0.10, 0.30] m`，生成后固定。桌板尺寸 `0.80 × 1.10 × 0.05 m`，中心为 `(d + 0.40, 0, h - 0.025) m`；距离沿机器人前方 `+X` 从底座原点测量。桌上物体与固定支架同步调整，桌腿随高度伸缩、脚垫保持落地。`--table-distance` 仍可显式覆盖桌距，省略时无需交互，headless 同样适用。默认随机任务换场景会重新采样；相同 seed 可复现，暂停／恢复／回退不重采样。随机范围不构成全姿态可达或避碰保证。

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

模型现有临时碰撞例外为左侧 `Link5_L–Link7_L` 和右侧 `Link5_R–Link7_R`，记录在 `collision.temporary_excludes`。它们绕过原始凸包腕部干涉，也会忽略这两对连杆的真实碰撞；不是硬件安全保证。限位、分组 hold 和目标差门限同样不构成实机安全认证，SPD 不承担上游 IK/轨迹规划或避碰。

模型编译默认拒绝覆盖非空产物目录，检查新模型应使用新目录：

```bash
pixi run spd-model --output /tmp/spd-model-check
pixi run spd-envs-check
```

正式资源在 `src/tianji_wuji2/tianji_wuji2/{assets,generated}`。不要在采集会话中替换模型；更改资产后重新编译、验证再使用，不混合不同机器人配置的数据。

## 配置化采集与状态查看

`pixi run spd-collect` 与 `spd-sim` 使用同一仿真／录制入口，二选一运行，均使用首页的三键逻辑。上游和 SPD 分别操作，SPD 不请求上游标定、暂停或跟随。

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
- 到达正数上限自动结束、校验并保存为 **`success=false`**，不会自动开启下一段；默认 `0` 不限帧数。只有操作者暂停后 `r → r` 确认保存才标记成功。
- 每段记录实际生效的配置与配置文件路径；采样直接读取当前物理状态，不等待新的 ROS cmd，也不补写录制前缓存。

默认仅发布 `/spd/collection/status`（`std_msgs/msg/String` JSON，可靠、transient-local），包含状态、帧数、路径、`physics_paused`、`checkpoint_frames` 等。`checkpoint_frames=0` 是合法起始检查点，不代表无检查点。底层采集类保留管理接口供专用集成，但 `spd-sim` 不开放外部 Trigger 控制；升级后须重启采集进程。

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

数据契约见 [docs/schema-v2.md](docs/schema-v2.md)。在线无接触检查点、暂停与失败回退已实现；超过 10 秒无接触裁剪和 30 Hz 训练样本构建仍未实现，属于后续数据处理。离线渲染生成所有保留源帧的图像，不改变采样时间网格。旧 `align_30hz`、`filter_contacts` 入口依赖已废弃契约，已移除；不能从 state-only 文件恢复未记录的原始命令。

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

当前是 NumPy CPU 增强接口，不是 CUDA／Torch 增强内核或完整策略训练器。纹理坐标共享于归一化图像空间，不宣称三维表面投影一致。读取器每次读完关闭 HDF5 句柄，适用于 dataloader worker；检查源文件身份和选中帧关联，拒绝 partial、未知实例和跨回退边界序列。完整图像校验和／恢复校验仍由渲染器负责。临时相机标定状态不会因增强而升级为正式标定。

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

正式运行入口包括订阅仿真、本地三键采集及只读采集状态查询、模型编译、场景查看／检查、轨迹恢复和 EGL 离线渲染。当前仿真入口不开放外部采集 Trigger 服务；不存在 SPD 内的 PICO 启动、H5 命令发布、Zenoh tracking 或遥操作兼容入口。

真实头显到上游再到 SPD 的端到端采集、硬件安全、跨主机网络、录制负载下的实时性能及论文数据等价性必须分别验收，不能以进程启动或模块存在替代。上游发布契约已定稿；本次 SPD 验证范围见下文，不宣称已完成真实头显联调。架构与职责见 [docs/architecture.md](docs/architecture.md)。

### 历史：普通键盘三键采集验证（2026-09-22）

本节及下节记录独立采集方案确定前的操作；旧跳过按键、对齐门限和外部 Trigger 控制不属于当前入口。当前操作以首页本地 R/S/D 流程为准，独立采集的既有验收记录见文末。

- 真实 PTY 向 ControlTerminal 写入无换行 `e/g/r/s/d/f/q`，驱动真实 ROS／MuJoCo 采集：首次无检查点 `d` 只提示，其他操作取消确认，第二次 `d` 才跳过；检查点回退恢复原物理状态，中键拒绝未对齐目标并在对齐后授权续采。
- 生成的 7 帧 HDF5 通过独立恢复，机器人和物体状态最大误差均为 0；PTY 原有终端属性在关闭后恢复。8 项当前回归通过，包括即时单键输入、EOF、线程启动失败和终端恢复。
- 实际 `spd-sim --headless` 与交互式 `spd-collect-trigger` 使用无换行按键完成启动、两次确认跳过、检查点及本地暂停／回退／恢复／保存；最终 838 帧文件独立恢复误差为 0，两个终端按 `q` 正常退出。未验证实体踩踏、桌面焦点或长按自动重复行为；没有设备权限依赖。

### 历史：检查点与失败回退验证（2026-09-22）

- 真实 Fast DDS 服务、MuJoCo `cups/pyramid` 场景和 HDF5 写入验证：暂停期间新命令不改变物理状态或帧数；无授权恢复被拒绝；实际手–杯接触拒绝替换已有检查点。
- 失败分支后连续两次回退，再授权续采并保存；10 帧完整文件通过独立恢复，机器人位置／速度和物体位姿最大误差均为 0；回退事件保留、tick 间隔为 8、主机时间严格递增。跳过仅删除当前段，暂停退出保留不完整文件。
- 实际 `spd-sim --headless` 终端 `e/r/u/p/k` 与独立 `spd-collect-trigger --command checkpoint/revert/save` 完成操作；917 帧回退到 441 帧，保存的 441 帧逐帧恢复误差为 0。
- `pixi run python -m unittest discover -s tests -v` 两项物理回归通过：完整检查点继续积分结果逐元素一致、当前接触检查不修改在线状态、重复裁剪保留前缀并更新分支边界。
- 未验证桌面 Viewer 的实际按键／HUD、实体脚踏板或真实 PICO 上游重新对齐；未扩展离线渲染、无接触裁剪或训练样本生成。

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

### state-only 契约校正（2026-09-22）

- 移除文件中的命令目标、命令序号／会话等扩展，采样接口不再接收 AppliedCommand。此前验证记录属于校正前的输出，不能作为当前 state-only 文件契约的证明。
- 57 项测试通过；新增真实 MuJoCo 回归，验证没有新 cmd 时仍可采集实际 qpos，并与保留目标明确区分。
- 再次执行真实 DDS → MuJoCo → 采集服务：新文件仅有 `observations/{arms,hands}` 和 `images`，无 commands/actions；状态各 12 帧、每路 RGB 3 帧，校验通过。录制状态与本次测试源发送目标的最大差约 0.0633 rad，未将 cmd 当 state 写入。

### 历史：统一 R/S/D 控制通道迁移验证（2026-09-22）

- 当时控制曾切换到 ROS `PicoControl`／`PicoControlStatus`。当前确认方案已移除 SPD 远端控制客户端：上游独立 R/S，SPD 只订阅 JointCommand、本地执行三键状态机。以下记录仅作历史，当前验收见后续章节。
- 修正 SPD 双手启动目标：关节范围中点与上游零位不一致，原先默认 `0.15 rad` 门限拒绝接入。现使用同一零位并校验范围，不放宽授权门限。
- 上游 `test_control_server` 与 `test_spd_publisher` 共 16 项定向测试通过；发布器单测使用独立临时锁目录，不抢占正在运行的 domain `121` 自测。
- 键盘适配前，隔离 ROS domain `122` 的合成 PICO TCP 输入经真实 DLS／Hand2、当时的本地控制通道／DDS 与无图形 `RosViewerApp` 跑通标定、采集、检查点、暂停恢复、回退、保存和丢弃。保存 19 帧轨迹，`success=true`、`complete=true`、文件校验通过，包含 1 个回退记录。该记录不作为当前 ROS 控制或键盘交互的验收依据。
- 当前入口只使用普通键盘，不读取或独占设备、不检测长按，暂停后 `r → r` 确认保存。真实头显、桌面 Viewer 焦点及实时性能仍需现场验收。
- 普通键盘适配后、此次 ROS 控制迁移前，加载本地 ROS 接口 overlay 的 13 项回归通过；真实 PTY 单键输入经现有终端和应用分发，验证暂停、保存确认取消、再次 `r → r` 保存成功及 HDF5 校验。当时的无图形三键入口无需设备参数，`r/s` 无需回车即响应，未连接上游时保持未授权，`q` 正常退出（退出码 0）。该历史记录不是当前入口的验收结果；真实头显联调仍需现场验证。

### 历史：随机任务与中文任务条验证（2026-09-22）

以下联调记录包含旧远端控制和丢弃流程，不能作为当前独立三键入口的验收；场景／任务 API 保持不变，当前只在保存完成后切换任务。

- 不传 `--task` 时按段随机选择；指定 `--task` 保持固定，`--scene` 可缩小随机范围，显式 `--seed` 可复现序列。实际无 `--task/--seed` 的启动命令选中 `cups/unstack` 并打印中文名称／目标；固定任务、场景过滤及种子复现另经运行验证。
- 19 项回归通过：覆盖保存／丢弃后换场景、暂停和保存失败不换场景、旧输入不授权新场景、不同模型维度间机器人状态连续、新物体保持初始状态、无效状态迁移拒绝。
- 合成 PICO TCP 输入经真实 DLS／Hand2、ROS 与无图形仿真，完成“悬挂马克杯 → 保存 → 叠放餐盘 → 确认丢弃 → 多米诺骨牌”。两次换任务均保留机器人关节位置、速度和目标，并保持未授权；首段 4 帧轨迹通过文件校验。
- 在隔离 X 显示器上运行真实 MuJoCo Viewer，截图确认中文任务名称／目标位于窗口顶部，640 像素窄窗口下长目标正确换行。该软件渲染环境的端到端图形联调触发过失鲜保护，因此原生连续换任务验证使用无图形模式；没有放宽新鲜度或授权门限。真实头显及实际 GPU 图形实时性仍需现场验收。

### 独立 R/S、零号检查点与一秒恢复验收（2026-09-22）

- 上游 R 标定后保持 ready，显式 S 后才跟随；124 项裸手包回归通过（含11项定向生命周期回归）。原生 Viewer/worker 已重建；对齐当前上游源码时修复了两处阻止编译的表达式／字段引用错误，没有修改既有 YAML 运动参数。
- SPD 当前 29 项回归通过，覆盖0号检查点、实际状态连续性、过渡逐 tick 更新、ready／session／失鲜撤权、60 Hz 包间隔下的自动回退接入、软限位实测超调的合法命令起点、恢复标签裁剪及写盘失败保护。
- 实际原生 DLS/Hand2＋60 Hz 合成 PICO TCP＋ROS domain122＋无图形 SPD 经 PTY 按键跑通：r 开段、0号检查点、接入期间忽略 r/s/d、s 暂停期间上游继续发布、s 一秒恢复、r 更新检查点、暂停 d 自动回退接入、r→r 成功保存。
- 保存 200 帧，恢复标签计数为 0=18、1=61、2=60、3=61；文件校验与独立恢复通过，机器人位置／速度最大恢复误差为0。失败后缀已裁掉，恢复标签与轨迹行数一致；这是合成输入验证，不是实际头显精度／实时性验收。
- 真实 MuJoCo 窗口截图验证检查点／实时目标两种蓝色半透明手部虚影以及正常跟随时隐藏；只有42个左右 wrist 子树视觉几何，机器人 qpos/qvel/ctrl 和模型外观数组未被虚影计算修改。中文任务条显示“检查点目标／实时目标（1秒接入）”。

### 独立采集单话题收敛验收（2026-09-23）

- 两端只保留既有 `JointCommand` 跨项目数据链；控制 socket、远程控制客户端和试迁移的
  `PicoControl`／`PicoControlStatus` 已移除，生成绑定清理后分别重建，消息原始指纹不变。
- 本次 SPD 独立三键、任务生命周期、接入过渡定向回归分别 **4／1／10 项通过**；
  上游裸手包回归 **113 项通过**。这些是本次范围，不反写前面的历史测试数量。
- 隔离 domain **173**，实际两个前台程序经合成 PICO TCP、真实 DLS／Hand2 和直接 DDS
  跑通上游 R→S、SPD R 开段／检查点、S 暂停、D 回退自动恢复、S→R→R 成功保存。
  SPD 暂停时上游仍为 TELEOP；DDS 目标话题为一个发布端、一个订阅端，无跨项目控制端点。
- `hardware_free` 场景保存 **6374 帧**，`validate_episode` 和 `replay_episode` 通过，
  `complete=true`、`success=true`、恢复标注有效，机器人 qpos/qvel 最大恢复误差为 **0**。
  两端 Q 退出码均为 0，临时测试源和数据已清理。
- 本次没有真实头显、任务物体交互、图形虚影或 GPU 吞吐验收，不把单话题结构当作端到端低延迟证明。

### ROS 2 C++ 在线执行迁移验证（2026-09-23）

- `pixi run spd-native-build` 构建原生可执行程序与扩展；`pixi run spd-test` 的 29 项回归通过，包括一秒接入、失鲜／换 session 撤权、真实 DDS、三键回退、接触检查、完整检查点和跨场景机器人状态保留。
- 实际 `spd_executor` 进程在 `cups/pyramid` seed 0 下接收独立 DDS 发布端，通过真实终端完成开段、检查点、暂停、回退自动接入及 `r→r` 成功保存，`q` 退出码为 0；1,795 帧文件通过 schema-v2 校验，保留恢复标签。
- 默认 Python 环境的 `replay_episode` 独立恢复上述全部帧，机器人位置／速度与物体位姿最大误差均为 0；无需原生执行器参与离线恢复。
- 修复内嵌 Python 的 `sys.executable` 指向，使 GLFW 等库启动 Python 探测子进程时使用实际解释器，而非递归启动执行器。物理步进释放 GIL，后台文件写入与终端输入正常。
- 本次未验证真实头显、主动手–物操作或图形 Viewer；迁移不构成 Jenga 已达到 480 Hz 墙钟实时的证明。

### 采集窗口精简验证（2026-09-23）

- 已删除关节曲线、对应历史缓存和 F8/F9 处理，并移除通信统计、关节误差和 operation ID；状态与操作提示以中文合并到顶部任务条右侧，不再绘制左上角独立状态框。
- 实际打开 MuJoCo Viewer 并检查待开始、接入、录制、暂停、保存确认、失鲜和异常截图。顶部任务条与核心状态保留；接入／暂停存在 42 个双手虚影几何，正常录制为 0；异常原因可见，窗口正常关闭。
- 中文双列任务条已在实际 Viewer 的 1280 与 640 像素宽度下验证，覆盖录制、暂停、接入、保存确认、失鲜和非法关节目标；窄窗口自动换行，异常使用醒目文字颜色。MuJoCo 3.12 图像叠加前通过空文本渲染调用初始化 2D 状态，空文本底板由任务条完全覆盖，不恢复旧 HUD。

### 场景对齐与视觉增强验证（2026-09-23）

- 18 个任务 × seeds 0–2 的 54 个机器人组合场景均通过生成、物体子树映射、HOME 初始净空和 96 步物理检查；同 seed 场景／manifest 可复现，观察到全部三种杯／马克杯／盘／箱几何和全部六种瓶子资产。
- seeds 0–2 的 9 个带载抽屉在 ±6 N 外力下完成关闭和拉开，3／3／2 块物体留在对应托盘内；软限位下端点误差小于 0.5 mm。盘子在槽内落稳、杯柄挂稳、瓶子落入箱内均通过真实重力／接触推进。修复抽屉搁板共面接触引起的卡滞，未关闭碰撞。
- `pixi run spd-test` 33 项通过；新增带载抽屉与语义增强隔离回归。实际渲染检查普通木块、彩色字母、鲜明杯色和可见抽屉内腔。
- 含实际抽屉运动的 4 帧轨迹完成三视角 EGL 渲染及独立恢复，最大恢复误差为 0；增强同 seed 重复一致、跨帧／视角参数一致，机器人像素、掩码和状态标签不变，源文件哈希不变。纹理库预览 CLI 通过。
- 以上为受控物理／渲染验证，不是完整遥操作成功率、实物参数标定、相机最终标定或论文原资产等价性证明。

### 单窗口双视角验证（2026-09-23）

- 实际 GLFW/MuJoCo 窗口验证：仅一个可见窗口，顶部中文提示，左侧自由视角、右侧固定头部视角；两侧均显示暂停双手虚影。1600×900 与奇数尺寸 961×601 的绘制／换行通过。
- 真实鼠标输入验证左侧旋转、平移、缩放；右侧拖动／滚轮后画面不变。鼠标操作前后主仿真完整积分状态及训练相机位置／姿态逐元素一致。
- 图形窗口 r 开段、s 暂停、r→r 成功保存并通过轨迹校验，窗口聚焦后 q 正常退出。原生入口启动／退出和 headless 场景推进通过。本次没有真人遥操作或负载下 480 Hz 实时性验收。
