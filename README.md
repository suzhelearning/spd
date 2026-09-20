# SPD Simulation Collection — Tianji + Wuji Hand 2

本项目面向《Pre-training Visual Dexterity in Simulation》的仿真示范采集，目标机器人为双侧 **tianji_arm + wuji-hand2**。ROS 2 订阅入口只接收 54-DoF `JointCommand`：发布侧 ROS 2 → 直连 Fast DDS（两端 domain 120）→ SPD 校验与名称映射 → MuJoCo 执行、可视化和录制。发布侧可选外部 `tianji_teleop`、H5 播放器，或本项目独立的 PICO 实时发布进程；PICO 输入、IK 与手部重定向不进入 SPD 订阅进程。旧 PICO 跟踪运行链保留独立入口。论文原文见 [docs/papers/2608.15917v1.pdf](docs/papers/2608.15917v1.pdf)；不包含实机控制、脚部 IMU、鱼眼相机或 Odin。

## 目录

```text
pixi.toml / pixi.lock       唯一运行环境和命令入口
packages/
  spd-vr/                  仿真运行、机械臂 IK、录制/回放、机器人模型编译
  spd-envs/                六类场景、任务注册、随机重置、环境模型生成
  pico-hand-tracking/       PICO_2 协议接收包、APK、上游诊断工具
  wuji-retargeting/         Wuji 手部重定向依赖及机器人描述子模块
scripts/                   实时链路启动、停止和端到端检查
assets/tianji_wuji2/        机器人 URDF 与网格资产
data/                      本地采集数据；保留已有样本
docs/                      架构说明与论文
```

Python import 名称为 `spd_vr`、`spd_envs`、`pico_hand_tracking`、`wuji_retargeting`；目录使用 package 名称，不按历史设备工作空间划分。`spd-envs` 不依赖遥操作包，可独立生成和验证环境；`spd-vr` 声明依赖 `spd-envs`。

### 环境包

`spd-envs` 管理 `jenga`、`spelling_blocks`、`mugs`、`dishes`、`cups`、`bottles` 六类程序化场景，保留论文 Table 2 的 17 个任务，另加附录 A.4 的 `jenga/playing`。运行 `pixi run spd-envs-check` 检查所有任务的随机重置。程序化资产并不意味着逐项复现论文的视觉与接触参数。

```python
from spd_envs import get_task
from spd_envs.model_scene import write_scene_model

task = get_task("cups/unstack")
scene = task.reset(seed=42)
write_scene_model(
    "packages/spd-vr/generated/unified_plant.xml",
    scene,
    "/tmp/spd-cups/model.xml",
)
```

增加或调整场景时修改 `packages/spd-envs/spd_envs/`，机器人控制仍由 `spd-vr` 负责。场景在启动时选择；不支持运行中切换。

### 六个论文参考场景

五个操作任务参考 Figure 4 / 附录 A.4，拼字积木参考 Figure 2 / Table 2。使用当前 Tianji/Wuji 机器人分别运行：

```bash
pixi run spd-scene --task dishes/rack_dishes --seed 0
pixi run spd-scene --task mugs/hang_mug --seed 0
pixi run spd-scene --task jenga/playing --seed 0
pixi run spd-scene --task cups/pyramid --seed 0
pixi run spd-scene --task bottles/toss_in_bin --seed 0
pixi run spd-scene --task spelling_blocks/spelling --seed 0
```

每次启动先在终端询问桌子近侧边缘距离机器人底座原点多少米，确认位置后才打开 MuJoCo 窗口；关闭后可启动另一个任务。鼠标操作沿用 MuJoCo，`Esc` / `q` 退出。独立查看器保持机器人 HOME 目标，不启动 ROS、PICO 或自动任务策略。

| 任务 | 初始场景与接触几何 |
|---|---|
| Plate racking | 两只平放盘子、带三道开放插槽的固定盘架 |
| Mug hanging | 一只空心带孔杯柄马克杯、四分支固定挂杯架 |
| Playing Jenga | 18 层交错排列、54 块独立自由积木；中层抽取目标记录在 manifest |
| Cup stacking | 六只真实套叠的空心锥形杯，可拆开并倒置搭成 3–2–1 金字塔 |
| Bottles in bin | 四只自由瓶子、带底和四壁的开口固定收纳箱 |
| Spelling blocks | 八只 40 mm 自由字母方块，六面均有字母，打乱后拼出 `ROBOTICS` |

桌面高度为 **0.75 m**，适配 Tianji HOME 前臂和手掌的碰撞间隙。盘子半径 100 mm、积木 75×25×15 mm、杯高 90 mm、瓶高 180 mm、箱内尺寸 350×250×150 mm；这些是工程选型，不是论文提供的精确尺寸。物体使用真实碰撞、重力和摩擦，不以焊接、禁用接触或允许杯子穿透来维持摆放。随机质量、摩擦、颜色与位置可由 seed 重现。

桌子尺寸为 `0.80 × 1.10 × 0.05 m`。输入距离 `d` 表示沿机器人前方 `+X` 从**底座原点到近侧桌沿**的距离，不是桌面中心距离，也不是底座外表面的净空；桌面中心为 `(d + 0.40, 0, 0.725) m`。桌子、任务物体和固定支架一起平移，保留相对摆放、字母分配和物理参数；进入仿真后桌子固定，不增加滑动关节。

交互启动留空回车采用原位置 `d=0.10 m`；该位置与桌面高度处的基座立柱约有 `17.5 mm` 间隙。无效输入会重新询问，`Ctrl+C` / 输入结束可在开窗前取消。可用 `--table-distance 0.25` 显式指定位置并跳过询问；无交互终端时必须指定此参数。接受有限非负数，但输入距离不等于已验证的碰撞净空或机械臂可达范围。实际桌沿、中心、尺寸和平移后的工作区写入 `scene_manifest.json` 的 `table` 字段。

机器人主体结构材质为 **6061 铝合金**（用户提供的实物信息），不表示电机、减速器、轴承和手部接触面均为铝合金。机器人动力学仍使用 URDF 给定的各连杆质量、质心和惯性张量；不以外观/碰撞网格的实心铝体积重算装配体惯性。当前采用刚体仿真，不模拟铝合金弹性或屈服；接触摩擦需结合实际表面处理和接触材料确定，不能仅由 6061 牌号指定。

查看窗口默认将左右机械臂和机械手（含手掌、手指）的外观网格设为 `alpha=0.45`（45% 不透明度）；基座及场景物体保持原样。此显示设置不修改碰撞几何、质量或惯性。

任务物体采用以下刚体材质工程默认值，尚未经过实物标定。质量保留 `0.8～1.2` 倍随机化，摩擦在表中范围采样；惯性由原有几何及采样质量计算。

| 物体 | 材质 | 基准质量 | 滑动摩擦系数范围 |
| --- | --- | --- | --- |
| 叠叠乐积木 / 多米诺 | 木材 | 18.28 g / 32.76 g | 0.40～0.60 |
| 字母积木 | 木材 | 41.60 g | 0.40～0.60 |
| 挂杯 | 陶瓷 | 220 g | 0.25～0.40 |
| 盘碟 | 陶瓷 | 603.19 g | 0.25～0.40 |
| 套杯 | 塑料 | 30 g | 0.20～0.35 |
| 瓶子 | 玻璃 | 250 g | 0.15～0.30 |

木块质量按密度 `650 kg/m³` 与实心几何计算，陶瓷盘按 `2400 kg/m³` 与当前实心圆盘几何计算；杯、瓶使用名义空容器质量，不把玻璃瓶的实心碰撞代理当作实心玻璃计算质量。杯瓶的惯性仍按现有组合几何分配质量近似，不是实测薄壁惯性。指定物体的碰撞优先级为 1，使其摩擦不被桌面或机器人较大的默认值覆盖；同优先级物体之间仍使用 MuJoCo 的最大值合并规则。接触保持 `condim=3`，不新增滚动/扭转摩擦、碎裂或柔性变形。桌面、支架、箱子、机器人以及物体外观和摆放保持原样。材质、基准质量和参数来源记录在各物体清单中，分类摩擦范围记录在 `sampled_values.friction_range_by_class`。

拼字场景在窗口和终端提示目标单词 `ROBOTICS`；`--seed` 改变字母分配及摆放。目标单词和每个物体对应的字母保存在 `scene_manifest.json` 的 `sampled_values` 中。字母标记仅用于显示，不改变方块质量或碰撞形状；当前不含自动拼字策略或成功评分。

导出带机器人场景、采样参数、最终状态和截图：

```bash
pixi run spd-scene --task cups/pyramid --seed 0 \
  --table-distance 0.10 --headless --duration 3 --output data/task_scenes
```

输出为 `data/task_scenes/cups/pyramid/seed_0/{scene.xml,scene_manifest.json,state.json,final.png}`。其他任务目录同理；重复运行同一输出目录、任务和 seed 会更新这些输出，不修改已验证的基础机器人模型。`--duration 0` 导出初始状态；`--screenshot PATH` 指定截图路径。无图形模式默认使用 EGL。

接入既有外部关节控制链路时，在 SPD 启动命令中选任务：

```bash
pixi run spd-teleop-ros --task mugs/hang_mug --seed 0 --attach
```

发布端使用下述同域 Fast DDS 配置，仍需本地 `e` 授权。场景自由关节不改变 54 维机器人命令顺序，录制保存采样场景 manifest。HDF5 观测回放仅驱动关节，**不会自动完成这些任务**。现已检查初始稳定性、盘架承托、杯柄悬挂、套杯/金字塔支撑、积木抽取和瓶子落入箱内；未验收机器人自主抓取、完整任务策略或论文成绩复现。

## 环境与运行

以下命令均在项目根目录运行。支持 Linux x86-64；ADB 是系统前置依赖，图形查看需要可用显示环境。

```bash
# 首次克隆需要机器人描述子模块
git submodule update --init --recursive
pixi install

# 检查输入；头显需已进入手部跟踪 APK
adb devices
pixi run pico2-hand --duration 5

# 实时仿真查看（不是一键论文数据集采集）
pixi run spd-teleop
pixi run spd-teleop-status
pixi run spd-teleop-stop

# 回归测试
pixi run test
```

多设备使用 `PICO_ADB_SERIAL`；端口使用 `PICO2_PORT` / `PICO2_DEVICE_PORT`。启动脚本可用 `bash scripts/start_spd_vr.sh --dry-run` 查看实际进程与资源路径。

### ROS 2 运行链与采集

ROS 环境是独立的 `ros-jazzy` Pixi environment，使用 Jazzy/Fast DDS。当前 JointCommand 路径不安装或启动任何 ROS/DDS 桥；旧非 ROS tracking/Zenoh 入口及其依赖保留。Linux x86-64 首次安装（此 ROS 订阅路径不需要 ADB）：

```bash
pixi install
pixi run ros-build-interfaces
```

接口构建等价于在 ROS Pixi 环境运行 `colcon build --base-paths packages/tianji-spd-interfaces --build-base .ros/build --install-base .ros/install --merge-install`；发布主机也须构建同一接口并 source 对应 `setup.sh`。

两端默认连接为 **发布侧 domain 120 → 直连 Fast DDS → SPD domain 120**，默认仅同机发现：

```bash
# 启动 MuJoCo 订阅 Viewer；发布应用在独立进程中启动
pixi run spd-teleop-ros --attach
# 无图形运行：
# pixi run spd-teleop-ros --headless --output /tmp/spd-episodes

# 仅停止当前项目的 SPD Viewer 会话，不影响发布端或其他 tmux 会话
pixi run spd-teleop-ros-stop
```

使用外部 `tianji_teleop` 时，发布应用由用户在其工作区独立启动。发布器和 Viewer 均须使用 `ROS_DOMAIN_ID=120`、`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`、`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`、`ROS_STATIC_PEERS=''`。这些变量必须在 Pixi 环境激活及 source 接口 overlay **之后**显式 export，避免被环境或 overlay 覆盖。`spd-pico`、`spd-demo` 和 `spd-teleop-ros` 启动器均应用这一同机配置；手动发布示例见下文。`spd-teleop-ros` 只启动订阅侧，不启动发布器、PICO、IK 或实机控制，也不会自动重建接口。

同机 `LOCALHOST` 是默认运行范围，不是跨主机配置。跨主机部署须显式配置两端 DDS 发现范围／静态 peers 或发现服务、网络接口与防火墙，并同步主机 UTC 时钟；仍使用相同 domain 和兼容的 QoS。默认启动器固定使用上述同机配置，不能仅设置外部环境变量就把它当成已配置好的跨主机入口。DDS domain 不是认证或安全边界，应限制到可信网络；本项目不承诺跨主机或负载下的延迟、频率与实时性能。

SPD 会话使用项目专属 tmux socket `.pixi/spd-ros.tmux.sock`，只有 `viewer` 窗口；启动输出给出 attach 命令。控制终端和 Viewer 均用 `e` 启用/禁用命令应用、`c` 清除控制与授权；终端输入需要回车，Viewer 按键无需回车。清除不重置物理模型。启用前校验新鲜候选和当前目标差，默认门限 `0.15 rad`，可用 `--max-enable-delta-rad` 调整。Viewer 用 `F8` / `F9` 切换前一个／后一个关节的目标和实际位置曲线，避免与 MuJoCo 原生相机、关节、坐标轴快捷键冲突。`r` 开始录制、`s` 保存成功 episode、`d` 丢弃；录制目录可用 `SPD_EPISODE_OUTPUT` 或 `--output` 指定。

启用门限只检查候选命令中 ready 组的每个关节，相对保留的实际目标计算；未 ready 的组保持上次目标。合法的新 session 会撤销已有授权，但保留候选供再次显式启用。每组超过 `100 ms` 未获得新鲜 ready 命令后锁定 hold，须显式重新启用才能恢复，其他仍新鲜的组可继续。物理 tick 消费命令时也会复查 UTC 新鲜度，不把接收时合法等同于应用时仍有效。

HUD 显示接收/有效/拒绝计数、接收频率、会话、序号、目标年龄、ready/hold 和各组最大跟踪误差；曲线橙色为实际应用目标，蓝色为 MuJoCo 实际关节位置，不把命令当作观测。超时后仍有新消息不代表自动恢复；使用 `e` 先禁用再启用。相机首次创建和同步渲染可能阻塞主循环并触发 100 ms 保持，尚不保证录制开启时的实时控制频率；保持门槛不会为渲染而放宽。

消息接口位于 `packages/tianji-spd-interfaces/msg/JointCommand.msg`，ROS 类型为 `tianji_spd_interfaces/msg/JointCommand`，topic 为 `/spd/tianji_wuji2/v1/joint_command`，`schema_version=1`、`robot_config="tianji_wuji2_v1"`，QoS 为 `BEST_EFFORT / KEEP_LAST(1) / VOLATILE`。消息携带 54 个关节名称和弧度目标，由订阅端校验并按名称映射，不直接信任数组顺序；时间戳使用主机 UTC 纳秒，跨主机须同步时钟。schema-v1 episode 的 `observations/commands` 保存实际应用的 54-DoF target、sequence/session、ready/hold mask 和物理应用时间。

ROS Viewer 在已验证模型上按 `config/sim_cameras.yaml` 添加三路真实仿真相机，不修改已生成的模型文件或复制 overview 图像冒充多视角。JPEG/HDF5 和保存/丢弃由后台线程处理；当前相机配置仍标为 provisional，不宣称已校准。录制中的命令扩展与 `docs/schema-v1.md` 的“只存实际状态和 RGB”契约不同，不能用当前校验器通过来宣称符合该原始 schema。

应用就绪仅表示进程已完成自身初始化，不代表 DDS 已发现对端、Viewer 已收到新鲜且通过校验的候选，或控制已授权。应结合 Viewer 的接收／有效计数、候选年龄与 ready/hold 状态判断数据链路；只有符合原有新鲜度及目标差门限的候选才能经显式操作启用。PICO 有效跟踪、H5 播放状态与 SPD 授权也各自独立，直连 DDS 不绕过任何控制门。

### PICO 实时发布 → 选择并确认任务场景

在头显中打开 PICO 手部跟踪 APK，启用手部跟踪并授权 USB 调试，然后在电脑图形桌面的项目终端运行：

```bash
pixi run spd-pico
# 多设备时：
# pixi run spd-pico --adb-serial SERIAL
```

此入口从现有注册表列出六类场景的全部任务，按以下顺序启动：

1. 输入任务编号或完整 `SCENE/TASK`；回车选择 `spelling_blocks/spelling`，`q` 取消。
2. 输入桌沿距离，桌子与物体同步定位。
3. 查看最终的**场景、任务、seed、桌沿距离**，输入 `y` 才连接 PICO 并打开两个窗口。回车或 `n` 取消；`Ctrl+C` / 输入结束同样不会启动连接。

可用 `pixi run spd-pico --task mugs/hang_mug --seed 0 --table-distance 0.30` 预选配置，但**仍必须在交互终端最终确认**，非交互启动不会自动放行。只有确认后才检查设备、建立所需 ADB 转发、启动 Viewer 和发布端；确认启动不等于授权机器人运动。场景接入复用原控制路径，不改变物体尺寸、物理参数、标定或控制逻辑，不启动桥。原有 `spd-demo` H5 入口不变，仍使用无桌子场景。控制路径为：

```text
PICO_2 TCP → 发布侧真实双臂 IK + Wuji 手部重定向
    → 54-DoF JointCommand / ROS domain 120
    → 直连 Fast DDS → SPD / ROS domain 120
    → SPD 授权、名称映射与保持 → 本次已确认的 SCENE/TASK
```

操作全部在 **PICO 控制窗口**完成，不再需要切到 MuJoCo 按 `e`：

1. 确认头部和双手跟踪有效、机器人已保持并静止。头朝操作正前方，双手**手指向前、掌心向下**，按 **K 标准掌姿校准**。连续采集至少 10 个稳定样本、覆盖至少 150 ms；头部及双腕相对窗口首帧不超过 1 cm / 0.08 rad。K 固定操作者前／左／上方向，并分别标定左右腕到掌面的刚体偏置，不启动运动。
2. 按 **C 位置对齐 / 预览**。基准取 SPD 新鲜的**实际关节状态 FK**，不是上次发送目标；要求反馈不超过 250 ms、双臂关节速度不超过 0.1 rad/s、实际与保留目标差不超过 0.1 rad。C 保留 K 的朝向标定。移动双手检查前／左／上三个方向及掌面坐标轴；此时机器人目标保持不动。
3. 查看三视图中的目标掌面 T（实线）、实际掌面 A（虚线）和源目标 FK S（点线），以及实际位置／朝向误差。缺少实际反馈或尚未测量 IK 时显示 unavailable，不显示虚假的零误差。确认方向正确且三组就绪后，按 **F 确认并跟随**；SPD 校验当前会话、候选新鲜度和目标差，成功回执到达且本地仍有效才开始跟随。
4. 按 **Space 保持**：立即停止源目标变化并请求 SPD 撤销授权，不跳回 HOME。跟踪失效、SPD 撤权或反馈／授权通道中断后不自动续动；恢复需 **C → F**。设备重连或跟踪时钟回退使参考原点失效，需重新 **K → C → F**。

快捷键只在 PICO 控制窗口获得焦点时生效，按住不连续触发；不是全局快捷键。整个标定、授权、跟随和保持流程不需要切换窗口。MuJoCo 窗口继续显示仿真，原有录制操作保持不变。

启动器为本次会话创建私有本机 Unix socket，传递授权请求、状态确认及物理线程采样的双臂实际位置／速度、保留目标和当前场景 XML；54 维运动目标走直连 ROS/Fast DDS（两端 domain 120），不走该 socket。状态以 50 ms 周期轮询，读取不会授权或移动机器人。请求绑定会话、唯一编号和有效期；取消、重新标定或输入失效后的迟到确认不能启动运动。控制请求在独立线程中等待，不阻塞 IK 或界面。普通 ROS Viewer/H5 演示不改变操作方式，不使用此 PICO 专属控制 socket。生产 PICO 发布器必须提供 `--control-socket`；缺少实际反馈时拒绝启动，不再提供无反馈的手动授权模式。

双臂共享一个 ready 位，任一腕失效会保持双臂；左右手独立失效。手指关键点丢失只撤销对应手的就绪状态，不连带撤销仍有效的手腕；未参与 21 点重定向的掌心/掌骨点失效不影响手指求解。源侧按实际接收时间检查 50 ms 新鲜度，订阅端继续执行原有 100 ms 超时和 0.15 rad 启用门限。IK 目标调度周期 5 ms，ROS 发布目标频率 60 Hz；双臂速度上限为 `min(模型关节限速, 1.5 rad/s)`，手指为 `min(模型关节限速, 6.0 rad/s)`。双臂源限速同时纳入 QP 约束，避免求解后逐关节截断改变末端运动方向。控制按实际经过时间积分，单次最多 50 ms；过期输入仍保持，不补走失联期间轨迹。积压的连续有效帧合并为最新手势，失效、设备时钟回退和控制事件保持顺序。这些是调度/安全配置，不保证负载下始终达到目标频率；手指几何映射与低通滤波未改变。

默认连接本机 TCP `10002`，转发到设备 `10002`。`--host`、`--port`、`--device-port`、`--adb-path`、`--adb-serial`、`--reconnect` 可配置输入。已有转发仅在设备、两端端口完全匹配时复用，退出时保留；新建转发使用 `--no-rebind`，仅清理本次拥有的映射。已有 TCP 输入可用 `--no-adb-forward --host HOST --port PORT`，不操作 ADB。USB 拔插若使转发消失，可关闭后重新运行入口。

对齐完成后等待确认期间，源端继续更新腕部对齐器但保持机器人目标不动。初始对齐仍要求连续稳定帧（每帧不超过 2 cm / 0.15 rad）；运行中改用设备采样时间计算运动速度，超过 5 m/s 或 20 rad/s 才判定为不可信跳变，避免把正常快速运动或 TCP 集中到达误判为失去对齐。这是输入合理性检查，不是机器人输出速度。真正的跳变、无效腕部跟踪和超时仍会撤销就绪状态。

PICO 输入已是 FLU（X 前、Y 左、Z 上）。K 用头部水平前向建立固定旋转 `B`，把操作者前／左／上对应到机器人工作坐标；之后转头不会拖动目标。标准掌面坐标为 X 沿手指、Y 向左、Z 沿手背（掌心向下时 Z 向上）。机器人掌面由 URDF 的腕部和食指／中指／小指 MCP 固定根部定义，掌心参考点取腕到中指根部的中点；不从会随手指弯曲的链接估计朝向。

目标掌位为 `p_actual_palm0 + B × (p_human_palm - p_human_palm0)`，目标掌面朝向为 `B × R_human_palm`。左右人体腕到掌面的旋转／平移偏置分别由 K 标定；最后用机器人腕到掌面的逆刚体变换还原 `l_wrist` / `r_wrist` IK 目标，因此转腕时不会漏掉腕掌杆臂。位置重对齐不会把当前任意机器人朝向重新当作“掌心向下”。手指重定向几何、低通滤波和限速保持不变。

掌面目标不使用旧腕部模式的厘米级静止死区，缓慢、小幅的平移与旋转也保持上述映射；命令平滑由 QP 的速度／加速度约束负责。

双臂 QP 包含关节位置、速度、8 rad/s² 加速度和关节限位制动约束；次级连续性、关节中位与肘部向外偏好不得覆盖主位置／朝向任务。碰撞约束使用实际场景的碰撞几何，覆盖躯干、静态桌面、非相邻自身连杆和另一条机械臂；不把可移动任务物体或未反馈的活动手指当成静态障碍。双臂提议姿态及实际姿态到提议姿态的路径还会联合采样检查；这是离散模型约束，不是连续扫掠或硬件安全认证。不可达／受约束目标保持有界并报告 blocked 和残差，不冒充到位，也不等同于跟踪丢失；真正的无效输入或数值求解失败仍触发保持和重新授权。

临时仿真例外：按用户要求，当前仅额外排除左侧 `Link5_L–Link7_L` 和右侧 `Link5_R–Link7_R` 的碰撞检测，以绕过原始凸包造成的腕部干涉。物理接触与 IK 避碰共同遵守这两项排除；其他碰撞对、手指映射和标定稳定性门槛不变。生成清单的 `collision.temporary_excludes` 单独列出此例外。它也会忽略这两对连杆的真实碰撞，不可作为实机安全保证；修复腕部碰撞几何后需移除生成器中的 `TEMPORARY_WRIST_EXCLUDES` 并重新生成模型。

当前验证边界：合成头手输入驱动真实 MuJoCo 物理仿真，已通过 K/C、目标预览、连续跟随和 Hold；未使用真实头显或实机。测试中的 20 mm 前移、10 mm 上移与 0.1 rad 旋转，实际前移约 19.8～19.9 mm、上移约 7.9～8.1 mm、旋转约 0.1005 rad，同时约有 4.3 mm 侧向漂移。实际掌面绝对稳态残差仍约 45～47 mm、4.1～4.2°；界面分别显示实际误差与 IK 残差，IK 收敛不代表实际到位。本次未调整位置伺服增益或添加重力补偿。上述运动中受保护碰撞对的采样最小实际间隙约 2.42 mm，不能将规划的 5 mm 裕量理解为动力学跟随的硬保证。私有 IPC／真实 ROS 隔离域另行验证了未授权保持、显式启用、Hold、会话拒绝及超时后重新授权。

进程窗口就绪不代表头显已提供有效跟踪；`WAITING / STALE` 时不能启动控制。APK 的安装、授权和只显示手骨架的限制见 [输入包说明](packages/pico-hand-tracking/README.md)。本入口不安装或自动打开 APK，也不向头显回传 MuJoCo 画面。

关闭任一窗口或终端 Ctrl+C 会清理本次进程。日志在 `.pixi/pico-teleop/`；与 `spd-demo` 继续使用共享会话锁互斥，不能同时启动。现有 `r/s/d` 录制操作保持不变，录制实时性限制仍适用。

**“回退”的当前恢复基准**：`.pixi/checkpoints/pre-arm-mapping-20260919T115122Z/` 中的 `workspace.tar.gz` 与 `checkpoint.json`，入口为 `.pixi/checkpoints/current.json`。存档包含本次手臂映射改造前的代码、资产和当时尚未提交的修改；归档 SHA-256 为 `e451cd7b8b4b3eb9fde67d51b716bbb8c24b2b2c4395b7d6496ce04d83ad044c`。用户说“回退”时恢复到此状态，**不再默认回到更早的 `277330f`**。恢复前保全之后新增的录制和无关修改，只恢复此次改动涉及的文件，不执行全目录清理。存档不包含运行进程、窗口或内存中的物理状态。

### HDF5 观测数据模拟发布

#### 双窗口一键演示

在项目目录打开终端，直接运行：

```bash
pixi run spd-demo
```

不再默认加载任何文件。终端先提示输入 H5 文件完整路径，回车确认后才统一启动无任务场景的 MuJoCo 订阅窗口和 H5 原始画面播放窗口。H5 发布器与 Viewer 通过 Fast DDS 在 domain 120 直接传输 JointCommand，不启动桥。留空回车或按 Ctrl+C 取消，不启动任何演示进程。路径支持 `~` 和成对引号。不使用 tmux，也不加载或修改六个任务场景。

先在 MuJoCo 窗口按 `e`，再在播放器点击 **Play**；**Next** 切换文件，默认循环整个列表。WAITING 时可能尚无对应相机帧，播放经过初始过渡后显示原始录像。脚本不会自动授权或开始播放。

也可以在命令中明确指定一个或多个文件，直接使用所指定的文件启动，不再询问：

```bash
pixi run spd-demo "/完整路径/文件一.h5" "/完整路径/文件二.h5"
```

终端保持打开。按 **Ctrl+C** 或关闭任一演示窗口，会关闭本次启动的两个窗口，不停止其他进程。日志保存在 `.pixi/recorded-demo/`。重复启动会被拒绝，并通过共享会话锁与 `spd-pico` 互斥。不要与下面的手动分步启动同时运行，也不要同时运行向同一命令话题发送目标的其他发布器。

#### 手动分步启动

先运行 `pixi run spd-teleop-ros --attach` 启动 SPD Viewer，再在另一终端启动独立播放面板；激活 Pixi 并加载接口 overlay 后显式设置同域 DDS 环境：

```bash
pixi run -e ros-jazzy bash -c \
  'source .ros/install/setup.sh && \
   export ROS_DOMAIN_ID=120 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
     ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_STATIC_PEERS="" && \
   exec python -m spd_vr.ros_recorded_publisher "$@"' -- \
  /home/summer/下载/20260914_153712_352406_take001.h5 \
  /home/summer/下载/20260914_182335_419850_take003.h5 --loop
```

播放器只读 `schema-v1` 的双臂/双手 `qpos` 和各自时间戳，以双方已有首样本的公共起点开始，到双方仍有数据的公共终点结束；按时间向后取最近样本，不把两个异步数组按行号强行拼接。54 维顺序为左臂、右臂、左手、右手；若存在关节名称或配置元数据，拒绝与此契约冲突的内容。

这些文件没有原始控制命令，故此入口明确把**实测观测作为合成测试目标**，不是恢复原始动作。发布使用新 UUID 会话、递增序号和当前 UTC 时间，原始时间戳只控制播放进度。默认 60 Hz、原速播放；`--speed` 修改文件播放速度，`--loop` 循环文件列表。初始与段间使用明确标注的平滑过渡，持续至少 2 秒、最大关节速率 0.3 rad/s；不夹紧文件中的非法目标。

启动后处于 WAITING，并持续发送 manifest HOME 目标。在 SPD Viewer 按 `e` 授权后，点击面板 **Play**；**Pause** 持续发送最后目标，不使接收端误判断流；**Next** 切换下一份文件并做过渡。终端也接受 `play` / `pause` / `next`。关闭面板停止发布，SPD 按超时规则保持。

播放面板显示文件名、进度、发布数，以及本地读取的 `top` / `left_wrist` 原始 JPEG 预览（最高 10 Hz）。**经直连 DDS 命令话题传输的只有 JointCommand，RGB 不是订阅端重渲染结果，也不经此命令话题发送。** MuJoCo 窗口检验关节目标接收和物理执行，不自动重建原始图像中的物体、接触或任务场景。

### 无硬件检查

```bash
pixi run benchmark_sim --duration 1 --headless
pixi run spd-viewer --headless --synthetic --ticks 2
pixi run python -m spd_vr.runtime --output /tmp/spd-smoke --duration 0.05 --mock
pixi run validate_episode /tmp/spd-smoke/episodes/1
pixi run replay_episode /tmp/spd-smoke/episodes/1
```

`--mock` 生成的是明确标记为 synthetic 的测试 episode，不是有效的人类示范。`replay_episode` 命令只校验并报告轨迹；只有程序接口传入 simulator 才执行物理回放。输出目录应使用新的空目录。

模型编译器默认保护已有产物，拒绝覆盖非空目录。验证重新编译时使用新目录，例如 `pixi run spd-model --output /tmp/spd-model-check`；验证后再显式替换正式模型，避免破坏正在运行的会话。

端到端检查使用本地 TCP 跟踪夹具，不使用真实头显：`pixi run spd-teleop-e2e --endpoint tcp/127.0.0.1:18888`。已有会话占用默认端口时应选择空闲端口，不要为了检查停止操作人员的会话。

## 能力边界

当前保留了跟踪接收、机械臂 IK、Wuji 手部重定向、MuJoCo 仿真、场景构建、episode 录制/校验/回放、接触过滤和 30 Hz 对齐模块。模块存在不等于论文流程已经全部接通。

尤其需要区分：

- 当前 PICO APK 仅发送跟踪并显示手骨架，**不显示电脑 MuJoCo 场景**。
- `spd-teleop` 是跟踪桥、求解器和 Viewer 的启动入口，**不能据此宣称已自动录制训练 episode**。
- 人体跟踪是操纵输入；训练数据应保存仿真机器人状态与实际控制目标，而非直接把人体手关节当成机器人动作标签。
- 论文中的六场景、多视角离线渲染、检查点回退和完整采集闭环，必须按实际实现与验证情况逐项验收。

架构与论文对齐范围见 [docs/architecture.md](docs/architecture.md)。PICO APK 安装及输入协议见 [packages/pico-hand-tracking/README.md](packages/pico-hand-tracking/README.md)。
