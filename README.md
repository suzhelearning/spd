# SPD Simulation Collection — Tianji + Wuji Hand 2

本项目面向《Pre-training Visual Dexterity in Simulation》的仿真示范采集，目标机器人为双侧 **tianji_arm + wuji-hand2**。ROS 2 入口仅接收外部 `tianji_teleop` 生成的 54-DoF `JointCommand`：外部 ROS 2 → DDS/Zenoh 桥 → DDS → SPD 校验与名称映射 → MuJoCo 执行、可视化和录制；SPD 不再负责该入口的 PICO 输入、IK 或手部重定向。旧 PICO 跟踪运行链保留独立入口。论文原文见 [docs/papers/2608.15917v1.pdf](docs/papers/2608.15917v1.pdf)；不包含实机控制、脚部 IMU、鱼眼相机或 Odin。

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

每条命令打开一个 MuJoCo 窗口；关闭后可启动另一个任务。鼠标操作沿用 MuJoCo，`Esc` / `q` 退出。独立查看器保持机器人 HOME 目标，不启动 ROS、PICO 或自动任务策略。

| 任务 | 初始场景与接触几何 |
|---|---|
| Plate racking | 两只平放盘子、带三道开放插槽的固定盘架 |
| Mug hanging | 一只空心带孔杯柄马克杯、四分支固定挂杯架 |
| Playing Jenga | 18 层交错排列、54 块独立自由积木；中层抽取目标记录在 manifest |
| Cup stacking | 六只真实套叠的空心锥形杯，可拆开并倒置搭成 3–2–1 金字塔 |
| Bottles in bin | 四只自由瓶子、带底和四壁的开口固定收纳箱 |
| Spelling blocks | 八只 40 mm 自由字母方块，六面均有字母，打乱后拼出 `ROBOTICS` |

桌面高度为 **0.75 m**，适配 Tianji HOME 前臂和手掌的碰撞间隙。盘子半径 100 mm、积木 75×25×15 mm、杯高 90 mm、瓶高 180 mm、箱内尺寸 350×250×150 mm；这些是工程选型，不是论文提供的精确尺寸。物体使用真实碰撞、重力和摩擦，不以焊接、禁用接触或允许杯子穿透来维持摆放。随机质量、摩擦、颜色与位置可由 seed 重现。

拼字场景在窗口和终端提示目标单词 `ROBOTICS`；`--seed` 改变字母分配及摆放。目标单词和每个物体对应的字母保存在 `scene_manifest.json` 的 `sampled_values` 中。字母标记仅用于显示，不改变方块质量或碰撞形状；当前不含自动拼字策略或成功评分。

导出带机器人场景、采样参数、最终状态和截图：

```bash
pixi run spd-scene --task cups/pyramid --seed 0 \
  --headless --duration 3 --output data/task_scenes
```

输出为 `data/task_scenes/cups/pyramid/seed_0/{scene.xml,scene_manifest.json,state.json,final.png}`。其他任务目录同理；重复运行同一输出目录、任务和 seed 会更新这些输出，不修改已验证的基础机器人模型。`--duration 0` 导出初始状态；`--screenshot PATH` 指定截图路径。无图形模式默认使用 EGL。

接入既有外部关节控制链路时，在 SPD 启动命令中选任务：

```bash
pixi run spd-teleop-ros --task mugs/hang_mug --seed 0 --attach
```

发布端和桥的启动方式不变，仍需本地 `e` 授权。场景自由关节不改变 54 维机器人命令顺序，录制保存采样场景 manifest。HDF5 观测回放仅驱动关节，**不会自动完成这些任务**。现已检查初始稳定性、盘架承托、杯柄悬挂、套杯/金字塔支撑、积木抽取和瓶子落入箱内；未验收机器人自主抓取、完整任务策略或论文成绩复现。

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

ROS 环境是独立的 `ros-jazzy` Pixi environment，使用 Jazzy/Fast DDS。桥使用官方独立二进制 **zenoh-bridge-ros2dds 1.10.0**，不是旧 tracking Zenoh 协议，也不是 `rmw_zenoh`。Linux x86-64 首次安装（系统需 `curl`、`unzip`、`sha256sum`；此路径不需要 ADB）：

```bash
pixi install
pixi run ros-bridge-install
pixi run ros-build-interfaces
```

安装器从 [Eclipse 固定版本发布目录](https://download.eclipse.org/zenoh/zenoh-plugin-ros2dds/1.10.0/) 下载，并校验已固定的官方 SHA-256；二进制位于 `.pixi/tools/zenoh-bridge-ros2dds/1.10.0/zenoh-bridge-ros2dds`，被现有 `.gitignore` 忽略。无需 Rust 编译或系统级安装。接口构建等价于在 ROS Pixi 环境运行 `colcon build --base-paths packages/tianji-spd-interfaces --build-base .ros/build --install-base .ros/install --merge-install`；发布主机也须构建同一接口并 source 对应 `setup.sh`。

两端默认连接为 **发布侧 domain 120 → Zenoh TCP `127.0.0.1:7447` → SPD domain 121**：

```bash
# 终端 A：仅启动发布侧桥；前台运行，Ctrl-C 只停止此桥
pixi run ros-bridge-publisher

# 终端 B：启动 SPD 侧桥和 MuJoCo 订阅 Viewer；有显示器时省略 --headless
pixi run spd-teleop-ros --attach
# 无图形运行：
# pixi run spd-teleop-ros --headless --output /tmp/spd-episodes

# 仅停止当前项目的 SPD 桥/Viewer 会话，不影响发布端或其他 tmux 会话
pixi run spd-teleop-ros-stop
```

发布侧实际应用由用户在外部 `tianji_teleop` 工作区独立启动。其进程必须使用 `ROS_DOMAIN_ID=120`、`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`、`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`、`ROS_STATIC_PEERS=''`，并 source 接口 overlay。SPD 环境默认 domain 为 121；通用模拟发布进程须在 Pixi 激活之后设置 domain 120，避免发布到 SPD 同域。下述 HDF5 播放器在代码中明确使用 domain 120。SPD 启动脚本绝不启动 `ros_publisher`、PICO、IK 或实机控制，也不会自动重建接口。

独立调试 SPD 桥可运行 `pixi run ros-bridge-spd`（前台 Ctrl-C 停止），不要与一键入口重复启动。跨主机时，在 SPD 主机设 `SPD_ZENOH_LISTEN=tcp/0.0.0.0:7447` 后运行一键入口；发布主机设 `SPD_ZENOH_CONNECT=tcp/<SPD主机IP>:7447` 后启动发布侧桥。仅显式开放该 TCP 端口到可信网络；默认不提供认证或加密。两端 ROS 应用仍分别与本机桥通信，domain 120/121 不可合并，否则 DDS 可绕过桥。`config/ros2dds.json5` 关闭 Zenoh multicast/gossip 自动发现，仅允许关节命令 topic 的 publisher/subscriber 路由，禁止其他 topic、service 和 action；本机 ROS DDS 发现不等同于 Zenoh 自动发现。

SPD 会话使用项目专属 tmux socket `.pixi/spd-ros.tmux.sock`，含 `bridge` / `viewer` 两个窗口；启动输出给出 attach 命令，可用 tmux `Ctrl-b n` 切换窗口查看桥日志。控制终端和 Viewer 均用 `e` 启用/禁用命令应用、`c` 清除控制与授权；终端输入需要回车，Viewer 按键无需回车。清除不重置物理模型。启用前校验新鲜候选和当前目标差，默认门限 `0.15 rad`，可用 `--max-enable-delta-rad` 调整。Viewer 用 `F8` / `F9` 切换前一个／后一个关节的目标和实际位置曲线，避免与 MuJoCo 原生相机、关节、坐标轴快捷键冲突。`r` 开始录制、`s` 保存成功 episode、`d` 丢弃；录制目录可用 `SPD_EPISODE_OUTPUT` 或 `--output` 指定。

启用门限只检查候选命令中 ready 组的每个关节，相对保留的实际目标计算；未 ready 的组保持上次目标。合法的新 session 会撤销已有授权，但保留候选供再次显式启用。每组超过 `100 ms` 未获得新鲜 ready 命令后锁定 hold，须显式重新启用才能恢复，其他仍新鲜的组可继续。物理 tick 消费命令时也会复查 UTC 新鲜度，不把接收时合法等同于应用时仍有效。

HUD 显示接收/有效/拒绝计数、接收频率、会话、序号、目标年龄、ready/hold 和各组最大跟踪误差；曲线橙色为实际应用目标，蓝色为 MuJoCo 实际关节位置，不把命令当作观测。超时后仍有新消息不代表自动恢复；使用 `e` 先禁用再启用。相机首次创建和同步渲染可能阻塞主循环并触发 100 ms 保持，尚不保证录制开启时的实时控制频率；保持门槛不会为渲染而放宽。

消息接口位于 `packages/tianji-spd-interfaces/msg/JointCommand.msg`，ROS 类型为 `tianji_spd_interfaces/msg/JointCommand`，topic 为 `/spd/tianji_wuji2/v1/joint_command`，`schema_version=1`、`robot_config="tianji_wuji2_v1"`，QoS 为 `BEST_EFFORT / KEEP_LAST(1) / VOLATILE`。消息携带 54 个关节名称和弧度目标，由订阅端校验并按名称映射，不直接信任数组顺序；时间戳使用主机 UTC 纳秒，跨主机须同步时钟。schema-v1 episode 的 `observations/commands` 保存实际应用的 54-DoF target、sequence/session、ready/hold mask 和物理应用时间。

ROS Viewer 在已验证模型上按 `config/sim_cameras.yaml` 添加三路真实仿真相机，不修改已生成的模型文件或复制 overview 图像冒充多视角。JPEG/HDF5 和保存/丢弃由后台线程处理；当前相机配置仍标为 provisional，不宣称已校准。录制中的命令扩展与 `docs/schema-v1.md` 的“只存实际状态和 RGB”契约不同，不能用当前校验器通过来宣称符合该原始 schema。

桥配置依据官方 [使用说明](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/tree/1.10.0#usage) 和 [1.10.0 配置定义](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/blob/1.10.0/DEFAULT_CONFIG.json5)；官方明确要求桥两端避免直接 DDS 通信。桥内部使用 CycloneDDS，与此处 Fast DDS 节点通过标准 DDS UDP 互通，不能依赖 Fast DDS 专用共享内存跨桥传输。

桥进程通过 `config/ros2dds-cyclone.xml` 使用 loopback UDP 单播发现和固定范围参与者端口；Linux `lo` 未开启 MULTICAST 标志时也不需要 sudo 修改网卡。该配置只用于桥内的 CycloneDDS，不会把 ROS 节点的 Fast DDS 后端换掉。

### HDF5 观测数据模拟发布

先启动上面的两侧桥和 SPD Viewer，再启动独立播放面板：

```bash
pixi run -e ros-jazzy bash -c \
  'source .ros/install/setup.sh && exec python -m spd_vr.ros_recorded_publisher "$@"' -- \
  /home/summer/下载/20260914_153712_352406_take001.h5 \
  /home/summer/下载/20260914_182335_419850_take003.h5 --loop
```

播放器只读 `schema-v1` 的双臂/双手 `qpos` 和各自时间戳，以双方已有首样本的公共起点开始，到双方仍有数据的公共终点结束；按时间向后取最近样本，不把两个异步数组按行号强行拼接。54 维顺序为左臂、右臂、左手、右手；若存在关节名称或配置元数据，拒绝与此契约冲突的内容。

这些文件没有原始控制命令，故此入口明确把**实测观测作为合成测试目标**，不是恢复原始动作。发布使用新 UUID 会话、递增序号和当前 UTC 时间，原始时间戳只控制播放进度。默认 60 Hz、原速播放；`--speed` 修改文件播放速度，`--loop` 循环文件列表。初始与段间使用明确标注的平滑过渡，持续至少 2 秒、最大关节速率 0.3 rad/s；不夹紧文件中的非法目标。

启动后处于 WAITING，并持续发送 manifest HOME 目标。在 SPD Viewer 按 `e` 授权后，点击面板 **Play**；**Pause** 持续发送最后目标，不使接收端误判断流；**Next** 切换下一份文件并做过渡。终端也接受 `play` / `pause` / `next`。关闭面板停止发布，SPD 按超时规则保持。

播放面板显示文件名、进度、发布数，以及本地读取的 `top` / `left_wrist` 原始 JPEG 预览（最高 10 Hz）。**跨桥传输的只有 JointCommand，RGB 不是订阅端重渲染结果，也不经此命令话题发送。** MuJoCo 窗口检验关节目标接收和物理执行，不自动重建原始图像中的物体、接触或任务场景。

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
