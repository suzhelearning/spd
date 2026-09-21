# SPD Simulation Collection — Tianji + Wuji Hand 2

SPD **只负责仿真数据采集**：订阅外部 ROS 2 `JointCommand`，在 MuJoCo 中推进 Tianji 双臂 + Wuji Hand 2 双手与任务物体的真实物理状态，生成仿真相机图像并录制 HDF5。SPD 不接收 PICO 原始输入，不做标定、IK 或手部重定向，不提供命令发布器，也不控制实机。

上游 `tianji_teleop-ros2` 已定稿：负责 PICO 标定、共享根坐标下的 Franka DLS + Ruckig、Hand2 映射及默认 ROS 关节命令发布。其入口是在上游工作区运行 `bash bash/run_pico_hand_sim.sh --height-m 1.75`。SPD 启动器不会启动或管理上游进程；上游功能定稿不替代本工作区的真实头显端到端验收。

论文参考：[Pre-training Visual Dexterity in Simulation](docs/papers/2608.15917v1.pdf)。程序化场景、相机与数据流程不是论文结果的等价性证明。

## 源码与资源职责

```text
pixi.toml / pixi.lock                 受维护运行环境与命令
src/
  spd/                               spd-vr Python 包与测试
    spd_vr/
      interfaces/                    JointCommand 校验、邮箱、会话与授权
      simulation/                    MuJoCo 物理执行、ROS Viewer、场景查看
      cameras/                       仿真多视角相机
      data_collector/                episode、HDF5、校验与数据检查
      description/                   manifest、资源定位、机器人模型编译
    test/
  environments/spd_envs/             独立环境包：任务、随机重置、场景生成
  description/tianji_wuji2/
    assets/                          原始 URDF、网格与碰撞资产
    generated/                       编译后的模型与 manifest
  interfaces/tianji_spd_interfaces/   ROS 2 JointCommand 消息包
config/                              相机等运行配置
bash/                                SPD 订阅仿真的前台启动入口
data/                                采集输出和已有数据；不随代码清理删除
docs/                                架构、数据契约与论文
```

Python import 名称保留 `spd_vr`、`spd_envs`；运行时按职责分组，不保留旧模块转发层。`spd-vr → spd-envs`，环境包不依赖 ROS、PICO 或机器人输入算法，也不硬编码 Tianji 模型路径。

## 安装与启动

在项目根目录操作，支持 Linux x86-64。环境固定 Python 3.12，保留 MuJoCo 3.12，并锁定兼容的 ROS 2 Jazzy / Fast DDS 依赖。图形查看需要可用显示环境；SPD 本身不需要 ADB 或头显。模型和配置从本 checkout 的唯一资源目录读取，不支持将 Python wheel 脱离工作区单独部署。

```bash
pixi install --locked
pixi run ros-build-interfaces

# 在当前终端前台启动订阅器；发布端在上游工作区单独启动
pixi run spd-sim

# 选择采集任务；未给桌距时先在终端询问，确认后才开窗
pixi run spd-sim --task mugs/hang_mug --seed 0

# 无图形订阅；当前终端仍可控制授权和录制
pixi run spd-sim --headless --output data/headless-episodes

# 退出：在运行 SPD 的终端按 Ctrl+C，不停止上游发布器
```

`ros-build-interfaces` 在 `ros-jazzy` Pixi 环境中执行：

```bash
colcon build --base-paths src/interfaces/tianji_spd_interfaces \
  --build-base .ros/build --install-base .ros/install --merge-install
```

发布侧须使用相同消息定义并加载对应 ROS 接口 overlay。SPD 启动器不会自动重建接口。默认录制目录为 `data/episodes`，可通过 `SPD_EPISODE_OUTPUT` 或 `--output PATH` 指定。

SPD 直接在当前终端前台运行，不使用 tmux，不后台启动，也不提供 `--attach` 或独立停止命令。`Ctrl+C` 退出当前 SPD；Viewer 中 `q` / `Esc` 也可退出。需要成功保存时先按 `s` 并等保存完成，再退出；中断不是成功确认，未完成段保留为 partial。一次只启动一个采集进程，避免多个实例同时写入同一个输出目录。

### 与定稿发布端配合

在 `/home/current/syz/tianji_teleop-ros2` 的独立终端启动：

```bash
bash bash/run_pico_hand_sim.sh --height-m 1.75
# 只关闭上游辅助 Viewer，仍生成并发布 ROS 关节目标：
bash bash/run_pico_hand_sim.sh --height-m 1.75 --headless
```

两条命令二选一，身高填写操作者实测值。发布最多 60 Hz 的双臂＋双手 54 维绝对关节目标，单位 rad；原生 Viewer 不是命令数据来源。C、跟随、失鲜及 P／H／Q 制动和回程的 readiness／session 语义全部由上游维护，SPD 不重做这些逻辑，也不根据发布端窗口关闭推断仿真已回程。

随后在 SPD 工作区启动 `pixi run spd-sim --task cups/pyramid --seed 0`。上游提供新鲜且满足接收端启用目标差门限的候选后，在 SPD 按 `e` 显式启用，再按 `r` 开始采集。消息到达、上游进入跟随和 SPD 本地授权是不同状态；新 session 或失鲜后的恢复仍遵循下述接收端规则。

### ROS 连接与授权

```text
上游 JointCommand 发布器（独立工作区）
    → Fast DDS / ROS domain 120
    → SPD 校验与最新目标邮箱
    → 本地显式授权 / 分组 hold / 名称映射
    → MuJoCo position actuators 与物理积分
    → 实际 qpos、多视角 RGB、实际应用命令 → HDF5
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

54 维顺序为左臂 7、右臂 7、左手 20、右手 20，单位 rad。订阅端校验固定关节名称契约、维度、有限值、ready 组关节限位、session、递增 sequence 和 UTC 新鲜度，随后按 manifest 名称映射到机器人 qpos/actuator 地址。场景 free joints 不占用这 54 维；收到目标不意味着授权运动。

| 操作 | 作用 |
|---|---|
| `e` | 显式启用／禁用命令应用；启用须有新鲜候选并通过目标差门限 |
| `c` | 清除候选与授权，保持已有目标；不回 HOME、不重置物理场景 |
| `F8` / `F9` | Viewer 切换所选关节的实际应用目标与实际 qpos 曲线 |
| `r` | 已启用控制后准备新 episode；准备完成后才记录 |
| `s` | 停止接收本段数据，后台校验并保存为成功 episode |
| `d` | 丢弃当前未完成段，不删除以前保存的数据 |

Viewer 按键不需要回车；控制终端输入需要回车。运动授权与录制独立，启动订阅器或按 `e` 不会自动录制。

启用检查只针对 ready 组，相对当前保留目标计算，默认最大差值 `0.15 rad`（`--max-enable-delta-rad` 可调整）。双臂共用 ready 位，左右手各自独立；未 ready 组保持已有目标。每组超过 **100 ms** 未获得新鲜 ready 命令后锁存 hold，仍新鲜的其他组可继续。消息恢复不自动解除 hold，需要显式禁用后重新启用。合法新 session 撤销原有授权；物理 tick 消费目标时再次检查年龄，不把接收时合法等同于应用时仍有效。

应用输出 `ready` 仅表示初始化完成，不代表发现了发布端、已收到有效候选或已授权。查看 HUD 的接收／有效／拒绝计数、候选年龄、session、ready/hold 和跟踪误差。曲线的实际应用目标与 MuJoCo 实际 qpos 是两条不同数据，目标不是观测。

## 任务场景与模型

环境包管理 `jenga`、`spelling_blocks`、`mugs`、`dishes`、`cups`、`bottles` 六类场景，保留论文 Table 2 的 17 个任务和附录 A.4 的 `jenga/playing`。独立场景查看保持机器人 HOME 目标，不启动 ROS 或发布器：

```bash
pixi run spd-scene --task dishes/rack_dishes --seed 0
pixi run spd-scene --task mugs/hang_mug --seed 0
pixi run spd-scene --task jenga/playing --seed 0
pixi run spd-scene --task cups/pyramid --seed 0
pixi run spd-scene --task bottles/toss_in_bin --seed 0
pixi run spd-scene --task spelling_blocks/spelling --seed 0

# 导出物理场景、随机参数、状态和截图
pixi run spd-scene --task cups/pyramid --seed 0 \
  --table-distance 0.10 --headless --duration 3 --output data/task_scenes
```

桌高为 `0.75 m`，尺寸 `0.80 × 1.10 × 0.05 m`。距离 `d` 沿机器人前方 `+X`，从**底座原点到近侧桌沿**测量；中心为 `(d + 0.40, 0, 0.725) m`。交互回车默认 `d=0.10 m`；`--table-distance` 可显式指定，非交互场景启动必须提供。桌子、物体与固定支架整体平移，不重新采样，桌子保持静态碰撞体。接受有限非负距离，不代表已验证碰撞净空或可达范围。

种子可重现位置、质量、摩擦、颜色及字母分配；任务 manifest 记录采样参数与桌距。物体使用重力、碰撞与摩擦，不直接写 qpos 播放、不焊住自由物体、不用禁用接触或允许穿透伪造稳定。盘架、杯架、箱体固定，任务物体自由运动；程序化场景不是自动策略、任务评分或论文视觉资产的精确复刻。

机器人动力学保留 URDF 的质量、质心和惯性，不以 6061 铝外观网格体积重算装配惯性。木、陶瓷、塑料、玻璃物体的质量和摩擦是工程默认值，未经过实物标定；材质和参数来源保存在 manifest。刚体仿真不模拟材料屈服、破碎或柔性。机器人外观透明度只影响显示，不改变接触和动力学。

模型现有临时碰撞例外为左侧 `Link5_L–Link7_L` 和右侧 `Link5_R–Link7_R`，记录在 `collision.temporary_excludes`。它们绕过原始凸包腕部干涉，也会忽略这两对连杆的真实碰撞；不是硬件安全保证。限位、分组 hold 和目标差门限同样不构成实机安全认证，SPD 不承担上游 IK/轨迹规划或避碰。

模型编译默认拒绝覆盖非空产物目录，检查新模型应使用新目录：

```bash
pixi run spd-model --output /tmp/spd-model-check
pixi run spd-envs-check
```

正式资源在 `src/description/tianji_wuji2/{assets,generated}`。不要在采集会话中替换模型；更改资产后重新编译、验证再使用，不混合不同机器人配置的数据。

## 相机、HDF5 与数据检查

`config/sim_cameras.yaml` 配置世界固定 `top` 和左右腕部相机，生成各自真实仿真视角，不复制同一画面充当多视角。采集只渲染契约需要的 RGB，不额外生成未保存的分割图。配置仍为 `provisional-v1`，不宣称已经完成相机标定；腕部画面可能受手指遮挡，需要独立完成视角验收。

录制保存双臂实际 qpos（14 维）、双手实际 qpos（40 维）和启用相机的 RGB/JPEG；状态目标采样 120 Hz、图像目标 30 Hz。使用同一主机单调时钟，以 episode 起点归零；真实频率必须由时间戳统计。仿真与图像均由物理状态产生，不把人体输入或命令角度当作机器人实测状态。

**当前采集器保留已有的命令扩展** `observations/commands`，记录实际应用的 54 维目标、session/sequence、源 UTC 时间、ready/hold 和物理应用时间。它不是原始“只存状态与 RGB”契约的一部分；扩展和原始契约的区别见 [docs/schema-v1.md](docs/schema-v1.md)。本次目录迁移不改字段、版本或文件格式，也不把未来实测 qpos 伪称原始动作。

录制期间文件名为 `episode_XXXXXX.partial.h5`。按 `s` 确认成功后，后台写入者完成字段、维度、有限值、时间戳和 JPEG 校验，通过后发布为 `.h5`；中断或失败保留 partial，不自动成功。相机创建和同步渲染仍可能阻塞主循环并触发 100 ms hold；JPEG/HDF5 在后台处理不代表渲染无阻塞，也不保证录制时的实时频率。

```bash
pixi run validate_episode data/episodes/episode_000001.h5
pixi run replay_episode data/episodes/episode_000001.h5
```

`replay_episode` 是**只读校验与统计检查**，不会启动命令发布器、恢复场景或用记录 qpos 驱动物理回放。训练侧负责时间对齐、state/action 配对和图像预处理；不能从原始观测文件恢复未记录的控制命令。

## 能力与验证边界

正式运行入口仅包括订阅仿真、模型编译、场景查看／检查和数据检查。不存在 SPD 内的 PICO 启动、H5 命令发布、Zenoh tracking 或遥操作兼容入口。

真实头显到上游再到 SPD 的端到端采集、硬件安全、跨主机网络、录制负载下的实时性能及论文数据等价性必须分别验收，不能以进程启动或模块存在替代。上游发布契约已定稿；本次 SPD 验证范围见下文，不宣称已完成真实头显联调。架构与职责见 [docs/architecture.md](docs/architecture.md)。

### spd-syz 重构验证（2026-09-21）

- `pixi install --all --locked`、`pixi run ros-build-interfaces` 成功；发布端与本仓库的 `JointCommand.msg` 已逐字节比较一致。
- `pixi run test -q`：49 项通过。无效碰撞输入回归触发一条 trimesh 数值警告，不影响通过结果。
- 新路径下 `cups/pyramid` 场景已完成无图形物理推进、manifest/状态导出和截图检查，机器人 54 维索引与场景自由关节共存。
- 独立 ROS domain 127 的真实 Fast DDS 测试发布进程驱动 MuJoCo：未授权保持、显式启用、关节物理响应、失鲜后锁存保持及新会话撤权通过。测试发布器不是产品入口，未复制上游遥操作算法。
- 实际无图形控制终端 `e/r/s/d` 完成保存和丢弃，中断保留未标记成功的 partial 文件。保存的示范包含双臂／双手各 2,202 帧、每路 RGB 551 帧，`validate_episode` 和 `replay_episode` 均通过。去除 tmux 后，前台 `spd-sim --headless` 启动、终端输入及 SIGINT 退出已重新验证，无残留订阅进程。
- 三路 1280×720 RGB 已实际渲染并检查；本次未验证桌面 Viewer 交互、真实头显到上游发布端的完整链路、相机标定或录制实时性能。帧数不是采集频率保证。
