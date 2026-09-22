# SPD Simulation Collection — Tianji + Wuji Hand 2

SPD **只负责仿真数据采集**：订阅外部 ROS 2 `JointCommand`，在 MuJoCo 中推进 Tianji 双臂 + Wuji Hand 2 双手与任务物体，在线记录 60 Hz 完整场景物理轨迹及接触信息。训练图像由后续离线渲染产生；在线采集不创建相机渲染器、不保存 RGB。SPD 不接收 PICO 原始输入，不做标定、IK 或手部重定向，不提供命令发布器，也不控制实机。

上游 `tianji_teleop-ros2` 已定稿：负责 PICO 标定、共享根坐标下的 Franka DLS + Ruckig、Hand2 映射及默认 ROS 关节命令发布。其入口是在上游工作区运行 `bash bash/run_pico_hand_sim.sh --height-m 1.75`。SPD 启动器不会启动或管理上游进程；上游功能定稿不替代本工作区的真实头显端到端验收。

论文参考：[Pre-training Visual Dexterity in Simulation](docs/papers/2608.15917v1.pdf)。程序化场景、相机与数据流程不是论文结果的等价性证明。

## 源码与资源职责

```text
pixi.toml / pixi.lock                 受维护运行环境与命令
setup.py                            Python 安装配置与 CLI 入口
src/
  interfaces/                       JointCommand 校验、邮箱、会话与授权
  simulation/                       MuJoCo 物理执行、ROS Viewer、场景查看
  cameras/                          仿真多视角相机
  data_collector/                   完整场景轨迹、模型快照、ROS 采集控制及独立恢复检查
  offline_rendering/                预留：从物理轨迹离线生成 RGB／实例分割，尚未实现
  description/                      manifest、资源定位、机器人模型编译
  environments/spd_envs/             独立环境包：任务、随机重置、场景生成
  tianji_wuji2/tianji_wuji2/
    assets/                          原始 URDF、网格与碰撞资产
    generated/                       编译后的模型与 manifest
  interfaces/tianji_spd_interfaces/   ROS 2 JointCommand 消息包
config/                              collect_sim.yaml 采集配置与相机配置
bash/                                仿真／采集前台入口与独立采集触发器
data/                                采集输出和已有数据；不随代码清理删除
docs/                                架构、数据契约与论文
```

运行时 Python 包直接位于 `src/`，使用 `simulation.*`、`data_collector.*`、`cameras.*`、`interfaces.*`、`description.*` 导入，不保留统一外层包或旧导入兼容层。根目录 `setup.py` 安装这些包，发行包名仍为 `spd`；独立环境包保持 `spd_envs`，依赖方向为 `spd → spd-envs`，环境包不依赖 ROS、PICO 或机器人输入算法，也不硬编码 Tianji 模型路径。

## 安装与启动

在项目根目录操作，支持 Linux x86-64。环境固定 Python 3.12，保留 MuJoCo 3.12，并锁定兼容的 ROS 2 Jazzy / Fast DDS 依赖。图形查看需要可用显示环境；SPD 本身不需要 ADB 或头显。在线仿真从本 checkout 读取资源；轨迹内嵌已编译模型和网格／纹理，可脱离原场景 XML 恢复，但必须使用记录时的精确 MuJoCo 版本。不支持将 Python wheel 脱离工作区作为完整在线仿真部署。

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

发布侧须使用相同消息定义并加载对应 ROS 接口 overlay。SPD 启动器不会自动重建接口。`config/collect_sim.yaml` 默认采集根目录为 `/data/TianjiSim`，新 episode 自动存入开始当天的 `YYYYMMDD/` 子目录；显式 `--output PATH` 优先于 `SPD_EPISODE_OUTPUT`，两者均未提供时使用配置的 `data_dir`。

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

54 维顺序为左臂 7、右臂 7、左手 20、右手 20，单位 rad。订阅端校验固定关节名称契约、维度、有限值、ready 组关节限位、session、递增 sequence 和 UTC 新鲜度，随后按 manifest 名称映射到机器人 qpos/actuator 地址。场景 free joints 不占用这 54 维；收到目标不意味着授权运动。

| 操作 | 作用 |
|---|---|
| `e` | 显式启用／禁用命令应用；启用须有新鲜候选并通过目标差门限 |
| `c` | 清除候选与授权，保持已有目标；不回 HOME、不重置物理场景 |
| `F8` / `F9` | Viewer 切换所选关节的实际应用目标与实际 qpos 曲线 |
| `r` | 已启用且至少一组命令新鲜、ready、未 hold 时准备新 episode |
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

种子可重现位置、质量、摩擦、资产型号、材质变体及字母分配；任务 manifest 记录实际参数、桌距和 `geometry_revision=detailed-scenes-v1`。物体使用重力、碰撞与摩擦，不直接写 qpos 播放、不焊住自由物体、不用禁用接触或允许穿透伪造稳定。盘架、杯架、箱体固定，任务物体自由运动；场景不是自动策略、任务评分或论文视觉资产的精确复刻。

场景按 ABC 的方式分开**外观资产与碰撞代理**：外观使用有纹理的网格、圆滑表面和细节零件，质量为零且不参与接触；碰撞几何独立承担质量、惯性与实际接触。场景碰撞组为 3，Viewer／相机默认隐藏这一组，只影响显示、不禁用物理。

| 场景 | 细粒度内容 |
|---|---|
| Jenga／多米诺 | 圆角木纹外观、不同木色、多米诺点数与分隔线；保留实体积木碰撞 |
| 字母积木 | 全部字母任务都有 A–Z 六面贴图；拼词保留 ROBOTICS，元音／辅音任务包含两类字母 |
| 马克杯 | 圆滑中空杯壁、杯沿与釉线、开放椭圆杯柄；32 段杯柄碰撞不封住开口 |
| 餐盘 | 浅盘内凹面和盘沿，不再是一个实心圆柱；盘架保留真实槽位并增加金属杆／木底座细节 |
| 杯子 | 平滑中空轮廓、杯沿、底圈、32 面物理杯壁及真实防卡叠放支点 |
| 瓶子／箱子 | 6 种 ABC 瓶子网格与原始贴图、各自分块凸碰撞；箱体有开口、边沿和面板细节 |

桌面增加木纹、边框、桌腿和统一双光源；桌腿／边饰／背景地面为外观元素，桌面仍使用原尺寸静态碰撞体。物体开口、杯柄和箱体内腔不是贴图伪造。物理代理与细小外观倒角并非逐三角面完全相同。

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

## 配置化采集与独立触发终端

借鉴 `protype-yam-update` 的配置、触发服务和采集状态流程，复用 SPD 的同一个 MuJoCo 物理所有者和后台 HDF5 写入器。**独立的是触发客户端，不是第二套仿真或写入器**；不复制 CAN、真机、PICO、IK、缓存补帧或缺失数据补零。

终端一启动采集会话（与 `spd-sim` 二选一，不要同时启动）：

```bash
pixi run spd-collect --task cups/pyramid --table-distance 0.25
# 同等 shell 入口：
# bash bash/run_data_collector.sh --task cups/pyramid --table-distance 0.25
```

在这个终端输入 `e` 并回车，显式启用新鲜且满足目标差门限的关节候选。启动采集进程不会自动运动或录制。已有 `spd-sim` 也提供同样的采集服务；升级代码后需重启旧进程。

终端二启动触发器：

```bash
pixi run spd-collect-trigger
# 或 bash bash/collect_trigger.sh
```

触发终端直接按键，无需回车：`r` 开始、`s` 保存并确认成功、`d` 丢弃、`q`／`Ctrl+C` 仅退出触发器。退出触发器**不会结束正在进行的录制**，也不会保存、丢弃或停止 SPD。它没有运动使能按键；SPD 自身终端的 `e/c/r/s/d` 仍需回车。

非交互调用：

```bash
pixi run spd-collect-trigger --command status
pixi run spd-collect-trigger --command start
pixi run spd-collect-trigger --command save
pixi run spd-collect-trigger --command discard
```

这些是按需执行的独立操作，不是一组连续运行的脚本。客户端等待匹配的 collector／operation 完成，显示状态、帧数和路径；`save completed` 才表示关闭、校验及文件发布完成。拒绝、服务缺失、多实例冲突、状态失鲜或超时会明确报错，不自动重试；超时不能解释为操作未发生。默认 `--timeout 30`，失败后先检查 SPD 终端和当前状态。

采集配置 `config/collect_sim.yaml`：

```yaml
version: 2
data_dir: /data/TianjiSim
state_rate_hz: 60
writer_queue_size: 256
max_frames: 0
```

- `data_dir` 是采集根目录；相对路径基于配置文件所在目录解析，可用 `--collection-config PATH` 指定配置。每次开始新段按本机本地日期选择 `YYYYMMDD/`，例如 `/data/TianjiSim/20260922/`。跨午夜正在录制的段不拆分，仍保存到开始当天；下一段自动进入新日期，无需重启。`--output` 和环境变量覆盖的根目录也遵循此规则。
- 轨迹固定每 8 个 480 Hz 物理步采一帧，即仿真时间 60 Hz。记录物理步编号、仿真时间和主机单调时间；实际墙钟频率必须另行统计，不能以名义调度推断。旧配置中的 `camera_rate_hz` 已移除，配置版本改为 2。
- `writer_queue_size` 限制后台写入队列；溢出报错并保留 partial，不静默丢帧。
- `max_frames: 0` 表示不限；正数限制完整场景轨迹帧数，不按接收的命令数计数。可用 `--max-frames N` 覆盖。
- 到达上限自动结束、校验并保存为 **`success=false`**，不会自动开启下一段。只有显式 `s`／`save` 标记成功。
- 每段记录实际生效的配置与配置文件路径；采样直接读取当前物理状态，不等待新的 ROS cmd，也不补写录制前缓存。

服务为 `/spd/collection/{start,save,discard}`（`std_srvs/srv/Trigger`）；状态为 `/spd/collection/status`（`std_msgs/msg/String` JSON，可靠、transient-local）。服务成功响应表示操作已接受，最终结果由携带 `collector_id`、`operation_id` 的状态确认。采集服务不授予运动权限，不放宽 100 ms 失鲜保持或启用目标差门限。

## 物理轨迹、场景恢复与离线渲染边界

**state-only 不等于只存机器人关节角。** schema-v2 每帧保存全场景 `qpos/qvel`、按名称排列的机器人 54 维实际位置／速度、任务物体世界位姿、左右手接触状态及接触对象。模型存在的执行器内部状态、mocap 状态和 equality 启用状态也随帧保存。没有 `ctrl`、ROS 命令、actions 或在线 RGB。

每段内嵌 MuJoCo 编译模型（含网格、纹理与相机）、版本与 SHA-256、任务和实际随机参数、关节／物体映射与相机元数据。采集开始后的样本直接来自完成物理积分的场景，不等新的 ROS 命令、不补录旧缓存。手–物接触在每个物理步观察，按采样区间累计，避免只看 60 Hz 瞬间漏掉短接触；不会把桌面接触和机器人自碰撞当作手–物接触。

`config/sim_cameras.yaml` 的 `top`、`left_wrist`、`right_wrist` 定义仍注入模型并记录，但不在采集循环中渲染。在线 Viewer 是操作者反馈，不是训练图像流。配置仍为 `provisional-v1`，尚未完成相机标定。`src/offline_rendering/` 继续仅预留，尚不生成 RGB 或实例分割。

新段写入 `episode_<UUID>.partial.h5`。每个后台队列事件是一整帧，所有数据集严格同长。重复／缺失物理步、非递增时间戳、非有限状态、队列溢出或写盘失败都保留不完整段，不静默覆盖或丢帧。显式保存或达到帧数上限后，关闭并校验数据、模型和元数据，完整通过才发布 `.h5`。`complete` 表示数据完成，`success` 表示操作者确认任务成功，二者不同；帧数上限完成为 `complete=true, success=false`。

每天的目录独立保存 `dataset_config.json` 和当天的 HDF5。schema-v2 不与旧的机器人 qpos＋JPEG schema-v1 混写；同日契约不匹配会拒绝追加，不覆盖原配置。**升级后请指定新的采集根目录，例如 `--output /data/TianjiSim-trajectories`。** 历史数据不迁移、不删除。

```bash
# 将路径替换为采集状态输出的实际文件路径
pixi run validate_episode '/data/TianjiSim-trajectories/YYYYMMDD/episode_<UUID>.h5'
pixi run replay_episode '/data/TianjiSim-trajectories/YYYYMMDD/episode_<UUID>.h5'
```

`replay_episode` 在独立 MuJoCo 模型中逐帧恢复记录状态，计算机器人状态及物体位姿的最大恢复误差；不发送控制目标，不推进物理，不渲染图像，不修改文件。拒绝不完整段、版本或模型校验不匹配。它是离线渲染前的重建验证，不是检查点继续仿真：文件没有保存重启原控制循环所需的命令和全部积分器内部历史。

数据契约见 [docs/schema-v2.md](docs/schema-v2.md)。检查点／回退、超过 10 秒无接触裁剪、30 Hz 训练样本构建和离线图像渲染尚不属于当前采集实现。旧 `align_30hz`、`filter_contacts` 入口依赖已废弃且不匹配的 action／image schema，已移除，避免误处理轨迹；不能从 state-only 文件恢复未记录的原始命令。

## 能力与验证边界

正式运行入口包括订阅仿真、配置化采集及独立采集触发、模型编译、场景查看／检查和数据检查。不存在 SPD 内的 PICO 启动、H5 命令发布、Zenoh tracking 或遥操作兼容入口。

真实头显到上游再到 SPD 的端到端采集、硬件安全、跨主机网络、录制负载下的实时性能及论文数据等价性必须分别验收，不能以进程启动或模块存在替代。上游发布契约已定稿；本次 SPD 验证范围见下文，不宣称已完成真实头显联调。架构与职责见 [docs/architecture.md](docs/architecture.md)。

### 精细场景验证（2026-09-22）

- 18 个任务 × seeds 0–2，共 54 个场景：重复生成的 manifest／完整 MJCF 一致，初始接触检查通过，每个独立场景推进 480 步后状态有限、物体根 body 高度均大于 0.70 m。确认动态物体质量与配置一致，固定支架无 free joint，外观层不参与碰撞。
- 六类代表场景分别组合真实 Tianji／Wuji 模型并渲染截图；重复合并不修改源场景。各记录 3 帧 schema-v2 轨迹，移除临时场景 XML 后内嵌模型恢复通过，物体位姿误差小于 1e-12。
- 实际球体落入杯腔／箱体并停在内底上；12 mm 探针可置于杯柄孔内，无接触穿透。字母 R 的实际渲染方向与原图一致，已修正贴图 V 方向；陶瓷盘沿与内凹面在渲染检查中可见。
- 环境包 wheel 包含 104 个运行资产文件；从临时解包安装位置加载瓶子／字母／马克杯场景通过，全部六种瓶子资产和导出哈希一致。`pixi run spd-envs-check --seed-count 3` 和实际 `spd-scene` 无图形截图入口通过。
- 这些是有限种子、短时物理与视觉检查，不代替真实头显操作者的抓取／挂杯／堆叠验收或实物参数标定。没有新增永久测试目录或离线渲染实现。

### schema-v2 完整场景轨迹验证（2026-09-22）

- 真实 Fast DDS（隔离 domain 149）→ MuJoCo → 采集服务完成：未授权开始被拒绝；没有新命令时仍采状态；12 帧上限保存为 `complete=true, success=false`，显式保存为 `success=true`，丢弃不影响已保存段。
- 实际 `cups/pyramid` 场景包含 54 个机器人关节和 6 个任务物体；受控初始落体产生真实物体运动。记录 qpos/qvel 与采样时物理状态逐元素相等，tick 间隔恰好为 8。在线 Renderer 被替换为抛错探针，整个采集仍通过。
- 关闭采集并删除临时场景 XML 后，新 Python 进程仅加载内嵌模型恢复 12 帧，机器人位置／速度和物体位置／四元数最大误差均为 0；恢复进程禁止 mj_step 和 Renderer，两个公开校验／恢复 CLI 均通过。
- 漏物理步、中断、队列溢出均保留不完整 partial；模型字节、元数据哈希和 tick 损坏被拒绝。另一真实 MuJoCo 小场景验证左右手区间接触、非任务接触排除、act/mocap/equality 恢复、零物体／零相机及旧数据集／版本冲突拒绝。
- 实际无图形 RosViewerApp 和独立触发器状态查询在 domain 150 启动验证通过。未验证真实头显示范、桌面 Viewer、离线图像或长期采集实时性能；没有新增永久测试目录。

### spd-syz 重构验证（2026-09-21）

以下为旧 schema 的历史验收记录，不代表当前完整场景轨迹契约；当前工作区已移除回归测试目录及测试命令。

- `pixi install --all --locked`、`pixi run ros-build-interfaces` 成功；发布端与本仓库的 `JointCommand.msg` 已逐字节比较一致。
- 当时回归测试 49 项通过。无效碰撞输入回归触发一条 trimesh 数值警告，不影响通过结果。
- 新路径下 `cups/pyramid` 场景已完成无图形物理推进、manifest/状态导出和截图检查，机器人 54 维索引与场景自由关节共存。
- 独立 ROS domain 127 的真实 Fast DDS 测试发布进程驱动 MuJoCo：未授权保持、显式启用、关节物理响应、失鲜后锁存保持及新会话撤权通过。测试发布器不是产品入口，未复制上游遥操作算法。
- 实际无图形控制终端 `e/r/s/d` 完成保存和丢弃，中断保留未标记成功的 partial 文件。保存的示范包含双臂／双手各 2,202 帧、每路 RGB 551 帧，`validate_episode` 和 `replay_episode` 均通过。去除 tmux 后，前台 `spd-sim --headless` 启动、终端输入及 SIGINT 退出已重新验证，无残留订阅进程。
- 三路 1280×720 RGB 已实际渲染并检查；本次未验证桌面 Viewer 交互、真实头显到上游发布端的完整链路、相机标定或录制实时性能。帧数不是采集频率保证。

### protype 采集流程适配验证（2026-09-22）

- 完整保留测试共 56 项通过；首次采样状态提示修正后，相关采集／录制 8 项回归再次通过。
- 独立 ROS domain 128 的真实 Fast DDS、MuJoCo 和三路相机验证：未授权开始被拒绝；帧数上限 12 自动保存双臂／双手各 12 帧、各路 RGB 3 帧，`success=false`；显式保存段为状态各 57 帧、各路 RGB 15 帧，`success=true`，均通过文件校验。
- 实际独立终端 `r` 启动、`q` 退出后录制继续；另一客户端 `discard` 只删除当前段，已保存段保留。中断仿真后保留非成功 partial，未发布正式文件。
- 采集服务验收使用测试发布源，不覆盖真实头显、桌面 Viewer 操作、相机标定或录制实时性能。未停止用户现有发布端或采集进程。

### state-only 契约校正（2026-09-22）

- 移除文件中的命令目标、命令序号／会话等扩展，采样接口不再接收 AppliedCommand。此前验证记录属于校正前的输出，不能作为当前 state-only 文件契约的证明。
- 57 项测试通过；新增真实 MuJoCo 回归，验证没有新 cmd 时仍可采集实际 qpos，并与保留目标明确区分。
- 再次执行真实 DDS → MuJoCo → 采集服务：新文件仅有 `observations/{arms,hands}` 和 `images`，无 commands/actions；状态各 12 帧、每路 RGB 3 帧，校验通过。录制状态与本次测试源发送目标的最大差约 0.0633 rad，未将 cmd 当 state 写入。
