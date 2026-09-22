# 仿真采集架构

## 1. 边界与数据流

SPD 是 **ROS 关节命令订阅端与 MuJoCo 仿真数据采集端**。输入设备接入、人体标定、共享根坐标下的 Franka DLS + Ruckig、Hand2 映射和 ROS 发布属于独立上游 `tianji_teleop-ros2`；已定稿入口是在其工作区运行 `bash bash/run_pico_hand_sim.sh --height-m 1.75`。SPD 不启动上游，不包含 PICO/IK/重定向/命令发布器，不发布实机控制。上游最多 60 Hz 发布 54 维 rad 目标，`--headless` 只关闭辅助窗口，发布继续；C、跟随、失鲜和 P／H／Q 制动／回程的 readiness 与会话由上游负责。

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
       offline_rendering：预留，后续生成训练 RGB／实例分割
```

正式入口为前台 `pixi run spd-sim`，以及模型、场景、数据检查命令。不存在独立停止命令、旧 PICO、HDF5 命令发布或 Zenoh 遥操作启动器和转发 shim。`replay_episode` 从文件内嵌模型恢复状态并验证，不把历史观测用作合成发布目标。

## 2. 源码与资源职责

| 路径 | 职责 |
|---|---|
| `src/interfaces/` | JointCommand wire 契约、校验、邮箱、订阅执行器与授权／保持门 |
| `src/simulation/` | 机器人 MuJoCo 物理执行、ROS Viewer、窗口与场景查看 |
| `src/cameras/` | 世界／腕部仿真相机与 RGB 获取 |
| `src/data_collector/` | 配置、采集状态机、ROS 控制、完整物理轨迹与模型快照、独立恢复验证 |
| `src/offline_rendering/` | 仅预留；不包含渲染器或训练图像输出 |
| `src/description/` | manifest、模型编译与资源定位 |
| `src/environments/spd_envs/` | 独立环境包，任务注册、随机化、场景生成与重置检查 |
| `src/tianji_wuji2/tianji_wuji2/assets/` | 原始 URDF、网格和碰撞资产 |
| `src/tianji_wuji2/tianji_wuji2/generated/` | 编译后的可加载模型与 manifest |
| `src/interfaces/tianji_spd_interfaces/` | ROS 2 `JointCommand.msg` 与接口构建元数据 |
| `config/` | `collect_sim.yaml` 采集配置与 `sim_cameras.yaml` 相机配置 |
| `bash/` | 前台订阅／采集启动入口与独立触发终端 |
| `data/` | 采集产物和已有样本，不随代码清理删除 |

运行时 Python 包直接位于 `src/`，按职责使用 `interfaces`、`simulation`、`cameras`、`data_collector`、`description`，不保留统一外层包或旧导入兼容层。根目录 `setup.py` 安装这些包，发行包名仍为 `spd`；依赖方向为 `spd → spd-envs`，独立环境包保持 `spd_envs`，只负责场景，不依赖 ROS 或遥操作算法，也不硬编码机器人路径。资源定位通过 `description/model_builder.py` 的 `workspace_root()`、`description_root()` 和 `config_root()`，不以调用者当前目录猜测资源位置。

`pixi.toml` / `pixi.lock` 是受维护运行环境；ROS 接口构建到 `.ros/{build,install}`，与原始源文件和机器人生成模型分离。`ros-build-interfaces` 使用 `src/interfaces/tianji_spd_interfaces`，SPD 不自动构建上游工作区。

## 3. 订阅契约与授权状态

接口类型 `tianji_spd_interfaces/msg/JointCommand`，话题 `/spd/tianji_wuji2/v1/joint_command`，`schema_version=1`，`robot_config=tianji_wuji2_v1`。54 维顺序是左臂 7、右臂 7、左手 20、右手 20，单位 rad。消息保留原 wire 字段，不因目录迁移改变。

订阅回调校验名称顺序、维度、有限值、ready 组限位、session、递增 sequence 与 UTC 新鲜度，完整通过后才原子替换最新候选。物理 tick 消费时再次校验年龄；按 manifest 名称预计算 qpos/actuator 地址，场景 free joints 不改变机器人索引。

显式 `e` 启用要求新鲜候选，且 ready 组每个关节相对保留目标的差不超过默认 `0.15 rad`；该门限可通过 `--max-enable-delta-rad` 调整。新 session 撤销已有授权，`c` 清除控制与授权但不回 HOME 或重置场景。

ready/hold 三组分别为双臂（bit 0）、右手（bit 1）、左手（bit 2）。未 ready 组保持目标；每组超过 100 ms 没有新鲜 ready 目标后锁存 hold，其他新鲜组仍可执行。数据恢复不能自动恢复该组运动，须再次显式禁用／启用。目标保持不是冻结 qpos：物理积分、接触和跟随误差继续存在。

Viewer 用 `F8/F9` 选择关节，展示实际应用目标与实际 qpos；HUD 展示接收／有效／拒绝计数、候选年龄、session、ready/hold 与跟踪误差。进程 ready、DDS 对端发现、有效候选和运动授权是四种不同状态。

## 4. 进程与网络边界

`bash/start_spd_sim.sh` 在 ROS Pixi 环境中以 `exec` 启动前台订阅进程，直接使用当前终端，不创建 tmux 会话、不后台运行。`pixi run spd-sim` 直接选择 `ros-jazzy` 环境，避免嵌套任务启动；脚本从裸 shell 调用时自行进入同一环境。当前终端 `Ctrl+C` 或 Viewer 退出只结束 SPD，不停止上游或硬件控制器。启动不代替运动授权；一次只运行一个采集进程，不共享输出目录。

Jazzy/Fast DDS 使用 domain 120，QoS 为 `BEST_EFFORT / KEEP_LAST(1) / VOLATILE`。在 Pixi 激活和接口 overlay 加载后显式设置 `ROS_DOMAIN_ID=120`、`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`、`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`、`ROS_STATIC_PEERS=''`；无桥接进程。

启动器固定同机发现。跨主机需另外配置发现范围／静态 peers 或发现服务、网络接口、防火墙和 UTC 同步。domain 不是安全边界；DDS 直连也不是延迟、丢包或实时性能保证。

## 5. 物理场景、随机化与模型限制

环境注册表保留六类场景、Table 2 的 17 个任务和 A.4 的 `jenga/playing`。`spd-scene --task SCENE/TASK --seed N` 可独立查看；`spd-sim` 使用同样的任务／seed 参数接入订阅仿真。场景在启动时选择，不运行中切换。

先验证基础机器人资产，再组合任务模型，不覆盖基础 MJCF。桌高 0.75 m；固定盘架、杯架和箱体，任务物体通过 free joints、重力、摩擦与真实接触运动。场景不包含自动策略或成功评分。物理状态不是命令 qpos 回放，RGB 来自该状态渲染，不是录制画面的替代粘贴。

精细场景采用 `SceneBuildResult.assets + worldbody`：`abc_assets.py` 读取环境包自带的六种 ABC 瓶子网格、贴图及分块碰撞，`visual_details.py` 生成其余物体的圆角／旋转曲面、材质和细节。局部资产路径从环境包位置解析，wheel 包含所需 OBJ／PNG／JSON，运行时不依赖原 ABC 目录。ABC 源资产与变换、导出文件哈希写入 provenance；公开再分发权利尚未确认。

场景外观 geom 为 group 2、mass=0、contype=conaffinity=0；独立碰撞代理为 group 3，实际承担接触和质量分配。隐藏 group 3 不等于关闭碰撞。杯／杯柄／箱子的空腔通过真实分块碰撞保留，盘子使用中心浅盘和倾斜环状盘沿。桌腿和装饰仅为外观，不扩大可交互物理范围。木纹、釉面与 A–Z 贴图为原创程序生成，字母任务不再用大量小 box 拼字。

种子同时选择瓶子型号／尺寸、材质和纹理变体；`geometry_revision=detailed-scenes-v1`、对象资产 ID、原始／导出哈希、外观选择、物理参数与字母分配进入场景 manifest。碰撞调试色与真实外观材质分开记录。桌距变化同步平移物体、桌面外观、灯光和相关元数据，不重新采样。

模型合并先解析原模型资源，再深拷贝场景资产和实体；重复资产名明确拒绝，不修改或消耗原 SceneBuildResult。重复合并输出一致。新模型的网格、纹理和相机继续由 schema-v2 的 MJB 快照完整携带，状态恢复与场景精细化解耦；不因此实现离线渲染。

带桌场景在创建窗口前解析 `simulation.scene.resolve_table_distance`；`--table-distance` 显式指定或交互询问，非交互必须显式提供。距离沿 +X 从底座原点到近侧桌沿，默认 0.10 m。桌子、物体和固定支架整体平移，不重新采样随机参数；实际距离、几何和 seed 保存到 scene/task manifest。

机器人保留 URDF 质量、质心和惯性；物体材质参数是工程默认值，不是实物标定结果。显示透明度不改变碰撞。模型保留 `Link5_L–Link7_L`、`Link5_R–Link7_R` 两对临时碰撞排除，记录在 `collision.temporary_excludes`；它们也会忽略真实碰撞，不得作为硬件安全保证。SPD 保留模型限位和控制门，不承担上游轨迹求解或避碰。

## 6. 完整场景轨迹采集与恢复

`config/collect_sim.yaml` 使用 version 2：物理 480 Hz、固定轨迹 60 Hz，每 8 个物理步记录一帧。三路相机定义仍由 `config/sim_cameras.yaml` 注入模型并保存，但在线采集不创建渲染器、不采 RGB。Viewer 是操作反馈，与后续训练渲染无关。相机配置仍为 provisional；名义仿真频率不是负载下墙钟性能保证。

运动与录制独立：已启用且至少一组新鲜 ready／非 hold 命令时，`r` 准备新 episode；`s` 确认成功并保存；`d` 丢弃当前段。准备／保存／丢弃期间拒绝新操作。退出、撤权、队列溢出或数据错误保留不完整 partial。没有新 ROS 命令时仍按物理步记录实际状态，分组 hold 不冻结物理。

`TrajectorySource` 在会话初始化时序列化完整 MuJoCo 模型，包含网格、纹理和相机。SHA-256、精确 MuJoCo 版本、物理设置、源资产溯源、任务随机参数及关节／物体地址映射随 episode 保存。原场景 XML 删除或搬迁不影响恢复；不支持不同 MuJoCo 版本之间直接加载二进制快照。

物理线程每步观察手–物接触，按左右手及任务物体累计到下一个轨迹样本。接触分类使用 `l_wrist/r_wrist` 子树和 task manifest 的物体子树，不把机器人自碰撞、手–桌接触算作手–物接触。每帧保存全场景 qpos/qvel、机器人实际 54 维状态、任务物体世界位姿，以及存在时的 act/mocap/equality 状态。派生位姿通过独立数据对象刷新，不修改在线物理状态。

每帧记录严格递增的物理 tick、绝对仿真秒数和主机单调纳秒。物理 tick 差必须为 8，仿真间隔必须为 1/60 秒；墙钟间隔单独保留，不伪造实时频率。首帧来自准备完成后的第一个物理步；接触首区间只覆盖该步，之后覆盖 8 个物理步。

`CollectionSession` 是本地按键和 ROS 请求共享的唯一录制状态机。单个协调 worker 负责准备／保存／丢弃；单个 HDF5 写线程接收有界整帧队列。队列满、数据不合法或漏 tick 立即报错，绝不覆盖旧样本或静默丢帧。完成后校验全部行、数据类型、时间轴、模型和元数据哈希，才发布 `.h5`；完整性 `complete` 与任务结果 `success` 分离。帧数上限为 `success=false`，仅显式保存为 true。

每次接受 start 时按本机日期固定 `YYYYMMDD/`。跨午夜不拆段，下一段重新选日期；每日 `dataset_config.json` 仅约束模型无关的共享 schema。旧 schema-v1 目录拒绝追加，不修改历史数据。输出优先级保持 `--output`、`SPD_EPISODE_OUTPUT`、配置 `data_dir`。

`replay_episode` 加载内嵌模型，在独立 MjData 逐帧赋值并调用前向计算，验证机器人投影和物体位姿；不调用 `mj_step`、不发送目标、不渲染。记录不包含 ctrl 或全部积分器历史，不能当作恢复原控制运行的检查点。公开校验和恢复拒绝 partial／未完成文件，且拒绝 schema、模型、元数据和 MuJoCo 版本不匹配。详见 [schema-v2.md](schema-v2.md)。

`CollectionRosControl` 仍在同一节点提供 `/spd/collection/{start,save,discard}`；服务响应只表示接受，最终状态带 collector／operation ID。可靠 transient-local `/spd/collection/status` 包含状态、完整轨迹帧数、路径、错误和 elapsed，不再含 camera_frames。独立触发客户端同步采用该字段集合，无运动授权能力。升级后两端均须重启。

本次不实现检查点／回退、接触裁剪、30 Hz 样本构建或离线渲染。旧 `align_30hz`、`filter_contacts` 实现依赖已废弃的动作／图像契约，已移除；不为新轨迹保留无效入口。

## 7. 研究与验证边界

论文参考为 [Pre-training Visual Dexterity in Simulation](papers/2608.15917v1.pdf) §3.1、附录 A.1。论文物理 480 Hz、控制／流传输／记录 60 Hz、训练网格 30 Hz 是不同阶段的契约，不等同于当前实现各流频率或机器性能保证。

上游发布契约已定稿；这里不宣称真实 PICO → 上游 → SPD 已完成端到端验收。多视角离线批量渲染、训练增强、完整任务示范质量与论文等价性均须单独验证。实机控制、真实传感器融合、策略训练与部署不属于 SPD。启动窗口、模块存在或接口匹配，都不能替代物理行为和采集数据的实际验收。
