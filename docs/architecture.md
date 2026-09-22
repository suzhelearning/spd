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
              实际关节 qpos / 任务物体与接触状态
                     ↙                    ↘
            cameras：真实多视角       Viewer：状态与目标曲线
                     ↓
       data_collector：MuJoCo 实际状态 + RGB + 时间戳
                     ↓
            .partial.h5 → 校验 → .h5
```

正式入口为前台 `pixi run spd-sim`，以及模型、场景、数据检查命令。不存在独立停止命令、旧 PICO、HDF5 命令发布或 Zenoh 遥操作启动器和转发 shim。`replay_episode` 只读检查数据，不把历史观测用作合成发布目标。

## 2. 源码与资源职责

| 路径 | 职责 |
|---|---|
| `src/spd/spd_vr/interfaces/` | JointCommand wire 契约、校验、邮箱、订阅执行器与授权／保持门 |
| `src/spd/spd_vr/simulation/` | 机器人 MuJoCo 物理执行、ROS Viewer、窗口与场景查看 |
| `src/spd/spd_vr/cameras/` | 世界／腕部仿真相机与 RGB 获取 |
| `src/spd/spd_vr/data_collector/` | 配置、共享采集会话、ROS 服务／状态及触发客户端、HDF5 写入／校验 |
| `src/spd/spd_vr/description/` | manifest、模型编译与资源定位 |
| `src/spd/test/` | 与保留运行时对应的测试 |
| `src/environments/spd_envs/` | 独立环境包，任务注册、随机化、场景生成与重置检查 |
| `src/description/tianji_wuji2/assets/` | 原始 URDF、网格和碰撞资产 |
| `src/description/tianji_wuji2/generated/` | 编译后的可加载模型与 manifest |
| `src/interfaces/tianji_spd_interfaces/` | ROS 2 `JointCommand.msg` 与接口构建元数据 |
| `config/` | `collect_sim.yaml` 采集配置与 `sim_cameras.yaml` 相机配置 |
| `bash/` | 前台订阅／采集启动入口与独立触发终端 |
| `data/` | 采集产物和已有样本，不随代码清理删除 |

Python 包名保留 `spd_vr`、`spd_envs`。依赖方向为 `spd-vr → spd-envs`；环境包只负责场景，不依赖 ROS 或遥操作算法，也不硬编码机器人路径。资源定位通过 `description/model_builder.py` 的 `workspace_root()`、`description_root()` 和 `config_root()`，不以调用者当前目录猜测资源位置。

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

带桌场景在创建窗口前解析 `simulation.scene.resolve_table_distance`；`--table-distance` 显式指定或交互询问，非交互必须显式提供。距离沿 +X 从底座原点到近侧桌沿，默认 0.10 m。桌子、物体和固定支架整体平移，不重新采样随机参数；实际距离、几何和 seed 保存到 scene/task manifest。

机器人保留 URDF 质量、质心和惯性；物体材质参数是工程默认值，不是实物标定结果。显示透明度不改变碰撞。模型保留 `Link5_L–Link7_L`、`Link5_R–Link7_R` 两对临时碰撞排除，记录在 `collision.temporary_excludes`；它们也会忽略真实碰撞，不得作为硬件安全保证。SPD 保留模型限位和控制门，不承担上游轨迹求解或避碰。

## 6. 相机与录制

`config/sim_cameras.yaml` 定义世界 `top` 和左右腕相机；每路使用真实仿真视角，不复制 overview。配置仍为 provisional，不能宣称完成标定。`collect_sim.yaml` 默认实际关节状态目标采样 120 Hz、RGB 目标 30 Hz；二者独立配置为 480 的正整数约数，名义调度不是负载下频率保证。

运动与录制独立：已启用且至少一组新鲜 ready／非 hold 命令时，`r` 请求准备新 episode；`s` 由操作员确认成功并请求保存；`d` 丢弃当前段。Viewer 无需回车，SPD 终端输入需回车。独立 `trigger.py` 终端无需回车，`q` 只退出客户端。采样从当前 MuJoCo 物理状态开始，不等待新的 AppliedCommand，也不补写旧缓存。准备／保存／丢弃期间明确拒绝新操作。清除授权、采集错误或退出不自动发布成功文件。

录制使用同一主机单调时间并按 episode 起点归零。状态源为 MuJoCo 实际 qpos，图像时间在完整帧可用且 JPEG 编码之前记录。JPEG/HDF5 写入由后台线程执行，保存／丢弃协调不在控制回调同步等待；相机创建和同步渲染仍可能阻塞物理循环并触发 100 ms hold，不为渲染放宽门限。

写入中为 `episode_<UUID>.partial.h5`，关闭后校验字段、维数、有限值、时间戳与 JPEG，通过后原子发布 `.h5`；中断／失败保留 partial。`max_frames` 为状态样本上限，0 不限，达到上限自动保存 `success=false`；仅显式保存标记成功，不自动开始下一段。输出按 `--output`、`SPD_EPISODE_OUTPUT`、配置 `data_dir` 的优先级确定，配置相对路径基于配置所在目录；数据集 schema 配置不兼容时拒绝追加，有效采集参数按 episode 记入 task manifest。

上述输出路径是根目录；`CollectionSession` 在每次接受 start 时按本机本地日期固定 `YYYYMMDD/`，将目录传给后台 recorder。创建目录、校验每日 `dataset_config.json` 和写入都在后台执行。跨午夜不拆分或移动当前段，下一段重新选日期；状态中的 partial／最终路径都指向该段固定的日期目录。数据集配置按日隔离，不在根目录新建共享配置，不迁移旧数据。

当前文件遵循 state-only schema-v1：双臂／双手实际 qpos、RGB、各流时间戳和 task manifest。ROS JointCommand、执行器目标、命令序号／session／ready／hold 只用于执行与实时显示，不写入采集文件；`CollectionSession.tick` 只接收物理步信息，从 plant 读取实际 qpos。已删除旧 `observations/commands` 扩展，当前校验器拒绝带该扩展的旧文件，不自动迁移已有数据。详见 [schema-v1.md](schema-v1.md)。

`CollectionSession` 是本地按键和 ROS 请求共同使用的唯一录制状态机。物理线程管理状态、采样和相机；单个协调 worker 处理磁盘生命周期，沿用单个后台 HDF5 写入者。`ros_viewer` 不再维护另一套录制状态。`spd-collect` 是同一前台仿真进程的采集入口，与 `spd-sim` 二选一，不能同时启动两套写入者。

`CollectionRosControl` 在同一节点提供 `/spd/collection/start`、`save`、`discard` Trigger 服务；回调只接受／拒绝操作，不等待写盘或改变运动授权。响应 JSON 返回 collector／operation ID，`success` 只表示接受。`/spd/collection/status` 为可靠 transient-local String JSON，状态变化立即发布，并以 250 ms 名义间隔更新；包含状态、计数、路径和错误。物理循环阻塞也会延迟心跳，不伪造健康。

独立触发客户端只消费状态和调用服务，无 MuJoCo、输入设备或写盘所有权；它检查单一同属节点的服务和状态发布者、心跳及 operation ID，等候完成，不自动重试未知结果。该工作流借鉴 protype 的配置／触发组织，不引入其硬件数据处理器、缺失补零或同步帧 schema。

## 7. 研究与验证边界

论文参考为 [Pre-training Visual Dexterity in Simulation](papers/2608.15917v1.pdf) §3.1、附录 A.1。论文物理 480 Hz、控制／流传输／记录 60 Hz、训练网格 30 Hz 是不同阶段的契约，不等同于当前实现各流频率或机器性能保证。

上游发布契约已定稿；这里不宣称真实 PICO → 上游 → SPD 已完成端到端验收。多视角离线批量渲染、训练增强、完整任务示范质量与论文等价性均须单独验证。实机控制、真实传感器融合、策略训练与部署不属于 SPD。启动窗口、模块存在或接口匹配，都不能替代物理行为和采集数据的实际验收。
