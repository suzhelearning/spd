# 仿真采集架构

## 唯一目标

参考 [Pre-training Visual Dexterity in Simulation](papers/2608.15917v1.pdf) §3.1、附录 A.1，在 MuJoCo 内通过人类遥操作采集目标机器人本体的示范。机器人替换为 tianji_arm + wuji-hand2，跟踪输入替换为 PICO_2。实机控制与真实传感器融合不在范围内。

## 外部关节订阅入口

`pixi run spd-teleop-ros` 只启动 SPD 侧 DDS/Zenoh 桥和 ROS Viewer：

```text
外部 tianji_teleop（标定 / IK / 手部重定向）或模拟发布器
    → ROS JointCommand，domain 120
    → 发布侧 zenoh-bridge-ros2dds
    → Zenoh TCP
    → SPD 侧 zenoh-bridge-ros2dds，domain 121
    → 校验 / 会话授权 / 最新目标邮箱 / 名称映射
    → MuJoCo position actuators / 物理积分
    → Viewer 目标与实际位置曲线 / 仿真 episode
```

SPD 此入口不接 PICO、不初始化手部重定向、不运行 IK，也不启动发布器；`PlantController(command_only=True)` 拒绝旧 tracking/arm-target 输入。两侧使用不同 ROS domain，桥采用显式 TCP 端点和 loopback DDS 单播发现，避免同机 DDS 绕过桥。ROS 节点使用 Jazzy/Fast DDS，桥内部的 CycloneDDS 不改变 ROS 节点后端。

54 维目标必须满足固定名称顺序、有限值、ready 组范围、会话、递增序号和 UTC 新鲜度。范围检查通过后才原子更新目标；实际消费再次检查年龄。映射按 manifest 名称预计算 qpos/执行器地址，场景 free joints 不占用机器人索引。启用须检查新鲜候选相对保持目标的差值；100 ms 组级超时锁存，新会话解除授权，恢复需本地显式启用。

Viewer 显示接收/拒绝/保持状态和所选关节的实际应用目标、MuJoCo qpos 曲线；`F8/F9` 切换关节，`e/c` 启用或清除控制，`r/s/d` 独立控制录制。相机按既有配置附着在世界/腕部，仿真状态按名称取值；不把未 ready 输入占位值记录为实际动作。当前同步相机渲染仍可能触发控制超时，不宣称已满足录制开启时的实时频率。启动命令和网络配置见根 README。

## PICO 发布侧与拼字场景

`pixi run spd-pico` 是独立的同机编排入口：`PICO_2 → TCP → ros_publisher / PicoTeleopCore → JointCommand(domain 120) → 两侧 DDS/Zenoh 桥 → ros_viewer(domain 121, spelling_blocks/spelling)`。订阅器仍为 `command_only=True`，不加载 PICO 或求解器；其他场景和 H5 演示入口不变。

发布侧复用经过模型校验的双臂 IK、`SideAlignment` 和 `WujiRetargetPair`，输出规范名称顺序的有限、限位内 54 维目标。接收线程只排队帧及接收时间；求解状态由单线程拥有，避免重连回调与求解器并发重置。5 ms 求解调度与 60 Hz 发布调度分开，实际输入超过 50 ms 则撤销 ready，不以新的 ROS 时间戳伪装旧跟踪有效。

PICO 控制窗口的 `C 对齐 → F 确认并跟随 → 空格保持` 是显式控制边界，不再要求切到 MuJoCo 按 `e`。对齐保持当前目标、按其 FK 重新标定双腕，并更换 ROS 会话使接收端撤销旧授权。确认操作经本次会话私有 0700 目录内的 Unix socket 请求 SPD 授权；订阅器在物理线程检查当前候选会话、三组就绪状态，并调用原有新鲜度及 0.15 rad 差值授权门。发布端只有收到匹配且未过期的成功回执，且本地标定／输入仍有效时才开始跟随。Hold 立即冻结源目标并请求撤权，不重置姿态。

授权 IPC 由独立工作线程处理；编号、会话、有效期和源端操作代次隔离迟到回复。状态轮询不启动运动；撤权或控制通道不可用时停止跟随，恢复后仍需显式操作。订阅器仅在指定 `--control-socket` 时开放该接口，H5 和普通订阅入口不改变。运动目标仍通过 ROS/DDS/Zenoh，不经本机授权通道发送。失效组锁存；双臂共用 ready 位、左右手分别处理，仍有效组的局部跟随语义保留。订阅侧原有 100 ms 保持门限不放宽。

编排器只持有本次子进程组及新建的 ADB 转发；完全匹配的已有转发可复用，但不获取其清理所有权。窗口关闭或 Ctrl+C 清理本次资源。与 H5 编排器共享互斥锁，拒绝抢占 7447 端口。进程就绪、跟踪有效和本地控制授权是三个不同状态。

## 保留的 PICO 跟踪入口

```text
PICO_2 APK → ADB/TCP → pico_hand_tracking
                         ↓
                 spd_vr.pico2_bridge
                         ↓
              Zenoh spd/vr/v1/tracking
                    ↙             ↘
               arm_ik            viewer
                    ↘             ↑
                 机械臂目标 + 手部重定向
                         ↓
               Tianji/Wuji MuJoCo 模型
```

输入包只负责原始协议、有效标记与传输。统一 tracking 协议隔离设备差异；机器人模型、关节顺序和控制语义由 spd-vr 管理。场景状态及机器人实际状态必须由仿真产生，不能用操作者人体位姿替代。

`packages/spd-vr/spd_vr/` 内按现有职责组织：

| 模块 | 职责 |
|---|---|
| `pico2_bridge`, `wire`, `zenoh_transport` | 跟踪、控制和状态消息 |
| `arm_ik`, `qp_arm`, `retarget_pair`, `alignment` | 机械臂求解、灵巧手映射与操作者对齐 |
| `model_compiler`, `manifest` | URDF 编译、碰撞资产和关节契约 |
| `simulator`, `viewer`, `camera` | 物理状态推进、查看和相机数据 |
| `episode`, `recorder` | episode 生命周期与 schema-v1 HDF5 输出 |
| `replay` | schema-v1 文件校验与只读检查；时间对齐由训练端负责 |

`packages/spd-envs/spd_envs/` 独立管理环境：`registry` 注册六类场景、Table 2 的 17 个任务及 A.4 的 `jenga/playing`，`scene_builder` 生成带接触几何的物体和随机参数，`model_scene` 将场景合入调用者提供的机器人 MJCF，`validate` 检查重置。它只依赖 NumPy 和 MuJoCo，不依赖 PICO、Zenoh 或 `spd_vr`。未报告的 A.4 时长和数据集统计为 `None`，不伪造 Table 2 数据。

依赖方向为 `spd-vr → spd-envs`。原 `spd_vr.scenes` 已迁移为 `spd_envs`，不保留转发层。机器人 `generated/` 与 `config/` 留在 `spd-vr`；原始机器人资产独立放在根目录 `assets/`。环境包不硬编码 Tianji/Wuji 模型路径。

五个 Figure 4/A.4 场景可通过 `pixi run spd-scene --task SCENE/TASK --seed N` 独立查看，也可通过 `spd-teleop-ros --task SCENE/TASK --seed N` 加载到既有关节订阅链。`PlantController` 先验证基础机器人资产，再合入场景；不覆盖基础 MJCF，不因场景 free joints 改变机器人索引。桌高 0.75 m 适配 Tianji 初始姿态，盘架、杯架和箱体固定，盘/杯/积木/瓶自由运动；杯体和把手使用非凸组合碰撞，不允许初始穿透。构造参数及 seed 写入 scene manifest，ROS episode 保存完整 task manifest。此功能是可交互物理场景，不包含自动策略、任务评分或原论文视觉资产的精确复刻。

## 论文目标与当前实现的区别

论文仿真 480 Hz，控制/流传输/记录 60 Hz，训练网格 30 Hz。频率是不同阶段的契约，不是三个名称相同的循环，也不是当前机器的性能保证。

目标采集流为：

```text
任务与随机化 → 仿真交互 → 轨迹与动作记录
                              ↓
                  长时间无接触片段裁剪
                              ↓
                  多视角回放渲染 + 分割
                              ↓
           按时间对齐的视觉 / 本体状态 / 动作数据
```

当前存在相关 Python 模块，但还不能把 `spd-teleop` 等同为完整论文采集入口：

1. PICO_2 预构建 APK 不接收 MuJoCo 场景；VR 闭环场景显示未完成。
2. 实时查看与 episode 录制模块需要完整联调，启动 Viewer 不代表开始录制。
3. 与论文六场景、离线批量渲染及数据增强的等价性尚未验收。
4. 实机后训练、策略训练和部署不属于本次项目整理。

## schema-v1 订阅与发布

核心观测为仿真/设备反馈形成的双臂 14 维 qpos、双手 40 维 qpos，以及启用相机的 RGB 帧。ROS Viewer 当前只提供仿真反馈；关节与完整 RGB 帧使用采集主机单调时间，以 episode 起点归零。写入线程负责 JPEG 编码和 HDF5 发布，保存/丢弃由录制协调线程等待，不在控制回调中同步等待。

录制中的文件为 `episode_XXXXXX.partial.h5`。收到成功确认后完成字段、维度、有限值、时间戳和 JPEG 解码校验，通过后发布为 `.h5`；中断或校验失败保留 partial 文件。当前录制器还要求 `observations/commands`，保存实际应用目标及 session/sequence/UTC/ready/hold/物理应用时间；这是仿真命令扩展，并不符合 `docs/schema-v1.md` 排除命令的原始文件契约。文件不包含速度、深度、IMU、分割图或 30 Hz 训练副本。

## 数据与资源约定

- `assets/`：原始 URDF、网格与必须保留的机器人资产。
- `packages/spd-vr/generated/`：可加载的编译模型和 manifest；改模型后重新编译并验证。
- `data/`：采集输出与已有样本；不随代码清理删除。
- `docs/papers/`：研究依据；论文不作为可执行规范替代代码验证。
- 根目录 `pixi.toml` 和 `pixi.lock` 是唯一受维护运行环境；系统 ADB 和图形会话为外部前置条件。

此前移除的旧 ROS 2 采集、鱼眼、IMU、Odin、PXREA 和实机控制入口不恢复。新的 ROS 入口仅订阅外部关节目标驱动仿真，不发布实机控制命令。
