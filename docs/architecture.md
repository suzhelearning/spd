# SPD WebXR 仿真采集架构

SPD 负责本地裸手仿真采集、原始 schema-v2 轨迹校验和网页回放；项目只控制仿真，不控制实机。已完成轨迹由相邻 [data_process](../../data_process/README.md) 只读后处理，不是 SPD 的运行时模块。

启动参数、按键和操作流程见 [README](../README.md)；TCP、WebXR、管道和可选 DDS 通信及话题的权威说明见 [Pipeline.md](../Pipeline.md)；HDF5 文件字段、兼容性和校验规则见 [schema-v2.md](schema-v2.md)。本文说明模块和运行时边界，不重复使用步骤。

## 1. 目标与复现边界

本仓库以论文 [Pre-training Visual Dexterity in Simulation](papers/2608.15917v1.pdf) §3.1、§3.3 和附录 A.1 为任务与节奏参考，保留 Tianji 7×2 双臂与 Wuji Hand2 20×2 灵巧手。它采用头显本地 WebXR 显示、工作站唯一物理状态、人工示范和在线原始轨迹的分工。

论文机器人、原始场景资产和精确动作编码不在当前适配范围。`TaskSpec` 的参考时长由 Table 2 的平均时长计算，并非作者的原始配置。本仓库不提供训练准备、接触裁剪、30 Hz 重采样、训练文件导出或离线渲染命令；相机最终标定与真实 Quest 佩戴验收仍未完成，不能据此宣称论文结果、硬实时或吞吐复现。

## 2. 数据流与所有权

WebXR 模式的数据流如下：

```text
Quest Browser：WebXR viewer／head／25-joint hands
       │ tracking JSON（最多 60 Hz；源时间、sequence、generation）
       ▼
WebXRBridge：网络线程，验证／最新输入槽／场景确认
       │ PicoRawFrame（FLU，保留每关节有效性）
       ▼
TeleopSession：单 worker，相对参考＋DLS/Ruckig＋Hand2（60 Hz）
       │ 带 generation 和质量标志的不可变参考快照
       ▼
唯一物理 owner：CollectionControl → C++ executor → Physics（480 Hz）
       ├── 60 Hz 状态／实际目标／接触 → HDF5 writer
       ├── 60 Hz owned body poses → WebXRBridge → 二进制 SPDS → Quest
       └── 可选桌面 SplitViewRenderer：私有模型副本，只显示不推进物理
```

原生 `spd_executor` 嵌入 CPython；`run_loop` 协调本地输入、推进 MuJoCo，并每 8 个物理步在与采集一致的相位应用最新关节目标。WebXR、状态写入和渲染线程都不直接改变活的 MuJoCo 状态，也没有第二个裸手 ROS 发布器。

WebXR 入口的 `TeleopSession` 以 `start_receiver=False` 启动，由桥接的 `feed()` 接收已验证帧；未启用 `--webxr-port` 的本地头显入口仍使用 TCP 接收器。采集 DLS worker 的积分以及 Home／制动步长为 1/60 秒，辅助非采集 worker 保留其原周期。480／60 Hz 是仿真调度目标，不提供硬实时保证；Python 回调、GIL 与接触开销仍在热路径上。

## 3. 模块边界

| 路径 | 职责 |
| --- | --- |
| `src/spd_native/` | C++ `spd_executor`、目标校验与授权、可选 DDS 邮箱、原生 `Physics`/`run_loop`、检查点、接触及材料求解。 |
| `src/webxr/` | 本地 WebXR 场景导出、浏览器输入协议、同源服务与状态传输；仅通过回调接入遥操作。 |
| `src/pico2_hands/` | 本地 TCP 输入、相对参考绑定、每手重接入和 `TeleopSession` worker；只产出控制快照。 |
| `src/teleop_native/`、`tools/wuji_hand_native/` | 双臂 DLS/Ruckig、Viewer 辅助和独立 Hand2 原生求解环境。 |
| `packages/wuji-retargeting/` | 固定引用的 `mujoco-sim` 和 `wuji-description` 子模块；不构成运行时控制入口。 |
| `src/simulation/` | `CollectionControl`、模型准备、场景/Home 生命周期和在线 Viewer。 |
| `src/data_collector/` | 采集配置、文件生命周期、完整物理轨迹、接触累计和独立恢复校验。 |
| `src/description/`、`src/cameras/` | 模型编译、manifest、资源定位和机器人局部相机配置。 |
| `src/interfaces/` | Python wire 工具、终端输入及 ROS 消息定义。 |
| `src/environments/spd_envs/` | 独立环境包：任务注册、随机化、场景生成与重置检查。 |
| `src/tianji_wuji2/tianji_wuji2/{assets,generated}/` | 原始 URDF/网格/碰撞资产，以及可加载的编译模型和 manifest。 |
| `config/`、`data/` | 采集/相机配置与采集产物；`data/` 不是构建缓存，不应由代码清理删除。 |

Python 运行包直接位于 `src/`，发行包名为 `spd`。依赖方向是 `spd → spd_envs`：环境包不依赖 ROS 或遥操作算法，也不硬编码机器人路径。资源由 `description/model_builder.py` 的定位函数解析，不能依赖调用者当前目录。

`pixi.toml` 和 `pixi.lock` 定义受维护运行环境。ROS 原生组件安装在 `.ros/`，双臂／Viewer／Hand2 原生组件安装在 `.teleop/`；两套 ABI 不混用。`packages/wuji-retargeting/` 下的 `mujoco-sim` 和 `wuji-description` 是由 `.gitmodules` 固定的随仓库分发子模块，用于固定参考源码和资产，而不是第二个控制端。`config/input_provenance/` 只保存输入契约指纹所需的冻结源文件，不参与 Python 导入或原生编译。

## 4. 进程、线程与控制权属

一个前台采集主进程拥有控制、物理和文件生命周期；双臂与 Hand2 数值求解仍是该主进程管理的独立 worker 子进程。这不构成第二套控制终端或另一个物理 owner。

| Owner | 拥有的职责 | 明确不做 |
| --- | --- | --- |
| `spd_executor` 中的物理线程 / `CollectionControl` | 解释输入、绑定、授权、目标应用、MuJoCo 步进、冻结/恢复、采集和场景切换。 | 不把 MuJoCo 写权限交给输入、DDS 回调或渲染线程。 |
| `TeleopSession` 的单个 worker | 处理本地 TCP 或桥接送入的帧，并与 DLS/Hand2 worker 交互；发布带 `generation`/`sequence` 的不可变最新快照。 | 不写 MuJoCo 状态；重绑定或换场景前的结果不能重新授权。 |
| `WebXRBridge` 网络线程 | 校验同源 WebSocket、保留最新输入、提供静态场景和复制后的状态；将帧与按键投入既有回调。 | 不触碰 live model/data，也不直接授权运动。 |
| 可选 DDS 回调 | 校验消息并原子更新最新候选邮箱。 | 不推进物理；物理 owner 消费时会再次检查新鲜度和授权。 |
| `SplitViewRenderer` 渲染线程 | 自己的模型克隆、`MjData`、场景/相机和 GLFW/GL 资源；从短锁交换的最新状态包绘制。 | 不调用 `mj_step`，不阻塞采集等待 GPU，也不修改物理模型。 |
| `CollectionSession` 协调 worker 与单个 HDF5 写线程 | 文件准备、保存/丢弃及有界整帧队列。 | 不在 ROS 回调中同步等待写盘，不静默丢帧或覆盖旧样本。 |

WebXR、本地 TCP 裸手和可选外部 DDS 是互斥输入路由：WebXR 关闭 TCP 接收器，本地裸手模式禁用 DDS 目标订阅，外部 DDS 模式没有本地腕部重绑定和每手平滑重接入保证。话题、QoS、wire 字段和状态发布条件以 [Pipeline.md](../Pipeline.md) 为准；状态观察不是第二个操作者入口。有限但越限的目标按合法范围饱和，非有限值、失效输入或 worker 故障 fail-closed。headless 不创建桌面渲染后端。

## 5. WebXR 入口、场景和输入

`pixi run spd-webxr --height-m HEIGHT` 经 `bash/run_webxr.sh` 和 `bash/start_spd_sim.sh` 启动 `.ros/install/lib/spd_native/spd_executor`，再进入 `simulation.ros_viewer`。USB 使用 `adb reverse tcp:8080 tcp:8080`，头显访问 `http://localhost:8080`；多设备可用 `ANDROID_SERIAL` 选择，只有桌面检查时才显式设置 `SPD_WEBXR_NO_ADB=1`。`--headless` 只关闭工作站窗口，不关闭 Quest 的 WebXR 场景。

服务默认绑定 `127.0.0.1`，localhost 满足 WebXR 的可信上下文要求。不关闭浏览器安全策略，不开放通配绑定；非 localhost 远程地址需要独立的可信 HTTPS/WSS 部署、Origin/Host 配置和认证，本仓库不提供自动证书或远程代理。Three.js 0.180.0 及其 MIT 许可固定在 `src/webxr/static/vendor/three`，不依赖运行时 CDN。一次只允许一个 WebXR 控制 socket，第二个页面得到明确冲突，避免双写。

场景由物理 owner 通过 `set_scene()` 导出为不可变 generation。浏览器收到 `scene` 和 `/scene?generation=N` 后加载场景、发送 `ready`，再完成 nonce 时钟握手才可提交跟踪或按键；generation 不匹配的旧数据不会被消费。`/scene` 返回 version 1、generation、task、bodies、meshes、materials、textures、geoms 和 view；它支持 gzip，bootstrap 上限为 256 MiB，每张纹理上限为 16 M 像素。geometry 相对自己的 body，body pose 是绝对世界坐标，客户端不得再次叠加父子变换。天空盒和 MuJoCo 灯光不复制到浏览器，浏览器使用明确的显示光照；它不是训练渲染器。

坐标变换为：

```text
B: XR → FLU       (x,y,z) → (-z,-x,y)
C = B^-1: FLU → Three (x,y,z) → (-y,z,-x)
```

`view` 是 XR 跟踪原点到世界的固定刚体变换，由机器人肩部位置和操作者身高初始化，不随头部逐帧重置。客户端世界根使用 `C`，XR camera rig 使用 `C*view`；传回的位置为 `view.position + view.rotation*pXR`，姿态为 `Rview*RXR*C`。默认 `Rview=B` 时，中立头部得到单位姿态和 `+X` forward。

每帧 JSON 含 generation、严格递增的 sequence、浏览器 `time_ms`、head pose 或 null，以及左右各 25 个 pose 或 null。桥接把 XR wrist 复制到已有 26 点 PICO 结构中未使用的 palm 槽，保留每个关节独立的有效性；缺失腕、指尖或头部不会伪造有效零姿态。nonce 往返上限为 250 ms，源年龄上限为 150 ms；原始源时间与主机接收时间分离，晚到包的接收时刻不会伪装成采集时刻。陈旧包、重复 sequence、回退时间和非法四元数被拒绝，输入 JSON 限 48 KiB、限 240 消息/秒，网络发送和关闭均有界。45 ms 输入新鲜度和 120 ms 持续失跟踪预算仍由控制层执行。Host/Origin 同源检查不替代远程用户认证，因此默认仅支持 localhost＋USB。

## 6. 场景、物理与材质

场景按 seed 随机生成并把实际参数写入 manifest；暂停、恢复和回退不重采样。桌面上表面高度均匀采样于 `0.70–0.80 m`，近侧桌沿距机器人底座原点的 `+X` 距离均匀采样于 `0.10–0.30 m`。物体和固定支架随桌面定位，桌腿伸缩而脚垫保持落地；这些范围不是全姿态可达或避碰保证。

普通任务物体通过 free joint、重力、真实碰撞和摩擦运动，不直接写 `qpos` 播放或焊死。盘架、杯架、箱体和柜体固定；抽屉是 `0–250 mm` 的被动有界 slide joint，保留真实底板、侧壁、前板和把手。搁板和滑动底板保留 `1 mm` 运行间隙，避免共面接触导致的数值摩擦锁死，而不关闭接触。场景不是自动策略、任务评分或论文 CAD 的精确复刻。

外观与碰撞代理分离：外观 geom 为 group 2、质量为零且不参与接触；group 3 的碰撞代理承担质量、惯量和接触。Viewer/相机隐藏 group 3 只影响显示，不关闭物理。机器人保留 URDF 的质量、质心和惯量；物体质量、几何和摩擦是工程设定，不代表实物标定或论文物理等价。当前 `Link5_L–Link7_L` 与 `Link5_R–Link7_R` 是临时碰撞排除，会同时忽略真实碰撞，不能当作实机避碰或安全认证。

完整机器人、双臂投影、独立场景和初始化接触检查统一使用 `480 Hz`、`implicitfast`、`cone="elliptic"` 与 `noslip_iterations="1"`。接触密集场景不保证墙钟实时。ABC 瓶子只按随环境包提供的文件适配，公开再分发权利尚未确认。

### 执行器增益

手指使用 `_HAND_GAIN_SCALE=10.0`：左右手每根手指的实际 `Kp=(8.0, 2.5, 4.0, 2.0) N·m/rad`，执行器 `kv/Kd=(0.25, 0.15, 0.12, 0.08) N·m·s/rad`，顺序为 CMC/MCP 屈伸、CMC/MCP 外展、MCP/PIP、IP/DIP。`Kp` 与 `Kd` 均同乘 10，不采用 `sqrt(10)` 阻尼缩放或逐关节附加倍率；倍率只作用于 40 个手指关节，手指被动 `joint damping=0`，不作用于双臂，也不改变力矩/控制限幅、碰撞、摩擦或惯量。

左右臂 `Joint1` 至 `Joint7` 均直接使用 `Kp=(802, 802, 802, 602, 321, 321, 321) N·m/rad` 和 `kv/Kd=(67, 67, 41, 41, 11, 11, 11) N·m·s/rad`，不做角度换算、手指倍率缩放或 `dampratio` 推导；双臂被动关节阻尼为 `0.1`。编译模型、双臂投影、校准表和 manifest 必须一致重建；运行中的模型不热改，已记录 MJB 保留记录时的增益。仿真增益不是硬件 MIT 参数，且不保证受接触约束的关节必能到达命令角度。

### 材料与接触

材料策略使用对称的有效滑动摩擦系数，但数值不是一张不可变的全局表：源码 checkout 的 `config/material_friction.yaml` 是完整 26 个批准材料对的初始化模板；每个 `scene/task` 使用独立、完整的 `config/task_material_friction/<scene>/<task>.yaml` 档案。新档案仅在缺失时从模板原子创建，已有档案直接加载，损坏或读取失败会失败而不会回退到模板。seed 不参与档案路径。`SPD_TASK_MATERIAL_FRICTION_DIR` 可覆盖档案根，`SPD_MATERIAL_FRICTION_CONFIG` 只选择初始化模板及其同级档案根；已安装 wheel 使用可写的用户配置目录而不改写包资源。

构建时持有独立系数和路径快照，并把 `physical_materials.task_profile`、来源与完整矩阵写入 manifest；独立场景、初始化接触检查和机器人合并场景使用同一快照，后续文件或环境变量变化不使运行场景漂移。该策略不重算质量、惯量或碰撞几何，不改变配色、随机抽样、原 geom 摩擦、碰撞过滤、`condim`、法向 `solref/solimp`、`margin/gap`、扭转/滚动摩擦、关节或执行器参数，也不增加显式 geom 配对。

木质积木、字母块、多米诺、柜体和抽屉归为木；杯、瓶和箱归为 PE；盘和马克杯按朝下底面区分未上釉陶瓷；盘架/杯架支撑为裸铁；桌面和地面为涤纶织物。整个 distal 指尖、指腹和掌侧使用硅胶，掌侧以 body 局部外法线左手 `Y<0`、右手 `Y>0` 判定，背侧、裸露外壳、安装件和手臂保持旧摩擦。材料与表面区域只解释既有接触：不分割或替换 collision mesh，整体旋转和 geom 顺序不改变分类。批准对以外的 `-1` 矩阵值（包括硅胶—硅胶和织物—织物）保留原 geom 的 MuJoCo 混合规则。所有系数均为有限、非负、无人为上限的工程有效值，不是厂商硅胶数据或实测动摩擦；manifest 对其标记 `engineering_choice_not_measured`，同时保留木—木和 PE—PE 的参考来源。

MJB 的 custom numeric `spd_material_friction` 保存版本 2 的 row-major 8×8 矩阵，`geom_user` 字段 2/3 保存材料 ID 与表面区域。物理推进、续跑或材料求解力诊断必须使用匹配版本 MuJoCo 的 `_spd_native.material_step(model, data)` 或 `material_forward(model, data)`；原生 `Physics` 使用同一实现。裸 `mj_step`/`mj_forward` 不解释这些字段，不能等价推进带策略的模型。该运行时只支持 Euler、implicit 和 implicitfast，拒绝带策略的 RK4 或启用 EFM 的模型；只读回放只恢复记录姿态并使用普通 `mj_forward`，历史无策略 MJB 继续走普通路径。

桌面 `MaterialEditor` 在物理 owner 上运行：`M` 打开/取消，方向键选择与调整，`Enter` 应用并原子保存当前任务档案。它只列出当前可碰撞 geom 涉及的批准对，活动 episode（包括暂停）只读；面板打开时隔离 `r/s/d`。应用先在原位更新 numeric、刷新材料接触并准备新的 `TrajectorySource`，只有任务档案原子保存成功才发布新的 manifest 和 MJB 快照；失败会恢复原 numeric、完整 `MjData` 和旧快照，不重采样布局或推进物理。

## 7. 采集控制、轨迹与恢复

`CollectionControl` 是单键状态机：首次和新任务仅 `r` 发起绑定并开段；运动中 `r` 更新检查点、`s` 人工暂停、`d` 回退后自动重新绑定续采；人工暂停中 `s` 重新绑定继续、`r` 保存整条、`d` 丢弃整条。本地头、双腕持续失跟踪会自动冻结，稳定输入触发重新绑定续采，不回退、不更新人工检查点；外部 DDS 失效则由发布端对齐后按 `r` 重新接手。终端、桌面窗口和 WebXR 页面都只将原始 `r/s/d/q` 投递给这一个物理线程，状态观察不提供远程 Trigger 控制。

`CollectionControl` 驱动 `CollectionSession` 记录完整物理状态和手—物接触：物理为 480 Hz、每 8 个物理步采样一次，轨迹为 60 Hz。单一有界写队列只传递自有整帧；漏 tick、非有限状态、队列溢出或写盘错误都保留 `.partial.h5`，不会补帧、静默丢帧或覆盖样本。只有文件、模型、元数据、时钟和状态投影都通过校验才发布完整 `.h5`；`complete` 与操作者确认的 `success` 是不同概念。

每段内嵌可独立恢复的 MJB、模型/场景元数据和资产溯源。`TrajectorySource` 同时记录完整场景状态、任务物体世界位姿、手—物接触、严格的物理 tick／仿真时间／主机单调时间，以及存在时实际已应用的 `robot_target` 和 `action_definition`；目标在完成物理步后记录，与该步状态对应，不是未应用网络候选或未来实测 qpos。模型与 metadata 有哈希，源文件保持只读，失败的 partial 不会冒充 complete。完整字段、恢复/质量标记和校验规则以 [schema-v2.md](schema-v2.md) 为准。

每次接受 start 均以本机开段时间生成 `YYYYMMDD/episode_YYYYMMDD_HHMMSS_ffffff.partial.h5`；完成并校验后发布同名 `.h5`。跨午夜不拆当前段，下一段重新选择日期；同日 `dataset_config.json` 只约束模型无关的 schema 契约。输出优先级为 `--output`、`SPD_EPISODE_OUTPUT`、采集配置的 `data_dir`。回退只保留选中的前缀和续采，主机单调时间不倒退；`recovery_transition` 和 `control_flags` 与轨迹同事务写入或裁剪，重新接手本身不生成 `rewind`。

任务注册为六类 18 项：17 项来自论文 Table 2，另加 Jenga playing。物体资产、位置、质量、摩擦、桌高和桌距的随机结果都写入 manifest；任务只提示目标，成功由操作者在人工暂停后按 `r` 显式确认，没有自动任务策略或评分。`bottles/toss_in_bin` 每条生成一个自由运动瓶子和一个收纳箱，保留瓶子的种子化位置/姿态、六种资产型号与半径/高度随机采样；箱体最终世界 X 坐标采样于 `0.40–0.60 m`，不会因桌距平移而越界。`--repeat-task` 固定任务类型但每段仍使用新 seed 与新布局；暂停和回退不重采样，保存或丢弃完成后才生成新的 Home 场景并清空授权。

`replay_episode` 从内嵌 MJB 在独立 `MjData` 中逐帧恢复并普通前向计算，验证机器人投影和物体位姿；它不调用 `mj_step`、不发送目标、不渲染，也不是能重启原控制循环的检查点。SPD 生成、校验并通过 `spd-web` 网页回放原始 schema-v2 轨迹，不在线保存 RGB、完整执行器 `ctrl`、动作或训练文件。

## 8. 后处理交接

SPD 只生成和校验原始 schema-v2 轨迹。离线恢复成图像、render schema、训练读取和视觉增强已迁至相邻 [data_process](../../data_process/README.md)，它通过显式本地依赖复用 SPD 的 `cameras.camera`、`data_collector.recorder` 与 `data_collector.trajectory`，不复制协议，也不向 SPD 添加反向依赖。详见 [data_process 架构](../../data_process/docs/architecture.md)、[render schema 与训练读取规范](../../data_process/docs/render-schema-v2.md) 和 [VLA 规范](../../data_process/docs/schema-vla.md)。本仓库没有 `src/offline_rendering`、`src/training_data`、render Pixi 环境、`spd-render` 或 `spd-augment` 命令。

## 9. 研究与技术边界

- 模拟接触、随机场景、限位和控制门不构成实机避碰、硬件安全或人体安全认证。
- 名义仿真频率、WebSocket/TCP/DDS 传输都不承诺延迟、丢包恢复或墙钟实时性能；DDS domain 也不是安全边界。
- 场景物理、材质与增益是受控工程近似，不是抓取成功率、实物材料标定、论文等价性或任意突变目标/接触负载稳定性的证明。
- 真实头显完整操作、最终相机标定和吞吐尚待验收。PICO v1 没有显式 tracking-origin epoch；连接、源时钟异常和大幅姿态跳变可撤销参考，但同一连接的小幅坐标重置无法可靠地区分为正常运动或重定位。
- 在线采集不提供第二套远程 Trigger 控制、离线训练渲染器或自动任务成功评分；后处理边界以 data_process、README 和 Pipeline 为准。

**未来计划：CONTROL-TYPE 辅助，尚未实现。** approach／alignment／grasp／insertion（接近／对齐／抓取／插入）辅助必须作为独立控制类型设计和标注，不能与纯人工动作混标。当前 Viewer 只有任务、状态、检查点和手指等待提示，不包含这些辅助策略。
