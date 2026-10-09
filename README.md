# SPD：裸手仿真采集与网页回放

使用 Quest／PICO 控制 MuJoCo 中的 Tianji 双臂与 Wuji Hand 2，采集完整场景物理轨迹。**只控制仿真，不控制实机。** 一个入口统一管理头显输入、求解、物理和采集；无需另开 ROS 控制终端。

| 文档 | 内容 |
| --- | --- |
| [Pipeline.md](Pipeline.md) | 数据流、脚本、TCP／管道／ROS 话题及消息字段 |
| [架构说明](docs/architecture.md) | 模块职责、线程权属、物理材质、模型参数与限制 |
| [HDF5 Schema v2](docs/schema-v2.md) | 原始轨迹字段、时钟、接触、恢复与校验契约 |

## 1. 安装与头显准备

支持 Linux x86-64，使用 Pixi 锁定 Python 3.12、MuJoCo 3.12 和 ROS 2 Jazzy。图形模式需要显示环境及系统 `fonts-noto-cjk` 字体；`--headless` 不创建窗口。在线运行依赖本工作区资源，不能只复制 Python wheel 部署。

在项目根目录执行：

```bash
git lfs install --local
git lfs pull --include="apps/pico/pico_hand_tracking_adb.apk"
pixi install --locked
pixi run --locked spd-teleop-build

adb devices -l
# Quest：开启开发者模式并授权 USB 调试
adb install -r apps/quest/quest3s_hand_tracking.apk
# PICO 使用裸手跟踪 APK，二选一安装
# adb install -r apps/pico/pico_hand_tracking_adb.apk
```

在头显中打开对应应用，启用手部跟踪并授予权限。`adb devices -l` 必须显示 `device`；多设备时设置 `ANDROID_SERIAL`。诊断接收器和采集入口不能同时占用头显输入。

- `unauthorized`：在头显内接受 USB 调试授权。
- `no permissions`：主机 USB 权限不足，不是头显授权问题；不要用 `sudo` 启动采集。
- Quest 3S 可使用以下 udev 规则；当前用户需属于 `plugdev` 组，设置后拔插 USB：

```bash
printf '%s\n' 'SUBSYSTEM=="usb", ATTR{idVendor}=="2833", ATTR{idProduct}=="5013", MODE="0660", GROUP="plugdev", TAG+="uaccess"' | sudo tee /etc/udev/rules.d/70-spd-quest3s.rules
sudo udevadm control --reload-rules
```

可选输入诊断：`pixi run spd-quest-receive --print`。诊断结束后 Ctrl+C，再启动采集。头显共用 TCP `10002`，ADB 转发由本地后端检查／建立，协议见 [Pipeline](Pipeline.md)。

求解器、原生源码或依赖更新后重新运行 `spd-teleop-build`。它分环境构建双臂、Hand2 和仿真执行器；仅修改主仿真原生模块时可运行 `pixi run spd-native-build`。

## 2. 启动采集

一次只启动一个采集进程，不共享输出目录。将 `1.75` 换成操作者身高（米）。

```bash
# Quest：默认每条重新随机任务和布局
pixi run --locked spd-quest-teleop --height-m 1.75

# PICO：与 Quest 二选一
pixi run --locked spd-pico-teleop --height-m 1.75

# 固定任务类型，每条仍重采样布局
pixi run --locked spd-quest-teleop --height-m 1.75 \
  --task bottles/toss_in_bin --repeat-task
```

| 参数 | 含义 |
| --- | --- |
| `--task SCENE/TASK` | 指定首个任务 |
| `--repeat-task` | 必须与 `--task` 同用；后续保持该任务类型 |
| `--scene cups` | 仅限制首个任务的场景范围 |
| `--scene hardware_free` | 首次为无任务物体场景 |
| `--seed 0` | 复现随机选择序列 |
| `--table-distance 0.2` | 只覆盖首个场景的近侧桌沿距离，单位米 |
| `--headless` | 无图形运行，仍可通过终端按键操作 |
| `--output PATH` | 指定采集根目录 |
| `--collection-config PATH` | 指定采集配置 |
| `--max-frames N` | 限制每条轨迹帧数，0 为不限 |

默认保存或丢弃完成后，从完整任务目录重新抽取任务。两种任务模式均重新生成 seed、物体布局、桌高 `0.70–0.80 m` 与桌距 `0.10–0.30 m`；暂停和回退不重采样。新场景在 Home 等待接手，不自动录制。

### 任务目录

将下面的标识传给 `--task`。共 6 类、18 个任务；需要连续采同一类型时加 `--repeat-task`。

| 场景 | 任务标识 | 操作 |
| --- | --- | --- |
| 木质积木 | `jenga/hollow_tower` | 空心积木塔 |
| 木质积木 | `jenga/tower` | 搭建积木塔 |
| 木质积木 | `jenga/dominos` | 多米诺骨牌 |
| 木质积木 | `jenga/criss_cross` | 交错积木塔 |
| 木质积木 | `jenga/handover_lr` | 左手交给右手 |
| 木质积木 | `jenga/handover_rl` | 右手交给左手 |
| 木质积木 | `jenga/playing` | 抽取中间积木并放到塔顶 |
| 字母积木 | `spelling_blocks/spelling` | 按任务提示拼词 |
| 字母积木 | `spelling_blocks/sort_and_unload` | 拉开抽屉，分类并取出积木 |
| 字母积木 | `spelling_blocks/pyramid` | 字母积木金字塔 |
| 字母积木 | `spelling_blocks/vowel_consonant_sort` | 元音／辅音分类 |
| 马克杯 | `mugs/hang_mug` | 悬挂马克杯 |
| 餐盘 | `dishes/rack_dishes` | 餐盘入架 |
| 餐盘 | `dishes/plate_dishes` | 叠放餐盘 |
| 杯子 | `cups/pyramid` | 杯子金字塔 |
| 杯子 | `cups/stack_two_threes` | 两组三杯叠放 |
| 杯子 | `cups/unstack` | 拆分套叠杯 |
| 瓶子 | `bottles/toss_in_bin` | 单个自由运动瓶子投入收纳箱 |

## 3. 按键、暂停与恢复

面向前方，将双手放在舒适、稳定的腰间准备位置，使头部和双腕可跟踪，再按 `r`。系统将当前手腕位置／朝向绑定到机器人保留目标，绑定本身不引起运动，不要求前伸标定。“腰间”是操作者选择的姿势，不是腰部跟踪器测得的位置。

| 当前状态 | `r` | `s` | `d` |
| --- | --- | --- | --- |
| 待接手：首次、新任务、失跟踪后 | 绑定并开始／重新接手，不存检查点 | 无操作 | 无操作 |
| 运动／录制中 | 更新检查点，继续录制 | 人工暂停 | 回退检查点，裁掉失败后缀，重新绑定后自动续采 |
| 人工暂停中 | 保存整条，进入下一条 | 重新绑定并继续 | 丢弃整条，进入下一条 |

- 单键即时处理，无需回车。请点按，不要长按；终端可能收到系统重复字符。
- 运动中 `d`、人工暂停中 `s` 都会等待稳定输入后自动续采，无需再按 `r`。
- 失跟踪会冻结现场。恢复后的首次 `r` 只重新绑定当前目标，不回退、不裁剪，也不覆盖人工检查点。
- 左右手指独立保持与平滑重接入；单侧手指短时缺口不直接阻塞另一手和双臂。持续头／腕失效会暂停整个世界，求解故障停止控制。
- **保存／丢弃整条只在人工暂停中执行。** 保存失败不切场景；任务成功由操作者判断，没有自动评分。
- `q`、Ctrl+C 或窗口 Esc 退出，**不代替保存**；未完成段保留 `.partial.h5`。
- 检查点只属于当前进程和 episode，不能重启后恢复。

在线窗口左侧可旋转／平移／缩放，右侧为固定头部观察视角；鼠标只改变左侧观察相机，不改变采集相机配置。顶部显示任务、采集状态、帧数和检查点；暂停／接入时显示双手虚影。

## 4. 配置与采集文件

[config/collect_sim.yaml](config/collect_sim.yaml) 定义输出目录、固定 60 Hz 状态采样、有界写队列和帧数上限。输出目录优先级为 `--output` → `SPD_EPISODE_OUTPUT` → 配置 `data_dir`；配置中的相对路径以配置文件目录为基准。

```text
data/episodes/YYYYMMDD/
├── dataset_config.json
├── episode_<UUID>.partial.h5
└── episode_<UUID>.h5
```

日期在开段时固定，跨午夜不拆当前段。物理每秒 480 步，每 8 步记录一帧，保存全场景状态、54 维机器人实际状态、物体位姿、手物接触、恢复标签及内嵌模型。**不记录在线 RGB、执行器 ctrl 或 actions。** 频率是仿真时间契约，不是墙钟性能保证。

完整校验后才发布 `.h5`。队列溢出、漏 tick 或写入故障不静默丢帧；不完整文件不能通过改扩展名冒充成功。达到正数帧数上限时保存为 `success=false`，人工暂停中按 `r` 显式保存才标记成功。不同数据契约不混写同一日期目录。

将路径替换为实际已完成文件：

```bash
pixi run validate_episode 'data/episodes/YYYYMMDD/episode_<UUID>.h5'
pixi run replay_episode 'data/episodes/YYYYMMDD/episode_<UUID>.h5'
```

`replay_episode` 逐帧恢复并报告误差，不推进物理、不显示图像、不发控制目标。恢复要求与记录时精确相同的 MuJoCo 版本，不依赖原始 XML／网格目录；记录不能当作原控制循环的重启检查点。完整定义见 [Schema v2](docs/schema-v2.md)。

## 5. 网页回放与相机调整

```bash
pixi run --locked spd-web --directory data/episodes --port 8765
```

打开 `http://127.0.0.1:8765`，扫描**服务端目录**并选择轨迹。每个轨迹目录需要配套 `dataset_config.json`，模型要求匹配 MuJoCo 版本。网页直接读取原始轨迹，不要求离线渲染；播放、暂停、逐帧、拖动进度和循环均不修改源文件。空格播放／暂停，左右方向键逐帧；后台标签页自动暂停。

只调整相机时，使用已完成轨迹的内嵌模型：

```bash
pixi run --locked spd-web --scene /path/to/episode.h5 --port 8765
```

此模式显示模型编译后的 `qpos0`，不是轨迹第一帧，不读取轨迹序列。侧栏可调整三路相机局部位置和旋转，并下载 v2 YAML；网页修改不会自动覆盖配置或 HDF5，刷新前应下载保存。

[config/sim_cameras.yaml](config/sim_cameras.yaml) 指定相机安装：

- `parent`：机器人 body，不能为 world；默认 top 挂 `Link_Stand`，左右相机挂各自 wrist。
- `position`：相对 parent 的 `[x,y,z]`，单位米。
- `rpy_deg`：`[roll,pitch,yaw]`，单位度，旋转为 `Rz(yaw) Ry(pitch) Rx(roll)`；相机光轴为局部 `-Z`，上方为 `+Y`。

修改配置后需重启网页服务或重建采集会话，不热更新已有轨迹。配置仍为 provisional；WebGL 预览不是正式训练 RGB，也不是实测标定证明。

服务默认仅本机可访问。`--host 0.0.0.0` 仅用于可信局域网：**无身份认证，可读取服务端所选数据，不要暴露到公网。** Ctrl+C 停止服务。

## 6. 开发与外部输入

```bash
# 独立查看场景，不需要头显
pixi run spd-scene --task cups/pyramid --seed 0

# 编译到新的空目录，避免覆盖现有模型
pixi run spd-model --output /tmp/spd-model-check
pixi run spd-envs-check

# 原生构建完成后运行回归
pixi run spd-test
```

不带 `--height-m` 的 `pixi run spd-sim` 使用可选外部 DDS 模式。上游需自行提供兼容 JointCommand；本地裸手输入不会同时启用，外部输入也不提供本地手腕重绑定保证。默认 ROS domain 为 120。

**只有外部 DDS 模式发布 `/spd/collection/status`**，可用 `pixi run spd-collect-trigger --command status` 观察；本地裸手模式直接通过窗口／终端看状态。生产入口不提供远程采集 Trigger 服务。完整话题及 QoS 见 [Pipeline](Pipeline.md)。

## 7. 技术边界

- 物理材质和质量含工程近似，参数、来源和材料运行时要求见 [架构说明](docs/architecture.md)；不代表实物标定或硬件安全认证。
- 模型仍有两对临时连杆碰撞排除，不保证全姿态避碰。不要在采集中替换模型或资产。
- 真实头显完整操作、长期负载实时性及正式相机标定不能由合成输入验证推断。PICO v1 的同连接小幅跟踪原点变化也不能可靠区分正常运动。
- ABC 瓶子资产的公开再分发授权尚未确认，公开发布前须核实权利。
- 本项目不包含自动任务策略或成功评分，也不从状态文件伪造未记录的动作命令。
