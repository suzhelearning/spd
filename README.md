# SPD Simulation Collection — Tianji + Wuji Hand 2

本项目仅面向《Pre-training Visual Dexterity in Simulation》的仿真示范采集，目标机器人为双侧 **tianji_arm + wuji-hand2**，操作者输入为 **PICO_2**。论文原文见 [docs/papers/2608.15917v1.pdf](docs/papers/2608.15917v1.pdf)。不包含实机控制、ROS 2 传感器采集、脚部 IMU、鱼眼相机或 Odin。

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

`spd-envs` 管理 `jenga`、`spelling_blocks`、`mugs`、`dishes`、`cups`、`bottles` 六类程序化场景，共 17 个任务。运行 `pixi run spd-envs-check` 检查所有任务的随机重置。程序化资产并不意味着已经逐项复现论文的视觉与接触参数。

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

增加或调整场景时修改 `packages/spd-envs/spd_envs/`，机器人控制仍由 `spd-vr` 负责。当前这是环境构建接口，不是 Viewer 已支持运行时切换场景的声明。

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
