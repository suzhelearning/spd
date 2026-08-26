# TJ Arm PICO SPARK Headroom Feedforward Velocity QP

这是天玑双臂在 MuJoCo 中的 PICO 遥操工程。当前 `main` 的推荐算法和 Viewer 默认值均为：

```text
spark_upper_qpoases_headroom_feedforward_velocity_qp
```

该路径以 SPARK 两阶段 qpOASES IK 保持人形手臂构型，以 Headroom 余量感知前馈提高跟踪响应，
并通过 Velocity QP、连续性代价和 settled-hold 抑制关节限位附近及静止阶段的可见抖动。当前工程只驱动 MuJoCo，不连接真实机械臂。

## 构建与测试

要求 Linux x86-64 和 [Pixi](https://pixi.sh/)。以下命令在 `TJ_arm_control` 仓库根目录执行：

```bash
pixi install
pixi run configure
pixi run build
ctest --test-dir build --output-on-failure
```

完整测试当前为 81 项，包含配置、SPARK、Headroom、Velocity QP、PICO UDP、TJVR 录制以及 Viewer 集成测试。

## PICO 实时遥操

PICO 侧使用 `PICO_tracker` 的 `main` 分支，将占位符替换为本机仓库目录：

```bash
cd <PICO_tracker目录>
git switch main
pixi shell
./scripts/start_tianji_pico_teleop.sh
```

该脚本启动 PICO 驱动、corrected M0 骨架、骨架 Viewer 和 TJVR v4 UDP bridge。

天机侧在 `TJ_arm_control` 仓库根目录无参数启动当前主算法：

```bash
OMP_WAIT_POLICY=ACTIVE OMP_PROC_BIND=close OMP_PLACES=cores \
  ./build/tianji_qp_ik_viewer
```

### 默认值

| 项目 | 默认值 |
|---|---|
| 算法 | `spark_upper_qpoases_headroom_feedforward_velocity_qp` |
| 配置 | `config/qp_ik_pico_teleop.yaml` |
| 模型 | `models/marvin_m6_qp_pico_fast.xml` |
| 控制层 | `velocity`，200 Hz |
| 控制状态源 | `model_reference` |
| PICO UDP | `127.0.0.1:15000` |
| 骨架 Overlay | 启用 |

没有 PICO 数据时控制器进入 stale/hold，不回退到脚本轨迹。若端口 15000 已被占用，请先关闭其他 Viewer 或显式指定另一端口。

## 主算法

```text
PICO TJVR v4
→ SPARK 两阶段 qpOASES IK
→ 关节速度与笛卡尔前馈
→ Headroom 余量衰减
→ Velocity QP
→ 输出连续性与 settled-hold
→ MuJoCo model_reference
```

SPARK 使用 corrected 肩、肘、腕和掌心数据，按机器人骨长求解连续的双臂关节构型。前馈路径从 SPARK 关节参考和掌心运动提取速度信息，减少纯反馈路径的相位滞后。

Headroom 根据关节位置、速度、加速度和 jerk 可用余量调节前馈强度：余量充足时保留
跟踪响应；接近硬约束时快速衰减，避免前馈持续把关节推向限位。QP 最终仍强制执行
关节动态边界、上臂外侧安全边界和笛卡尔任务约束。

输出连续性项抑制 `qdot` 和 jerk 方向突变。手部已静止或约束余量过低时，
stationary-reference hold 与 settled-hold 停止追逐不可实现的骨架抖动。恢复运动后，
前馈按配置的恢复时间平滑重新进入。

当前关键配置位于 `config/qp_ik_pico_teleop.yaml` 的：

```text
spark_feedforward_velocity_qp
spark_headroom_feedforward_velocity_qp
spark_upper_qpoases
hierarchical_qp
joint_limits
upper_arm_outward
```

## 完整显式启动

需要固定所有参数并保存遥测时：

```bash
mkdir -p benchmark_results/pico_live/traces
OMP_WAIT_POLICY=ACTIVE OMP_PROC_BIND=close OMP_PLACES=cores \
  ./build/tianji_qp_ik_viewer \
  --config config/qp_ik_pico_teleop.yaml \
  --model models/marvin_m6_qp_pico_fast.xml \
  --pico-teleop \
  --pico-skeleton-overlay \
  --pico-bind 127.0.0.1 \
  --pico-port 15000 \
  --control-level velocity \
  --algorithm spark_upper_qpoases_headroom_feedforward_velocity_qp \
  --model-state-only \
  --pico-record benchmark_results/pico_live/traces/output_continuity_retest.tjvr \
  --telemetry benchmark_results/pico_live/headroom_live.csv \
  --joint-telemetry benchmark_results/pico_live/headroom_live_joints.csv
```

遥测包含末端误差、QP slack、任务缩放、关节状态、有效动态边界、Headroom、前馈状态、
控制周期分位数和 PICO 输入统计。CSV 写线程与 200 Hz 控制线程隔离。TJVR 录制器不会
覆盖已有文件；重复测试时请保存旧轨迹或为 `--pico-record` 指定新文件名。

## MuJoCo TJVR 可视化回放

TJVR 实测轨迹是本地测试资产，不随仓库发布。可以通过上一节的 `--pico-record` 自行
录制，也可以由测试人员放到约定位置。两个终端均从 `TJ_arm_control` 仓库根目录启动。

本节回放身份固定为：

- 回放算法：`spark_upper_qpoases_headroom_feedforward_velocity_qp`
- 回放控制层：`velocity`
- 回放模型：`models/marvin_m6_qp_pico_fast.xml`
- 回放状态源：`model_reference`

终端 1：

```bash
mkdir -p benchmark_results/pico_live
OMP_WAIT_POLICY=ACTIVE OMP_PROC_BIND=close OMP_PLACES=cores \
  ./build/tianji_qp_ik_viewer \
  --duration 120 \
  --telemetry benchmark_results/pico_live/pico_headroom_main_replay.csv \
  --joint-telemetry benchmark_results/pico_live/pico_headroom_main_replay_joints.csv
```

看到 `pico_udp_bind=127.0.0.1:15000` 后，在终端 2 运行：

```bash
TRACE_FILE=benchmark_results/pico_live/traces/output_continuity_retest.tjvr
test -f "$TRACE_FILE" || { echo "missing TJVR trace: $TRACE_FILE" >&2; exit 1; }
python3 <vr_data目录>/tools/replay_pico_udp_trace.py \
  --input "$TRACE_FILE" \
  --host 127.0.0.1 \
  --port 15000 \
  --lead 0.5
```

当前本地基准轨迹包含 9442 帧，时长 106.887 秒；使用其他录制文件时以回放工具的实际
统计为准。该基准成功时回放工具输出 `replayed_frames=9442`。两份回放 CSV 位于
`benchmark_results/pico_live/`，便于与实测结果统一分析。

## 常用操作

| 输入 | 功能 |
|---|---|
| `P` | 启用或关闭 PICO 输入 |
| `G` | 切换 PICO 臂角参考模式 |
| `F2` | 显示或隐藏七轴关节曲线 |
| `F3` | 切换 `q`、`dq`、`ddq`、`jerk` |
| `F4` | 切换并锁定左/右臂曲线 |
| `F5` | 曲线恢复跟随当前选择臂 |
| `Space` | 暂停或恢复 |
| `F1` / `Esc` | 显示帮助 / 退出 |

## 验证默认路径

```bash
./build/tianji_qp_ik_viewer --headless --duration 1
```

启动摘要应包含：

```text
viewer_config=config/qp_ik_pico_teleop.yaml
control_state_source=model_reference
pico_udp_bind=127.0.0.1:15000
algorithm=spark_upper_qpoases_headroom_feedforward_velocity_qp
control_level=velocity
```

## 安全与范围

- 当前 Viewer 只驱动 MuJoCo，不包含真实机械臂 SDK、力矩控制或硬件急停。
- 当前速度、加速度、jerk 和制动参数是运动学验证值，不是实机安全认证参数。
- 工程不包含完整碰撞/自碰撞、双臂 14 自由度耦合优化或执行器动力学。
- 单臂参考无效或 QP 不安全时只冻结故障臂；健康臂继续运行。
- 非 PICO 运行需显式使用 `--no-pico-teleop --no-pico-skeleton-overlay` 并指定对应
  `--config`、`--model`、`--control-level` 和 `--algorithm`。

## 历史算法

Hierarchical QP、null-space DLS、Cartesian OTG velocity/acceleration、旧 SPARK A/B
模式、数学公式、基准指令和完整旧键位说明已迁移至
[历史算法与回归入口](docs/legacy_algorithms.md)。这些路径继续保留用于回归，但不是
当前 `main` 的推荐默认算法。
