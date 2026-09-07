# 仿真采集架构

## 唯一目标

参考 [Pre-training Visual Dexterity in Simulation](papers/2608.15917v1.pdf) §3.1、附录 A.1，在 MuJoCo 内通过人类遥操作采集目标机器人本体的示范。机器人替换为 tianji_arm + wuji-hand2，跟踪输入替换为 PICO_2。实机控制与真实传感器融合不在范围内。

## 已有代码的数据路径

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
| `episode`, `recorder` | episode 状态、检查点和 HDF5 输出 |
| `replay`, `filter_contacts`, `align_30hz` | 回放、接触裁剪与时间对齐 |

`packages/spd-envs/spd_envs/` 独立管理环境：`registry` 注册六类场景与 17 个任务，`scene_builder` 生成物体和随机参数，`model_scene` 将场景合入调用者提供的机器人 MJCF，`validate` 检查重置。它只依赖 NumPy 和 MuJoCo，不依赖 PICO、Zenoh 或 `spd_vr`。

依赖方向为 `spd-vr → spd-envs`。原 `spd_vr.scenes` 已迁移为 `spd_envs`，不保留转发层。机器人 `generated/` 与 `config/` 留在 `spd-vr`；原始机器人资产独立放在根目录 `assets/`。环境包不硬编码 Tianji/Wuji 模型路径。

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

## 数据与资源约定

- `assets/`：原始 URDF、网格与必须保留的机器人资产。
- `packages/spd-vr/generated/`：可加载的编译模型和 manifest；改模型后重新编译并验证。
- `data/`：采集输出与已有样本；不随代码清理删除。
- `docs/papers/`：研究依据；论文不作为可执行规范替代代码验证。
- 根目录 `pixi.toml` 和 `pixi.lock` 是唯一受维护运行环境；系统 ADB 和图形会话为外部前置条件。

本次移除旧 ROS 2 采集、鱼眼、IMU、Odin、PXREA 和实机控制入口，不为它们保留兼容启动脚本。旧项目文档由根 README 与本文件替代。
