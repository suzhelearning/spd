# Task 5 交付报告：PICO 双侧腕部对齐

## 状态

已完成并提交 Task 5 的可独立验证范围。实现不包含 XRoboToolkit SDK、Manus/SMPL、ROS relay 或旧 skeleton 输入；Task 3 deferred 边界保持明确。

提交：`3a25ddd` — `feat: align PICO wrist poses per arm`

## 实现文件

- `TJ_arm_control/include/tianji_qp_ik/pico_wrist_alignment.hpp`
  - `WristAlignmentConfig`、双侧结果类型和 `PicoWristAlignment` API。
  - 每侧候选稳定窗口、参考位姿、HOLD 状态和稳定计数。
- `TJ_arm_control/src/pico_wrist_alignment.cpp`
  - active/finite/proper-rotation 校验。
  - 位置/姿态连续性阈值和单侧状态机。
  - `inverse(T_pico_reference) * T_pico_current`、仅平移缩放、再乘 `T_robot_reference` 的 SE(3) 对齐。
  - global reset、单侧 reset、epoch discontinuity 和 `alignment_reset` 处理。
- `TJ_arm_control/tests/test_pico_wrist_alignment.cpp`
  - 稳定窗口第十帧建立、基准无跳变、平移缩放、旋转复合、单侧 inactive/invalid、reset、epoch 和 post-reset reacquisition 覆盖。
- `TJ_arm_control/apps/run_qp_ik_viewer.cpp`
  - wrist site CLI、阈值校验、显式 `EndEffectorSiteNames`、wrist-frame 安全拒绝 legacy frame。
  - 控制循环单侧 `setManualTarget`、pending frame 仍 commit、单侧 HOLD/stale、暂停/nominal/toggle reset。
  - 独立 wrist alignment telemetry、CSV 字段和 viewer overlay。
  - actual wrist pose/error 继续使用 `endEffectorPose` 和 `armKinematicsAt(...).end_effector_pose`。
- `TJ_arm_control/include/tianji_qp_ik/pico_teleop_protocol.hpp`
  - 仅添加后续 relay 使用的 neutral `PicoTeleopFrame` 字段：`wrist_pose_input`、`wrist_alignment_reset`、双侧 active；默认 false，legacy frame 不会被静默当作 wrist input。
  - 未改变 TJVR wire 编解码或实现 Task 3 relay。
- `TJ_arm_control/include/tianji_qp_ik/telemetry.hpp`
  - 添加双侧 alignment count/aligned/hold reason telemetry 字段，使用 `string_view` 避免控制循环中不必要的字符串分配。
- `TJ_arm_control/CMakeLists.txt`
  - 注册库源和 `test_pico_wrist_alignment`。

`pico_teleop_session.hpp/.cpp` 未做行为改动：现有 freshness、epoch、sequence、resynchronization gate 已满足本任务；viewer 在 stable window pending 时显式调用现有 `commitApplied`，因此不会吞掉后续 sequence。

## 窄验证命令与结果

1. `cmake --build TJ_arm_control/build --target test_pico_wrist_alignment -j2`
   - 结果：环境顶层没有 `cmake` 命令，返回 `command not found`。随后改用项目既有 pixi 环境执行等价命令。
2. `pixi run cmake --build build --target test_pico_wrist_alignment -j2`（cwd=`TJ_arm_control`）
   - 结果：PASS，CMake 重配置成功，新增库源和测试目标编译链接成功。
3. `pixi run cmake --build build --target tianji_qp_ik_viewer test_pico_wrist_alignment -j2`（cwd=`TJ_arm_control`）
   - 结果：PASS，viewer 和 focused alignment test 均编译链接成功。
4. `pixi run ctest --test-dir build -R 'test_pico_wrist_alignment|test_pico_teleop_session|test_pico_udp_receiver' --output-on-failure`（cwd=`TJ_arm_control`）
   - 结果：PASS，3/3：`test_pico_udp_receiver`、`test_pico_teleop_session`、`test_pico_wrist_alignment`。
5. `pixi run ./build/tianji_qp_ik_viewer --headless --duration 0.1 --no-arm-target-output --algorithm hierarchical_qp --pico-wrist-input --left-end-effector-site l_wrist_target --right-end-effector-site r_wrist_target`（cwd=`TJ_arm_control`）
   - 结果：PASS，headless viewer 完成控制循环，`pico_headless_complete` 正常输出。
6. `pixi run ./build/tianji_qp_ik_viewer --headless --duration 0.1 --no-arm-target-output --algorithm hierarchical_qp --pico-wrist-input`（cwd=`TJ_arm_control`）
   - 结果：按 contract 拒绝，提示必须显式提供 `l_wrist_target`/`r_wrist_target`，证明 wrist 模式不会静默使用 legacy TCP site。
7. `pixi run ./build/tianji_qp_ik_viewer --headless --wrist-position-scale 0 --no-arm-target-output`（cwd=`TJ_arm_control`）
   - 结果：按 contract 拒绝非正 scale。

## Task 3 deferred 边界

- 未实现 XRoboToolkit SDK bridge、XRoboToolkit relay、TJVR wrist wire flags 的编码/解码、ROS/Manus/SMPL 集成。
- neutral frame 字段只为测试和后续 relay 保留，不宣称 live wrist source；当前旧 TJVR decode 产生的 `wrist_pose_input=false` frame 在 `--pico-wrist-input` 模式被安全拒绝。
- 真实硬件 wrist 输入、pause relay reset bit 的端到端传输，需后续恢复 Task 3 后再验证。

## Review fix round 1

针对 reviewer 的 5 项问题已在提交 `60d5b67`（`fix: harden PICO wrist viewer alignment`）修复：

- wrist 模式收到 legacy frame 时仅丢弃该 frame，不再 `continue` 跳过外层控制周期，因此 IK、safety、HOLD、snapshot 和 sleep 仍会执行；同时移除每帧阻塞式 stderr 输出。
- 左右侧分别保存 `setManualTarget` 的返回值；stale、commit validity 和单侧 HOLD 以对应侧接受结果为准，避免一侧时间戳被拒绝时误将另一侧标为有效。
- epoch reset 即使稳定窗口仍 pending，也将全新的 `TargetManager` 写回，清除旧 manual timestamp/filter 状态；后续重新对齐从新状态接受。
- 在 controller 创建前拒绝 `--pico-wrist-input` 与任意 `spark_*` 算法组合，仅允许 `hierarchical_qp`。
- telemetry hold reason 的默认返回值改为非空指针的静态空字符串，避免 `%s` 使用默认空 `string_view::data()` 的未定义行为风险。

修复后的窄验证：

- `pixi run cmake --build build --target tianji_qp_ik_viewer test_pico_wrist_alignment -j2`：PASS。
- `pixi run ctest --test-dir build -R 'test_pico_wrist_alignment|test_pico_teleop_session|test_pico_udp_receiver' --output-on-failure`：PASS，3/3。
- `pixi run ./build/tianji_qp_ik_viewer --headless --duration 0.1 --no-arm-target-output --pico-wrist-input --left-end-effector-site l_wrist_target --right-end-effector-site r_wrist_target --algorithm spark_upper_qpoases_direct`：按 contract 拒绝，提示 `--pico-wrist-input requires hierarchical_qp`。

- 额外提交 `742ba1c` 清理 pause、enable/reset-nominal 时的左右侧 acceptance 状态，确保 reset 后不复用旧侧有效标记；同一窄构建和 3/3 focused ctest 已再次 PASS。

- 额外提交 `ed13971` 修正 reset/accepted 路径中的 `TargetManager` candidate 双重 move；修复后同一窄构建与 3/3 focused ctest 再次 PASS。
