# Task 12 报告：硬件无关 Python teleoperation E2E

## 交付

- `PICO_tracker/scripts/test_spd_teleop_e2e.py`：单一 E2E harness。
- 生产路径使用真实 `spd_vr.pxrea_bridge` fake-source queue、`spd_vr.arm_ik`、`spd_vr.viewer` 模块命令；所有子进程使用 argv、独立 process group、bounded wait，并在异常/超时时终止并回收。
- 启动前运行 `spd_vr.preflight` 并调用 authoritative `verify_artifacts`。缺少或验证失败时不启动任何生产进程，JSON `blocked=true`、exit 2。
- `--synthetic` 只运行已有 `arm_ik --self-test` 与 `viewer --headless --synthetic` smoke，JSON 全部标记 `synthetic=true`，不代表生产 artifact PASS。
- `PICO_tracker/pixi.toml` 增加 `spd-teleop-e2e`，保留用户已有 `pico-adb` 行。

## 生产 artifact gate（当前预期阻塞）

命令：

```bash
cd PICO_tracker
OMP_NUM_THREADS=1 pixi run python scripts/test_spd_teleop_e2e.py --json /tmp/spd-teleop-e2e.json
```

实际结果：命令以 exit 2 结束；未启动 bridge/IK/viewer；`/tmp/spd-teleop-e2e.json` 的关键输出为：

```text
blocked=true
synthetic=false
stage=artifact-gate
status=blocked
reason=authoritative artifacts blocked: Link_Base.STL p95 surface error 0.036785362 m exceeds the required arm/base 0.003 m gate; required artifacts missing: unified_plant.xml, arm_ik.xml, model_manifest.yaml, collision_manifest.yaml, actuator_calibration.yaml
preflight.exit_code=1
```

Preflight 原始 stdout 中 artifact 检查为：

```text
{"detail": "missing generated artifacts: unified_plant.xml, arm_ik.xml, model_manifest.yaml, collision_manifest.yaml, actuator_calibration.yaml", "name": "artifacts", "ok": false}
```

这保留 Task 7 authoritative blocker：`Link_Base.STL` 的 arm/base p95 为 `0.036785362 m`，超过必需 `0.003 m`；没有 synthetic fallback，也没有伪造 authoritative PASS。

## Explicit synthetic framework evidence

命令：

```bash
cd PICO_tracker
OMP_NUM_THREADS=1 pixi run python scripts/test_spd_teleop_e2e.py --synthetic --json /tmp/spd-teleop-e2e-synthetic.json
```

实际结果：exit 0。脚本确实运行已有模块：

```text
self-test: ticks=400 finite=400 solver_failures=0 elapsed_s=1.997 rate_hz=200.33 synthetic=true
headless=True ticks=480 simulated_seconds=1.000000 finite=True synthetic=True
```

JSON 关键字段：

```text
blocked=false
synthetic=true
stage=synthetic-framework
status=pass
reason=synthetic framework smoke only; not authoritative production artifact evidence
evidence.finite=true
evidence.solver.failures=0
evidence.physics.ticks=480
```

## Scoped verification

```bash
cd PICO_tracker
pixi run python -m py_compile scripts/test_spd_teleop_e2e.py
```

输出为空，exit 0。

碰撞动力学和独立性能大套件按用户裁决 deferred；本任务保留单一可运行 E2E harness、真实进程清理、finite/status/control boundary，以及 artifact fail-closed 证据。
