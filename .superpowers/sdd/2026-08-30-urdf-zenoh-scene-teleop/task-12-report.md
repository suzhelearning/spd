# Task 12 报告：硬件无关 Python teleoperation E2E

## Round 1 修复

- `test_spd_teleop_e2e.py` 的同步命令也使用独立 process group；超时对整个 group 执行 TERM/KILL 并 wait。
- fake JSONL 按 `pico_frames.decode_hand` 的真实布局生成：`active=1` offset 0、`scale=<f` offset 1、26×7 `<f4` offset 5；左右 wrist translation/rotation 和 finger flex 非零，并在写盘前 decode 断言。
- 生产 gate 对 preflight 任意非零立即 fail-closed；缺失 artifact 文案只列本次实际 missing，不拼接历史 Link_Base 数字。
- bridge 增加显式 `--wait-for-shutdown`（默认 fake CLI 行为不变），E2E 使用该模式，避免 JSONL EOF 先于 SHUTDOWN 自然退出。
- 生产 PASS 现在要求 status ready、control ack、全部 invariant、SHUTDOWN 后三进程自然 exit code 0、无 orphan；未观测的 finite/contact/HOLD/epoch 等不会伪造为 PASS。

## 生产 artifact gate（当前阻塞）

命令：

```bash
cd PICO_tracker
OMP_NUM_THREADS=1 pixi run python scripts/test_spd_teleop_e2e.py --json /tmp/spd-teleop-e2e.json
```

实际输出（exit 2）：

```text
blocked=true
synthetic=false
stage=artifact-gate
status=blocked
reason=required artifacts missing: unified_plant.xml, arm_ik.xml, model_manifest.yaml, collision_manifest.yaml, actuator_calibration.yaml
preflight.exit_code=1
```

脚本没有启动任何生产子进程。`preflight` 原始 artifact 检查为 `ok=false`，并报告同一组实际 missing 文件。Task 7 的历史 authoritative blocker 仍记录在 Task 7 报告中，但本次 missing-only 检查没有把未在本次验证读取到的 `Link_Base.STL` p95 数字冒充本次结果。

## Explicit synthetic framework evidence

命令：

```bash
cd PICO_tracker
OMP_NUM_THREADS=1 pixi run python scripts/test_spd_teleop_e2e.py --synthetic --json /tmp/spd-teleop-e2e-synthetic.json
```

实际输出（exit 0）：

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
pixi run python -m py_compile scripts/test_spd_teleop_e2e.py src/spd_vr/spd_vr/pxrea_bridge.py
pixi run python -m pytest src/spd_vr/test/test_pxrea_bridge.py::test_fake_source_jsonl_uses_worker_path_and_rejects_malformed_line -q
```

结果：py_compile 无输出 exit 0；focused bridge smoke `1 passed`。另以无 shell argv 子进程验证 `_run` 超时，结果 `timeout=0.2`、`exit=-15`，说明整个 process group 被回收。

碰撞动力学和独立性能大套件按用户裁决 deferred；本任务保留单一可运行 E2E harness、真实进程清理、finite/status/control boundary，以及 artifact fail-closed 证据。
