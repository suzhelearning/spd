# Task 11 报告：Python teleoperation lifecycle

## 交付

- 新增 `preflight.py`：结构化 `CheckResult(name, ok, detail)`，检查 ADB reverse、PXREA SDK load、Python 依赖、display、五个 generated artifact/hash、Zenoh `7447` 端口和 `spd-teleop` session；只读探测，不启动设备。
- 新增 `control_cli.py`：`START`、`PAUSE`、`RESUME`、`REALIGN`、`RESET`、`SHUTDOWN` 控制帧，持久化单调序列，使用 monotonic timestamp，通过 connect-only Zenoh peer 发布到 `spd/vr/v1/control`。
- 新增 `status_cli.py`：connect-only 订阅 bridge/IK/viewer status，缺失状态输出 `null`，保持结构化诊断。
- `viewer.py` 发布 `spd/vr/v1/status/viewer`，连接、控制状态和关闭均输出结构化状态。
- start/stop 固定 session `spd-teleop` 和 `pxrea_bridge` → `arm_ik` → `viewer` 三个 Python module；stop 先 `spd-control shutdown`，再按 viewer → IK → bridge 有界等待，只升级已由 metadata 验证的 pane。
- setup 注册七个新 console scripts；package.xml 移除 spd_vr live exec 的 ROS 依赖；删除 obsolete `ros_input.py`；Pixi 增加四个任务并保留用户行 `pico-adb = "bash ../adb.sh"`。

## 验证证据

```text
$ pixi run python -m pytest src/spd_vr/test/test_preflight.py src/spd_vr/test/test_lifecycle_scripts.py -q
....                                                                     [100%]
4 passed, 1 warning in 0.36s
```

warning 为既有环境 `hppfcl` import warning。

```text
$ bash -n scripts/start_spd_vr.sh scripts/stop_spd_vr.sh
$ bash scripts/start_spd_vr.sh --dry-run
session=spd-teleop
pxrea_bridge: python -m spd_vr.pxrea_bridge ... --listen
arm_ik: python -m spd_vr.arm_ik ...
viewer: python -m spd_vr.viewer ...
$ bash scripts/stop_spd_vr.sh --dry-run
session=spd-teleop
spd-control shutdown --endpoint tcp/127.0.0.1:7447
viewer: wait
arm_ik: wait
pxrea_bridge: wait
```

start dry-run 恰好列出三个 runtime module，无旧输入链路、ROS、DDS、router 或 C++ runtime。

```text
$ pixi run python -m py_compile src/spd_vr/spd_vr/preflight.py src/spd_vr/spd_vr/control_cli.py src/spd_vr/spd_vr/status_cli.py src/spd_vr/spd_vr/viewer.py
$ pixi run python -m spd_vr.control_cli shutdown --dry-run --sequence-file /tmp/spd-control-seq-task11
{"command": "shutdown", "endpoint": "tcp/127.0.0.1:7447", "key": "spd/vr/v1/control", "monotonic_timestamp_ns": 238947942508218, "sequence": 1}
$ pixi run python -m spd_vr.status_cli --timeout 0.05
{"endpoint": "tcp/127.0.0.1:7447", "key": "spd/vr/v1/status", "status": {"bridge": null, "ik": null, "viewer": null}}
```

真实 preflight（无设备启动）：

```text
$ pixi run python -m spd_vr.preflight --repo-root "$PWD"
{"detail": "ADB reverse is available", "name": "adb_reverse", "ok": true}
{"detail": "loaded /opt/apps/roboticsservice/SDK/x64/libPXREARobotSDK.so", "name": "sdk", "ok": true}
{"detail": "mujoco, osqp, coacd, zenoh", "name": "python_dependencies", "ok": true}
{"detail": ":0", "name": "display", "ok": true}
{"detail": "missing generated artifacts: unified_plant.xml, arm_ik.xml, model_manifest.yaml, collision_manifest.yaml, actuator_calibration.yaml", "name": "artifacts", "ok": false}
{"detail": "tcp/127.0.0.1:7447 is unavailable: [Errno 98] Address already in use", "name": "port_7447", "ok": false}
{"detail": "session is absent: spd-teleop", "name": "session", "ok": true}
```

命令以非零退出，且在缺少 artifact 时 fail-closed。

## Task7 blocker

权威 URDF 的 `Link_Base` 在固定 `seed=0,max_pieces=16,max_vertices=64` 和 `decimate=True` 约束下产生 16 个不超过 64 顶点的 pieces，但双向表面质量门为 `surface_p95=0.036785362 m`，超过 arm/base 的 `0.003 m` 阈值。Task7 因此以 `CollisionError` fail-closed，未发布半成品目录；Task11 preflight 如实报告五个 generated artifact 缺失，不生成替代碰撞或绕过质量门。

## Review round 1 修复与验证

- 控制序列改为 session-aware、文件锁保护的共享 allocator；`spd-control` 使用 reliable/blocking publisher，并等待 bridge、IK、viewer 对同一 sequence 的状态确认。viewer/bridge/IK 均订阅同一 `CONTROL_KEY`。
- bridge 收到 `SHUTDOWN` 后停止 SDK、worker 与 Zenoh；viewer 状态定期节流发布，使用 `plant.tick` 和 `session.snapshot.last_sequence`，未收到控制时 sequence 为 `null`。
- preflight 现在校验选中的在线 ADB PICO、设备对应 reverse、`RoboticsService`，默认 manifest 为 `src/spd_vr/generated/model_manifest.yaml`；SDK 优先级为 `PXREA_SDK_LIBRARY`，其次 `${PXREA_SDK_ROOT}/x64/libPXREARobotSDK.so`。
- start 对每个 argv 参数 shell-quote，未知选项返回非零，并在部分创建失败时仅清理自己创建的 `spd-teleop` session；viewer 与 preflight/IK 使用同一显式 URDF。stop 读取 metadata endpoint（显式 `--endpoint` 除外），先控制 SHUTDOWN，再等待自然退出，最后只对重新验证身份的 pane 升级。

```text
$ python3 -m py_compile src/spd_vr/spd_vr/preflight.py src/spd_vr/spd_vr/control_sequence.py src/spd_vr/spd_vr/control_cli.py src/spd_vr/spd_vr/status_cli.py src/spd_vr/spd_vr/pxrea_bridge.py src/spd_vr/spd_vr/arm_ik.py src/spd_vr/spd_vr/viewer.py src/spd_vr/spd_vr/zenoh_transport.py
$ bash -n scripts/start_spd_vr.sh scripts/stop_spd_vr.sh
$ pixi run python -m pytest src/spd_vr/test/test_preflight.py src/spd_vr/test/test_lifecycle_scripts.py src/spd_vr/test/test_pxrea_bridge.py src/spd_vr/test/test_viewer.py -q
................                                                         [100%]
16 passed, 1 warning in 0.52s
```

warning 为既有环境 `hppfcl` import warning。

```text
$ bash scripts/start_spd_vr.sh --dry-run --endpoint 'tcp/host name:7447' --sdk-library '/tmp/sdk path.so' --manifest '/tmp/model dir/model_manifest.yaml' --urdf '/tmp/urdf file.urdf'
session=spd-teleop
pxrea_bridge: python -m spd_vr.pxrea_bridge --sdk-library /tmp/sdk\ path.so --endpoint tcp/host\ name:7447 --listen
arm_ik: python -m spd_vr.arm_ik --model /tmp/model\ dir/arm_ik.xml --manifest /tmp/model\ dir/model_manifest.yaml --urdf /tmp/urdf\ file.urdf --endpoint tcp/host\ name:7447
viewer: python -m spd_vr.viewer --model /tmp/model\ dir/unified_plant.xml --manifest /tmp/model\ dir/model_manifest.yaml --urdf /tmp/urdf\ file.urdf --endpoint tcp/host\ name:7447
$ bash scripts/start_spd_vr.sh --bogus
unknown-option-exit=nonzero
```

真实 preflight（硬件/生成 artifact 状态保持 fail-closed）：

```text
$ pixi run spd-preflight --repo-root "$PWD" --manifest "$PWD/src/spd_vr/generated/model_manifest.yaml" --urdf "$PWD/../assets/tianji_wuji2/tianji_wuji2.urdf" --sdk-library /opt/apps/roboticsservice/SDK/x64/libPXREARobotSDK.so --endpoint tcp/127.0.0.1:7447
preflight-exit=1
{"detail": "online PICO: PA921DMGK8270070G", "name": "pico_device", "ok": true}
{"detail": "expected reverse entry missing: tcp:7447 tcp:7447", "name": "adb_reverse", "ok": false}
{"detail": "adb command failed", "name": "robotics_service", "ok": false}
{"detail": "loaded /opt/apps/roboticsservice/SDK/x64/libPXREARobotSDK.so", "name": "sdk", "ok": true}
{"detail": "mujoco, osqp, coacd, zenoh", "name": "python_dependencies", "ok": true}
{"detail": ":0", "name": "display", "ok": true}
{"detail": "missing generated artifacts: unified_plant.xml, arm_ik.xml, model_manifest.yaml, collision_manifest.yaml, actuator_calibration.yaml", "name": "artifacts", "ok": false}
{"detail": "tcp/127.0.0.1:7447 is unavailable: [Errno 98] Address already in use", "name": "port_7447", "ok": false}
{"detail": "session is absent: spd-teleop", "name": "session", "ok": true}
```

## Review round 2 修复与验证

- `control_cli` 补齐 JSON status decoder；控制序列 allocator 对损坏/空状态 fail-closed，使用独立 session lock 和临时文件 `fsync` + `os.replace`，并把取号与 publish 放进同一锁。
- package.xml 恢复 `ament_python` 构建元数据，仅不恢复 live ROS exec 依赖；bridge 初始化/清理路径保留首个异常并保证 SDK、worker、Zenoh、signal 清理。
- preflight 依据 `ss` 中唯一非 loopback `RoboticsService` listener 动态端口校验所选 serial 的 reverse；Zenoh 7447 仍只用于 peer。start 支持 `--serial`/`PICO_ADB_SERIAL` 并传入 preflight 与 bridge；collision cache 移到 output 外。
- SHUTDOWN 状态先发布并保留 Zenoh peer 0.5 秒 bounded grace，viewer production/synthetic 分支均显式设置状态。

```text
$ python3 -m py_compile src/spd_vr/spd_vr/preflight.py src/spd_vr/spd_vr/control_sequence.py src/spd_vr/spd_vr/control_cli.py src/spd_vr/spd_vr/pxrea_bridge.py src/spd_vr/spd_vr/viewer.py
$ bash -n scripts/start_spd_vr.sh scripts/stop_spd_vr.sh
$ pixi run python -m pytest src/spd_vr/test/test_control_cli.py src/spd_vr/test/test_preflight.py src/spd_vr/test/test_lifecycle_scripts.py src/spd_vr/test/test_pxrea_bridge.py src/spd_vr/test/test_viewer.py -q
...................                                                      [100%]
19 passed, 1 warning in 0.51s
```

warning 为既有环境 `hppfcl` import warning。

```text
$ PICO_ADB_SERIAL=PICO-1 bash scripts/start_spd_vr.sh --dry-run --endpoint 'tcp/host name:7447'
session=spd-teleop
pxrea_bridge: python -m spd_vr.pxrea_bridge --sdk-library /opt/apps/roboticsservice/SDK/x64/libPXREARobotSDK.so --endpoint tcp/host\ name:7447 --device-id PICO-1 --listen
arm_ik: python -m spd_vr.arm_ik --model .../arm_ik.xml --manifest .../model_manifest.yaml --urdf .../tianji_wuji2.urdf --endpoint tcp/host\ name:7447
viewer: python -m spd_vr.viewer --model .../unified_plant.xml --manifest .../model_manifest.yaml --urdf .../tianji_wuji2.urdf --endpoint tcp/host\ name:7447
$ pixi run spd-preflight --repo-root "$PWD" --manifest "$PWD/src/spd_vr/generated/model_manifest.yaml" --urdf "$PWD/../assets/tianji_wuji2/tianji_wuji2.urdf" --sdk-library /opt/apps/roboticsservice/SDK/x64/libPXREARobotSDK.so --endpoint tcp/127.0.0.1:7447
preflight-exit=1
{"detail": "online PICO: PA921DMGK8270070G", "name": "pico_device", "ok": true}
{"detail": "expected reverse entry missing: tcp:63901 tcp:63901", "name": "adb_reverse", "ok": false}
{"detail": "non-loopback RoboticsService listener: 63901", "name": "robotics_service", "ok": true}
{"detail": "loaded /opt/apps/roboticsservice/SDK/x64/libPXREARobotSDK.so", "name": "sdk", "ok": true}
{"detail": "mujoco, osqp, coacd, zenoh", "name": "python_dependencies", "ok": true}
{"detail": ":0", "name": "display", "ok": true}
{"detail": "missing generated artifacts: unified_plant.xml, arm_ik.xml, model_manifest.yaml, collision_manifest.yaml, actuator_calibration.yaml", "name": "artifacts", "ok": false}
{"detail": "tcp/127.0.0.1:7447 is unavailable: [Errno 98] Address already in use", "name": "port_7447", "ok": false}
{"detail": "session is absent: spd-teleop", "name": "session", "ok": true}
```

## Review round 3 修复与验证

- viewer parser 同时保留 `--model`、`--manifest`、`--urdf`，production 默认 `synthetic=False`；新增真实 `main()` parser smoke，确认 manifest/URDF 到达 production constructor。
- allocator 的 session lock 现在覆盖 sequence allocation、publish 和全部 status ACK 等待；异常时 context manager 释放锁，状态文件保持临时文件 `fsync` + `os.replace`。
- start 解析 `--serial`（默认 `PICO_ADB_SERIAL`），一致传给 preflight 与 bridge；preflight 用 `adb -s SERIAL reverse --list`，对 serial/device/host 三字段精确比较动态 RoboticsService 端口。

```text
$ python3 -m py_compile src/spd_vr/spd_vr/preflight.py src/spd_vr/spd_vr/control_sequence.py src/spd_vr/spd_vr/control_cli.py src/spd_vr/spd_vr/pxrea_bridge.py src/spd_vr/spd_vr/viewer.py
$ bash -n scripts/start_spd_vr.sh scripts/stop_spd_vr.sh
$ pixi run python -m pytest src/spd_vr/test/test_control_cli.py src/spd_vr/test/test_preflight.py src/spd_vr/test/test_lifecycle_scripts.py src/spd_vr/test/test_pxrea_bridge.py src/spd_vr/test/test_viewer.py -q
....................                                                     [100%]
20 passed, 1 warning in 0.52s
```

warning 为既有环境 `hppfcl` import warning。

```text
$ pixi run python -m pytest src/spd_vr/test/test_viewer.py::test_main_production_parser_passes_manifest_and_urdf -q
1 passed, 1 warning
```

```text
$ PICO_ADB_SERIAL=PICO-1 bash scripts/start_spd_vr.sh --dry-run --endpoint 'tcp/host name:7447'
session=spd-teleop
pxrea_bridge: python -m spd_vr.pxrea_bridge --sdk-library /opt/apps/roboticsservice/SDK/x64/libPXREARobotSDK.so --endpoint tcp/host\ name:7447 --device-id PICO-1 --listen
arm_ik: python -m spd_vr.arm_ik --model /home/current/syz/spd/PICO_tracker/src/spd_vr/generated/arm_ik.xml --manifest /home/current/syz/spd/PICO_tracker/src/spd_vr/generated/model_manifest.yaml --urdf /home/current/syz/spd/PICO_tracker/../assets/tianji_wuji2/tianji_wuji2.urdf --endpoint tcp/host\ name:7447
viewer: python -m spd_vr.viewer --model /home/current/syz/spd/PICO_tracker/src/spd_vr/generated/unified_plant.xml --manifest /home/current/syz/spd/PICO_tracker/src/spd_vr/generated/model_manifest.yaml --urdf /home/current/syz/spd/PICO_tracker/../assets/tianji_wuji2/tianji_wuji2.urdf --endpoint tcp/host\ name:7447
```

真实 preflight 仍按 Task7 artifact blocker fail-closed：

```text
preflight-exit=1
{"detail": "online PICO: PA921DMGK8270070G", "name": "pico_device", "ok": true}
{"detail": "expected reverse entry missing: tcp:63901 tcp:63901", "name": "adb_reverse", "ok": false}
{"detail": "non-loopback RoboticsService listener: 63901", "name": "robotics_service", "ok": true}
{"detail": "loaded /opt/apps/roboticsservice/SDK/x64/libPXREARobotSDK.so", "name": "sdk", "ok": true}
{"detail": "mujoco, osqp, coacd, zenoh", "name": "python_dependencies", "ok": true}
{"detail": ":0", "name": "display", "ok": true}
{"detail": "missing generated artifacts: unified_plant.xml, arm_ik.xml, model_manifest.yaml, collision_manifest.yaml, actuator_calibration.yaml", "name": "artifacts", "ok": false}
{"detail": "tcp/127.0.0.1:7447 is unavailable: [Errno 98] Address already in use", "name": "port_7447", "ok": false}
{"detail": "session is absent: spd-teleop", "name": "session", "ok": true}
```
