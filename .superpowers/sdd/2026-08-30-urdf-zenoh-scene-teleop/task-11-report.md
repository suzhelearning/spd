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
