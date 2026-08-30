# Task 8 实施报告

## 变更

- `src/spd_vr/spd_vr/alignment.py`：单侧 10-frame 中立对齐、显式 4x4 变换、jump/epoch/timestamp/stale/inactive HOLD、REALIGN/RESET。
- `src/spd_vr/spd_vr/qp_arm.py`：单侧 MuJoCo site Jacobian + SO(3) log、7-variable box-constrained persistent OSQP workspace；仅接受 `solved`/`solved inaccurate` 的 finite bounded 解。
- `src/spd_vr/spd_vr/arm_ik.py`：独立左右 controller、tracking/control codec 与 Zenoh mailbox/publisher 接入、绝对 5 ms deadline loop、production manifest gate、synthetic CLI self-test。
- `src/spd_vr/test/test_alignment.py`、`test_qp_arm.py`、`test_arm_ik.py`：仅覆盖对齐、求解边界/持久 workspace、双侧隔离与 loop contract。

## 验证命令与输出

```text
$ cd PICO_tracker
$ pixi run python -m pytest src/spd_vr/test/test_alignment.py src/spd_vr/test/test_qp_arm.py src/spd_vr/test/test_arm_ik.py -q
........                                                                 [100%]
8 passed, 15 warnings in 0.37s

$ pixi run python -m py_compile src/spd_vr/spd_vr/alignment.py src/spd_vr/spd_vr/qp_arm.py src/spd_vr/spd_vr/arm_ik.py
# no output; exit 0

$ pixi run python -m spd_vr.arm_ik --self-test --ticks 400
self-test: ticks=400 finite=400 solver_failures=0 rate_hz=200.00
```

测试过程先执行同一 focused pytest 命令确认缺模块红灯：收集阶段报告 `ModuleNotFoundError: No module named 'spd_vr.alignment'`，随后实现并得到上述绿灯。测试运行仅有现有 `hppfcl` import、OSQP `polish`/`raise_error` warning，无失败。

## Authoritative artifact 状态

真实 `arm_ik.xml` 未生成，生产入口保持 fail-closed：必须同时提供 `--model --manifest --urdf`，并调用 `verify_artifacts()` 校验 manifest 自身 hash、authoritative URDF/mesh hash、全部输出 hash、模型维度与 joint/actuator 映射；不以 synthetic fixture 冒充生产 artifact。

Task 7 已记录的真实 blocker：对 authoritative `Link_Base.STL` 执行固定 CoACD 参数（seed 0、每 link 最多 16 pieces、每 piece 最多 64 vertices）时，arm/base p95 surface error 为 `0.036785362 m`，超过固定 `0.003 m` gate。按约束未放宽阈值、未增加 piece/vertex 上限、未添加 fallback；synthetic 7DoF fixture 只用于 focused tests 与 CLI smoke。
## Review round 1 修复验证

补强了生产入口：artifact 校验成功后构造 manifest 绑定的左右 7-joint solver、初始化对应 wrist site neutral pose、连接 `tracking/control/arm_targets` Zenoh key，并进入绝对 5 ms runtime；校验失败仍 fail-closed。另修复 `dq`（rad/s）积分为 `q + dq * dt`、发布 `qdot=dq`，严格 tracking sequence/timestamp gate、ordered control queue、stale 清空 alignment、无限 loop 丢弃历史 outputs，以及 ALIGNED 相邻 frame 更新。

```text
$ cd PICO_tracker
$ pixi run python -m pytest src/spd_vr/test/test_alignment.py src/spd_vr/test/test_qp_arm.py src/spd_vr/test/test_arm_ik.py -q
...........                                                              [100%]
11 passed, 17 warnings in 0.37s

$ pixi run python -m py_compile src/spd_vr/spd_vr/alignment.py src/spd_vr/spd_vr/qp_arm.py src/spd_vr/spd_vr/arm_ik.py
# no output; exit 0

$ pixi run python -m spd_vr.arm_ik --self-test --ticks 400
self-test: ticks=400 finite=400 solver_failures=0 elapsed_s=1.998 rate_hz=200.24 synthetic=true
```

`synthetic=true` 明确表示该 CLI 仅验证 loop/solver smoke，不是 authoritative model 验收；真实 artifact blocker 状态保持不变。
