# Task 3 Report

## RED

- `cd PICO_tracker && pixi run python -m pytest src/spd_vr/test/test_zenoh_transport.py -q`
- 真实结果：exit 4；collection 报 `ModuleNotFoundError: No module named 'spd_vr.zenoh_transport'`，`1 warning, 1 error in 0.15s`。

## GREEN

- `cd PICO_tracker && pixi run python -m pytest src/spd_vr/test/test_zenoh_transport.py -q`
- 真实结果：`5 passed, 1 warning in 0.22s`，pytest 进程正常退出。

## Files

- 新增 `PICO_tracker/src/spd_vr/spd_vr/zenoh_transport.py`。
- 新增 `PICO_tracker/src/spd_vr/test/test_zenoh_transport.py`。
- 新增本报告 `task-3-report.md`。

## Self-review

- `LatestSample` 仅持有一个 `Lock`、一个 value 和单调 generation；覆盖写不累积，`invalidate()` 清空槽位。
- callback 仅执行 `bytes(sample.payload)`、decoder 和 `mailbox.put()`；测试用两个真实 in-process peer 验证 bytes decode、latest-only 与关闭后 generation 不再变化。
- `ZenohNode` 独占其声明的 subscriber/publisher/session；`close()` 先逆序 undeclare subscriber，再逆序 undeclare publisher，最后关闭 session。首次错误延后到其余资源释放后原样抛出；session 先置空，因此重复 close 不重复释放。
- endpoint 由 OS 临时端口动态选择，测试无固定端口冲突；未创建 router 或外部进程，两个 context/session 均显式关闭。
- 仅新增本任务实现、聚焦测试和报告；未触碰已有无关 dirty files。

## Concerns

- 锁定的 eclipse-zenoh 1.10.0 `Config` 没有简报示例中的 `to_json5()`，只有 `get_json()` 与 `str(config)`；测试用这两个实际 API 验证 endpoint 只出现一次。
- 唯一 warning 来自环境 `hppfcl` 提示改用 `coal`，与本任务无关；无其他 concern。

## Review fix round 1

- 修复：真实 peer 测试在首个 publish 前通过 eclipse-zenoh 1.10 `Publisher.matching_status` 属性和 matching listener 的 `threading.Event` 有界等待 subscriber match；不再用固定 sleep 猜测声明传播时序，listener 在成功和超时路径均 undeclare。
- 覆盖：新增 matching 等待超时与 listener 触发测试；原真实双 peer 测试覆盖 match 后批量发送路径。
- RED：`pixi run python -m pytest src/spd_vr/test/test_zenoh_transport.py -q` → `3 failed, 4 passed, 1 warning in 0.14s`，三个失败均为等待函数尚未实现的 `NotImplementedError`。
- GREEN：同命令 → `7 passed, 1 warning in 0.22s`。
