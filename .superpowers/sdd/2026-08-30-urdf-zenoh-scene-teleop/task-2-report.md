# Task 2 Report

## RED

- `pixi run python -m pytest src/spd_vr/test/test_wire_protocols.py src/spd_vr/test/test_pico_frames.py -q`
- 真实结果：exit 4；collection 报 `ModuleNotFoundError: No module named 'spd_vr.pico_frames'`，`2 errors in 0.17s`。
- 严格整数/布尔语义补测真实结果：`2 failed, 1 warning in 0.12s`；失败原因为 float sequence 未被拒绝。

## GREEN

- `pixi run python -m pytest src/spd_vr/test/test_wire_protocols.py src/spd_vr/test/test_pico_frames.py src/spd_vr/test/test_arm_target_protocol.py -q`
- 真实结果：`38 passed, 1 warning in 0.12s`。

## Files

- 新增 `spd_vr/wire/{__init__,crc,tracking,control,keys}.py`、`spd_vr/pico_frames.py`。
- 新增 `test_wire_protocols.py`、`test_pico_frames.py`；扩展 `test_arm_target_protocol.py`。
- `arm_target_protocol.py` 仅新增 HOLD 4–7，272-byte v2 layout 未改。

## Self-review

- 对外名称与简报一致；按主代理裁决固定 `PicoFrame`/`PicoPose`/`PicoHand`/`PairedHands` 字段，stream framing 保留 immutable `bytes`，typed decode 独立，pairer 不生成 epoch。
- Tracking header 为 `<IHHIIQQqqff>`（56 bytes），head/left/right offsets 为 56/84/812，CRC 覆盖 `[16,1540)`；control 为 `<IHHIIQq8s>`，CRC 覆盖 `[16,40)`；PICO header 14 bytes、payload 上限 733、总上限 747。
- 任务路径检查仅包含本任务新增/修改；未触碰已有无关 dirty files，未修改 C/C++ 遥操作代码。

## Concerns

- 聚焦 pytest 唯一 warning 来自环境 `hppfcl` 提示改用 `coal`，与本任务无关；无其他 concern。

## Review fix round 1

- 修改：active hand 的 26 个 quaternion 增加 `1e-3` norm gate；`HandPairer` 严格接受非 bool `Integral` epoch、拒绝 rollback 且仅在更大 epoch 清 pending；`TrackingFrame`、`PicoPose`、`PicoHand` 持有独立只读数组副本。
- 测试：新增 active/inactive quaternion、epoch rollback/non-integral 且 pending 保留、三类 frozen dataclass 外部源隔离与写保护回归。
- RED：4 个新增 nodeid 真实结果 `4 failed, 1 warning in 0.14s`。
- 回归 GREEN：相同 4 个 nodeid 真实结果 `4 passed, 1 warning in 0.11s`。
- 最终聚焦：`pixi run python -m pytest src/spd_vr/test/test_wire_protocols.py src/spd_vr/test/test_pico_frames.py src/spd_vr/test/test_arm_target_protocol.py -q` → `42 passed, 1 warning in 0.13s`；涉及文件为 `pico_frames.py`、`wire/tracking.py`、`test_pico_frames.py`、`test_wire_protocols.py` 和本报告。
