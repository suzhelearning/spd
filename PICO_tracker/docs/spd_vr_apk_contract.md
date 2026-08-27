# SPD VR PICO APK 合同

`spd-vr` 只接受一个 PICO 前台应用。forthcoming `pico_wholebody_stream.apk` 必须在现有 14-byte little-endian frame header 下发送：

- `0x38`：左手；`0x39`：右手。
- payload 固定 `733` bytes：`active:uint8`、`scale:float32`、随后按 OpenXR 顺序的 26 个 `xyz + quaternion_xyzw` `float32` pose。
- 位置单位为米；两侧只有相同 `ts_ms` 且相同 `tracking_epoch` 才组成一次 `/pico/hands` 原子 frame。
- `active=0` 的手不得进入 Wuji retarget，只能进入该侧 HOLD。

`pico_bridge` 在 TCP 重连和 `TYPE_WORLD_RESET` 时清空未配对帧并递增 tracking epoch。缺少 `0x38/0x39` 时 host 仍可运行协议/mock 测试，但 `spd-vr` live readiness 必须失败，不得把 controller pose 冒充 optical hand。

若交付 APK 不实现本合同或不支持虚拟场景显示，只能使用同一 wire contract 的 XRoboToolkit Unity Client 构建兼容 APK（commit `cdc53166b0bf412efae71046c6a225eb5091605f`）。不得并行运行两个 PICO 前台应用；PICO 双目/鱼眼操作者流也不进入策略三相机数据集。
