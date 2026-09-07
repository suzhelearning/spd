# PICO hand tracking input

本包是 SPD 仿真采集系统的操作者输入，不是仿真渲染客户端或训练数据录制器。

- Python 包：`pico_hand_tracking`；命令：`pico-hand-tracking`。
- APK：`apk/pico_hand_tracking_adb.apk`。
- Android 包名：`com.PICO.hand.tracking.wired.unity`。
- USB ADB forward，设备端 TCP `10002`。
- `0x40` 帧：头部、左右手腕、每手 26 个关节；基础 payload 1968 字节。
- 位姿：`[x, y, z, qx, qy, qz, qw]`；FLU 坐标，X 前、Y 左、Z 上。

## 从项目根目录运行

```bash
pixi install
adb devices
adb install -r packages/pico-hand-tracking/apk/pico_hand_tracking_adb.apk
adb shell monkey -p com.PICO.hand.tracking.wired.unity -c android.intent.category.LAUNCHER 1
pixi run pico2-hand --duration 5
pixi run pico2-hand --save-jsonl data/hand_tracking.jsonl
```

多设备时传 `--adb-serial SERIAL` 或设置 `PICO_ADB_SERIAL`。接收器自动执行端口转发并支持重连；具体选项用 `pixi run pico-hand-tracking --help` 查看。`L0/R0` 表示该帧没有有效手部跟踪，不代表 TCP 断开。

系统适配器 `spd_vr.pico2_bridge` 把输入转换成统一 tracking 帧，发布到 Zenoh `spd/vr/v1/tracking`。从项目根目录运行 `pixi run spd-teleop` 启动实时仿真查看链路。

## APK 限制

此 APK 只发送跟踪数据并在头显中显示手骨架，不接收或显示 MuJoCo 场景。沉浸式仿真场景回传仍需要单独实现，不能将头显手骨架画面当作仿真反馈。首次运行若提示缺少授权缓存，需要头显联网并完成 PICO 账号授权。

`pc_scripts/hand_tracking/` 保留上游独立手部可视化工具，不属于系统默认启动路径。APK 校验文件为 `SHA256SUMS`；上游来源为 https://github.com/vbgfbft/PICO_2 的 `PICO_Hand_Tracking` 分支。
