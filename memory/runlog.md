# spd workspace runlog（最新在上）

- 2026-09-07 omp agent — 打通 PICO_tracker spd-vr 遥操作仿真会话（PXREA SDK）。完成：装 adb(~/.local/android-platform-tools，软链 ~/.local/bin/adb)；adb.sh --offline 建 reverse tcp:63901；spd-preflight 9/9；`pixi run spd-teleop` 三进程(tmux spd-teleop: pxrea_bridge/arm_ik/viewer)全就绪，bridge 经 gRPC 连 RoboticsServiceProcess(60061) 成功("server start feedback")。
- 关键坑（均已定位）：
  1. tmux 版本分裂：start_spd_vr.sh 在 pixi run 内用 pixi env 的 tmux 3.7c，系统 tmux 3.4 连不上其 server → 操作会话一律用 .pixi/envs/default/bin/tmux。
  2. proxy 环境变量（http_proxy/all_proxy 全套指向 127.0.0.1:7890 代理）→ PXREA SDK 内嵌 gRPC 走代理握手失败（现象：init rc=0 但无 SERVER_CONNECT，"client cancel server stream"）。修复：`env -u http_proxy -u https_proxy -u HTTP_PROXY -u HTTPS_PROXY -u all_proxy -u ALL_PROXY pixi run spd-teleop`。spd-status 同理需去 proxy。
  3. RoboticsServiceProcess 单客户端：官方 RobotLinuxDemo(桌面 run3D.sh) 占住 60061 时 spd bridge 连不上；且 SDK init 不重试，需重启会话。
  4. 服务发现：PC service 经 UDP 广播(每 5s 到各网段 .255)发现头显端 XRoboToolkit client(com.xrobotoolkit.client, UnityPlayerActivity)。头显无 WiFi(仅 lo)时发现失败。service 日志：/home/fcl/.local/share/PICOBusinessSuitData/log/YYYYMMDD.txt。
- 遗留（等用户头显操作）：头显连 WiFi(与 PC 192.168.110.x 同段) 或 client 内 USB 直连；头显里 XRoboToolkit client 界面确认连接。设备上线后 bridge device_id 非 null，然后 `spd-control start` 开跑。会话初始 paused，控制命令 spd-control start/resume/pause。
