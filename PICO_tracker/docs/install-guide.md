# pico_project installation guide

This project is a pixi-managed ROS 2 Humble workspace. ROS is provided by the
pixi environment, not by `/opt/ros`.

The steps below were verified on:

- Ubuntu 22.04 arm64 / aarch64, RK3588 host
- Project path: `/home/current/project/pico_project`
- pixi: `0.67.2`
- adb: `1.0.41`, Debian package `adb`

## 1. Install system tools

Install adb from Ubuntu and curl for installing pixi:

```bash
sudo apt-get update
sudo apt-get install -y adb curl
```

Check adb:

```bash
adb version
adb devices
```

If no headset is connected, `adb devices` should still print `List of devices
attached`.

## 2. Install pixi

If pixi is not already installed:

```bash
curl -fsSL https://pixi.sh/install.sh | bash
```

Restart the shell or add pixi to PATH:

```bash
export PATH="$HOME/.pixi/bin:$PATH"
pixi --version
```

## 3. Prepare the workspace

Go to the project directory:

```bash
cd /home/current/project/pico_project
```

The workspace supports both `linux-64` and `linux-aarch64` in `pixi.toml`.

If this directory was copied from another CPU architecture, remove or rename the
old pixi environment before installing. For example, on an ARM64 host with an
old x86_64 `.pixi` environment:

```bash
mv .pixi/envs/default .pixi/envs/default.x86_64_bak_$(date +%Y%m%d%H%M%S)
```

Install the pixi environment:

```bash
pixi install
```

## 4. Build and test

Build all workspace packages:

```bash
pixi run build
```

Run tests:

```bash
pixi run test
```

Expected result:

- `pico_bridge` builds and its `test_pico_frame` gtest passes.
- `fisheye_camera` builds.
- `data_collector` builds.

Some CMake deprecation warnings from ROS dependencies are expected.

## 5. Smoke test without a PICO headset

Use the mock server to verify the ROS bridge:

```bash
pixi run bash -lc '
source install/setup.bash
python3 src/pico_bridge/scripts/mock_pico_server.py --fps 5 --duration 8 &
mock_pid=$!
sleep 1
ros2 run pico_bridge pico_bridge_node &
bridge_pid=$!
sleep 3
ros2 topic list
timeout 5 ros2 topic echo --once /pico/pose/head
kill $bridge_pid $mock_pid 2>/dev/null || true
'
```

Expected topics include:

- `/pico/cam_left/compressed`
- `/pico/cam_right/compressed`
- `/pico/pose/head`
- `/pico/pose/left_hand`
- `/pico/pose/right_hand`
- `/pico/ble/left`
- `/pico/ble/right`

## 6. Run with a real PICO

In every terminal, enter the pixi environment and source the built workspace:

```bash
cd /home/current/project/pico_project
pixi shell
source install/setup.bash
```

Terminal 1, forward the PICO TCP port and start the bridge:

```bash
adb devices
adb forward tcp:9999 tcp:9999
ros2 launch pico_bridge start_pico_bridge.launch.py
```

Terminal 2, optionally start fisheye cameras:

```bash
ros2 launch fisheye_camera fisheye_camera.launch.py
```

Terminal 3, start data collection:

```bash
ros2 run data_collector data_collector_node \
  --config src/data_collector/config/collect_config.yaml
```

Keyboard controls in the collector terminal:

- `s`: start recording
- `d`: stop and save
- `q`: quit

Recorded output is written under `data/` by default. This directory is ignored
by git.

## 7. Useful maintenance commands

Clean build artifacts:

```bash
pixi run clean
```

Rebuild after cleaning:

```bash
pixi run clean
pixi run build
```

Check the ROS environment is coming from pixi:

```bash
pixi run which ros2
pixi run ros2 --help
```

The `ros2` path should be inside `.pixi/envs/default/bin`.
