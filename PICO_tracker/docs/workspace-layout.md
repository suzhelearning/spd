# Workspace layout

```
pico_project/
├── pixi.toml                                ← pixi env (ROS2 Humble via robostack-staging)
├── README.md
├── src/
│   ├── pico_bridge/                          (C++, PICO TCP → ROS)
│   ├── fisheye_camera/                       (C++, V4L2 USB cams → ROS)
│   └── data_collector/                       (Python, ROS → HDF5 + MP4)
├── docs/
│   ├── PICO_Streaming_Guide.md
│   ├── workspace-layout.md
│   └── superpowers/
│       ├── specs/2026-04-16-pico-recorder-design.md
│       └── plans/2026-04-16-pico-recorder.md
└── reference/
    ├── receiver.py                            (original PICO receiver, for protocol reference)
    └── visualizer.py                          (if present)
```

## Build
```bash
cd /home/eai/project/pico_project
pixi shell                                   # enter Humble environment
colcon build --symlink-install --packages-select pico_bridge fisheye_camera data_collector
source install/setup.bash
```

## Run
```bash
adb forward tcp:9999 tcp:9999
ros2 launch pico_bridge start_pico_bridge.launch.py
ros2 launch fisheye_camera fisheye_camera.launch.py     # optional
ros2 run data_collector data_collector_node \
    --config src/data_collector/config/collect_config.yaml
# Then in data_collector terminal: 's' start / 'd' stop+save / 'q' quit
```

## Smoke test without real PICO
```bash
python3 src/pico_bridge/scripts/mock_pico_server.py --fps 30 --duration 30 &
ros2 run pico_bridge pico_bridge_node &
# ...then start data_collector as above
```
