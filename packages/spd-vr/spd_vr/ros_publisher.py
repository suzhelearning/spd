"""In-process PICO_2 -> arm IK -> Wuji Hand 2 -> ROS publisher."""
from __future__ import annotations
try:
    import rclpy
except ImportError:  # default simulation environment does not include ROS
    rclpy = None  # type: ignore[assignment]

import argparse
import signal
import threading
import time
import uuid
from pathlib import Path
from typing import Any

import numpy as np

from .arm_ik import _production_controller, _verified_model
from .model_builder import workspace_root
from .pico_ros_source import PicoRosSourceCore
from .ros_joint_command import (
    ARMS_READY,
    LEFT_HAND_READY,
    RIGHT_HAND_READY,
    TOPIC,
    JointCommandSnapshot,
    best_effort_qos,
    message_from_snapshot,
)
from .wire import decode_tracking


class _LatestFrame:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._frame: Any | None = None
        self._generation = 0

    def put(self, frame: Any) -> None:
        with self._lock:
            self._frame = frame
            self._generation += 1

    def take(self, generation: int) -> tuple[int, Any] | None:
        with self._lock:
            if self._generation == generation or self._frame is None:
                return None
            return self._generation, self._frame


def _paths() -> tuple[Path, Path, Path, Path]:
    package_root = Path(__file__).resolve().parents[1]
    generated = package_root / "generated"
    root = workspace_root()
    return (
        generated / "arm_ik.xml",
        generated / "model_manifest.yaml",
        root / "assets" / "tianji_wuji2" / "tianji_wuji2.urdf",
        root / "assets" / "tianji_wuji2",
    )


def run_publisher(args: argparse.Namespace) -> int:
    import rclpy
    from pico_hand_tracking import Pico2Receiver
    from tianji_spd_interfaces.msg import JointCommand
    from .retarget_pair import WujiRetargetPair

    arm_model_path, manifest_path, urdf_path, asset_root = _paths()
    model, verified = _verified_model(arm_model_path, manifest_path, urdf_path)
    arm = _production_controller(model, verified)
    hand_pair = WujiRetargetPair.from_manifest(
        asset_root / "wuji2_pico_left.yaml" if (asset_root / "wuji2_pico_left.yaml").is_file() else Path(__file__).resolve().parents[1] / "config" / "wuji2_pico_left.yaml",
        asset_root / "wuji2_pico_right.yaml" if (asset_root / "wuji2_pico_right.yaml").is_file() else Path(__file__).resolve().parents[1] / "config" / "wuji2_pico_right.yaml",
        manifest_path,
        urdf_path,
    )
    frame_mailbox = _LatestFrame()
    core = PicoRosSourceCore()
    stop = threading.Event()

    def on_connect() -> None:
        core.reset_stream()

    receiver = Pico2Receiver(
        frame_mailbox.put,
        host=args.host,
        port=args.port,
        device_port=args.device_port,
        adb_path=args.adb_path,
        adb_serial=args.adb_serial,
        reconnect_seconds=max(args.reconnect, 0.1),
        auto_adb_forward=not args.no_adb_forward,
        on_connect=on_connect,
    )

    def stop_signal(*_: Any) -> None:
        stop.set()

    old_handlers = {signal.SIGINT: signal.getsignal(signal.SIGINT), signal.SIGTERM: signal.getsignal(signal.SIGTERM)}
    for sig in old_handlers:
        signal.signal(sig, stop_signal)
    session_id = uuid.uuid4().hex
    sequence = 0
    generation = 0
    node = None
    try:
        rclpy.init()
        node = rclpy.create_node("spd_pico_joint_command_publisher")
        publisher = node.create_publisher(JointCommand, TOPIC, best_effort_qos())
        receiver.start()
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.0)
            sample = frame_mailbox.take(generation)
            if sample is None:
                stop.wait(0.001)
                continue
            generation, raw_frame = sample
            packet = core.accept_frame(raw_frame)
            if packet is None:
                continue
            tracking = decode_tracking(packet)
            arm.accept_tracking(tracking)
            arm_target = arm.tick(time.monotonic_ns())
            from .viewer import PlantController

            hand_input = PlantController._pico_hand_frame(tracking)
            hands = hand_pair.retarget(hand_input)
            position = np.concatenate(
                (
                    np.asarray(arm_target.left_q, dtype=np.float64),
                    np.asarray(arm_target.right_q, dtype=np.float64),
                    np.asarray(hands.left_qpos, dtype=np.float64),
                    np.asarray(hands.right_qpos, dtype=np.float64),
                )
            )
            ready_mask = 0
            if int(arm_target.valid_mask) == 3:
                ready_mask |= ARMS_READY
            if hands.left_valid:
                ready_mask |= LEFT_HAND_READY
            if hands.right_valid:
                ready_mask |= RIGHT_HAND_READY
            sequence += 1
            snapshot = JointCommandSnapshot.from_values(
                session_id=session_id,
                sequence=sequence,
                ready_mask=ready_mask,
                position_rad=position,
                stamp_ns=time.time_ns(),
            )
            publisher.publish(message_from_snapshot(snapshot))
    finally:
        receiver.stop()
        receiver.join(timeout=1.0)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9876)
    parser.add_argument("--device-port", type=int, default=8888)
    parser.add_argument("--adb-path", default="adb")
    parser.add_argument("--adb-serial")
    parser.add_argument("--reconnect", type=float, default=2.0)
    parser.add_argument("--no-adb-forward", action="store_true")
    return run_publisher(parser.parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
