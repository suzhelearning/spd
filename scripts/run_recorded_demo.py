"""Run the two-window HDF5 demonstration; own and clean up only its children."""
from __future__ import annotations

import argparse
import fcntl
import os
from pathlib import Path
import signal
import socket
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("files", nargs="*", type=Path, help="Explicit H5 paths; omitted paths are requested interactively")
    args = parser.parse_args()
    if not args.files:
        try:
            selected = input("请输入要演示的 H5 文件完整路径（回车确认，留空取消）：\n> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\n已取消，未启动演示。")
            return 0
        if not selected:
            print("已取消，未启动演示。")
            return 0
        if len(selected) >= 2 and selected[0] == selected[-1] and selected[0] in ("'", '"'):
            selected = selected[1:-1]
        args.files = [Path(selected)]
    paths = [path.expanduser().resolve() for path in args.files]
    for path in paths:
        if not path.is_file():
            parser.error(f"File not found: {path}; supply your H5 paths after pixi run spd-demo")
    if not os.environ.get("DISPLAY"):
        parser.error("No DISPLAY; run this command from the desktop terminal")
    if not (ROOT / ".ros/install/setup.sh").is_file():
        parser.error("ROS interfaces missing; run pixi run ros-build-interfaces")
    if not (ROOT / ".pixi/tools/zenoh-bridge-ros2dds/1.10.0/zenoh-bridge-ros2dds").is_file():
        parser.error("ROS bridge missing; run pixi run ros-bridge-install")

    directory = ROOT / ".pixi/recorded-demo"
    directory.mkdir(parents=True, exist_ok=True)
    children: list[tuple[str, subprocess.Popen, Path, object]] = []
    with (directory / "session.lock").open("w") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.error("spd-demo is already running; use its windows or Ctrl+C in its terminal")
        # Do not silently reuse or terminate another operator's bridge session.
        with socket.socket() as probe:
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                probe.bind(("127.0.0.1", 7447))
            except OSError:
                parser.error("Port 7447 is in use. Stop the previous demo first: Ctrl+C in its bridge terminal and pixi run spd-teleop-ros-stop")

        environment = dict(os.environ, PYTHONUNBUFFERED="1", ROS_DOMAIN_ID="121",
                           RMW_IMPLEMENTATION="rmw_fastrtps_cpp",
                           ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST", ROS_STATIC_PEERS="",
                           SPD_ZENOH_LISTEN="tcp/127.0.0.1:7447",
                           SPD_ZENOH_CONNECT="tcp/127.0.0.1:7447")

        def start(name: str, command: list[str]) -> subprocess.Popen:
            log_path = directory / f"{name}.log"
            log = log_path.open("w")
            try:
                process = subprocess.Popen(command, cwd=ROOT, env=environment,
                                           stdin=subprocess.DEVNULL, stdout=log,
                                           stderr=subprocess.STDOUT, start_new_session=True)
            except BaseException:
                log.close()
                raise
            children.append((name, process, log_path, log))
            return process

        def wait_ready(name: str, marker: str | None = None) -> None:
            deadline = time.monotonic() + 40
            while time.monotonic() < deadline:
                for child_name, process, log_path, _ in children:
                    if process.poll() is not None:
                        raise RuntimeError(f"{child_name} exited during startup.\n{log_path.read_text(errors='replace')[-6000:]}")
                if marker is None:
                    try:
                        with socket.create_connection(("127.0.0.1", 7447), timeout=.2):
                            return
                    except OSError:
                        pass
                elif marker in (directory / f"{name}.log").read_text(errors="replace"):
                    return
                time.sleep(.1)
            raise RuntimeError(f"Timed out starting {name}; see {directory / (name + '.log')}")

        def ros_command(module: str, *arguments: str) -> list[str]:
            return ["pixi", "run", "-e", "ros-jazzy", "bash", "-c",
                    'source .ros/install/setup.sh && exec python -u -m "$@"',
                    "--", module, *arguments]

        def interrupt(*_: object) -> None:
            raise KeyboardInterrupt

        previous = {sig: signal.signal(sig, interrupt) for sig in (signal.SIGINT, signal.SIGTERM)}
        try:
            print(f"Starting two-window demo; logs: {directory}", flush=True)
            start("spd-bridge", ["bash", "scripts/run_ros_bridge.sh", "spd"])
            wait_ready("spd-bridge")
            start("publisher-bridge", ["bash", "scripts/run_ros_bridge.sh", "publisher"])
            start("viewer", ros_command("spd_vr.ros_viewer", "--output", str(ROOT / "episodes")))
            wait_ready("viewer", "SPD subscriber ready")
            start("player", ros_command("spd_vr.ros_recorded_publisher", *(str(path) for path in paths), "--loop"))
            wait_ready("player", "PHASE WAITING")
            print("Demo ready: MuJoCo + H5 player. Press e in MuJoCo, then Play in the player.", flush=True)
            print("Next switches files. Ctrl+C or closing either window stops this demo. No scene loaded.", flush=True)
            while True:
                for name, process, log_path, _ in children:
                    code = process.poll()
                    if code is not None:
                        if code:
                            raise RuntimeError(f"{name} exited ({code}).\n{log_path.read_text(errors='replace')[-6000:]}")
                        print(f"{name} closed; stopping this demo.", flush=True)
                        return 0
                time.sleep(.2)
        except KeyboardInterrupt:
            return 0
        except (OSError, RuntimeError) as exc:
            print(f"Demo failed: {exc}", flush=True)
            return 1
        finally:
            for sig in previous:
                signal.signal(sig, signal.SIG_IGN)
            # Each child has its own process group, including wrappers/descendants.
            for _, process, _, _ in reversed(children):
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
            deadline = time.monotonic() + 5
            for _, process, _, log in reversed(children):
                try:
                    process.wait(timeout=max(.01, deadline - time.monotonic()))
                except subprocess.TimeoutExpired:
                    pass
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait()
                log.close()
            for sig, handler in previous.items():
                signal.signal(sig, handler)
            print("Demo stopped; owned bridge/player/viewer processes cleaned up.", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
