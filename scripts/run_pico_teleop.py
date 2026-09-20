"""Select and confirm a task scene before connecting PICO; clean up owned resources."""
from __future__ import annotations

import argparse
from contextlib import ExitStack
import fcntl
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]


def adb(args: argparse.Namespace, *arguments: str) -> str:
    command = [args.adb_path]
    if args.adb_serial:
        command.extend(("-s", args.adb_serial))
    try:
        result = subprocess.run(
            [*command, *arguments], stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=10,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise RuntimeError(f"ADB failed ({args.adb_path}): {exc}") from exc
    if result.returncode:
        raise RuntimeError(f"ADB {' '.join(arguments)} failed: {result.stderr.strip() or result.stdout.strip()}")
    return result.stdout


def preflight_device(args: argparse.Namespace) -> bool:
    """Return whether this session must create a new ADB forward."""
    if args.no_adb_forward:
        return False
    if args.host not in ("127.0.0.1", "localhost", "::1"):
        raise RuntimeError("ADB forwarding requires a loopback --host; use --no-adb-forward for an explicit remote TCP endpoint")
    devices = {}
    for line in adb(args, "devices").splitlines():
        fields = line.split()
        if len(fields) >= 2 and not line.startswith(("List of devices", "*")):
            devices[fields[0]] = fields[1]
    if args.adb_serial:
        state = devices.get(args.adb_serial, "not attached")
        if state != "device":
            raise RuntimeError(f"PICO ADB device {args.adb_serial!r} is {state}; connect it and authorize USB debugging, then check adb devices")
    else:
        if len(devices) > 1:
            raise RuntimeError("Multiple ADB devices attached; select the PICO with --adb-serial SERIAL (see adb devices)")
        if not devices:
            raise RuntimeError("No ADB device attached. Connect the PICO by USB, enable/authorize USB debugging, and check adb devices. For an existing TCP input use --no-adb-forward --host HOST --port PORT")
        args.adb_serial, state = next(iter(devices.items()))
        if state != "device":
            raise RuntimeError(f"PICO ADB device {args.adb_serial!r} is {state}; authorize USB debugging and check adb devices")
    local = f"tcp:{args.port}"
    for line in adb(args, "forward", "--list").splitlines():
        fields = line.split()
        if len(fields) == 3 and fields[1] == local:
            if fields == [args.adb_serial, local, f"tcp:{args.device_port}"]:
                return False
            raise RuntimeError(f"ADB forward {local} already exists ({line}); stop its owner or choose another --port. It will not be replaced")
    return True


def preflight_display() -> None:
    if not os.environ.get("DISPLAY"):
        raise RuntimeError("No DISPLAY; run pixi run spd-pico from a graphical desktop terminal")
    try:
        import tkinter
        root = tkinter.Tk()
        try:
            root.withdraw()
            root.update_idletasks()
        finally:
            root.destroy()
    except Exception as exc:
        raise RuntimeError(f"Cannot open the control UI on DISPLAY={os.environ['DISPLAY']!r}: {exc}. Use a working desktop display with Tk support") from exc


def ros_command(module: str, *arguments: str) -> list[str]:
    return ["pixi", "run", "-e", "ros-jazzy", "bash", "-c",
            'source .ros/install/setup.sh && '
            'export ROS_DOMAIN_ID=120 RMW_IMPLEMENTATION=rmw_fastrtps_cpp '
            'ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_STATIC_PEERS="" && '
            'exec python -u -m "$@"',
            "--", module, *arguments]


def log_tail(path: Path) -> str:
    with path.open("rb") as stream:
        stream.seek(max(0, path.stat().st_size - 6000))
        return stream.read().decode(errors="replace")


def confirm_scene(args: argparse.Namespace) -> bool:
    """Resolve the task and table placement before any device or window access."""
    from spd_envs.registry import TASKS, TASK_REGISTRY
    from spd_vr.scene import resolve_table_distance

    if not sys.stdin.isatty():
        raise ValueError("PICO 启动需要在交互终端选择并确认场景，未连接设备或打开窗口。")
    if args.task is None:
        print("请选择 PICO 接入的场景 / 任务：", flush=True)
        for index, spec in enumerate(TASKS, start=1):
            print(f"  {index:2d}. {spec.qualified_name} — {spec.prompt}")
        while True:
            selected = input("输入编号或完整 SCENE/TASK，回车选 spelling_blocks/spelling，q 取消：\n> ").strip()
            if selected.lower() == "q":
                return False
            if not selected:
                selected = "spelling_blocks/spelling"
            elif selected.isascii() and selected.isdecimal() and 1 <= int(selected) <= len(TASKS):
                selected = TASKS[int(selected) - 1].qualified_name
            if selected in TASK_REGISTRY:
                args.task = selected
                break
            print("无效选择，请输入列表中的编号或完整 SCENE/TASK。", flush=True)
    if args.task not in TASK_REGISTRY:
        raise ValueError(f"未知场景 / 任务：{args.task}；省略 --task 可查看并选择。")
    spec = TASK_REGISTRY[args.task]
    args.table_distance = resolve_table_distance(args.table_distance)
    print(
        f"\n即将接入的 PICO 场景：\n"
        f"  场景：{spec.scene}\n"
        f"  任务：{spec.name} — {spec.prompt}\n"
        f"  seed：{args.seed}\n"
        f"  桌沿距离：{args.table_distance:g} m（底座原点沿 +X 到近侧桌沿）\n"
        "  桌子与物体同步定位，进入后桌子固定；启动不等于运动授权。",
        flush=True,
    )
    while True:
        answer = input("确认接入以上场景并连接 PICO？[y/N] ").strip().lower()
        if answer in ("y", "yes", "是", "确认"):
            return True
        if answer in ("", "n", "no", "否", "q"):
            return False
        print("请输入 y 确认，或 n / 回车取消。", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1", help="PICO TCP host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=10002, help="Local PICO TCP port (default: 10002)")
    parser.add_argument("--device-port", type=int, default=10002, help="PICO device TCP port (default: 10002)")
    parser.add_argument("--adb-path", default="adb")
    parser.add_argument("--adb-serial", help="Select the PICO when multiple ADB devices are attached")
    parser.add_argument("--no-adb-forward", action="store_true", help="Use an existing TCP endpoint; do not require or modify ADB")
    parser.add_argument("--reconnect", type=float, default=2.0, help="Source TCP reconnect delay in seconds")
    parser.add_argument("--task", help="Registered SCENE/TASK; prompts for selection when omitted")
    parser.add_argument("--seed", type=int, default=0, help="Task scene seed (default: 0)")
    parser.add_argument("--table-distance", type=float, help="Robot base to near table edge in metres; prompts if omitted (default: 0.10)")
    args = parser.parse_args()
    if not all(1 <= port <= 65535 for port in (args.port, args.device_port)):
        parser.error("--port and --device-port must be in 1..65535")
    if not math.isfinite(args.reconnect) or args.reconnect <= 0:
        parser.error("--reconnect must be positive and finite")

    directory = ROOT / ".pixi/pico-teleop"
    children: list[tuple[str, subprocess.Popen, Path, object]] = []
    environment = dict(os.environ, PYTHONUNBUFFERED="1", ROS_DOMAIN_ID="120",
                       RMW_IMPLEMENTATION="rmw_fastrtps_cpp",
                       ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST", ROS_STATIC_PEERS="")

    def start(name: str, command: list[str]) -> None:
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

    def wait_ready(name: str, marker: str) -> None:
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline:
            for child_name, process, log_path, _ in children:
                code = process.poll()
                if code is not None:
                    raise RuntimeError(f"{child_name} exited during startup ({code}); log: {log_path}\n{log_tail(log_path)}")
            if marker in log_tail(directory / f"{name}.log"):
                return
            time.sleep(.1)
        path = directory / f"{name}.log"
        raise RuntimeError(f"Timed out waiting for {name} readiness; log: {path}\n{log_tail(path)}")

    def interrupt(*_: object) -> None:
        raise KeyboardInterrupt

    with ExitStack() as resources:
        # Hold the demo's existing lock too: both launchers must exclude one another.
        for session in (directory, ROOT / ".pixi/recorded-demo"):
            session.mkdir(parents=True, exist_ok=True)
            lock = resources.enter_context((session / "session.lock").open("a"))
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                parser.error("spd-pico or spd-demo is already running; close its windows or use Ctrl+C in its terminal")
        try:
            if not confirm_scene(args):
                print("已取消，未连接 PICO 或打开窗口。", flush=True)
                return 0
            create_forward = preflight_device(args)
            preflight_display()
            if not (ROOT / ".ros/install/setup.sh").is_file():
                raise RuntimeError("ROS interfaces missing; run pixi run ros-build-interfaces")
        except (EOFError, KeyboardInterrupt):
            print("\n已取消，未连接 PICO 或打开窗口。", flush=True)
            return 130
        except (OSError, RuntimeError, ValueError) as exc:
            parser.error(str(exc))
        control_directory = resources.enter_context(tempfile.TemporaryDirectory(prefix="spd-pico-control-"))
        control_socket = str(Path(control_directory) / "control.sock")

        owned_forward = False
        previous = {sig: signal.signal(sig, interrupt) for sig in (signal.SIGINT, signal.SIGTERM)}
        try:
            if create_forward:
                adb(args, "forward", "--no-rebind", f"tcp:{args.port}", f"tcp:{args.device_port}")
                owned_forward = True
                print(f"ADB forward established for {args.adb_serial}; this does not yet establish live tracking.", flush=True)
            elif not args.no_adb_forward:
                print(f"Reusing existing PICO ADB forward tcp:{args.port}; it remains externally owned and will not be removed.", flush=True)
            print(f"Starting PICO task {args.task} via direct Fast DDS (domain 120); logs: {directory}", flush=True)
            # This supervisor owns the forward; the source must neither create nor remove it.
            source_arguments = ["--host", args.host, "--port", str(args.port),
                                "--device-port", str(args.device_port), "--adb-path", args.adb_path,
                                "--reconnect", str(args.reconnect), "--no-adb-forward",
                                "--control-socket", control_socket]
            if args.adb_serial:
                source_arguments.extend(("--adb-serial", args.adb_serial))
            start("viewer", ros_command("spd_vr.ros_viewer", "--output", str(ROOT / "episodes"),
                                        "--task", args.task, "--seed", str(args.seed),
                                        "--table-distance", str(args.table_distance),
                                        "--control-socket", control_socket))
            wait_ready("viewer", "SPD subscriber ready")
            start("source", ros_command("spd_vr.ros_publisher", *source_arguments))
            wait_ready("source", "PICO publisher ready")
            print(f"PICO UI + {args.task} viewer ready; this is process/UI readiness, not proof of live PICO tracking.", flush=True)
            print("Use only the PICO window: Palm calibration (K) -> Align / Preview (C) -> Confirm & Follow (F). SPD checks and confirms authorization before motion.", flush=True)
            print("Hold (Space) stops following. After tracking loss: Align -> Confirm & Follow again. No MuJoCo e key required.", flush=True)
            print("Ctrl+C or closing either window stops this session and only its owned resources.", flush=True)
            while True:
                for name, process, log_path, _ in children:
                    code = process.poll()
                    if code is not None:
                        if code:
                            raise RuntimeError(f"{name} exited ({code}); log: {log_path}\n{log_tail(log_path)}")
                        print(f"{name} closed; stopping the PICO session.", flush=True)
                        return 0
                time.sleep(.2)
        except KeyboardInterrupt:
            return 0
        except (OSError, RuntimeError) as exc:
            print(f"PICO session failed: {exc}", flush=True)
            return 1
        finally:
            for sig in previous:
                signal.signal(sig, signal.SIG_IGN)
            for _, process, _, _ in reversed(children):
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
            deadline = time.monotonic() + 8
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
            if owned_forward:
                try:
                    expected = [args.adb_serial, f"tcp:{args.port}", f"tcp:{args.device_port}"]
                    if any(line.split() == expected for line in adb(args, "forward", "--list").splitlines()):
                        adb(args, "forward", "--remove", f"tcp:{args.port}")
                except RuntimeError as exc:
                    print(f"Warning: could not clean up owned ADB forward tcp:{args.port}: {exc}", flush=True)
            for sig, handler in previous.items():
                signal.signal(sig, handler)
            print("PICO session stopped; owned source/viewer processes cleaned up.", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
