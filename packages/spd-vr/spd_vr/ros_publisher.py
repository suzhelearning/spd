"""Live PICO TCP -> production 54-DoF solver -> ROS domain 120 publisher."""
from __future__ import annotations

import argparse
from collections import deque
import queue
import signal
import subprocess
import sys
import threading
import time
from typing import Any



class _InputQueue:
    """Bounded, ordered callback handoff, preserving loss/reconnect events."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._events: deque[tuple[str, Any, int]] = deque()

    def put(self, kind: str, value: Any = None) -> None:
        receipt = time.monotonic_ns()
        with self._lock:
            if len(self._events) >= 128:
                self._events.clear()
                self._events.append(("disconnect", None, receipt))
            self._events.append((kind, value, receipt))

    def drain(self) -> list[tuple[str, Any, int]]:
        with self._lock:
            result = list(self._events)
            self._events.clear()
        return result


class _ControlWorker:
    """Only this thread may perform blocking receiver-control IPC."""

    def __init__(self, client: Any, events: _InputQueue) -> None:
        self.client, self.events = client, events
        self._condition = threading.Condition()
        self._context = (0, "")
        self._requests: deque[tuple[str, int, str]] = deque()
        self._closing = False
        self.thread = threading.Thread(target=self._run, name="pico-spd-control", daemon=False)

    def context(self, generation: int, session: str) -> None:
        with self._condition:
            self._context = (generation, session)
            self._condition.notify()

    def submit(self, op: str, generation: int, session: str) -> None:
        with self._condition:
            request = (op, generation, session)
            if request not in self._requests:
                self._requests.append(request)
            self._condition.notify()

    def close(self) -> None:
        with self._condition:
            self._closing = True
            self._condition.notify()
        self.thread.join()

    def _run(self) -> None:
        next_poll = 0.0
        while True:
            with self._condition:
                while not self._requests and not self._closing and time.monotonic() < next_poll:
                    self._condition.wait(next_poll - time.monotonic())
                if self._requests:
                    op, generation, session = self._requests.popleft()
                    if op == "enable" and (self._closing or (generation, session) != self._context):
                        continue
                elif self._closing:
                    return
                else:
                    generation, session = self._context
                    op = "status"
            try:
                response = self.client.request(op, session_id=session)
                error = ""
            except RuntimeError as exc:
                response, error = None, str(exc)
            self.events.put("control", (op, generation, session, response, error))
            next_poll = time.monotonic() + 0.2


class _FollowControl:
    """Solver-owned authorization handshake; never starts on a status poll."""

    def __init__(self, core: Any, worker: _ControlWorker) -> None:
        self.core, self.worker = core, worker
        self.generation = 0
        self.pending = False
        self.following = False
        self.session = core.session_id
        self.receiver: dict[str, Any] | None = None
        self.detail = "先对齐，再确认并跟随；必须收到 SPD 授权确认"
        self.worker.context(self.generation, self.session)

    @staticmethod
    def _ready(status: dict[str, Any]) -> bool:
        return (status["fresh"] and status["input_mask"] == 7
                and status["ready_mask"] == 7 and status["state"] != "aligning")

    def _hold(self, reason: str) -> None:
        old_session = self.session
        self.generation += 1
        self.pending = self.following = False
        self.core.command("hold")
        self.session = self.core.session_id
        self.worker.context(self.generation, self.session)
        self.worker.submit("hold", self.generation, old_session)
        self.detail = reason

    def command(self, command: str) -> None:
        if command in {"align", "hold"}:
            self._hold("本地已保持；正在请求 SPD 保持。恢复需要重新对齐并确认")
            if command == "align":
                aligned = self.core.command("align")
                self.session = self.core.session_id
                self.worker.context(self.generation, self.session)
                self.detail = ("正在按保持位置对齐；请稳住双手腕" if aligned else
                               "对齐被拒绝：" + self.core.status()["reason"])
            return
        if command not in {"start", "confirm"} or self.pending or self.following:
            return
        status = self.core.status()
        if not self._ready(status):
            self.detail = ("确认被拒绝：双臂、左手、右手均需新鲜有效，且对齐已完成；"
                           + status["reason"])
            return
        self.generation += 1
        self.session = self.core.session_id
        self.pending = True
        self.worker.context(self.generation, self.session)
        self.worker.submit("enable", self.generation, self.session)
        self.detail = "保持目标不动；正在等待 SPD 确认当前会话授权"

    def observe(self) -> None:
        status = self.core.status()
        invalid = (self.core.session_id != self.session or not status["fresh"]
                   or (self.pending and not self._ready(status))
                   or (self.following and not status["running_mask"]))
        if (self.pending or self.following) and invalid:
            self._hold("跟踪或对齐失效，已保持：" + status["reason"]
                       + "；请重新对齐并确认")

    def result(self, result: tuple[str, int, str, Any, str]) -> None:
        op, generation, session, response, error = result
        if generation != self.generation or session != self.session:
            # A cancelled in-flight enable can still have reached SPD.
            if op == "enable":
                self.worker.submit("hold", self.generation, session)
            return
        if op == "hold":
            if error or not response["ok"]:
                self.detail = "本地已保持；SPD 保持尚未确认：" + (error or response["error"])
            return
        self.receiver = response
        authorized = (not error and response["ok"] and response["enabled"]
                      and response["authorized_session"] == session
                      and response["candidate_session"] == session)
        if op == "status":
            if (self.following or (self.pending and error)) and not authorized:
                self._hold("SPD 授权已失效：" + (error or response.get("reason")
                           or response["error"] or "disabled, held, or session changed")
                           + "；请重新对齐并确认")
            elif error:
                self.detail = "SPD 控制连接不可用：" + error
            return
        if op != "enable" or not self.pending:
            return
        self.observe()
        if not self.pending:
            return
        if not authorized or response["ready_mask"] != 7 or response["hold_mask"] != 0:
            self._hold("确认被拒绝：" + (error or response["error"]
                       or "SPD acknowledgment does not authorize this fully-ready session"))
            return
        if not self.core.command("start") or self.core.running_mask != 7:
            self._hold("确认已取消：启动前源端就绪状态发生变化")
            return
        self.pending = False
        self.following = True
        self.detail = "正在跟随：SPD 已确认当前会话；保持可同时停止源端与 SPD"

    def status(self) -> dict[str, Any]:
        return {"control_detail": self.detail, "spd": self.receiver,
                "control_phase": "confirming" if self.pending else ("following" if self.following else "held")}


class _AdbForward:
    """Own only forwards created here, never replace somebody else's port."""

    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.mapping: tuple[str, str, str] | None = None
        self._last_error = ""

    def _run(self, *args: str, serial: str | None = None) -> str:
        command = [self.args.adb_path]
        selected = serial or self.args.adb_serial
        if selected:
            command += ["-s", selected]
        result = subprocess.run(command + list(args), capture_output=True, text=True, timeout=5, check=False)
        if result.returncode:
            raise RuntimeError((result.stderr or result.stdout).strip() or "adb failed")
        return result.stdout

    def ensure(self) -> None:
        if self.args.no_adb_forward:
            return
        try:
            local, remote = f"tcp:{self.args.port}", f"tcp:{self.args.device_port}"
            mappings = [tuple(line.split()) for line in self._run("forward", "--list").splitlines()]
            occupied = [mapping for mapping in mappings if len(mapping) == 3 and mapping[1] == local]
            if occupied:
                if self.mapping in occupied:
                    return
                raise RuntimeError(f"ADB {local} already forwarded; will not replace it (use --no-adb-forward to reuse intentionally)")
            self.mapping = None
            serial = self._run("get-serialno").strip()
            if not serial or serial == "unknown":
                raise RuntimeError("No selected ADB device; connect/authorize PICO or use --adb-serial")
            self._run("forward", "--no-rebind", local, remote)
            self.mapping = (serial, local, remote)
            self._last_error = ""
        except (OSError, subprocess.SubprocessError, RuntimeError) as exc:
            detail = str(exc)
            if detail != self._last_error:
                print(f"PICO waiting: {detail}", flush=True)
                self._last_error = detail
            raise RuntimeError(detail) from exc

    def close(self) -> None:
        if self.mapping is None:
            return
        try:
            mappings = [tuple(line.split()) for line in self._run("forward", "--list").splitlines()]
            if self.mapping in mappings:
                self._run("forward", "--remove", self.mapping[1], serial=self.mapping[0])
        except (OSError, subprocess.SubprocessError, RuntimeError) as exc:
            print(f"PICO forward cleanup: {exc}", file=sys.stderr, flush=True)
        self.mapping = None


def _control_window(events: _InputQueue, stop: threading.Event, *, integrated: bool = False) -> tuple[Any, Any]:
    import tkinter as tk
    from tkinter import font as tkfont

    root = tk.Tk()
    default_font = tkfont.nametofont("TkDefaultFont")
    if "Noto Sans CJK SC" in tkfont.families(root):
        default_font.configure(family="Noto Sans CJK SC", size=16)
    else:
        default_font.configure(family="fixed", size=18)
    root.title("PICO -> SPD spelling | domain 120")
    root.geometry("1000x650")
    status = tk.StringVar(value="等待 PICO 输入；机器人目标尚未就绪")
    tk.Label(root, text="PICO 实时控制 — 仅 spelling 仿真场景",
             font=(default_font.actual("family"), 16, "bold")).pack(pady=12)
    tk.Label(root, textvariable=status, font=default_font, justify="left", wraplength=950).pack(padx=18, pady=12)
    controls = tk.Frame(root)
    controls.pack(pady=8)
    follow_label = "确认并跟随 (F)" if integrated else "启动：先手动 SPD e (F)"
    for label, command in (("对齐 Align (C)", "align"), (follow_label, "start"), ("保持 Hold (Space)", "hold")):
        tk.Button(controls, text=label, font=default_font, width=22,
                  command=lambda command=command: events.put("command", command)).pack(side="left", padx=6)
    steps = (
        "1. 对齐：稳住双手腕；按当前保持位置校准，不会跳回 HOME。\n"
        "2. 确认并跟随：三组全部就绪后点击；收到 SPD 当前会话授权确认才启动。\n"
        "3. 保持：立即停止源端并撤销 SPD 授权。失效组恢复需要重新对齐并确认。"
        if integrated else
        "手动模式 — 未连接 SPD 控制接口。\n"
        "1. 按保持位置对齐。2. 到 SPD 窗口按 e，再回到这里启动。\n"
        "3. 保持只停止本源端。跟踪丢失后需要重新对齐、SPD e、启动。"
    )
    tk.Label(root, font=default_font, justify="left", wraplength=950, text=steps + (
        "\nC / F / 空格仅在本窗口获得焦点时有效；建议优先使用按钮。"
    )).pack(padx=18, pady=12)
    pressed: set[str] = set()
    releases: dict[str, Any] = {}
    shortcuts = {"c": "align", "f": "start", "space": "hold"}

    def key_down(event: Any) -> str | None:
        key = event.keysym.lower()
        if key not in shortcuts:
            return None
        if key in releases:
            root.after_cancel(releases.pop(key))
        if key not in pressed:
            pressed.add(key)
            events.put("command", shortcuts[key])
        return "break"

    def key_up(event: Any) -> str | None:
        key = event.keysym.lower()
        if key not in shortcuts:
            return None
        def released() -> None:
            releases.pop(key, None)
            pressed.discard(key)
        releases[key] = root.after_idle(released)
        return "break"

    # Bind on buttons before their class binding: Space must Hold, never
    # activate whichever Align/Confirm button happens to have keyboard focus.
    for widget in (root, *controls.winfo_children()):
        widget.bind("<KeyPress>", key_down)
        widget.bind("<KeyRelease>", key_up)
    root.bind("<FocusOut>", lambda _: pressed.clear())
    root.protocol("WM_DELETE_WINDOW", stop.set)
    root.update()
    return root, status


def _status_text(status: dict[str, Any], integrated: bool) -> str:
    groups = []
    for name, bit in (("双臂（双腕）", 1), ("左手", 4), ("右手", 2)):
        state = ("跟随中" if status["running_mask"] & bit else
                 "就绪 / 保持" if status["ready_mask"] & bit else
                 "已跟踪 / 待对齐" if status["input_mask"] & bit else "丢失 / 保持")
        groups.append(f"{name}: {state}")
    calibration = ("正在采集稳定手腕" if status["state"] == "aligning" else
                   "已完成" if status["ready_mask"] & 1 else "需要对齐")
    lines = [f"PICO: {'新鲜有效' if status['fresh'] else '等待 / 已过期'} | 校准: {calibration}",
             " | ".join(groups)]
    if integrated:
        receiver = status.get("spd")
        if receiver is None:
            lines.append("SPD: 控制连接尚未确认 — 未获授权时源端保持")
        else:
            lines.append(f"SPD: {receiver['state']} | "
                         f"{'已授权' if receiver['enabled'] else '未授权'} | "
                         f"就绪掩码={receiver['ready_mask']} 保持掩码={receiver['hold_mask']}")
        lines.append(status.get("control_detail", "正在等待 SPD 控制状态"))
    else:
        lines.append("SPD: 手动模式 / 无法在此验证 — 启动前请在 SPD 窗口按 e")
    lines.append("源端诊断: " + status["reason"])
    return "\n".join(lines)


def run_publisher(args: argparse.Namespace) -> int:
    import rclpy
    from pico_hand_tracking import Pico2Receiver
    from tianji_spd_interfaces.msg import JointCommand
    # Load ROS native libraries before NumPy/MuJoCo in the mixed ROS/pixi runtime.
    from .pico_ros_source import PicoTeleopCore
    from .ros_joint_command import TOPIC, best_effort_qos, message_from_snapshot

    core = PicoTeleopCore.from_production()
    events = _InputQueue()
    stop = threading.Event()
    statuses: queue.Queue[dict[str, Any]] = queue.Queue(maxsize=1)
    errors: queue.Queue[BaseException] = queue.Queue()
    forward = _AdbForward(args)
    control_worker = None
    integrated = bool(getattr(args, "control_socket", None))

    class OwnedReceiver(Pico2Receiver):
        def ensure_adb_forward(self) -> None:
            forward.ensure()

    receiver = OwnedReceiver(
        lambda frame: events.put("frame", frame), host=args.host, port=args.port,
        device_port=args.device_port, adb_path=args.adb_path, adb_serial=args.adb_serial,
        reconnect_seconds=args.reconnect, auto_adb_forward=False,
        on_connect=lambda: events.put("connect"), on_disconnect=lambda: events.put("disconnect"),
    )
    old_handlers = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    for sig in old_handlers:
        signal.signal(sig, lambda *_: stop.set())
    root = node = worker = None
    initialized = False
    try:
        if not args.headless:
            root, label = _control_window(events, stop, integrated=integrated)
        rclpy.init(domain_id=120)
        initialized = True
        node = rclpy.create_node("spd_pico_joint_command_publisher")
        publisher = node.create_publisher(JointCommand, TOPIC, best_effort_qos())
        if integrated:
            from .local_control import LocalControlClient

            control_worker = _ControlWorker(LocalControlClient(args.control_socket), events)
            control_worker.thread.start()

        def solve() -> None:
            next_tick = next_publish = time.monotonic_ns()
            last_status: tuple[Any, ...] | None = None
            control = _FollowControl(core, control_worker) if control_worker is not None else None
            try:
                while not stop.is_set() and rclpy.ok():
                    now = time.monotonic_ns()
                    if now < next_tick:
                        stop.wait((next_tick - now) * 1e-9)
                        continue
                    for kind, value, received in events.drain():
                        if kind == "frame":
                            core.accept_frame(value, received_ns=received)
                        elif kind == "connect":
                            core.connected()
                        elif kind == "disconnect":
                            core.disconnected()
                        elif kind == "control" and control is not None:
                            control.result(value)
                        elif kind == "command":
                            if value == "quit":
                                stop.set()
                            elif control is not None:
                                control.command(value)
                            else:
                                core.command("start" if value == "confirm" else value)
                        if control is not None:
                            control.observe()
                    now = time.monotonic_ns()
                    core.tick(now)
                    if control is not None:
                        control.observe()
                    if now >= next_publish:
                        publisher.publish(message_from_snapshot(core.snapshot()))
                        status = core.status()
                        if control is not None:
                            status.update(control.status())
                        key = (status["fresh"], status["input_mask"], status["ready_mask"],
                               status["running_mask"], status["state"], status["reason"],
                               status.get("control_detail"),
                               tuple((status.get("spd") or {}).get(field) for field in
                                     ("state", "enabled", "ready_mask", "hold_mask", "authorized_session",
                                      "candidate_session", "reason")))
                        if key != last_status:
                            print("PICO input " + str(status), flush=True)
                            last_status = key
                        try:
                            statuses.get_nowait()
                        except queue.Empty:
                            pass
                        statuses.put_nowait(status)
                        next_publish += 1_000_000_000 // 60
                        if next_publish <= now:
                            next_publish = now + 1_000_000_000 // 60
                    next_tick += core.PERIOD_NS
                    finished = time.monotonic_ns()
                    if next_tick <= finished:
                        next_tick = finished + core.PERIOD_NS
            except BaseException as exc:
                errors.put(exc)
                stop.set()
            finally:
                if control is not None:
                    control.command("hold")
                core.disconnected()
                try:
                    publisher.publish(message_from_snapshot(core.snapshot()))
                except Exception:
                    pass

        worker = threading.Thread(target=solve, name="pico-solvers", daemon=False)
        worker.start()
        receiver.start()
        if args.headless:
            def stdin_commands() -> None:
                for line in sys.stdin:
                    command = line.strip().lower()
                    if command in {"align", "start", "confirm", "hold", "quit"}:
                        events.put("command", command)
                    elif command:
                        print("Commands: align / confirm (or start) / hold / quit", flush=True)
            threading.Thread(target=stdin_commands, name="pico-stdin", daemon=True).start()
        print("PICO publisher ready (process/control ready; waiting for live tracking, not robot-ready)", flush=True)
        while not stop.is_set():
            if root is not None:
                try:
                    status = statuses.get_nowait()
                    label.set(_status_text(status, integrated))
                except queue.Empty:
                    pass
                root.update()
            stop.wait(0.02)
        if not errors.empty():
            raise errors.get_nowait()
    finally:
        stop.set()
        receiver.stop()
        # ensure_adb_forward may still be in a bounded subprocess call; join
        # before cleanup so it cannot create a forward after we remove it.
        receiver.join()
        if worker is not None:
            worker.join()
        if control_worker is not None:
            control_worker.close()
        forward.close()
        if node is not None:
            node.destroy_node()
        if initialized and rclpy.ok():
            rclpy.shutdown()
        if root is not None:
            root.destroy()
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=10002)
    parser.add_argument("--device-port", type=int, default=10002)
    parser.add_argument("--adb-path", default="adb")
    parser.add_argument("--adb-serial")
    parser.add_argument("--reconnect", type=float, default=2.0)
    parser.add_argument("--no-adb-forward", action="store_true")
    parser.add_argument("--control-socket", help="Private local SPD control socket; omitted means manual SPD e")
    parser.add_argument("--headless", action="store_true", help="stdin align/confirm/start/hold/quit; smoke testing only")
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535 or not 1 <= args.device_port <= 65535:
        parser.error("ports must be in 1..65535")
    import math
    if not math.isfinite(args.reconnect) or args.reconnect < 0.1:
        parser.error("--reconnect must be finite and >= 0.1 seconds")
    return run_publisher(args)


if __name__ == "__main__":
    raise SystemExit(main())
