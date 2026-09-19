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
    """Latest healthy pose per burst; loss, clock and control boundaries stay ordered."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._events: deque[tuple[str, Any, int]] = deque()

    @staticmethod
    def _continuous_frames(previous: Any, current: Any) -> bool:
        # Validation is needed only on backlog. Never hide a lost/invalid frame
        # or device-clock transition by replacing it with a later healthy pose.
        from .pico_ros_source import _canonical_hand, _pose_values

        try:
            if not 0 <= previous.timestamp_ms < current.timestamp_ms:
                return False
            for frame in (previous, current):
                if frame.head.valid:
                    _pose_values(frame.head, "head")
                if not all(_canonical_hand(getattr(frame, side), side)[0] for side in ("left", "right")):
                    return False
            return True
        except (AttributeError, TypeError, ValueError, OverflowError):
            return False

    def put(self, kind: str, value: Any = None) -> None:
        receipt = time.monotonic_ns()
        with self._lock:
            if (kind == "frame" and self._events and self._events[-1][0] == "frame"
                    and self._continuous_frames(self._events[-1][1], value)
                    and not (len(self._events) > 1 and self._events[-2][0] == "frame"
                             and self._events[-2][1].timestamp_ms >= self._events[-1][1].timestamp_ms)):
                self._events[-1] = (kind, value, receipt)
                return
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
            poll_started = time.monotonic()
            try:
                response = self.client.request(op, session_id=session)
                error = ""
            except RuntimeError as exc:
                response, error = None, str(exc)
            self.events.put("control", (op, generation, session, response, error))
            next_poll = poll_started + 0.05


class _FollowControl:
    """Solver-owned authorization handshake; never starts on a status poll."""

    def __init__(self, core: Any, worker: _ControlWorker) -> None:
        self.core, self.worker = core, worker
        self.generation = 0
        self.pending = False
        self.following = False
        self.session = core.session_id
        self.receiver: dict[str, Any] | None = None
        self.detail = "先 K 标准掌姿校准，再 C 位置对齐/预览；只有 F 授权确认后才跟随"
        self.worker.context(self.generation, self.session)

    @staticmethod
    def _ready(status: dict[str, Any]) -> bool:
        return (status["fresh"] and status["input_mask"] == 7
                and status["ready_mask"] == 7 and status["state"] not in {"aligning", "calibrating"})

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
        if command in {"calibrate", "align", "hold"}:
            self._hold("本地已保持；正在请求 SPD 保持。恢复需要 C 对齐及 F 确认")
            if command in {"calibrate", "align"}:
                accepted = self.core.command(command)
                self.session = self.core.session_id
                self.worker.context(self.generation, self.session)
                action = ("标准掌姿校准：头朝前、手指向前、掌心向下，请保持稳定"
                          if command == "calibrate" else
                          "正在按实际保持位置对齐；完成后仅预览，F 前不跟随")
                self.detail = (action if accepted else
                               "校准/对齐被拒绝：" + self.core.status()["reason"])
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
        # Measurement is independent of authorization. A hold reply can belong
        # to the prior session after C/K; the core rejects old sample timestamps.
        if not error and response is not None and response.get("ok"):
            feedback = response.get("feedback")
            if feedback is not None:
                self.core.update_feedback(feedback)
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
        default_font.configure(family="Noto Sans CJK SC", size=13)
    else:
        default_font.configure(family="sans", size=13)
    root.title("PICO -> SPD | palm calibration and target-only preview | domain 120")
    root.geometry("1240x900")
    root.minsize(1000, 820)
    status = tk.StringVar(value="等待 PICO 输入和 SPD 实际状态；保持，不会运动")
    tk.Label(root, text="PICO 实时控制 — K 标准掌姿 → C 位置对齐/预览 → F 确认跟随",
             font=(default_font.actual("family"), 15, "bold")).pack(pady=8)
    tk.Label(root, textvariable=status, font=default_font, justify="left", anchor="nw",
             height=7, wraplength=1180).pack(fill="x", padx=18, pady=4)
    controls = tk.Frame(root)
    controls.pack(pady=6)
    follow_label = "确认并跟随 (F)" if integrated else "未连接：禁止跟随 (F)"
    for label, command in (("标准掌姿校准 (K)", "calibrate"), ("位置对齐 / 预览 (C)", "align"),
                           (follow_label, "start"), ("保持 Hold (Space)", "hold")):
        tk.Button(controls, text=label, font=default_font,
                  command=lambda command=command: events.put("command", command)).pack(side="left", padx=6)
    steps = (
        "K：先保持；头朝正前、手指向前、掌心向下，稳住双腕，采集 10 个稳定样本。\n"
        "C：按机器人新鲜、静止的实际掌位对齐（保留 K 校准）；移动双手检查目标坐标轴，仅预览。\n"
        "F：三组全部就绪且 SPD 当前会话授权确认后才跟随。Space：源端和 SPD 保持；恢复需 C → F。"
        if integrated else
        "未连接 SPD 控制接口 — 仅显示，无法位置对齐或跟随。\n"
        "K：头朝前、手指向前、掌心向下并保持稳定。取得新鲜实际状态后才能 C 对齐/预览。\n"
        "生产启动请使用 pixi run spd-pico，或提供 --control-socket；无实际反馈时禁止跟随。"
    )
    tk.Label(root, font=default_font, justify="left", wraplength=1180, text=steps + (
        "\nK / C / F / 空格仅在本窗口获得焦点时有效；预览不是实际运动，也不是精度/避碰保证。"
    )).pack(fill="x", padx=18, pady=6)
    tk.Canvas(root, name="palm_preview", background="white", height=300,
              highlightthickness=0).pack(fill="both", expand=True, padx=12, pady=4)
    tk.Label(root, name="palm_errors", font=("sans", 11), justify="left", anchor="nw",
             height=8, wraplength=1180, text="Actual / solver diagnostics: unavailable").pack(fill="x", padx=18, pady=6)
    pressed: set[str] = set()
    releases: dict[str, Any] = {}
    shortcuts = {"k": "calibrate", "c": "align", "f": "start", "space": "hold"}

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


def _update_control_window(root: Any, label: Any, status: dict[str, Any], integrated: bool) -> None:
    """Render a production status without sending any control command."""
    from .palm_preview import palm_preview_text, render_palm_preview

    label.set(_status_text(status, integrated))
    render_palm_preview(root.nametowidget("palm_preview"), status)
    root.nametowidget("palm_errors").configure(text=palm_preview_text(status))


def _status_text(status: dict[str, Any], integrated: bool) -> str:
    groups = []
    for name, bit in (("双臂（双腕）", 1), ("左手", 4), ("右手", 2)):
        state = ("跟随中" if status["running_mask"] & bit else
                 "就绪 / 保持" if status["ready_mask"] & bit else
                 "已跟踪 / 待对齐" if status["input_mask"] & bit else "丢失 / 保持")
        groups.append(f"{name}: {state}")
    mapping = status.get("mapping") or {}
    calibration = ("正在采集标准掌姿" if mapping.get("calibrating") else
                   "已完成" if mapping.get("calibrated") else "需要 K 标准掌姿校准")
    alignment = ("正在采集稳定手腕" if status["state"] == "aligning" else
                 "已对齐 / 可预览" if status["ready_mask"] & 1 else "需要 C 位置对齐")
    lines = [f"PICO: {'新鲜有效' if status['fresh'] else '等待 / 已过期'} | K: {calibration} | C: {alignment}",
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
        lines.append("SPD: 未连接 / 无实际状态 — 无法校准、对齐或授权跟随")
    lines.append("源端诊断: " + status["reason"])
    return "\n".join(lines)


def run_publisher(args: argparse.Namespace) -> int:
    if not getattr(args, "control_socket", None):
        raise SystemExit("Actual SPD state is required: use pixi run spd-pico or provide "
                         "--control-socket for actual state. Manual SPD e cannot replace feedback.")
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
    integrated = True

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
        from .local_control import LocalControlClient

        control_worker = _ControlWorker(LocalControlClient(args.control_socket), events)
        control_worker.thread.start()

        def solve() -> None:
            next_tick = next_publish = time.monotonic_ns()
            last_status: tuple[Any, ...] | None = None
            control = _FollowControl(core, control_worker)
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
                        elif kind == "control":
                            control.result(value)
                        elif kind == "command":
                            if value == "quit":
                                stop.set()
                            else:
                                control.command(value)
                        control.observe()
                    now = time.monotonic_ns()
                    core.tick(now)
                    control.observe()
                    if now >= next_publish:
                        publisher.publish(message_from_snapshot(core.snapshot()))
                        status = core.status()
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
                        # Skip missed deadlines without adding another idle period.
                        next_tick = finished
            except BaseException as exc:
                errors.put(exc)
                stop.set()
            finally:
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
                    if command in {"calibrate", "align", "start", "confirm", "hold", "quit"}:
                        events.put("command", command)
                    elif command:
                        print("Commands: calibrate (standard palms/head forward) / align (preview) / "
                              "confirm (or start) / hold / quit", flush=True)
            threading.Thread(target=stdin_commands, name="pico-stdin", daemon=True).start()
        print("PICO publisher ready (process/control ready; waiting for live tracking, not robot-ready)", flush=True)
        while not stop.is_set():
            if root is not None:
                try:
                    status = statuses.get_nowait()
                    _update_control_window(root, label, status, integrated)
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
    parser.add_argument("--control-socket", help="Required private SPD control socket for actual state and authorization; use pixi run spd-pico")
    parser.add_argument("--headless", action="store_true", help="stdin calibrate/align/confirm/start/hold/quit; smoke testing only")
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535 or not 1 <= args.device_port <= 65535:
        parser.error("ports must be in 1..65535")
    import math
    if not math.isfinite(args.reconnect) or args.reconnect < 0.1:
        parser.error("--reconnect must be finite and >= 0.1 seconds")
    return run_publisher(args)


if __name__ == "__main__":
    raise SystemExit(main())
