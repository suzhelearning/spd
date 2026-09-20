"""Publish recorded observations as synthetic ROS targets, never original commands.

The desktop panel previews the original JPEGs. Run ros_viewer separately for
MuJoCo target/actual visualization; this process only publishes JointCommand.
"""
from __future__ import annotations

# ROS must load before NumPy/HDF5 and other native libraries (C++ symbol order).
try:
    import rclpy
except ImportError:
    rclpy = None

import argparse
from dataclasses import dataclass
from io import BytesIO
import math
import os
from pathlib import Path
import queue
import selectors
import signal
import sys
import threading
import time
import uuid

import numpy as np

from .manifest import ManifestJoint, load_manifest, resolve_home_positions
from .recorded_targets import RecordedTargets
from .ros_joint_command import (
    JOINT_NAME_TUPLE,
    ROBOT_CONFIG,
    TOPIC,
    VALID_READY_MASK,
    JointCommandSnapshot,
    best_effort_qos,
    message_from_snapshot,
)


@dataclass(frozen=True)
class PlaybackStatus:
    file_index: int
    phase: str
    elapsed_s: float
    transition_s: float
    transition_duration_s: float
    published: int
    error: str = ""


class RecordedPublisher:
    """One publishing owner; UI/stdin only enqueue commands or read snapshots."""

    def __init__(self, recordings: tuple[RecordedTargets, ...], home: np.ndarray,
                 limits: np.ndarray, *, rate_hz: float, speed: float, loop: bool) -> None:
        self.recordings = recordings
        self.rate_hz = rate_hz
        self.speed = speed
        self.loop = loop
        self.commands: queue.SimpleQueue[str] = queue.SimpleQueue()
        self.stop = threading.Event()
        self.session_id = uuid.uuid4().hex
        self._lock = threading.Lock()
        self._status = PlaybackStatus(0, "WAITING", 0.0, 0.0, 0.0, 0)
        self._index = 0
        self._phase = "WAITING"
        self._resume = "WAITING"
        self._elapsed = 0.0
        self._transition_elapsed = 0.0
        self._transition_duration = 0.0
        self._position = home.copy()
        self._limits = limits
        self._transition_start = home.copy()
        self._transition_delta = np.zeros(54, dtype=np.float64)
        self._published = 0
        self._error = ""
        self._check_position(self._position)

    def command(self, command: str) -> None:
        if command not in {"play", "pause", "next"}:
            raise ValueError(f"unknown playback command: {command}")
        self.commands.put(command)

    def status(self) -> PlaybackStatus:
        with self._lock:
            return self._status

    def _share(self) -> None:
        status = PlaybackStatus(
            self._index, self._phase, self._elapsed, self._transition_elapsed,
            self._transition_duration, self._published, self._error,
        )
        with self._lock:
            self._status = status

    def _log_file(self) -> None:
        recording = self.recordings[self._index]
        print(f"FILE {self._index + 1}/{len(self.recordings)} {recording.path} "
              f"duration={recording.duration_s:.3f}s task={recording.task!r} "
              "source=observations synthetic_targets=true", flush=True)

    def _set_phase(self, phase: str) -> None:
        if phase != self._phase:
            self._phase = phase
            detail = ""
            if phase == "TRANSITION":
                detail = (f" duration={self._transition_duration:.3f}s "
                          "synthetic_not_recorded=true max_joint_rate=0.3rad/s")
            elif phase == "PLAYING":
                detail = " observation_replay=true original_commands=false"
            elif phase in {"PAUSED", "DONE", "WAITING"}:
                detail = " publishing=held_target"
            print(f"PHASE {phase}{detail}", flush=True)

    def _check_position(self, position: np.ndarray) -> None:
        if position.shape != (54,) or not np.all(np.isfinite(position)):
            raise ValueError("recorded target must contain 54 finite joint values")
        invalid = np.flatnonzero((position < self._limits[:, 0]) | (position > self._limits[:, 1]))
        if invalid.size:
            index = int(invalid[0])
            raise ValueError(f"{JOINT_NAME_TUPLE[index]} target {position[index]} exceeds "
                             f"manifest range {tuple(self._limits[index])}; refusing to clamp")

    def _begin_transition(self) -> None:
        target = self.recordings[self._index].sample(0.0)
        self._check_position(target)
        self._elapsed = 0.0
        self._transition_elapsed = 0.0
        self._transition_start = self._position.copy()
        self._transition_delta = target - self._position
        # Cubic smoothstep's peak derivative is 1.5. Speed never accelerates
        # these synthetic transitions, including transitions after Next/loop.
        self._transition_duration = max(2.0, 1.5 * float(np.max(np.abs(self._transition_delta))) / 0.3)
        self._set_phase("TRANSITION")

    def _next(self) -> bool:
        if self._index + 1 == len(self.recordings) and not self.loop:
            self._set_phase("DONE")
            return False
        self._index = (self._index + 1) % len(self.recordings)
        self._elapsed = 0.0
        self._log_file()
        return True

    def _consume_commands(self) -> bool:
        changed = False
        while True:
            try:
                command = self.commands.get_nowait()
            except queue.Empty:
                return changed
            changed = True
            if command == "pause" and self._phase in {"TRANSITION", "PLAYING"}:
                self._resume = self._phase
                self._set_phase("PAUSED")
            elif command == "play":
                if self._phase == "PAUSED":
                    self._set_phase(self._resume)
                elif self._phase in {"WAITING", "DONE"}:
                    if self._phase == "DONE":
                        self._index = 0
                        self._log_file()
                    self._begin_transition()
            elif command == "next":
                prior = self._phase
                if self._next():
                    if prior == "WAITING":
                        continue
                    self._begin_transition()
                    if prior == "PAUSED":
                        self._resume = "TRANSITION"
                        self._set_phase("PAUSED")

    def _advance(self, dt: float) -> None:
        if self._phase == "TRANSITION":
            self._transition_elapsed = min(self._transition_duration, self._transition_elapsed + dt)
            fraction = self._transition_elapsed / self._transition_duration
            weight = fraction * fraction * (3.0 - 2.0 * fraction)
            self._position = self._transition_start + weight * self._transition_delta
            if self._transition_elapsed >= self._transition_duration:
                self._position = self.recordings[self._index].sample(0.0)
                self._set_phase("PLAYING")
        elif self._phase == "PLAYING":
            recording = self.recordings[self._index]
            self._elapsed = min(recording.duration_s, self._elapsed + dt * self.speed)
            self._position = recording.sample(self._elapsed)
            self._check_position(self._position)
            if self._elapsed >= recording.duration_s and self._next():
                self._begin_transition()

    def run(self, publisher: object) -> None:
        period = 1.0 / self.rate_hz
        previous = time.monotonic()
        deadline = previous
        self._log_file()
        print(f"SESSION {self.session_id} domain=120 topic={TOPIC} rate={self.rate_hz:g}Hz "
              f"speed={self.speed:g} loop={self.loop}", flush=True)
        print("PHASE WAITING publishing=manifest_home; authorize SPD, then Play", flush=True)
        try:
            while not self.stop.is_set() and rclpy.ok():
                if self.stop.wait(max(0.0, deadline - time.monotonic())):
                    break
                now = time.monotonic()
                dt = now - previous
                previous = now
                if self._consume_commands():
                    dt = 0.0
                self._advance(dt)
                snapshot = JointCommandSnapshot.from_values(
                    session_id=self.session_id,
                    sequence=self._published + 1,
                    ready_mask=VALID_READY_MASK,
                    position_rad=self._position,
                    stamp_ns=time.time_ns(),
                )
                publisher.publish(message_from_snapshot(snapshot))
                self._published += 1
                self._share()
                deadline += period
                finished = time.monotonic()
                if deadline <= finished:
                    # Missed ticks are discarded, never replayed in a burst.
                    deadline = finished + period
        except Exception as exc:
            self._error = f"{type(exc).__name__}: {exc}"
            self._set_phase("ERROR")
            print(f"ERROR {self._error}", flush=True)
        finally:
            self._share()
            self.stop.set()


def _stdin_commands(player: RecordedPublisher) -> None:
    """Polling raw fd reads make shutdown joinable even with an idle terminal."""
    try:
        fd = sys.stdin.fileno()
        pending = b""
        with selectors.DefaultSelector() as selector:
            selector.register(fd, selectors.EVENT_READ)
            while not player.stop.is_set():
                if not selector.select(timeout=0.1):
                    continue
                chunk = os.read(fd, 4096)
                if not chunk:
                    if pending.strip():
                        _stdin_line(player, pending)
                    return
                pending += chunk
                while b"\n" in pending:
                    line, pending = pending.split(b"\n", 1)
                    _stdin_line(player, line)
    except (OSError, ValueError) as exc:
        print(f"STDIN unavailable: {exc}; desktop buttons remain available", flush=True)


def _stdin_line(player: RecordedPublisher, line: bytes) -> None:
    command = line.decode("utf-8", errors="replace").strip().lower()
    if command in {"play", "pause", "next"}:
        player.command(command)
    elif command:
        print(f"STDIN ignored {command!r}; use play/pause/next", flush=True)


class PlaybackPanel:
    def __init__(self, root: object, player: RecordedPublisher) -> None:
        import tkinter as tk
        from tkinter import ttk
        from PIL import Image, ImageTk

        self.root = root
        self.player = player
        self._Image = Image
        self._ImageTk = ImageTk
        self._photos: dict[str, object] = {}
        self._last_preview: tuple[int, float] | None = None
        root.title("SPD recorded observations -> synthetic ROS targets (domain 120)")
        root.geometry("760x950")
        root.protocol("WM_DELETE_WINDOW", self.close)
        frame = ttk.Frame(root, padding=12)
        frame.pack(fill="both", expand=True)
        ttk.Label(frame, text="Observation replay / synthetic test targets — NOT original commands",
                  font=("TkDefaultFont", 12, "bold"), wraplength=720).pack(anchor="w")
        ttk.Label(frame, text="Start: hold manifest home, authorize SPD in its viewer, then Play. "
                  "Pause keeps publishing; closing this panel stops publishing.",
                  wraplength=720).pack(anchor="w", pady=5)
        self.file_text = tk.StringVar()
        self.state_text = tk.StringVar()
        self.progress_text = tk.StringVar()
        self.count_text = tk.StringVar()
        for variable in (self.file_text, self.state_text, self.progress_text, self.count_text):
            ttk.Label(frame, textvariable=variable, wraplength=720).pack(anchor="w", pady=3)
        self.progress = ttk.Progressbar(frame, maximum=1.0)
        self.progress.pack(fill="x", pady=8)
        buttons = ttk.Frame(frame)
        buttons.pack(anchor="w")
        for label, command in (("Play", "play"), ("Pause", "pause"), ("Next", "next")):
            ttk.Button(buttons, text=label, command=lambda value=command: player.command(value)).pack(
                side="left", padx=(0, 10))
        ttk.Label(frame, text="Original recorded RGB (10 Hz preview); during transitions these images "
                  "show the destination file's first frame, not the synthetic motion.",
                  wraplength=720).pack(anchor="w", pady=10)
        images = ttk.Frame(frame)
        images.pack(fill="both", expand=True)
        self.image_labels = {}
        for row, name in enumerate(("top", "left_wrist")):
            box = ttk.LabelFrame(images, text=f"Original RGB: {name}", padding=4)
            box.grid(row=row, column=0, sticky="nsew", pady=3)
            images.rowconfigure(row, weight=1)
            label = ttk.Label(box, anchor="center", text="No frame")
            label.pack(fill="both", expand=True)
            self.image_labels[name] = label
        images.columnconfigure(0, weight=1)
        self.preview_text = tk.StringVar()
        ttk.Label(frame, textvariable=self.preview_text, wraplength=720).pack(anchor="w")
        root.after(100, self.refresh)

    def close(self) -> None:
        self.player.stop.set()
        self.root.quit()

    def refresh(self) -> None:
        status = self.player.status()
        recording = self.player.recordings[status.file_index]
        self.file_text.set(f"File {status.file_index + 1}/{len(self.player.recordings)}: {recording.name} "
                           f"| task metadata: {recording.task}")
        descriptions = {
            "WAITING": "holding manifest home; waiting for Play",
            "TRANSITION": "synthetic smooth transition (not recorded samples)",
            "PLAYING": "replaying observations as synthetic targets",
            "PAUSED": "paused; continuing to publish last target",
            "DONE": "playlist ended; continuing to publish last target",
            "ERROR": status.error,
        }
        self.state_text.set(f"{status.phase}: {descriptions[status.phase]}")
        transition = ""
        if status.phase == "TRANSITION":
            transition = f" | transition {status.transition_s:.2f}/{status.transition_duration_s:.2f}s"
        self.progress_text.set(f"Recording time {status.elapsed_s:.2f}/{recording.duration_s:.2f}s"
                               f" | speed {self.player.speed:g}x{transition}")
        self.progress["value"] = status.elapsed_s / recording.duration_s if recording.duration_s else 0.0
        self.count_text.set(f"Published: {status.published} | ROS domain 120 | {TOPIC}")
        preview = (status.file_index, status.elapsed_s)
        if preview != self._last_preview:
            self._last_preview = preview
            errors = []
            for name, label in self.image_labels.items():
                try:
                    jpeg = recording.image_jpeg(name, status.elapsed_s) if name in recording.camera_names else None
                    if jpeg is None:
                        self._photos.pop(name, None)
                        label.configure(image="", text="No recorded frame at this time")
                    else:
                        with self._Image.open(BytesIO(jpeg)) as source:
                            image = source.convert("RGB")
                        image.thumbnail((700, 260))
                        photo = self._ImageTk.PhotoImage(image, master=self.root)
                        self._photos[name] = photo
                        label.configure(image=photo, text="")
                except Exception as exc:
                    self._photos.pop(name, None)
                    label.configure(image="", text=f"Preview unavailable: {exc}")
                    errors.append(f"{name}: {exc}")
            self.preview_text.set("; ".join(errors))
        if self.player.stop.is_set():
            self.root.quit()
        else:
            # Scheduling after decode also keeps slow storage/decoders below 10 Hz.
            self.root.after(100, self.refresh)


def _positive(value: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise argparse.ArgumentTypeError("must be a finite positive number")
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paths", nargs="+", type=Path)
    parser.add_argument("--loop", action="store_true")
    parser.add_argument("--rate-hz", type=_positive, default=60.0)
    parser.add_argument("--speed", type=_positive, default=1.0)
    args = parser.parse_args(argv)
    if rclpy is None:
        parser.error("rclpy is required; activate the ROS Jazzy environment and source the interface workspace")
    import tkinter as tk
    from tianji_spd_interfaces.msg import JointCommand

    manifest_path = Path(__file__).resolve().parents[1] / "generated" / "model_manifest.yaml"
    manifest = load_manifest(manifest_path)
    joints = [ManifestJoint(**entry) for entry in manifest["joints"]]
    home = resolve_home_positions(joints, manifest)
    by_name = {joint.joint: joint for joint in joints}
    if set(by_name) != set(JOINT_NAME_TUPLE):
        raise ValueError("model manifest does not match canonical JointCommand joints")
    canonical_home = np.asarray([home[by_name[name].index] for name in JOINT_NAME_TUPLE], dtype=np.float64)
    limits = np.asarray([by_name[name].range for name in JOINT_NAME_TUPLE], dtype=np.float64)
    recordings = tuple(RecordedTargets(path) for path in args.paths)
    for recording in recordings:
        if recording.robot_config != ROBOT_CONFIG:
            raise ValueError(f"{recording.path}: unsupported robot_config {recording.robot_config!r}")
    player = RecordedPublisher(recordings, canonical_home, limits,
                               rate_hz=args.rate_hz, speed=args.speed, loop=args.loop)
    root = tk.Tk()
    node = None
    workers: list[threading.Thread] = []
    handlers = {}
    initialized = False
    try:
        PlaybackPanel(root, player)
        # Both recorded publisher and SPD subscriber use direct DDS domain 120.
        rclpy.init(args=[], domain_id=120)
        initialized = True
        node = rclpy.create_node("spd_recorded_target_publisher")
        publisher = node.create_publisher(JointCommand, TOPIC, best_effort_qos())
        for sig in (signal.SIGINT, signal.SIGTERM):
            handlers[sig] = signal.getsignal(sig)
            signal.signal(sig, lambda *_: player.stop.set())
        for target, arguments, name in (
            (player.run, (publisher,), "recorded-ros-publisher"),
            (_stdin_commands, (player,), "recorded-stdin"),
        ):
            worker = threading.Thread(target=target, args=arguments, name=name)
            worker.start()
            workers.append(worker)
        root.mainloop()
    finally:
        player.stop.set()
        for worker in workers:
            worker.join()
        if node is not None:
            node.destroy_node()
        if initialized:
            rclpy.try_shutdown()
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
        root.destroy()
    return 1 if player.status().error else 0


if __name__ == "__main__":
    raise SystemExit(main())
