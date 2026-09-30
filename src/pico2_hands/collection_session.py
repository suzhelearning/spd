"""Single-owner, simulation-only relative PICO collection backend.

The physics owner freezes the scene before rebind/Home. Retained targets, not
measured plant qpos, are the authority. No ROS publisher or keyboard runs here.
"""
from __future__ import annotations

from contextlib import ExitStack
from dataclasses import dataclass
import errno
import hashlib
import math
from pathlib import Path
import select
import socket
import threading
import time
import uuid

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from .dls_worker import DlsWorker
from .hand_worker import NativeHandWorker
from .reference.models import PicoRawFrame
from .reference.official_pico import (
    pico_official_hand_observations, pico_official_hand2_retarget_input,
)
from .reference.runtime import PicoPacketStream
from .resources import controller_profile, display_model_path


_SIDES = ("left", "right")
_REGIONS = {"left": slice(14, 34), "right": slice(34, 54)}
_FRESH_NS = 45_000_000
_STABLE_NS = 100_000_000
_DROPOUT_NS = 120_000_000
_PERIOD = .005


@dataclass(frozen=True, slots=True)
class TeleopSnapshot:
    generation: int
    sequence: int
    position_rad: tuple[float, ...]
    generated_ns: int
    input_ns: int
    can_bind: bool
    arms_valid: bool
    needs_rebind: bool
    control_flags: int
    finger_modes: tuple[str, str]  # Left, right; independent of transient quality flags.
    mode: str
    fault: str


class _RelativeAnchor:
    def __init__(self, height_m):
        if isinstance(height_m, bool) or not isinstance(height_m, (int, float)) or not math.isfinite(height_m) or not 1 <= height_m <= 2.4:
            raise ValueError("shared-root requires height in metres within [1.0, 2.4]")
        height = float(height_m)
        human_width = height * .1828
        human_reach = height * .155882 + height * .152941 + height * .037037
        geometry = yaml.safe_load(controller_profile("shared_root_robot_geometry_dls.yaml").read_text())["robot_geometry"]
        for path_key, hash_key in (("urdf_path", "urdf_sha256"), ("mujoco_xml_path", "mujoco_xml_sha256")):
            if hashlib.sha256(display_model_path(Path(geometry[path_key]).name).read_bytes()).hexdigest() != geometry[hash_key]:
                raise ValueError("shared-root geometry/model fingerprint mismatch")
        self.robot_rotation = np.array(geometry["R_BCt"], float)
        width = np.linalg.norm(np.array(geometry["left_shoulder_B_m"])-geometry["right_shoulder_B_m"])
        reach = .5 * (sum(geometry["left_segment_lengths_m"]) + sum(geometry["right_segment_lengths_m"]))
        self.scale = np.array([reach/human_reach, width/human_width, reach/human_reach])
        self.bound = False

    @staticmethod
    def _pose(pose):
        p = np.asarray(pose, float)
        if p.shape != (7,) or not np.isfinite(p).all() or not 1e-12 < np.linalg.norm(p[3:]) < 1e12:
            raise ValueError("invalid tracked pose")
        return p, Rotation.from_quat(p[3:])

    @staticmethod
    def extract(frame):
        if not frame.head_valid:
            raise ValueError("head tracking invalid")
        head, head_rotation = _RelativeAnchor._pose(frame.head_pose)
        wrists, rotations = [], []
        for side in _SIDES:
            hand = frame.hands[side]
            # The v1 wrist flag is independent of per-finger joint validity.
            # Do not derive arm validity from a complete Hand2 skeleton.
            if not (hand.valid and hand.wrist_valid):
                raise ValueError("wrist tracking invalid")
            pose, rotation = _RelativeAnchor._pose(hand.wrist_pose)
            wrists.append(pose[:3])
            rotations.append(rotation)
        return head[:3], head_rotation, np.asarray(wrists), rotations

    def bind(self, geometry, reference):
        head, head_rotation, wrists, rotations = geometry
        heading = head_rotation.apply([1., 0., 0.])
        heading[2] = 0.
        length = np.linalg.norm(heading)
        if length < .5:
            raise ValueError("look approximately level while binding")
        heading /= length
        self.basis = np.column_stack((heading, np.cross([0., 0., 1.], heading), [0., 0., 1.]))
        self.relative = wrists - head
        self.positions = np.asarray([reference[s]["achieved_pose"][:3] for s in _SIDES])
        self.corrections = []
        for i, side in enumerate(_SIDES):
            desired = Rotation.from_quat(reference[side]["achieved_pose"][3:]).as_matrix()
            mapped = self.robot_rotation @ self.basis.T @ rotations[i].as_matrix()
            self.corrections.append(mapped.T @ desired)
        self.bound = True

    def targets(self, geometry):
        head, _, wrists, rotations = geometry
        displacement = ((wrists - head - self.relative) @ self.basis * self.scale) @ self.robot_rotation.T
        if not np.isfinite(displacement).all() or np.max(np.linalg.norm(displacement, axis=1)) > 1.5:
            raise ValueError("relative wrist displacement exceeds target gate")
        quaternions = [Rotation.from_matrix(
            self.robot_rotation @ self.basis.T @ rotations[i].as_matrix() @ self.corrections[i]
        ).as_quat() for i in range(2)]
        return np.column_stack((self.positions + displacement, quaternions))


class _FingerGate:
    def __init__(self):
        self.reset(np.zeros(20))

    def reset(self, retained):
        self.held = np.asarray(retained).copy()
        self.target = None
        self.stamp = 0
        self.blend_start = 0
        self.hold_start = 0
        self.valid = False
        self.mode = "waiting"

    def fresh(self, now):
        return self.valid and self.target is not None and 0 <= now - self.stamp <= _FRESH_NS

    def _hold(self, retained, now):
        if self.mode == "waiting":
            return
        if not 0 <= now - self.stamp < _DROPOUT_NS:
            self.reset(retained)
        elif not self.hold_start:
            self.hold_start = now if not self.valid else min(now, self.stamp + _FRESH_NS)

    def invalidate(self, retained, now):
        self.valid = False
        self._hold(retained, now)

    def observe(self, target, stamp, retained):
        if target.shape != (20,) or not np.isfinite(target).all():
            raise RuntimeError("invalid Hand2 target")
        if stamp <= self.stamp:
            return
        if self.stamp and stamp - self.stamp >= _DROPOUT_NS:
            self.reset(retained)
        elif self.stamp and stamp - self.stamp > _FRESH_NS and not self.hold_start:
            self.hold_start = self.stamp + _FRESH_NS
        self.valid = True
        self.target, self.stamp = target, stamp
        if self.mode != "waiting":
            return
        self.held = retained.copy()
        self.blend_start = 0
        self.mode = "blend"

    def advance(self, retained, now, dt):
        if not self.fresh(now):
            self._hold(retained, now)
            return
        if self.hold_start:
            if self.blend_start:
                self.blend_start += now - self.hold_start
            self.hold_start = 0
        if self.mode == "waiting":
            return
        desired = self.target
        if self.mode == "blend":
            if not self.blend_start:
                self.blend_start = now
            u = min(1., max(0., (now - self.blend_start) / 200_000_000))
            weight = u * u * u * (10. + u * (-15. + 6. * u))
            desired = self.held + weight * (self.target - self.held)
            if u >= 1.:
                self.mode = "live"
        retained += np.clip(desired - retained, -2. * dt, 2. * dt)


class _TcpInput:
    """Nonblocking TCP polling on the same owner as DLS and both Hand2 workers."""
    def __init__(self, host, port, connected=None):
        self.address = (host, port)
        self.receiver_id = uuid.uuid4().hex
        self.generation = int(connected is not None)
        self.sock = connected
        if connected is not None:
            connected.setblocking(False)
        self.connecting = False
        self.retry_at = 0.
        self.connect_started = 0.
        self.stream = (PicoPacketStream(receiver_instance_id=self.receiver_id,
                                       connection_generation=self.generation)
                       if connected is not None else None)

    def close(self):
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    def poll(self, now):
        if self.sock is None:
            if now < self.retry_at:
                return None, False
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.setblocking(False)
            result = self.sock.connect_ex(self.address)
            if result not in (0, errno.EINPROGRESS, errno.EWOULDBLOCK):
                self.close()
                self.retry_at = now + 1.
                return None, False
            self.connecting = True
            self.connect_started = now
        if self.connecting:
            _, writable, errors = select.select([], [self.sock], [self.sock], 0)
            if not writable and not errors and now - self.connect_started <= 3.:
                return None, False
            if errors or not writable or self.sock.getsockopt(socket.SOL_SOCKET, socket.SO_ERROR):
                self.close()
                self.retry_at = now + 1.
                return None, False
            self.connecting = False
            self.generation += 1
            self.stream = PicoPacketStream(receiver_instance_id=self.receiver_id,
                                           connection_generation=self.generation)
        latest = None
        try:
            # Bound transport work per tick; the input slot still contains only
            # the most recent complete frame. Partial TCP packets remain framed.
            for _ in range(4):
                try:
                    data = self.sock.recv(64 * 1024)
                except BlockingIOError:
                    break
                if not data:
                    raise ConnectionError("PICO disconnected")
                frames = self.stream.feed(data)
                if frames:
                    latest = frames[-1]
        except OSError:
            self.close()
            self.retry_at = now + 1.
            return None, True
        return latest, False


class TeleopSession:
    def __init__(self, height_m, *, host="127.0.0.1", port=10002, start_receiver=True):
        if isinstance(height_m, bool) or not isinstance(height_m, (int, float)) or not math.isfinite(height_m) or not 1. <= height_m <= 2.4:
            raise ValueError("height_m must be within [1.0, 2.4] metres")
        if not isinstance(host, str) or not host or type(port) is not int or not 1 <= port <= 65535:
            raise ValueError("valid TCP host and port required")
        self._height = height_m
        self._host, self._port, self._start_receiver = host, port, start_receiver
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._generation = self._revision = 0
        self._request = self._latest = self._follow = None
        self._intent = "waiting"
        self._closed = False
        self._startup_error = None
        self._snapshot = TeleopSnapshot(0, 0, (0.,) * 54, 0, 0, False, False, True, 7,
                                        ("waiting", "waiting"), "waiting", "")
        self._thread = threading.Thread(target=self._run, name="collection-teleop", daemon=False)
        self._thread.start()
        self._ready.wait()
        if self._startup_error is not None:
            self._thread.join()
            raise RuntimeError(f"teleop startup failed: {self._startup_error}") from self._startup_error

    @staticmethod
    def _joints(joints):
        values = np.asarray(joints, dtype=float)
        if values.shape != (54,) or not np.isfinite(values).all():
            raise ValueError("54 finite canonical joint targets required")
        return values.copy()

    def _available(self):
        if self._closed:
            raise RuntimeError("teleop session is closed")
        if self._snapshot.fault:
            raise RuntimeError(self._snapshot.fault)

    def feed(self, frame):
        if not isinstance(frame, PicoRawFrame):
            raise TypeError("feed requires a decoded PicoRawFrame")
        with self._lock:
            self._available()
            self._latest = frame

    def snapshot(self):
        with self._lock:
            return self._snapshot

    def rebind(self, joints):
        retained = self._joints(joints)
        with self._lock:
            self._available()
            self._generation += 1
            self._revision += 1
            self._intent = "rebind"
            self._follow = None
            self._request = (self._revision, self._generation, "rebind", retained)
            return self._generation

    def follow(self, generation):
        with self._lock:
            self._available()
            if type(generation) is int and generation == self._generation and self._intent == "rebind":
                self._follow = generation

    def pause(self):
        with self._lock:
            if self._closed or self._snapshot.fault or self._intent == "waiting":
                return
            self._revision += 1
            self._intent = "waiting"
            self._follow = None
            retained = np.asarray(self._snapshot.position_rad)
            self._request = (self._revision, self._generation, "waiting", retained)

    def close(self):
        with self._lock:
            self._closed = True
            self._stop.set()
        self._thread.join()

    def _invalidate_reference(self):
        self._needs_rebind = True
        self._anchor.bound = False
        self._stable_first = 0
        self._stable_count = 0
        self._stable_geometry = None
        self._geometry = None
        self._arms_valid = False
        for side in _SIDES:
            self._fingers[side].reset(self._q[_REGIONS[side]])

    def _observe(self, frame, now):
        identity = (frame.receiver_instance_id, frame.connection_generation)
        if identity != self._identity:
            self._invalidate_reference()
            self._identity = identity
            self._last_sequence = self._last_source = self._last_received = -1
            self._previous_geometry = None
        if frame.receiver_frame_sequence == self._last_sequence and frame.source_timestamp_ns == self._last_source:
            return
        if frame.receiver_frame_sequence <= self._last_sequence or frame.source_timestamp_ns < self._last_source:
            self._invalidate_reference()
            return
        self._last_sequence = frame.receiver_frame_sequence
        # Duplicate source samples are not fresh observations, even when TCP
        # delivery supplies a later reception time or sequence number.
        if frame.source_timestamp_ns == self._last_source or frame.received_timestamp_ns <= self._last_received:
            return
        self._last_source = frame.source_timestamp_ns
        self._last_received = frame.received_timestamp_ns
        if not 0 <= now - frame.received_timestamp_ns <= _FRESH_NS:
            self._arms_valid = False
            self._stable_first = self._stable_count = 0
            return
        self._frame = frame
        try:
            geometry = self._anchor.extract(frame)
        except ValueError:
            geometry = None
        if geometry is not None and self._previous_geometry is not None:
            previous, stamp = self._previous_geometry
            if frame.received_timestamp_ns - stamp <= _DROPOUT_NS:
                jumped = max(np.max(np.abs(geometry[0] - previous[0])),
                             np.max(np.abs(geometry[2] - previous[2]))) > .25
                rotated = (previous[1].inv() * geometry[1]).magnitude() > .8
                rotated = rotated or any((previous[3][i].inv() * geometry[3][i]).magnitude() > 1.2 for i in range(2))
                if jumped or rotated:
                    self._invalidate_reference()
        self._geometry = geometry
        self._arms_valid = geometry is not None
        if geometry is not None:
            self._input_ns = frame.received_timestamp_ns
            self._previous_geometry = (geometry, frame.received_timestamp_ns)
            if self._mode in ("waiting", "bound"):
                stable = self._stable_geometry is not None
                if stable:
                    old = self._stable_geometry
                    stable = max(np.max(np.abs(geometry[0] - old[0])),
                                 np.max(np.abs(geometry[2] - old[2]))) <= .02
                    stable = stable and (old[1].inv() * geometry[1]).magnitude() <= .10
                    stable = stable and all((old[3][i].inv() * geometry[3][i]).magnitude() <= .10 for i in range(2))
                if not stable or frame.received_timestamp_ns - self._stable_last > _FRESH_NS:
                    self._stable_first = frame.received_timestamp_ns
                    self._stable_count = 0
                    self._stable_geometry = geometry
                self._stable_last = frame.received_timestamp_ns
                self._stable_count += 1
        else:
            self._stable_first = self._stable_count = 0
            self._stable_geometry = None
        observations = pico_official_hand_observations(frame)
        self._hand_sequence += 1
        pending = []
        for side in _SIDES:
            if observations[side].valid:
                worker = self._hands[side]
                worker.submit(pico_official_hand2_retarget_input(observations[side].keypoints_m),
                              self._hand_sequence, frame.received_timestamp_ns)
                pending.append((side, worker))
            else:
                self._fingers[side].invalidate(self._q[_REGIONS[side]], frame.received_timestamp_ns)
        results = [(side, worker.receive()) for side, worker in pending]
        for side, result in results:
            self._fingers[side].observe(result, frame.received_timestamp_ns, self._q[_REGIONS[side]])

    def _can_bind(self, now):
        if not self._fresh(now) or self._stable_count < 5 or self._stable_last - self._stable_first < _STABLE_NS:
            return False
        heading = self._geometry[1].apply([1., 0., 0.])
        return np.linalg.norm(heading[:2]) >= .5

    def _fresh(self, now):
        return self._arms_valid and self._geometry is not None and 0 <= now - self._input_ns <= _FRESH_NS

    def _consume(self, result):
        phase = result["left"]["status"]
        if phase != result["right"]["status"] or phase == "FAULT":
            raise RuntimeError("native bilateral recovery fault")
        self._phase = phase
        for i, side in enumerate(_SIDES):
            self._q[i * 7:(i + 1) * 7] = result[side]["joints"]
        if not np.isfinite(self._q).all():
            raise RuntimeError("nonfinite DLS output")
        return all(result[s]["accepted"] for s in _SIDES)

    def _command(self, operation, now):
        return self._consume(self._ik.command(operation, self._q[:14].reshape(2, 7), now / 1e9))

    def _apply_request(self, request):
        self._active_revision, self._active_generation, action, retained = request
        self._mode = "waiting"
        self._bind_pending = action == "rebind"
        self._anchor.bound = False
        self._needs_rebind = True
        self._stable_first = self._stable_count = 0
        self._stable_geometry = None
        if retained is not None:
            self._q[:] = retained
        for side in _SIDES:
            self._fingers[side].reset(self._q[_REGIONS[side]])
        # Pause does not advance the old trajectory. A later frozen rebind is
        # the only way to replace that suspended native numerical state.

    def _advance(self, now):
        if self._bind_pending and self._can_bind(now):
            retained = self._q.copy()
            result = self._ik.bind_frozen(retained[:14].reshape(2, 7), now / 1e9)
            if not self._consume(result):
                raise RuntimeError("native frozen binding rejected")
            if not np.array_equal(self._q, retained):
                raise RuntimeError("frozen binding changed retained targets")
            self._anchor.bind(self._geometry, result)
            self._bind_pending = False
            self._needs_rebind = False
            self._mode = "bound"
            for side in _SIDES:
                self._fingers[side].reset(self._q[_REGIONS[side]])
            return
        with self._lock:
            follow = self._follow
        if self._mode == "bound" and follow == self._active_generation and self._fresh(now) and not self._needs_rebind:
            if not self._command(4, now):
                raise RuntimeError("native follow start rejected")
            self._mode = "follow"
            return
        if self._mode == "follow":
            fresh = self._fresh(now) and not self._needs_rebind
            if not fresh:
                if self._phase == "TELEOP":
                    self._command(6, now)
                if self._phase == "BRAKING":
                    self._command(7, now)
                if self._needs_rebind or now - self._input_ns > _DROPOUT_NS:
                    self._needs_rebind = True
            else:
                if self._phase in ("BRAKING", "HOLD"):
                    if not self._command(8, now):
                        self._needs_rebind = True
                    # A start/resume command consumes this native clock tick.
                    # The next solve retains q/v/a and uses a strictly newer time.
                    return
                try:
                    targets = self._anchor.targets(self._geometry)
                except ValueError:
                    self._invalidate_reference()
                    self._command(6, now)
                    return
                result = self._ik.solve(self._q[:14].reshape(2, 7), targets,
                    source_time=self._frame.source_timestamp_ns / 1e9,
                    received_time=self._frame.received_timestamp_ns / 1e9, now=now / 1e9)
                if not self._consume(result):
                    self._needs_rebind = True
            for side in _SIDES:
                self._fingers[side].advance(self._q[_REGIONS[side]], now, _PERIOD)

    def _publish(self, now, fault=""):
        fresh = self._fresh(now)
        flags = 0 if fresh and not self._needs_rebind else 1
        for side, waiting, blending in (("right", 2, 8), ("left", 4, 16)):
            gate = self._fingers[side]
            if gate.mode == "waiting" or not gate.fresh(now):
                flags |= waiting
            elif gate.mode == "blend":
                flags |= blending
        with self._lock:
            if fault or self._active_revision == self._revision:
                self._snapshot = TeleopSnapshot(self._active_generation, self._snapshot.sequence + 1,
                    tuple(float(x) for x in self._q), now, self._input_ns, self._can_bind(now),
                    fresh, self._needs_rebind, flags,
                    (self._fingers["left"].mode, self._fingers["right"].mode),
                    "fault" if fault else self._mode, fault)

    def _run(self):
        resources = ExitStack()
        try:
            self._q = np.zeros(54)
            self._active_revision = self._active_generation = 0
            self._mode, self._phase = "waiting", "WAITING"
            self._bind_pending = False
            self._needs_rebind = True
            self._arms_valid = False
            self._input_ns = self._stable_first = self._stable_last = self._stable_count = 0
            self._geometry = self._stable_geometry = self._previous_geometry = self._frame = None
            self._identity = None
            self._last_sequence = self._last_source = self._last_received = -1
            self._hand_sequence = 0
            self._fingers = {side: _FingerGate() for side in _SIDES}
            self._anchor = _RelativeAnchor(self._height)
            tcp = None
            if self._start_receiver:
                from .input_transport import ensure_adb_forward, input_lock
                resources.enter_context(input_lock(self._port))
                connected = None
                if self._host in ("localhost", "127.0.0.1"):
                    # A directly supplied local TCP server needs no ADB. If no
                    # listener exists, use the existing non-stealing ADB setup.
                    try:
                        connected = socket.create_connection((self._host, self._port), timeout=.1)
                    except OSError:
                        ensure_adb_forward(self._port)
                tcp = _TcpInput(self._host, self._port, connected)
                resources.callback(tcp.close)
            self._ik = resources.enter_context(DlsWorker(timeout_s=.1, collection_session=True))
            self._hands = {side: resources.enter_context(NativeHandWorker(side, timeout_s=.3)) for side in _SIDES}
            self._ready.set()
            while not self._stop.is_set():
                started = time.monotonic_ns()
                with self._lock:
                    request, self._request = self._request, None
                    frame, self._latest = self._latest, None
                if request is not None:
                    self._apply_request(request)
                if tcp is not None:
                    incoming, disconnected = tcp.poll(time.monotonic())
                    if disconnected:
                        self._invalidate_reference()
                    if incoming is not None and (frame is None or incoming.received_timestamp_ns > frame.received_timestamp_ns):
                        frame = incoming
                if frame is not None:
                    self._observe(frame, time.monotonic_ns())
                now = time.monotonic_ns()
                with self._lock:
                    current = self._active_revision == self._revision
                if current:
                    self._advance(now)
                    self._publish(time.monotonic_ns())
                self._stop.wait(max(0., _PERIOD - (time.monotonic_ns() - started) / 1e9))
        except BaseException as error:
            if not self._ready.is_set():
                self._startup_error = error
            else:
                self._needs_rebind = True
                self._arms_valid = False
                self._publish(time.monotonic_ns(), f"{type(error).__name__}: {error}")
        finally:
            try:
                resources.close()
            except BaseException as error:
                if not self._ready.is_set():
                    self._startup_error = self._startup_error or error
                else:
                    self._needs_rebind = True
                    self._arms_valid = False
                    self._publish(time.monotonic_ns(), f"worker shutdown failed: {error}")
            self._ready.set()
