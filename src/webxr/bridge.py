"""Single-controller HTTP/WebSocket bridge with no network work on physics ticks."""
from __future__ import annotations

import asyncio
import copy
from dataclasses import dataclass, field
import gzip
import ipaddress
import json
import logging
import math
from pathlib import Path
import threading
import time
from urllib.parse import urlsplit
import uuid

from aiohttp import web, WSMsgType
import mujoco

from .protocol import (
    COMMAND_KEYS, MAX_MESSAGE_BYTES, MAX_SOURCE_AGE_NS, ProtocolError,
    TrackingReceiver, decode_message, encode_state, positive_integer,
)
from .scene import MAX_SCENE_BYTES, copy_body_poses, export_scene

_LOG = logging.getLogger(__name__)
_STATIC = Path(__file__).with_name("static")
_STATUS_FIELDS = frozenset(("task_title", "task_goal", "stage", "notice", "state_frames",
                            "checkpoint_frames", "auto_checkpoint_frames", "control_flags",
                            "sim_time", "error", "target_duration_s", "recorded_duration_s"))


@dataclass
class _Peer:
    socket: web.WebSocketResponse
    receiver: TrackingReceiver
    generation: int = 0
    ready: bool = False
    last_state: int = 0
    last_status_ns: int = 0
    pending_frame: object = None
    send_lock: asyncio.Lock = field(default_factory=asyncio.Lock)


class WebXRBridge:
    """Network callbacks only enqueue input/commands; they must not touch physics.

    ``set_scene`` and ``publish`` belong to the same physics-owner thread.
    Only the current immutable scene and latest state/input samples are retained.
    ``start`` returns only after the socket is bound, or raises synchronously.
    """

    def __init__(self, host="127.0.0.1", port=8080, on_frame=None, on_command=None, height_m=1.7):
        if not callable(on_frame) or not callable(on_command):
            raise TypeError("WebXR requires on_frame and on_command queue callbacks")
        if not isinstance(host, str) or not host or any(c in host for c in "/@?#"):
            raise ValueError("host must be localhost or an explicit bind address")
        if host in ("0.0.0.0", "::", "*"):
            raise ValueError("bind to an explicit address so HTTP Host can be checked")
        if type(port) is not int or not 0 <= port <= 65535:
            raise ValueError("port must be in 0..65535")
        self.host, self.port, self.height_m = host, port, float(height_m)
        self.on_frame, self.on_command = on_frame, on_command
        try:
            loopback = ipaddress.ip_address(host).is_loopback
        except ValueError:
            loopback = host.lower() == "localhost"
        self._allowed_hosts = {"127.0.0.1", "localhost", "::1"} if loopback else {host.lower()}
        self._lock = threading.Lock()
        self._owner = None
        self._model = None
        self._generation = 0
        self._scene_json = self._scene_gzip = None
        self._scene_pending = False
        self._latest = None
        self._sequence = 0
        self._thread = None
        self._loop = None
        self._stop = None
        self._started = threading.Event()
        self._closed = False
        self._failure = None
        self._peer = None
        self._instance = "webxr-" + uuid.uuid4().hex
        self._connections = 0

    @property
    def generation(self):
        with self._lock:
            return self._generation

    @property
    def url(self):
        host = f"[{self.host}]" if ":" in self.host else self.host
        return f"http://{host}:{self.port}"

    def start(self):
        if self._closed:
            raise RuntimeError("WebXR bridge is closed")
        if self._thread is not None:
            raise RuntimeError("WebXR bridge already started")
        self._thread = threading.Thread(target=self._run, name="webxr-bridge", daemon=True)
        self._thread.start()
        if not self._started.wait(10):
            self.close()
            raise RuntimeError("WebXR bridge startup timed out")
        if self._failure is not None:
            self.close()
            raise RuntimeError(f"WebXR bridge startup failed: {self._failure}") from self._failure
        return self

    def _check_owner(self):
        owner = threading.get_ident()
        if self._owner is None:
            self._owner = owner
        elif self._owner != owner:
            raise RuntimeError("set_scene/publish must run on the physics owner thread")
        if self._closed:
            raise RuntimeError("WebXR bridge is closed")
        if self._failure is not None:
            raise RuntimeError(f"WebXR bridge failed: {self._failure}") from self._failure

    def begin_scene_change(self):
        """Revoke input and notify the headset before potentially slow model construction."""
        self._check_owner()
        if self._scene_pending:
            return
        with self._lock:
            if self._generation == 0xFFFFFFFF:
                raise RuntimeError("scene generation exhausted; restart the bridge")
            self._generation += 1
            self._scene_pending = True
            self._scene_json = self._scene_gzip = self._latest = self._model = None
            self._sequence = 0

    def set_scene(self, model, data, task):
        self._check_owner()
        if not self._scene_pending:
            self.begin_scene_change()
        generation = self._generation
        scene = export_scene(model, data, task, generation=generation, height_m=self.height_m)
        encoded = json.dumps(scene, separators=(",", ":"), allow_nan=False).encode("utf-8")
        if len(encoded) > MAX_SCENE_BYTES:
            raise ValueError("scene exceeds the 256 MiB bootstrap bound")
        compressed = gzip.compress(encoded, compresslevel=3, mtime=0)
        self._pose_data = mujoco.MjData(model)
        poses = self._capture_poses(model, data)
        with self._lock:
            self._model = model
            self._scene_json, self._scene_gzip = encoded, compressed
            self._scene_pending = False
            self._sequence = 1
            self._latest = (generation, 1, float(data.time), poses,
                            {"task_title": scene["task"]["title"], "task_goal": scene["task"]["goal"],
                             "sim_time": float(data.time)})

    def _capture_poses(self, model, data):
        # mj_step leaves derived poses at the pre-integration state. Recompute
        # on a private reusable buffer, without altering the physics owner.
        snapshot = self._pose_data
        snapshot.qpos[:] = data.qpos
        snapshot.mocap_pos[:] = data.mocap_pos
        snapshot.mocap_quat[:] = data.mocap_quat
        snapshot.time = data.time
        mujoco.mj_kinematics(model, snapshot)
        return copy_body_poses(model, snapshot)

    def publish(self, model, data, status):
        self._check_owner()
        if model is not self._model:
            raise ValueError("publish requires the current set_scene model")
        poses = self._capture_poses(model, data)
        # No serialization, mesh export, waiting on sockets, or live array views.
        fields = copy.deepcopy({key: value for key, value in status.items() if key in _STATUS_FIELDS})
        for key in ("target_duration_s", "recorded_duration_s"):
            value = fields.get(key)
            if value is not None:
                if type(value) not in (int, float):
                    raise ValueError(f"{key} must be null or finite nonnegative seconds")
                try:
                    seconds = float(value)
                except OverflowError as error:
                    raise ValueError(f"{key} must be finite nonnegative seconds") from error
                if not math.isfinite(seconds) or seconds < 0:
                    raise ValueError(f"{key} must be finite nonnegative seconds")
                fields[key] = seconds
        fields["sim_time"] = float(data.time)
        with self._lock:
            self._sequence = self._sequence % 0xFFFFFFFF + 1
            self._latest = (self._generation, self._sequence, float(data.time), poses, fields)

    def close(self):
        if self._closed:
            return
        self._closed = True
        loop, stop = self._loop, self._stop
        if loop is not None and stop is not None and not loop.is_closed():
            loop.call_soon_threadsafe(stop.set)
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=8)
            if thread.is_alive():
                raise RuntimeError("WebXR bridge did not shut down within 8 seconds")

    def _run(self):
        try:
            asyncio.run(self._serve())
        except Exception as error:
            self._failure = error
            _LOG.exception("WebXR bridge stopped")
        finally:
            self._started.set()

    async def _serve(self):
        self._loop = asyncio.get_running_loop()
        self._stop = asyncio.Event()
        app = web.Application(client_max_size=MAX_MESSAGE_BYTES, middlewares=[self._security])
        app.router.add_get("/scene", self._get_scene)
        app.router.add_get("/ws", self._websocket)
        app.router.add_get("/{path:.*}", self._static)
        runner = web.AppRunner(app, shutdown_timeout=1, access_log=None)
        try:
            await runner.setup()
            site = web.TCPSite(runner, self.host, self.port, shutdown_timeout=1)
            await site.start()
            addresses = runner.addresses
            if not addresses:
                raise RuntimeError("aiohttp did not bind a listening socket")
            self.port = int(addresses[0][1])
            self._started.set()
            if not self._closed:
                await self._stop.wait()
        finally:
            if self._peer is not None:
                self._invalidate(self._peer, "server shutdown")
                await self._close_socket(self._peer, 1001, "server shutdown")
            await runner.cleanup()

    @web.middleware
    async def _security(self, request, handler):
        def authority(value):
            try:
                parsed = urlsplit("http://" + value)
                if parsed.username is not None or parsed.password is not None or parsed.path or parsed.query or parsed.fragment:
                    raise ValueError
                return parsed.hostname, parsed.port or 80
            except ValueError as error:
                raise web.HTTPForbidden(text="invalid Host") from error

        host, port = authority(request.host)
        if host not in self._allowed_hosts or port != self.port:
            raise web.HTTPForbidden(text="Host is not the configured WebXR endpoint")
        origin = request.headers.get("Origin")
        if origin is not None:
            try:
                parsed = urlsplit(origin)
                valid = (parsed.scheme == "http" and not parsed.path and not parsed.query and not parsed.fragment
                         and parsed.username is None and parsed.password is None
                         and (parsed.hostname, parsed.port or 80) == (host, port))
            except ValueError:
                valid = False
            if not valid:
                raise web.HTTPForbidden(text="cross-origin WebXR access is forbidden")
        if request.headers.get("Sec-Fetch-Site") == "cross-site":
            raise web.HTTPForbidden(text="cross-site WebXR access is forbidden")
        if request.path == "/ws" and origin is None:
            raise web.HTTPForbidden(text="WebXR sockets require a same-origin browser")
        response = await handler(request)
        if not response.prepared:
            response.headers["X-Content-Type-Options"] = "nosniff"
            response.headers["Referrer-Policy"] = "no-referrer"
            response.headers["Permissions-Policy"] = "xr-spatial-tracking=(self)"
            response.headers["Cache-Control"] = "no-store"
        return response

    async def _static(self, request):
        relative = request.match_info["path"] or "index.html"
        candidate = (_STATIC / relative).resolve()
        if not candidate.is_relative_to(_STATIC.resolve()) or not candidate.is_file():
            raise web.HTTPNotFound()
        response = web.FileResponse(candidate)
        if candidate.suffix == ".html":
            response.headers["Content-Security-Policy"] = (
                "default-src 'self'; script-src 'self'; connect-src 'self' ws:; "
                "img-src 'self' data: blob:; style-src 'self' 'unsafe-inline'; "
                "object-src 'none'; base-uri 'none'; frame-ancestors 'none'")
        return response

    async def _get_scene(self, request):
        value = request.query.get("generation")
        try:
            requested = int(value) if value is not None else None
        except ValueError as error:
            raise web.HTTPBadRequest(text="invalid scene generation") from error
        with self._lock:
            if requested is not None and requested != self._generation:
                raise web.HTTPConflict(text="stale scene generation")
            scene, compressed = self._scene_json, self._scene_gzip
        if scene is None:
            raise web.HTTPServiceUnavailable(text="scene is not ready")
        if "gzip" in request.headers.get("Accept-Encoding", ""):
            return web.Response(body=compressed, content_type="application/json",
                                headers={"Content-Encoding": "gzip", "Vary": "Accept-Encoding"})
        return web.Response(body=scene, content_type="application/json")

    def _invalidate(self, peer, reason):
        peer.pending_frame = None
        frame = peer.receiver.invalidate(reason)
        try:
            self.on_frame(frame)
        except Exception:
            _LOG.exception("WebXR source invalidation callback failed")

    async def _send(self, peer, value):
        async with peer.send_lock:
            if isinstance(value, bytes):
                await asyncio.wait_for(peer.socket.send_bytes(value), .25)
            else:
                encoded = json.dumps(value, separators=(",", ":"), allow_nan=False)
                if len(encoded) > 64 * 1024:
                    raise ProtocolError("status exceeds 64 KiB")
                await asyncio.wait_for(peer.socket.send_str(encoded), .25)

    async def _close_socket(self, peer, code, message):
        try:
            await asyncio.wait_for(peer.socket.close(code=code, message=message.encode()[:120]), .5)
        except (TimeoutError, ConnectionError):
            peer.socket.force_close()

    async def _websocket(self, request):
        if self._peer is not None:
            raise web.HTTPConflict(text="another browser owns WebXR control")
        socket = web.WebSocketResponse(max_msg_size=MAX_MESSAGE_BYTES, heartbeat=5, compress=False)
        self._connections += 1
        peer = _Peer(socket, TrackingReceiver(f"{self._instance}:{self._connections}", 1))
        # Reserve before prepare awaits: concurrent upgrades cannot both win.
        self._peer = peer
        sender = None
        try:
            await socket.prepare(request)
            self._invalidate(peer, "new browser connection")
            sender = asyncio.create_task(self._pump(peer))
            window, messages = time.monotonic(), 0
            async for packet in socket:
                if packet.type != WSMsgType.TEXT:
                    if packet.type in (WSMsgType.CLOSE, WSMsgType.CLOSED, WSMsgType.ERROR):
                        break
                    raise ProtocolError("only JSON text input is supported")
                now = time.monotonic()
                if now - window >= 1:
                    window, messages = now, 0
                messages += 1
                if messages > 240:
                    raise ProtocolError("input exceeds 240 messages/second")
                message = decode_message(packet.data)
                generation = positive_integer(message.get("generation"), "generation", 0xFFFFFFFF)
                with self._lock:
                    current = self._generation
                if generation != current or generation != peer.generation:
                    # Old frames naturally remain in TCP during scene switch.
                    # Discard, but never refresh the source or execute a command.
                    continue
                kind = message["type"]
                if kind == "ready":
                    if peer.ready:
                        raise ProtocolError("scene already acknowledged")
                    peer.ready = True
                    await self._send(peer, peer.receiver.challenge())
                elif kind == "clock":
                    if not peer.ready:
                        raise ProtocolError("clock requires ready scene")
                    peer.receiver.acknowledge_clock(message)
                    await self._send(peer, {"type": "clock_ready", "generation": generation})
                elif kind == "tracking":
                    frame = peer.receiver.accept(message, packet.data)
                    if peer.receiver.invalidated:
                        # Losing XR visibility/wrist tracking is an identity
                        # edge, not a latest-slot sample that may be overwritten
                        # by rapid re-entry before the next 60 Hz dispatch.
                        peer.pending_frame = None
                        with self._lock:
                            if self._generation == generation:
                                self.on_frame(frame)
                    else:
                        peer.pending_frame = frame
                elif kind == "command":
                    if not peer.ready or peer.receiver.clock_source_ms is None:
                        raise ProtocolError("commands require acknowledged scene and clock")
                    key = message.get("key")
                    if not isinstance(key, str) or key not in COMMAND_KEYS:
                        raise ProtocolError("unknown collection command")
                    with self._lock:
                        if self._generation == generation:
                            self.on_command(key)
                else:
                    raise ProtocolError("unknown WebXR input type")
        except (ProtocolError, ValueError) as error:
            self._invalidate(peer, str(error))
            await self._close_socket(peer, 1008, str(error))
        except (ConnectionError, TimeoutError):
            self._invalidate(peer, "connection lost")
        except Exception:
            _LOG.exception("WebXR input callback failed")
            self._invalidate(peer, "input callback failed")
            await self._close_socket(peer, 1011, "input callback failed")
        finally:
            if sender is not None:
                sender.cancel()
                await asyncio.gather(sender, return_exceptions=True)
            self._invalidate(peer, "browser disconnected")
            if self._peer is peer:
                self._peer = None
        return socket

    async def _pump(self, peer):
        try:
            while not peer.socket.closed:
                started = time.monotonic()
                with self._lock:
                    generation, scene_ready, latest = self._generation, self._scene_json is not None, self._latest
                if generation != peer.generation:
                    self._invalidate(peer, "scene changed")
                    peer.receiver.change_scene(generation)
                    peer.generation, peer.ready, peer.last_state = generation, False, 0
                    peer.last_status_ns = 0
                    if not scene_ready:
                        await self._send(peer, {"type": "scene_loading", "generation": generation})
                if scene_ready and not peer.ready and peer.last_state == 0:
                    await self._send(peer, {"type": "scene", "generation": generation,
                                            "url": f"/scene?generation={generation}"})
                    peer.last_state = -1
                now_ns = time.monotonic_ns()
                if not peer.receiver.invalidated and now_ns - peer.receiver.latest_received_ns > MAX_SOURCE_AGE_NS:
                    self._invalidate(peer, "tracking timed out")
                frame, peer.pending_frame = peer.pending_frame, None
                if frame is not None:
                    with self._lock:
                        if self._generation == peer.generation:
                            self.on_frame(frame)
                if peer.ready and latest is not None and latest[0] == peer.generation:
                    state_generation, sequence, sim_time, poses, status = latest
                    if sequence != peer.last_state:
                        await self._send(peer, encode_state(state_generation, sequence, sim_time, poses))
                        peer.last_state = sequence
                    if now_ns - peer.last_status_ns >= 100_000_000:
                        await self._send(peer, {**status, "type": "status", "generation": state_generation})
                        peer.last_status_ns = now_ns
                await asyncio.sleep(max(0., 1 / 60 - (time.monotonic() - started)))
        except asyncio.CancelledError:
            raise
        except Exception:
            _LOG.exception("WebXR bounded state sender stopped")
            self._invalidate(peer, "state sender failed")
            await self._close_socket(peer, 1011, "state sender failed")
