"""Same-origin HTTP API and bundled WebGL client; no ROS or native viewer."""
from __future__ import annotations

from collections import OrderedDict
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import ipaddress
import json
import mimetypes
from pathlib import Path
import socket
import threading
from urllib.parse import parse_qs, unquote, urlsplit

from .dataset import ReplayEpisode, ReplayError, scan_directory
from .static_scene import StaticScene

_STATIC = Path(__file__).resolve().parent / "static"
_MAX_BODY_BYTES = 64 * 1024


def _resolve_initial_episode(directory: Path, initial_episode: str | None) -> str | None:
    if initial_episode is None:
        return None
    if not isinstance(initial_episode, str) or not initial_episode.strip():
        raise ValueError("--episode 必须是目录内非空的相对 .h5 路径")
    candidate = Path(initial_episode).expanduser()
    if candidate.is_absolute():
        raise ValueError("--episode 必须是相对于 --directory 的文件名或路径")
    try:
        resolved = (directory / candidate).resolve(strict=True)
    except (OSError, RuntimeError, ValueError) as exc:
        raise ValueError(f"--episode 路径无法解析: {initial_episode}: {exc}") from exc
    if not resolved.is_relative_to(directory):
        raise ValueError("--episode 必须位于 --directory 内")
    if not resolved.is_file():
        raise ValueError(f"--episode 不是普通文件: {initial_episode}")
    if not resolved.name.endswith(".h5") or resolved.name.endswith((".partial.h5", ".render.h5")):
        raise ValueError(f"--episode 不是已完成的轨迹 .h5 文件: {initial_episode}")
    return resolved.relative_to(directory).as_posix()


class ReplayApplication:
    """Catalog paths plus a bounded cache of selected models, never all models."""

    def __init__(self, directory: Path, initial_episode: str | None = None, *, scene_path: Path | None = None):
        try:
            root = Path(directory).expanduser().resolve(strict=True)
        except (OSError, RuntimeError, TypeError) as exc:
            raise ValueError(f"目录无法解析: {directory}: {exc}") from exc
        if not root.is_dir():
            raise ValueError(f"不是目录: {root}")
        self.directory = root
        if scene_path is not None and initial_episode is not None:
            raise ValueError("--scene 与 --episode 不能同时使用")
        self.scene_path = None
        if scene_path is not None:
            try:
                source = Path(scene_path).expanduser().resolve(strict=True)
            except (OSError, RuntimeError, TypeError) as exc:
                raise ValueError(f"场景模型路径无法解析: {scene_path}: {exc}") from exc
            if not source.is_file():
                raise ValueError(f"场景模型不是普通文件: {source}")
            self.scene_path = source
        self._scene: StaticScene | None = None
        self.initial_episode = _resolve_initial_episode(root, initial_episode)
        self._summaries: dict[str, dict] = {}
        self._paths: dict[str, Path] = {}
        self._episodes: OrderedDict[str, ReplayEpisode] = OrderedDict()
        self._lock = threading.RLock()

    def scan(self, directory: str) -> dict:
        if self.scene_path is not None:
            raise ReplayError("静态场景模式不扫描轨迹目录")
        root, summaries, paths, skipped = scan_directory(directory)
        with self._lock:
            self.directory = root
            self._summaries = {entry["id"]: entry for entry in summaries}
            self._paths = paths
            for key in list(self._episodes):
                if key not in paths:
                    del self._episodes[key]
        return {"directory": str(root), "episodes": summaries, "skipped": skipped}

    def episode(self, episode_id: str) -> ReplayEpisode:
        with self._lock:
            if episode_id not in self._paths:
                raise KeyError("轨迹未找到，请重新扫描目录")
            if episode_id not in self._episodes:
                episode = ReplayEpisode(self._paths[episode_id], self._summaries[episode_id])
                self._episodes[episode_id] = episode
                while len(self._episodes) > 2:
                    self._episodes.popitem(last=False)
            self._episodes.move_to_end(episode_id)
            return self._episodes[episode_id]

    def static_scene(self) -> StaticScene:
        with self._lock:
            if self.scene_path is None:
                raise KeyError("服务未启用静态场景模式")
            if self._scene is None:
                self._scene = StaticScene(self.scene_path)
            return self._scene


class ReplayHTTPServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address: tuple[str, int], application: ReplayApplication):
        self.application = application
        super().__init__(address, ReplayHandler)


class ReplayHandler(BaseHTTPRequestHandler):
    server: ReplayHTTPServer

    def _same_origin(self) -> bool:
        """Reject browser cross-origin access and hostname-based DNS rebinding."""
        host = self.headers.get("Host", "")
        try:
            target = urlsplit("http://" + host)
            hostname = target.hostname
            if not hostname or target.username is not None or target.path:
                return False
            port = target.port or 80
            if port != self.server.server_port:
                return False
            local_address = self.connection.getsockname()[0]
            names = {"localhost", socket.gethostname().lower(), socket.getfqdn().lower()}
            try:
                requested = ipaddress.ip_address(hostname)
                valid_host = requested == ipaddress.ip_address(local_address)
            except ValueError:
                valid_host = hostname.lower() in names
            if not valid_host:
                return False
            origin = self.headers.get("Origin")
            return origin is None or origin == "http://" + host
        except (ValueError, OSError):
            return False

    def _send(self, status: int, content: bytes, content_type: str, **headers: str) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("X-Frame-Options", "DENY")
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header(
            "Content-Security-Policy",
            "default-src 'self'; script-src 'self' 'unsafe-inline'; "
            "style-src 'self'; img-src 'self' blob: data:; connect-src 'self' blob:; "
            "object-src 'none'; base-uri 'none'; frame-ancestors 'none'",
        )
        for key, value in headers.items():
            self.send_header(key, value)
        self.end_headers()
        self.wfile.write(content)

    def _json(self, status: int, payload: dict) -> None:
        self._send(status, json.dumps(payload, ensure_ascii=False, allow_nan=False).encode(),
                   "application/json; charset=utf-8")

    def _failure(self, exc: Exception) -> None:
        if isinstance(exc, KeyError):
            status = HTTPStatus.NOT_FOUND
            message = str(exc.args[0])
        elif isinstance(exc, (ReplayError, ValueError, OSError)):
            status = HTTPStatus.BAD_REQUEST
            message = str(exc)
        else:
            status = HTTPStatus.INTERNAL_SERVER_ERROR
            message = "加载失败，请查看服务端日志"
            self.log_error("%s: %s", type(exc).__name__, exc)
        self._json(status, {"error": message})

    def do_GET(self) -> None:
        try:
            if not self._same_origin():
                self._json(HTTPStatus.FORBIDDEN, {"error": "仅允许同源访问"})
                return
            parsed = urlsplit(self.path)
            path = unquote(parsed.path)
            app = self.server.application
            if path == "/api/config":
                if app.scene_path is not None:
                    self._json(HTTPStatus.OK, {"mode": "scene", "scene_path": str(app.scene_path)})
                else:
                    self._json(HTTPStatus.OK, {
                        "mode": "replay",
                        "directory": str(app.directory),
                        "sample_rate": 60,
                        "initial_episode": app.initial_episode,
                    })
                return
            if path in {"/api/scene/info", "/api/scene/scene.glb"}:
                static_scene = app.static_scene()
                if path.endswith("/info"):
                    self._json(HTTPStatus.OK, static_scene.info)
                else:
                    self._send(HTTPStatus.OK, static_scene.scene_glb, "model/gltf-binary")
                return
            segments = path.strip("/").split("/")
            if len(segments) == 4 and segments[:2] == ["api", "episodes"]:
                if app.scene_path is not None:
                    raise KeyError("静态场景模式不提供轨迹帧")
                if segments[3] not in {"info", "scene.glb", "frames"}:
                    raise KeyError("接口不存在")
                episode = app.episode(segments[2])
                if segments[3] == "info":
                    self._json(HTTPStatus.OK, episode.info)
                elif segments[3] == "scene.glb":
                    self._send(HTTPStatus.OK, episode.scene_glb, "model/gltf-binary")
                elif segments[3] == "frames":
                    query = parse_qs(parsed.query)
                    start = int(query.get("start", ["0"])[0])
                    count = int(query.get("count", ["240"])[0])
                    if count < 1 or count > 240:
                        raise ReplayError("count 必须为 1–240")
                    content, actual_count = episode.frame_block(start, count)
                    self._send(HTTPStatus.OK, content, "application/octet-stream",
                               **{"X-Frame-Start": str(start), "X-Frame-Count": str(actual_count)})
                else:
                    raise KeyError("接口不存在")
                return
            if path == "/":
                relative = "index.html"
            elif path.startswith("/static/"):
                relative = path[len("/static/"):]
            else:
                raise KeyError("页面不存在")
            asset = (_STATIC / relative).resolve()
            if not asset.is_relative_to(_STATIC) or not asset.is_file():
                raise KeyError("资源不存在")
            content_type = mimetypes.guess_type(asset.name)[0] or "application/octet-stream"
            if asset.suffix in {".html", ".css", ".js"}:
                content_type += "; charset=utf-8"
            self._send(HTTPStatus.OK, asset.read_bytes(), content_type)
        except (BrokenPipeError, ConnectionResetError):
            return
        except Exception as exc:
            self._failure(exc)

    def do_POST(self) -> None:
        try:
            if not self._same_origin():
                self._json(HTTPStatus.FORBIDDEN, {"error": "仅允许同源访问"})
                return
            if urlsplit(self.path).path != "/api/catalog":
                raise KeyError("接口不存在")
            if self.headers.get_content_type() != "application/json":
                raise ReplayError("请求必须使用 application/json")
            length = int(self.headers.get("Content-Length", "0"))
            if length < 1 or length > _MAX_BODY_BYTES:
                raise ReplayError("请求体为空或超过 64 KiB")
            payload = json.loads(self.rfile.read(length))
            if not isinstance(payload, dict) or not isinstance(payload.get("directory"), str):
                raise ReplayError("请提供 directory 目录字符串")
            if not payload["directory"].strip():
                raise ReplayError("目录不能为空")
            self._json(HTTPStatus.OK, self.server.application.scan(payload["directory"]))
        except (BrokenPipeError, ConnectionResetError):
            return
        except Exception as exc:
            self._failure(exc)
