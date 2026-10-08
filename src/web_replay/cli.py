"""Serve a local browser-based SPD trajectory viewer."""
from __future__ import annotations

import argparse
from pathlib import Path

from .server import ReplayApplication, ReplayHTTPServer


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", type=Path, default=Path.cwd(),
                        help="Initial server-side dataset directory; change it in the website")
    parser.add_argument("--host", default="127.0.0.1",
                        help="Bind address; 0.0.0.0 exposes trusted local datasets to the LAN")
    parser.add_argument("--port", type=int, default=8765, help="HTTP port (default: 8765)")
    args = parser.parse_args(argv)
    if not 0 < args.port < 65536:
        parser.error("--port must be between 1 and 65535")
    directory = args.directory.expanduser().resolve()
    if not directory.is_dir():
        parser.error(f"Dataset directory does not exist: {directory}")
    try:
        with ReplayHTTPServer((args.host, args.port), ReplayApplication(directory)) as server:
            visible_host = "127.0.0.1" if args.host == "0.0.0.0" else args.host
            print(f"SPD replay ready: http://{visible_host}:{server.server_port}", flush=True)
            print(f"Initial directory: {directory}", flush=True)
            if args.host != "127.0.0.1":
                print("Trusted network only: this server has no authentication and reads selected local data.",
                      flush=True)
            try:
                server.serve_forever()
            except KeyboardInterrupt:
                print("\nSPD replay stopped.", flush=True)
    except OSError as exc:
        parser.exit(1, f"spd-web: {exc}\n")
    return 0
