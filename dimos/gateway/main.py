# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""`dimos gateway` / `python -m dimos.gateway`: serve the /dimos API on a unix socket (and optionally a local port).

Nothing heavy is imported here, so `dimos --help` stays fast; the app is imported when the gateway starts.
"""

from __future__ import annotations

import os
from pathlib import Path
import socket as sockets
import subprocess
import sys
import time

import typer

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.gateway import config


def default_socket() -> Path:
    return config.gateway_dir() / "dimos-gateway.sock"


def log_file() -> Path:
    return config.logs_dir() / "dimos-gateway.log"


def healthy(socket_path: Path, timeout: float = 2.0) -> bool:
    """Something answers `GET /healthz` with 200 on the socket."""
    try:
        with sockets.socket(sockets.AF_UNIX, sockets.SOCK_STREAM) as connection:
            connection.settimeout(timeout)
            connection.connect(str(socket_path))
            connection.sendall(b"GET /healthz HTTP/1.0\r\nHost: localhost\r\n\r\n")
            return connection.recv(64).split(b" ")[1:2] == [b"200"]
    except OSError:
        return False


def serve(
    socket_path: Path,
    dimos_dir: Path,
    port: int | None = None,
    zenoh_namespace: str | None = None,
    zenoh_connect: str | None = None,
    zenoh: bool = True,
) -> None:
    """Runs the gateway in this process until SIGTERM / Ctrl-C (or POST /dimos/server/stop)."""
    import uvicorn

    from dimos.gateway import zenoh_events
    from dimos.gateway.app import create_app, default_state

    namespace = zenoh_events.resolve_namespace(zenoh_namespace) if zenoh else None

    if healthy(socket_path):
        raise SystemExit(f"a dimos gateway is already answering on {socket_path}")
    socket_path.parent.mkdir(parents=True, exist_ok=True)
    socket_path.unlink(missing_ok=True)
    unix = sockets.socket(sockets.AF_UNIX, sockets.SOCK_STREAM)
    unix.bind(str(socket_path))
    os.chmod(socket_path, 0o600)
    listening = [unix]
    if port is not None:
        tcp = sockets.socket(sockets.AF_INET, sockets.SOCK_STREAM)
        tcp.setsockopt(sockets.SOL_SOCKET, sockets.SO_REUSEADDR, 1)
        tcp.bind(("127.0.0.1", port))
        listening.append(tcp)

    def exit_now() -> None:
        socket_path.unlink(missing_ok=True)
        os._exit(0)

    state = default_state(dimos_dir)
    state.exit = exit_now
    if namespace is not None:
        try:
            publisher = zenoh_events.open_publisher(
                namespace, zenoh_events.resolve_connect(zenoh_connect)
            )
            state.bus.sinks.append(publisher)
            state.bus.publishers.append(publisher.under)
            state.zenoh_namespace = namespace
            print(f"dimos gateway events -> zenoh {namespace}/dimos/events/<type>", flush=True)
        except Exception as error:
            print(f"dimos gateway: no zenoh session, events are on SSE only ({error})", flush=True)
    # an open event stream never ends on its own: give up on it after 2 s
    server = uvicorn.Server(
        uvicorn.Config(create_app(state), log_level="warning", timeout_graceful_shutdown=2)
    )
    print(
        f"dimos gateway -> {socket_path}{f' and 127.0.0.1:{port}' if port else ''} (dimos at {dimos_dir})",
        flush=True,
    )
    try:
        server.run(sockets=listening)
    finally:
        socket_path.unlink(missing_ok=True)


def detach(
    socket_path: Path,
    dimos_dir: Path,
    port: int | None = None,
    zenoh_namespace: str | None = None,
    zenoh_connect: str | None = None,
    zenoh: bool = True,
) -> None:
    """Starts the gateway as its own process (its own session, so it outlives whoever started it) and returns once it
    answers. One already answering = nothing to do."""
    if healthy(socket_path):
        print(f"the dimos gateway is already running on {socket_path}")
        return
    log_file().parent.mkdir(parents=True, exist_ok=True)
    command = [
        sys.executable,
        "-m",
        "dimos.gateway",
        "--socket",
        str(socket_path),
        "--dimos-dir",
        str(dimos_dir),
    ]
    if port is not None:
        command += ["--port", str(port)]
    if zenoh_namespace is not None:
        command += ["--zenoh-namespace", zenoh_namespace]
    if zenoh_connect is not None:
        command += ["--zenoh-connect", zenoh_connect]
    if not zenoh:
        command.append("--no-zenoh")
    with log_file().open("a") as log:
        child = subprocess.Popen(
            command,
            stdin=subprocess.DEVNULL,
            stdout=log,
            stderr=log,
            start_new_session=True,
        )
    for _ in range(60):
        if healthy(socket_path):
            print(f"the dimos gateway is running on {socket_path} (pid {child.pid})")
            return
        if child.poll() is not None:
            raise SystemExit(f"the dimos gateway exited ({child.returncode}); see {log_file()}")
        time.sleep(0.25)
    raise SystemExit(f"the dimos gateway didn't answer in 15 s; see {log_file()}")


def gateway(
    socket: Path = typer.Option(
        None, help="unix socket to serve on (default: <dimos state>/gateway/dimos-gateway.sock)"
    ),
    dimos_dir: Path = typer.Option(
        DIMOS_PROJECT_ROOT, help="the dimos checkout runs are launched from"
    ),
    port: int = typer.Option(None, help="also serve on 127.0.0.1:<port>"),
    detach_: bool = typer.Option(
        False, "--detach", help="start in the background and return once it answers"
    ),
    zenoh_namespace: str = typer.Option(
        None,
        help="Desktop's zenoh namespace; events go on <ns>/dimos/events/<type> "
        "(default: $DIMOS_ZENOH_NAMESPACE, DIMOS_APP.zenohNamespace, else Desktop's config.yaml)",
    ),
    zenoh_connect: str = typer.Option(
        None,
        help="zenoh endpoints to dial, comma-separated "
        "(default: DIMOS_APP.zenohConnect, else dimos's zenoh_connect, i.e. $ZENOH_CONNECT)",
    ),
    zenoh: bool = typer.Option(True, help="publish events on zenoh (off: SSE only)"),
    write_openapi: bool = typer.Option(
        False,
        "--write-openapi",
        help="write the API's OpenAPI document to dimos/gateway/openapi.json (dimos.yaml's api.openapi) and exit",
    ),
) -> None:
    """Serve the /dimos HTTP API (blueprints, runs, logs, events, cloud uploads) that dimOS Desktop uses."""
    if write_openapi:
        from dimos.gateway import openapi

        print(f"wrote {openapi.write()}")
        return
    socket_path = socket or default_socket()
    (detach if detach_ else serve)(
        socket_path, dimos_dir, port, zenoh_namespace, zenoh_connect, zenoh
    )


def main() -> None:
    typer.run(gateway)
