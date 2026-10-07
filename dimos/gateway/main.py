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

"""`dimos gateway` / `python -m dimos.gateway`: serve the /dimos API on 127.0.0.1:<port>.

Loopback only and unauthenticated: dimOS Desktop's proxy (/dimos/ behind its login) is the one door from other machines,
and loopback.py turns away other sites' pages and DNS rebinding. Any local user can still connect to the port.
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

# Desktop's default for it (its own 5555 + 2); Desktop picks a port once and saves it as config.yaml `dimos_gateway.port`
DEFAULT_PORT = 5557


def configured_port() -> int:
    """Desktop's config.yaml `dimos_gateway.port`, else DEFAULT_PORT."""
    port = config._section(config.load_desktop_config(), "dimos_gateway").get("port")
    return port if isinstance(port, int) and 0 < port < 65536 else DEFAULT_PORT


def log_file() -> Path:
    return config.logs_dir() / "dimos-gateway.log"


def healthy(port: int, timeout: float = 2.0) -> bool:
    """Something answers `GET /healthz` with 200 on 127.0.0.1:<port>."""
    try:
        with sockets.create_connection(("127.0.0.1", port), timeout) as connection:
            connection.settimeout(timeout)
            connection.sendall(b"GET /healthz HTTP/1.0\r\nHost: 127.0.0.1\r\n\r\n")
            return connection.recv(64).split(b" ")[1:2] == [b"200"]
    except OSError:
        return False


def taken(port: int) -> bool:
    """Something accepts connections on 127.0.0.1:<port> (a dimos gateway or any other program)."""
    try:
        with sockets.create_connection(("127.0.0.1", port), 0.5):
            return True
    except OSError:
        return False


def serve(
    port: int,
    dimos_dir: Path,
    zenoh_namespace: str | None = None,
    zenoh_connect: str | None = None,
    zenoh: bool = True,
) -> None:
    """Runs the gateway in this process until SIGTERM / Ctrl-C (or POST /dimos/server/stop)."""
    import uvicorn

    from dimos.gateway import zenoh_events
    from dimos.gateway.app import create_app, default_state
    from dimos.gateway.loopback import LoopbackOnly

    namespace = zenoh_events.resolve_namespace(zenoh_namespace) if zenoh else None

    if healthy(port):
        raise SystemExit(f"a dimos gateway is already answering on 127.0.0.1:{port}")
    if taken(port):
        raise SystemExit(
            f"127.0.0.1:{port} is in use by another program; pick another port "
            "(--port, or `dimos_gateway.port` in Desktop's config.yaml)"
        )
    tcp = sockets.socket(sockets.AF_INET, sockets.SOCK_STREAM)
    tcp.setsockopt(sockets.SOL_SOCKET, sockets.SO_REUSEADDR, 1)
    try:
        tcp.bind(("127.0.0.1", port))
    except OSError as error:
        raise SystemExit(f"can't listen on 127.0.0.1:{port}: {error}") from error

    state = default_state(dimos_dir)
    state.exit = lambda: os._exit(0)
    if namespace is not None:
        try:
            publisher = zenoh_events.open_publisher(
                namespace, zenoh_events.resolve_connect(zenoh_connect)
            )
            state.bus.sinks.append(publisher)
            state.bus.publishers.append(publisher.under)
            state.zenoh_namespace = namespace
            print(f"dimos gateway events -> zenoh {namespace}/dimos/events/<type>", flush=True)
            # every topic on the bus from now on (GET /dimos/topics/rates), on the same pooled session
            state.topics = zenoh_events.topic_watch(zenoh_events.resolve_connect(zenoh_connect))
            state.topics.start()
        except Exception as error:
            print(f"dimos gateway: no zenoh session, events are on SSE only ({error})", flush=True)
    # an open event stream never ends on its own: give up on it after 2 s
    server = uvicorn.Server(
        uvicorn.Config(
            LoopbackOnly(create_app(state)), log_level="warning", timeout_graceful_shutdown=2
        )
    )
    print(f"dimos gateway -> 127.0.0.1:{port} (dimos at {dimos_dir})", flush=True)
    server.run(sockets=[tcp])


def detach(
    port: int,
    dimos_dir: Path,
    zenoh_namespace: str | None = None,
    zenoh_connect: str | None = None,
    zenoh: bool = True,
) -> None:
    """Starts the gateway as its own process (its own session, so it outlives whoever started it) and returns once it
    answers. One already answering = nothing to do."""
    if healthy(port):
        print(f"the dimos gateway is already running on 127.0.0.1:{port}")
        return
    log_file().parent.mkdir(parents=True, exist_ok=True)
    command = [
        sys.executable,
        "-m",
        "dimos.gateway",
        "--port",
        str(port),
        "--dimos-dir",
        str(dimos_dir),
    ]
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
        if healthy(port):
            print(f"the dimos gateway is running on 127.0.0.1:{port} (pid {child.pid})")
            return
        if child.poll() is not None:
            raise SystemExit(f"the dimos gateway exited ({child.returncode}); see {log_file()}")
        time.sleep(0.25)
    raise SystemExit(f"the dimos gateway didn't answer in 15 s; see {log_file()}")


def gateway(
    dimos_dir: Path = typer.Option(
        DIMOS_PROJECT_ROOT, help="the dimos checkout runs are launched from"
    ),
    port: int = typer.Option(
        None,
        help=f"serve on 127.0.0.1:<port> (default: Desktop's config.yaml dimos_gateway.port, else {DEFAULT_PORT})",
    ),
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
    (detach if detach_ else serve)(
        port or configured_port(), dimos_dir, zenoh_namespace, zenoh_connect, zenoh
    )


def main() -> None:
    typer.run(gateway)
