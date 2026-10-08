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

"""`python -m dimos.gateway`: serve the /dimos API on a unix socket, configured by $DIMOS_GATEWAY (config.gateway_env).

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


def socket_path() -> Path:
    """$DIMOS_GATEWAY's `socket`, else <dimos state>/gateway/dimos-gateway.sock."""
    given = config.gateway_env().get("socket")
    return Path(given) if given else config.gateway_dir() / "dimos-gateway.sock"


def dimos_dir() -> Path:
    """$DIMOS_GATEWAY's `dimosDir`, else the checkout this code is in."""
    given = config.gateway_env().get("dimosDir")
    return Path(given) if given else DIMOS_PROJECT_ROOT


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


def serve(socket_path: Path, dimos_dir: Path, zenoh: bool = True) -> None:
    """Runs the gateway in this process until SIGTERM / Ctrl-C (or POST /dimos/server/stop)."""
    import uvicorn

    from dimos.gateway import zenoh_events
    from dimos.gateway.app import create_app, default_state

    namespace = zenoh_events.resolve_namespace() if zenoh else None

    if healthy(socket_path):
        raise SystemExit(f"a dimos gateway is already answering on {socket_path}")
    socket_path.parent.mkdir(parents=True, exist_ok=True)
    socket_path.unlink(missing_ok=True)
    unix = sockets.socket(sockets.AF_UNIX, sockets.SOCK_STREAM)
    unix.bind(str(socket_path))
    os.chmod(socket_path, 0o600)

    def exit_now() -> None:
        socket_path.unlink(missing_ok=True)
        os._exit(0)

    state = default_state(dimos_dir)
    state.exit = exit_now
    if namespace is not None:
        try:
            publisher = zenoh_events.open_publisher(namespace, zenoh_events.resolve_connect())
            state.bus.sinks.append(publisher)
            state.bus.publishers.append(publisher.under)
            state.zenoh_namespace = namespace
            print(f"dimos gateway events -> zenoh {namespace}/dimos/events/<type>", flush=True)
            # every topic on the bus from now on (GET /dimos/topics/rates), on the same pooled session
            state.topics = zenoh_events.topic_watch(zenoh_events.resolve_connect())
            state.topics.start()
        except Exception as error:
            print(f"dimos gateway: no zenoh session, events are on SSE only ({error})", flush=True)
    # an open event stream never ends on its own: give up on it after 2 s
    server = uvicorn.Server(
        uvicorn.Config(create_app(state), log_level="warning", timeout_graceful_shutdown=2)
    )
    print(f"dimos gateway -> {socket_path} (dimos at {dimos_dir})", flush=True)
    try:
        server.run(sockets=[unix])
    finally:
        socket_path.unlink(missing_ok=True)


def detach(socket_path: Path, zenoh: bool = True) -> None:
    """Starts the gateway as its own process (its own session, so it outlives whoever started it; it inherits
    $DIMOS_GATEWAY) and returns once it answers. One already answering = nothing to do."""
    if healthy(socket_path):
        print(f"the dimos gateway is already running on {socket_path}")
        return
    log_file().parent.mkdir(parents=True, exist_ok=True)
    command = [sys.executable, "-m", "dimos.gateway", *([] if zenoh else ["--no-zenoh"])]
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
    detach_: bool = typer.Option(
        False, "--detach", help="start in the background and return once it answers"
    ),
    print_socket: bool = typer.Option(
        False,
        "--print-socket",
        help="print the socket it serves on (dimos.yaml's socket:) and exit",
    ),
    zenoh: bool = typer.Option(True, help="publish events on zenoh (off: SSE only)"),
    write_provides: bool = typer.Option(
        False,
        "--write-provides",
        help="write the gateway's endpoints to dimos.yaml's provides: and exit",
    ),
) -> None:
    """Serve the /dimos HTTP API (blueprints, runs, logs, events, cloud uploads) that dimOS Desktop uses, as
    $DIMOS_GATEWAY says (dimos.yaml's start: documents it)."""
    if write_provides:
        from dimos.gateway import provides

        print(f"wrote {provides.write()}")
        return
    if print_socket:
        print(socket_path())
        return
    if detach_:
        detach(socket_path(), zenoh)
    else:
        serve(socket_path(), dimos_dir(), zenoh)


def main() -> None:
    typer.run(gateway)
