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

from __future__ import annotations

import os
from pathlib import Path
import socket as sockets
import subprocess
import sys

import typer

from dimos.constants import DIMOS_PROJECT_ROOT
from experimental.gateway import constants, store


# a gateway is listening when its socket takes a connection (a stale socket file refuses)
def healthy(socket_path: Path) -> bool:
    try:
        with sockets.socket(sockets.AF_UNIX, sockets.SOCK_STREAM) as connection:
            connection.connect(str(socket_path))
            return True
    except OSError:
        return False


def serve(
    socket_path: Path,
    dimos_dir: Path,
    port: int | None,
    host: str,
    zenoh_namespace: str | None,
    zenoh_connect: str | None,
    zenoh: bool,
    ready_fd: int | None = None,
) -> None:
    import uvicorn

    from experimental.gateway import events
    from experimental.gateway.app import create_app
    from experimental.gateway.state import new_state

    path_length = len(os.fsencode(str(socket_path)))
    if path_length > constants.UNIX_SOCKET_PATH_LIMIT:
        raise SystemExit(
            f"the gateway socket path {socket_path} is {path_length} bytes, over this platform's "
            f"{constants.UNIX_SOCKET_PATH_LIMIT}; point `dimos-gateway.socket` in ~/.dimos/config.yaml "
            "(or --socket) somewhere shorter"
        )
    if healthy(socket_path):
        raise SystemExit(f"a dimos gateway is already answering on {socket_path}")
    namespace = events.resolve_namespace(zenoh_namespace) if zenoh else None
    bus = events.Bus()
    if namespace is not None:
        import zenoh as zenoh_library

        try:
            bus = events.Bus(namespace, events.zenoh_put(events.resolve_connect(zenoh_connect)))
            print(f"dimos gateway events -> zenoh {namespace}/dimos/events/<type>", flush=True)
        except zenoh_library.ZError as error:
            print(f"dimos gateway: no zenoh session, no events ({error})", flush=True)
    socket_path.parent.mkdir(parents=True, exist_ok=True)
    socket_path.unlink(missing_ok=True)
    unix = sockets.socket(sockets.AF_UNIX, sockets.SOCK_STREAM)
    unix.bind(str(socket_path))
    os.chmod(socket_path, 0o600)
    listening = [unix]
    if port is not None:
        tcp = sockets.socket(sockets.AF_INET, sockets.SOCK_STREAM)
        tcp.setsockopt(sockets.SOL_SOCKET, sockets.SO_REUSEADDR, 1)
        tcp.bind((host, port))
        listening.append(tcp)
    for listener in listening:
        listener.listen()  # connections queue from here until uvicorn serves them

    def exit_now() -> None:
        socket_path.unlink(missing_ok=True)
        os._exit(0)

    state = new_state(dimos_dir, bus)
    state.exit = exit_now
    server = uvicorn.Server(uvicorn.Config(create_app(state), log_level="warning"))
    print(f"dimos gateway -> {socket_path}{f' and {host}:{port}' if port else ''}", flush=True)
    if ready_fd is not None:
        os.write(ready_fd, b"1")
        os.close(ready_fd)
    try:
        server.run(sockets=listening)
    finally:
        socket_path.unlink(missing_ok=True)


def detach(arguments: list[str], socket_path: Path) -> None:
    if healthy(socket_path):
        print(f"the dimos gateway is already running on {socket_path}")
        return
    log_file = store.logs_dir() / "dimos-gateway.log"
    log_file.parent.mkdir(parents=True, exist_ok=True)
    ready_read, ready_write = os.pipe()
    command = [sys.executable, "-m", "experimental.gateway", *arguments]
    with log_file.open("a") as log:
        child = subprocess.Popen(
            [*command, "--ready-fd", str(ready_write)],
            stdin=subprocess.DEVNULL,
            stdout=log,
            stderr=log,
            start_new_session=True,
            pass_fds=(ready_write,),
        )
    os.close(ready_write)
    # the child writes a byte once its socket listens; EOF means it exited first
    with os.fdopen(ready_read, "rb") as ready:
        signaled = ready.read(1)
    if not signaled:
        raise SystemExit(f"the dimos gateway exited ({child.wait()}); see {log_file}")
    print(f"the dimos gateway is running on {socket_path} (pid {child.pid})")


def gateway(
    socket: Path = typer.Option(None),
    dimos_dir: Path = typer.Option(DIMOS_PROJECT_ROOT),
    port: int = typer.Option(None),
    host: str = typer.Option(constants.DEFAULT_HOST),
    detach_: bool = typer.Option(False, "--detach"),
    zenoh_namespace: str = typer.Option(None),
    zenoh_connect: str = typer.Option(None),
    zenoh: bool = typer.Option(True),
    ready_fd: int = typer.Option(None, hidden=True),
) -> None:
    configured = store.setting("dimos-gateway", "socket")
    socket_path = socket or (
        Path(configured) if configured else store.gateway_dir() / "dimos-gateway.sock"
    )
    if detach_:
        detach(
            [arg for arg in sys.argv[1:] if arg != "--detach"] + ["--socket", str(socket_path)],
            socket_path,
        )
    else:
        serve(socket_path, dimos_dir, port, host, zenoh_namespace, zenoh_connect, zenoh, ready_fd)


if __name__ == "__main__":
    typer.run(gateway)
