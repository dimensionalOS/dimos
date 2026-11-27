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

"""Shared Rerun initialization. Call ``rerun_init()`` instead of ``rr.init()``."""

from __future__ import annotations

from importlib.resources import files
import os
import socket
import subprocess
import sys
import threading
from typing import Any
from urllib.parse import urlparse

from dimos.msgs.sensor_msgs.PointCloud2 import register_colormap_annotation
from dimos.utils.executable import find_executable
from dimos.utils.logging_config import setup_logger
from dimos.visualization.rerun.constants import RERUN_GRPC_PORT, RERUN_WEBSOCKET_URL

logger = setup_logger()


# Viewer-only log filter: silences the winit scale-factor line and re_viewer::app
# warnings such as "Data source has left unexpectedly" on shutdown. Kept out of
# os.environ because the Rerun SDK and zenoh in this process also read RUST_LOG.
_VIEWER_RUST_LOG = "info,re_viewer::app=error,winit::platform_impl::linux::x11::window=warn"


def _wayland_socket_fd(value: str) -> int:
    """Return the borrowed descriptor if it is a connected Unix stream socket."""
    fd = int(value)
    display = socket.socket(fileno=fd)
    try:
        if display.family != socket.AF_UNIX or display.type != socket.SOCK_STREAM:
            raise ValueError("expected a Unix stream socket")
        display.getpeername()
    finally:
        display.detach()
    return fd


def _launch_native(command: list[str], wayland_fd: int | None) -> bool:
    env = {**os.environ, "RUST_LOG": os.environ.get("RUST_LOG", _VIEWER_RUST_LOG)}
    if wayland_fd is not None:
        env["WAYLAND_SOCKET"] = str(wayland_fd)
    try:
        process = subprocess.Popen(
            command,
            stdin=None if wayland_fd == 0 else subprocess.DEVNULL,
            start_new_session=True,
            env=env,
            pass_fds=(wayland_fd,) if wayland_fd is not None else (),
        )
    except FileNotFoundError:
        return False
    except OSError:
        logger.warning("Failed to launch %s", command[0], exc_info=True)
        return False

    # The viewer outlives the bridge; reap it if it closes before this process exits.
    threading.Thread(target=process.wait, name="dimos-viewer-reaper", daemon=True).start()
    return True


def spawn_viewer(
    server_uri: str, memory_limit: str, websocket_url: str = RERUN_WEBSOCKET_URL
) -> bool:
    """Launch our custom viewer directly, falling back to stock Rerun if unavailable."""
    wayland_fd = None
    if sys.platform == "linux":
        if (value := os.environ.get("WAYLAND_SOCKET")) is not None:
            try:
                wayland_fd = _wayland_socket_fd(value)
            except (OSError, ValueError, OverflowError) as exc:
                logger.info("Cannot use WAYLAND_SOCKET for the native Rerun viewer: %s", exc)
                return False
        elif display := os.environ.get("WAYLAND_DISPLAY"):
            path = os.path.join(os.environ.get("XDG_RUNTIME_DIR", ""), display)
            if not os.path.isabs(path):
                logger.info("A relative WAYLAND_DISPLAY requires an absolute XDG_RUNTIME_DIR")
                return False
            try:
                with socket.socket(socket.AF_UNIX) as probe:
                    probe.setblocking(False)
                    probe.connect(path)
            except OSError as exc:
                logger.info(
                    "Cannot connect to WAYLAND_DISPLAY for the native Rerun viewer: %s", exc
                )
                return False
        elif not os.environ.get("DISPLAY"):
            logger.info("No Linux display available for the native Rerun viewer")
            return False

    options = ["--connect", server_uri, "--memory-limit", memory_limit, "--expect-data-soon"]
    custom = find_executable("dimos-viewer") or "dimos-viewer"
    if _launch_native([custom, *options, "--ws-url", websocket_url], wayland_fd):
        return True

    if wayland_fd is not None:
        # Both rr.spawn and the stock Python CLI wrapper can close the display descriptor.
        # Use the same executable (or override) as rerun_cli and pass the socket directly.
        stock = os.environ.get("RERUN_CLI_PATH")
        if stock is None:
            try:
                stock = str(files("rerun_cli").joinpath("rerun"))
            except ModuleNotFoundError:
                logger.info("The stock Rerun viewer is unavailable")
                return False
        return _launch_native([stock, *options], wayland_fd)

    import rerun as rr

    try:
        rr.spawn(connect=True, memory_limit=memory_limit, executable_path=find_executable("rerun"))
        return True
    except (RuntimeError, FileNotFoundError):
        logger.warning(
            "Rerun native viewer not available (headless?). "
            "Bridge will continue without a viewer — data is still "
            "accessible via --rerunbridgemodule.rerun-open web or by connecting a viewer to the gRPC server.",
            exc_info=True,
        )
        return False


def rerun_init(
    app_id: str = "dimos",
    *,
    start_grpc: bool = False,
    grpc_config: dict[str, Any] | None = None,
    **kwargs: Any,
) -> str | None:
    """
    Use this inside modules for direct visualization (see docs/usage/visualization.md)

    This exists to consolidate visualization settings across modules
    Note only the rerun bridge module should have start_grpc=True
    """
    import rerun as rr

    rr.init(app_id, **kwargs)  # type: ignore[arg-type]

    server_uri: str | None = None
    if start_grpc:
        if (
            not isinstance(grpc_config, dict)
            or not isinstance(grpc_config.get("connect_url"), str)
            or not isinstance(grpc_config.get("server_memory_limit"), str)
        ):
            raise TypeError(
                "rerun_init(start_grpc=True) requires grpc_config to be a dict with "
                "'connect_url' (str) and 'server_memory_limit' (str)"
            )

        connect_url = grpc_config["connect_url"]
        server_memory_limit = grpc_config["server_memory_limit"]
        parsed = urlparse(connect_url.replace("rerun+", "", 1))
        grpc_port = parsed.port or RERUN_GRPC_PORT
        grpc_host = parsed.hostname or "127.0.0.1"

        port_in_use = False
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            port_in_use = sock.connect_ex((grpc_host, grpc_port)) == 0

        if port_in_use:
            logger.info(f"gRPC port {grpc_port} already in use, connecting to existing server")
            rr.connect_grpc(url=connect_url)
            server_uri = connect_url
        else:
            server_uri = rr.serve_grpc(
                grpc_port=grpc_port,
                server_memory_limit=server_memory_limit,
            )
            logger.info(f"Rerun gRPC server ready at {server_uri}")

    # the important part of this function (consolidate them)
    register_colormap_annotation("turbo")
    return server_uri
