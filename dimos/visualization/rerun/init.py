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

import os
import socket
import subprocess
import threading
from typing import Any
from urllib.parse import urlparse

from dimos.msgs.sensor_msgs.PointCloud2 import register_colormap_annotation
from dimos.utils.logging_config import setup_logger
from dimos.visualization.rerun.constants import RERUN_GRPC_PORT

logger = setup_logger()


# Viewer-only log filter: silences the winit scale-factor line and re_viewer::app
# warnings such as "Data source has left unexpectedly" on shutdown. Kept out of
# os.environ because the Rerun SDK and zenoh in this process also read RUST_LOG.
_VIEWER_RUST_LOG = "info,re_viewer::app=error,winit::platform_impl::linux::x11::window=warn"


def spawn_viewer(server_uri: str, memory_limit: str) -> bool:
    """Launch our custom viewer directly, falling back to stock Rerun if unavailable."""
    try:
        process = subprocess.Popen(
            [
                "dimos-viewer",
                "--connect",
                server_uri,
                "--memory-limit",
                memory_limit,
                "--expect-data-soon",
            ],
            stdin=subprocess.DEVNULL,
            start_new_session=True,
            env={**os.environ, "RUST_LOG": os.environ.get("RUST_LOG", _VIEWER_RUST_LOG)},
        )
    except FileNotFoundError:
        pass
    except OSError:
        logger.warning(
            "dimos-viewer failed to spawn, falling back to stock rerun",
            exc_info=True,
        )
    else:
        # The viewer outlives the bridge; reap it if it closes before this process exits.
        threading.Thread(target=process.wait, name="dimos-viewer-reaper", daemon=True).start()
        return True

    import rerun as rr

    try:
        rr.spawn(connect=True, memory_limit=memory_limit)
        return True
    except (RuntimeError, FileNotFoundError):
        logger.warning(
            "Rerun native viewer not available (headless?). "
            "Bridge will continue without a viewer — data is still "
            "accessible via --rerun-open web or by connecting a viewer to the gRPC server.",
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
