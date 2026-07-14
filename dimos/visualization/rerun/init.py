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

from pathlib import Path
import socket
from typing import Any
from urllib.parse import urlparse

import rerun as rr

from dimos.msgs.sensor_msgs.PointCloud2 import register_colormap_annotation
from dimos.utils.logging_config import setup_logger
from dimos.visualization.rerun.constants import RERUN_GRPC_PORT

logger = setup_logger()

# Keeps the dedicated server recording alive when teeing to a file (see
# rerun_init): dropping it would shut down the gRPC server.
_server_recording: rr.RecordingStream | None = None


def rerun_init(
    app_id: str = "dimos",
    *,
    start_grpc: bool = False,
    grpc_config: dict[str, Any] | None = None,
    save_path: str | None = None,
    **kwargs: Any,
) -> str | None:
    """
    Use this inside modules for direct visualization (see docs/usage/visualization.md)

    This exists to consolidate visualization settings across modules
    Note only the rerun bridge module should have start_grpc=True

    ``save_path`` (only honored with ``start_grpc=True``) tees everything
    logged on this recording to a ``.rrd`` file in addition to the live gRPC
    stream.
    """
    global _server_recording

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
            if save_path is not None:
                _set_tee_sinks(connect_url, save_path)
            else:
                rr.connect_grpc(url=connect_url)
            server_uri = connect_url
        else:
            serve_kwargs: dict[str, Any] = {
                "grpc_port": grpc_port,
                "server_memory_limit": server_memory_limit,
            }
            if save_path is not None:
                # A recording's sink can be either the in-process gRPC server
                # or a (GrpcSink, FileSink) tee, not both. So give the server
                # its own recording (kept alive via _server_recording) and
                # point this recording's tee at it over loopback.
                if _server_recording is not None:
                    # A second serve+save in one process would drop the previous
                    # RecordingStream — and with it the live gRPC server.
                    logger.warning(
                        "rerun_init called again with start_grpc + save_path; "
                        "keeping the existing server recording alive"
                    )
                _server_recording = rr.RecordingStream(app_id)
                serve_kwargs["recording"] = _server_recording
            server_uri = rr.serve_grpc(**serve_kwargs)
            if save_path is not None:
                _set_tee_sinks(server_uri, save_path)
            logger.info(f"Rerun gRPC server ready at {server_uri}")

    # the important part of this function (consolidate them)
    register_colormap_annotation("turbo")
    return server_uri


def _set_tee_sinks(grpc_url: str, save_path: str) -> None:
    """Tee the active recording to the gRPC server at ``grpc_url`` and a file."""
    Path(save_path).parent.mkdir(parents=True, exist_ok=True)
    rr.set_sinks(rr.GrpcSink(url=grpc_url), rr.FileSink(save_path))
    logger.info(f"Rerun recording saving to {save_path}")
