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

import socket
from typing import Any
from urllib.parse import urlparse

import rerun as rr

from dimos.msgs.sensor_msgs.PointCloud2 import register_colormap_annotation
from dimos.utils.logging_config import setup_logger
from dimos.visualization.rerun.constants import RERUN_GRPC_PORT

logger = setup_logger()


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
