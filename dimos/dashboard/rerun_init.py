# Copyright 2025 Dimensional Inc.
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

"""Rerun initialization with multi-process support.

Architecture:
    - Main process calls init_rerun_server() to start gRPC server + viewer
    - Worker processes call connect_rerun() to connect to the server
    - All processes share the same Rerun recording stream

Viewer modes (set via RERUN_VIEWER environment variable):
    - "web" (default): Web viewer on port 9090
    - "native": Native Rerun viewer (requires display)
    - "none": gRPC only, connect externally with `rerun --connect`

Usage:
    # In main process (e.g., blueprints.build or robot connection):
    from dimos.dashboard.rerun_init import init_rerun_server
    server_addr = init_rerun_server()  # Returns server address

    # In worker modules:
    from dimos.dashboard.rerun_init import connect_rerun
    connect_rerun()  # Connects to server started by main process

    # On shutdown:
    from dimos.dashboard.rerun_init import shutdown_rerun
    shutdown_rerun()
"""

import atexit
import os

import rerun as rr

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

RERUN_GRPC_PORT = 9876
RERUN_WEB_PORT = 9090
RERUN_GRPC_ADDR = f"rerun+http://127.0.0.1:{RERUN_GRPC_PORT}/proxy"

# Environment variable to control viewer mode: "web", "native", or "none"
RERUN_VIEWER_MODE = os.environ.get("RERUN_VIEWER", "web").lower()

# Track initialization state
_server_started = False
_connected = False


def init_rerun_server() -> str:
    """Initialize Rerun server in the main process.

    Starts the gRPC server and optionally the web/native viewer.
    Should only be called once from the main process.

    Returns:
        Server address for workers to connect to.

    Raises:
        RuntimeError: If server initialization fails.
    """
    global _server_started

    if _server_started:
        logger.debug("Rerun server already started")
        return RERUN_GRPC_ADDR

    rr.init("dimos")

    if RERUN_VIEWER_MODE == "native":
        # Spawn native viewer (requires display)
        rr.spawn(port=RERUN_GRPC_PORT, connect=True)
        logger.info(f"Rerun: spawned native viewer on port {RERUN_GRPC_PORT}")
    elif RERUN_VIEWER_MODE == "web":
        # Start gRPC + web viewer (headless friendly)
        server_uri = rr.serve_grpc(grpc_port=RERUN_GRPC_PORT)
        rr.serve_web_viewer(web_port=RERUN_WEB_PORT, open_browser=False, connect_to=server_uri)
        logger.info(f"Rerun: web viewer on http://localhost:{RERUN_WEB_PORT}")
    else:
        # Just gRPC server, no viewer (connect externally)
        rr.serve_grpc(grpc_port=RERUN_GRPC_PORT)
        logger.info(
            f"Rerun: gRPC only on port {RERUN_GRPC_PORT}, "
            f"connect with: rerun --connect {RERUN_GRPC_ADDR}"
        )

    _server_started = True

    # Register shutdown handler
    atexit.register(shutdown_rerun)

    return RERUN_GRPC_ADDR


def connect_rerun(server_addr: str | None = None) -> None:
    """Connect to Rerun server from a worker process.

    Args:
        server_addr: Server address to connect to. Defaults to RERUN_GRPC_ADDR.
    """
    global _connected

    if _connected:
        logger.debug("Already connected to Rerun server")
        return

    addr = server_addr or RERUN_GRPC_ADDR

    rr.init("dimos")
    rr.connect_grpc(addr)
    logger.info(f"Rerun: connected to server at {addr}")

    _connected = True


def shutdown_rerun() -> None:
    """Disconnect from Rerun and cleanup resources."""
    global _server_started, _connected

    if _server_started or _connected:
        try:
            rr.disconnect()
            logger.info("Rerun: disconnected")
        except Exception as e:
            logger.warning(f"Rerun: error during disconnect: {e}")

            _server_started = False
            _connected = False
