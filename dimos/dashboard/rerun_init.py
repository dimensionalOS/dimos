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

Main process starts gRPC server + viewer, worker processes connect to it.
All processes share the same Rerun recording stream.

Viewer modes (set via RERUN_VIEWER environment variable):
    - "web" (default): Web viewer on port 9090
    - "native": Native Rerun viewer (requires display)
    - "none": gRPC only, connect externally with `rerun --connect`

Usage:
    # Web viewer (default)
    dimos --replay run unitree-go2

    # Native viewer
    RERUN_VIEWER=native dimos --replay run unitree-go2

    # No viewer (connect externally)
    RERUN_VIEWER=none dimos --replay run unitree-go2
    rerun --connect rerun+http://127.0.0.1:9876/proxy

Usage in modules:
    import rerun as rr
    from dimos.dashboard import rerun_init  # triggers initialization

    class MyModule(Module):
        def start(self):
            rr.log("my/entity", my_data.to_rerun())
"""

import os
import socket

import rerun as rr

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

RERUN_GRPC_PORT = 9876
RERUN_WEB_PORT = 9090
RERUN_GRPC_ADDR = f"rerun+http://127.0.0.1:{RERUN_GRPC_PORT}/proxy"

# Environment variable to control viewer mode: "web", "native", or "none"
RERUN_VIEWER_MODE = os.environ.get("RERUN_VIEWER", "web").lower()


def _is_port_in_use(port: int) -> bool:
    """Check if a port is already in use."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        return s.connect_ex(("127.0.0.1", port)) == 0


def _init_rerun() -> None:
    """Initialize Rerun - start server if main process, connect if worker."""
    import time

    rr.init("dimos")

    # Check if server is already running (with retry for workers starting
    # before main process server is fully up)
    for attempt in range(5):
        if _is_port_in_use(RERUN_GRPC_PORT):
            # Server already running (we're a worker) - connect to it
            rr.connect_grpc(RERUN_GRPC_ADDR)
            logger.info(f"Rerun: connected to server at {RERUN_GRPC_ADDR}")
            return
        if attempt == 0:
            # First attempt failed - we're likely the main process, start server
            break
        time.sleep(0.5)  # Wait for server to come up

    # Main process - start server based on viewer mode
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


# Initialize at import time
try:
    _init_rerun()
except Exception as e:
    logger.warning(f"Failed to initialize Rerun: {e}")


__all__ = ["rr"]
