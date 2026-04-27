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

import asyncio
from collections.abc import Callable
import time

import pytest
import websockets.asyncio.client as ws_client


def _wait_for_server(port: int, timeout: float = 5.0) -> None:
    """Block until the WebSocket server on *port* accepts a connection."""

    async def _probe() -> None:
        async with ws_client.connect(f"ws://127.0.0.1:{port}/ws"):
            pass

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            asyncio.run(_probe())
            return
        except Exception:
            time.sleep(0.05)
    raise TimeoutError(f"Server on port {port} did not become ready within {timeout}s")


@pytest.fixture()
def wait_for_server() -> Callable[[int, float], None]:
    """Fixture that returns a callable to wait for a WebSocket server."""
    return _wait_for_server
