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

"""The patched Portal client thread goes back to sleep once a burst of calls ends."""

from __future__ import annotations

from collections.abc import Iterator
import socket
import time

import pytest

from dimos.hardware.drive_trains.flowbase import portal_compat

portal = pytest.importorskip("portal")


def _free_port() -> int:
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


@pytest.fixture
def server_port() -> Iterator[int]:
    port = _free_port()
    server = portal.Server(port, name="test_portal_compat", errors=False)
    server.bind("echo", lambda data: data)
    server.start(block=False)
    try:
        yield port
    finally:
        server.close(timeout=2)


def _idle_cpu_seconds(window_s: float) -> float:
    """Process CPU time spent while this thread sleeps: what the other threads burn."""
    start = time.process_time()
    time.sleep(window_s)
    return time.process_time() - start


def test_install_patches_a_known_portal_once() -> None:
    assert portal_compat.portal_minor() in portal_compat.PATCHED_PORTAL_MINORS
    assert portal_compat.install()
    assert portal_compat.install(), "a second install is a no-op that still reports patched"


def test_client_thread_sleeps_after_a_burst_of_calls(server_port: int) -> None:
    """Unpatched, a sequential burst leaves the socket thread spinning at a full core
    (measured 99.9% CPU idle after one burst against the FlowBase)."""
    assert portal_compat.install()
    client = portal.Client(f"127.0.0.1:{server_port}", name="test_portal_compat")
    client.connect(timeout=5)
    try:
        for _ in range(200):
            assert client.echo({"x": 1}).result() == {"x": 1}
        busy = _idle_cpu_seconds(0.5)
    finally:
        client.close(timeout=2)
    assert busy < 0.2, f"the client thread burned {busy:.2f}s of CPU in a 0.5s idle window"
