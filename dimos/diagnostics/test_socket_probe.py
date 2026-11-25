# Copyright 2025-2026 Dimensional Inc.
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

"""Real loopback probe with caller-owned socket, no fixed timing sleeps."""

from concurrent.futures import ThreadPoolExecutor
import socket

from dimos.diagnostics.socket_probe import socket_probe


def test_udp_probe_matches_nonce_and_reports_visibility():
    with (
        socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server,
        ThreadPoolExecutor(max_workers=1) as pool,
    ):
        server.bind(("127.0.0.1", 0))
        server.settimeout(3)

        def echo_once():
            data, address = server.recvfrom(1024)
            server.sendto(data, address)

        reply = pool.submit(echo_once)
        result = socket_probe("127.0.0.1", server.getsockname()[1])
        reply.result(timeout=3)
    assert result["status"] == "ok"
    assert result["rtt_ns"] > 0
    assert result["scope"] == "dedicated echo socket; not application delivery or over-air timing"
