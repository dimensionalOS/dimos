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

"""Free local ports for tests that bind, so parallel test processes stay apart."""

from __future__ import annotations

import socket


def free_tcp_port(host: str = "127.0.0.1") -> int:
    """A TCP port nothing listens on right now.

    The kernel hands it out and we release it, so another caller could grab
    it in between; good enough to keep test processes off each other.
    """
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind((host, 0))
        port: int = sock.getsockname()[1]
    return port


def free_zmq_endpoints(host: str = "127.0.0.1") -> dict[str, str]:
    """`ZMQConfig` endpoint kwargs on two free loopback ports: a private bus."""
    return {
        "pub_endpoint": f"tcp://{host}:{free_tcp_port(host)}",
        "sub_endpoint": f"tcp://{host}:{free_tcp_port(host)}",
    }
