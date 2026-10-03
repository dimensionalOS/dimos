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

from __future__ import annotations

import ipaddress
import json
import os
import queue
import re
import socket
import sys
import threading
from typing import Any

from dimos.cli.network.model import PROTOCOL_VERSION, Settings
from dimos.cli.network.session import Endpoint, zenoh_version


def emit(message: dict[str, Any]) -> None:
    print(json.dumps(message, allow_nan=False), flush=True)


def peer_main() -> None:
    """SSH-owned responder. Stdin EOF, a stop message or the lease ends the session."""
    initial = sys.stdin.readline(16385)
    config = json.loads(initial)
    if config.get("protocol") != PROTOCOL_VERSION:
        raise ValueError("network-check protocol mismatch")
    token = config["token"]
    if not isinstance(token, str) or re.fullmatch(r"[0-9a-f]{32}", token) is None:
        raise ValueError("invalid session token")
    settings = Settings(**config["settings"])
    settings.validate()
    listen_host = str(ipaddress.ip_address(config["listen_host"]))
    port = int(config["port"])
    if not 0 <= port <= 65535:
        raise ValueError("invalid listen port")
    if port == 0:
        # Zenoh 1.10.1 exposes links but not the bound listener's ephemeral port.
        # Select a port first; a bind race fails visibly rather than reusing a service.
        family = socket.AF_INET6 if ":" in listen_host else socket.AF_INET
        with socket.socket(family) as sock:
            sock.bind((listen_host, 0))
            port = sock.getsockname()[1]
    address = f"[{listen_host}]" if ":" in listen_host else listen_host
    stop = threading.Event()
    commands: queue.Queue[dict[str, Any]] = queue.Queue(maxsize=8)

    def read_commands() -> None:
        try:
            for line in sys.stdin:
                if len(line) > 16384:
                    break
                request = json.loads(line)
                if request.get("op") == "stop":
                    stop.set()
                    break
                commands.put_nowait(request)
        except (ValueError, queue.Full):
            stop.set()
        finally:
            stop.set()

    lease = threading.Timer(settings.max_seconds, stop.set)
    # Last resort for a native transport teardown that does not return. This
    # kills only this responder, never an existing DimOS/robot process.
    hard_limit = threading.Timer(settings.max_seconds + 5, lambda: os._exit(124))
    lease.start()
    hard_limit.start()
    endpoint: Endpoint | None = None
    try:
        endpoint = Endpoint(token, "remote", f"tcp/{address}:{port}", stop)
        emit(
            {
                "op": "hello",
                "protocol": PROTOCOL_VERSION,
                "zenoh": zenoh_version(),
                "port": port,
                "pid": os.getpid(),
            }
        )
        threading.Thread(target=read_commands, name="network-check-stdin", daemon=True).start()
        used_bytes = 0
        while not stop.is_set():
            try:
                request = commands.get(timeout=0.1)
            except queue.Empty:
                continue
            op = request["op"]
            if op == "prepare":
                endpoint.receiver.reset(int(request["phase"]), float(request["duration"]))
                emit({"op": "prepared"})
            elif op == "send":
                rate = float(request["rate"])
                duration = float(request["duration"])
                if not 0 < rate <= settings.max_mbps or not 0 < duration <= settings.max_seconds:
                    raise ValueError("requested send exceeds session bounds")
                result = endpoint.send(
                    int(request["phase"]), rate, duration, settings, settings.max_bytes - used_bytes
                )
                used_bytes += result["bytes"]
                emit({"op": "sent", "sender": result})
            elif op == "stats":
                emit({"op": "stats", "receiver": endpoint.receiver.summary(int(request["sent"]))})
            else:
                raise ValueError(f"unknown operation {op}")
    finally:
        stop.set()
        if endpoint is not None:
            endpoint.close()
        lease.cancel()
        hard_limit.cancel()
    emit({"op": "bye", "cleanup": "confirmed"})
