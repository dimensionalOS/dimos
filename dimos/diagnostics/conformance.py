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

"""Real separate-process traced-carrier conformance, with an owned local test domain."""

from __future__ import annotations

from contextlib import ExitStack
from importlib.util import find_spec
import multiprocessing
from multiprocessing.connection import Connection
import os
import socket
import threading
from typing import Any
from uuid import uuid4

import zenoh

from dimos.diagnostics.factory import traced_link
from dimos.diagnostics.hosted_conformance import run_hosted
from dimos.diagnostics.model import TraceContext, context_scope, current_context
from dimos.diagnostics.traced_transport import TracedTransport
from dimos.protocol.pubsub.impl.webrtc.providers.spec import shutdown_all_providers


def _peer(pipe: Connection, backend: str, run: str, link: str, options: dict[str, Any]) -> None:
    transport = None
    try:
        transport = traced_link(backend, run, link, str, **options).build()
        assert isinstance(transport, TracedTransport)
        lock = threading.Lock()

        def report(message: Any) -> None:
            with lock:
                pipe.send(message)

        def received(value: str) -> None:
            ctx = current_context()
            report(("received", value, ctx.record() if ctx else None))

        transport.subscribe(received)
        report(("started", os.getpid(), transport.publisher))
        while True:
            command = pipe.recv()
            if command[0] == "stop":
                return
            if command[0] == "ready":
                transport.wait_ready(10, expected_peers=command[1])
                report(("ready", os.getpid()))
            elif command[0] == "send":
                ctx = TraceContext(
                    run, command[1], "input", command[2], command[3], "conformance/source"
                )
                with context_scope(ctx):
                    transport.publish("identical-payload")
    except Exception as error:
        try:
            pipe.send(("error", type(error).__name__, str(error)))
        except (EOFError, BrokenPipeError):
            pass
    finally:
        if transport is not None:
            transport.stop()
        shutdown_all_providers()
        pipe.close()


def _receive(pipe: Connection, kind: str, timeout: float = 15) -> Any:
    if not pipe.poll(timeout):
        raise TimeoutError(f"waiting for conformance {kind}")
    value = pipe.recv()
    if value[0] != kind:
        raise RuntimeError(f"conformance expected {kind}, got {value!r}")
    return value


def run_backend(backend: str, options: dict[str, Any] | None = None) -> dict[str, Any]:
    if backend in ("webrtc", "cloudflare"):
        return {**run_hosted(options), "backend": backend}
    dependency = {
        "ros": "rclpy",
        "dds": "cyclonedds",
        "webrtc": "aiortc",
        "cloudflare": "aiortc",
    }.get(backend)
    if dependency and find_spec(dependency) is None:
        return {"backend": backend, "status": "skipped", "reason": f"missing {dependency}"}
    context = multiprocessing.get_context("spawn")
    run, link = uuid4().hex, uuid4().hex
    settings = dict(options or {})
    with ExitStack() as cleanup:
        if backend == "zenoh" and not options:
            config = zenoh.Config()
            config.insert_json5("scouting/multicast/enabled", "false")
            config.insert_json5("scouting/gossip/enabled", "false")
            with socket.socket() as reservation:
                reservation.bind(("127.0.0.1", 0))
                endpoint = f"tcp/127.0.0.1:{reservation.getsockname()[1]}"
            config.insert_json5("listen/endpoints", f'["{endpoint}"]')
            router = zenoh.open(config)
            cleanup.callback(router.close)
            endpoints = [endpoint]
            settings.update(
                mode="client", connect=endpoints, multicast=False, gossip=False, scouting=False
            )
        if backend == "lcm" and not options:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
                reservation.bind(("127.0.0.1", 0))
                port = reservation.getsockname()[1]
            settings["url"] = f"udpm://239.255.76.67:{port}?ttl=0"

        def start_peer() -> tuple[Connection, Any, str]:
            parent, child = context.Pipe()
            process = context.Process(target=_peer, args=(child, backend, run, link, settings))
            process.start()
            child.close()

            def close(process: Any = process, pipe: Connection = parent) -> None:
                try:
                    pipe.send(("stop",))
                except (EOFError, BrokenPipeError, OSError):
                    pass
                process.join(3)
                if process.is_alive():
                    process.terminate()
                    process.join(3)
                pipe.close()

            cleanup.callback(close)
            started = _receive(parent, "started")
            return parent, close, started[2]

        peers = [start_peer() for _ in range(3)]
        pipes = [peer[0] for peer in peers]
        for pipe in pipes:
            pipe.send(("ready", [peer[2] for peer in peers]))
        for pipe in pipes:
            _receive(pipe, "ready")
        # Identical bytes, reordering and duplicate sequence: identity must be independent of payload.
        expected = [1, 3, 2, 3, 4]
        publisher = uuid4().hex
        for sequence in expected:
            pipes[0].send(("send", publisher, sequence, sequence * 1000))
            for pipe in pipes:
                result = _receive(pipe, "received")
                if (
                    result[1] != "identical-payload"
                    or result[2]["sequence"] != sequence
                    or result[2]["publisher"] != publisher
                ):
                    raise AssertionError("payload or identity changed across transport")
        peers[0][1]()
        peers[0] = start_peer()
        replacement = peers[0][0]
        pipes[0] = replacement
        for pipe in pipes:
            pipe.send(("ready", [peer[2] for peer in peers]))
        for pipe in pipes:
            _receive(pipe, "ready")
        restarted_publisher = uuid4().hex
        replacement.send(("send", restarted_publisher, 1, 9999))
        for pipe in pipes:
            result = _receive(pipe, "received")
            if result[2]["publisher"] != restarted_publisher or result[2]["sequence"] != 1:
                raise AssertionError("publisher restart aliased the old incarnation")
        return {
            "backend": backend,
            "status": "passed",
            "processes": 3,
            "deliveries": (len(expected) + 1) * len(pipes),
            "publisher_restart": True,
            "identical_payloads": True,
            "sequence_order": expected,
        }
