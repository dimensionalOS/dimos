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

"""Real broker/SFU qualification using only a newly created, owned test session."""

from __future__ import annotations

from contextlib import ExitStack
from importlib.util import find_spec
import multiprocessing
from multiprocessing.connection import Connection
import os
from queue import Queue
import threading
import time
from typing import Any, cast
from uuid import uuid4

from dimos.diagnostics.factory import traced_link
from dimos.diagnostics.hosted_fixture import OperatorFixtureConfig
from dimos.diagnostics.model import TraceContext, context_scope, current_context
from dimos.protocol.pubsub.impl.webrtc.providers.broker import BrokerConfig, BrokerProvider
from dimos.protocol.pubsub.impl.webrtc.providers.spec import shutdown_all_providers


def _robot(pipe: Connection, run: str, options: dict[str, Any]) -> None:
    transport = None
    lock = threading.Lock()

    def report(value: Any) -> None:
        with lock:
            pipe.send(value)

    try:
        config = BrokerConfig(**options)
        provider = cast("BrokerProvider", config.provider())
        provider.start()
        report(("session", provider.session_id))
        transport = traced_link(
            "cloudflare",
            run,
            "conformance",
            str,
            config=config,
            publish_channel="state_reliable_back",
            receive_channel="state_reliable",
        ).build()

        def received(value: str) -> None:
            ctx = current_context()
            report(("received", value, ctx.record() if ctx else None))

        transport.subscribe(received)
        # One broker operator is allowed; fan-out is to two actual application
        # subscribers in the receiver, not fictitious parallel operator sessions.
        transport.subscribe(received)
        while True:
            command = pipe.recv()
            if command[0] == "stop":
                return
            if command[0] == "ready":
                transport.wait_ready(30)
                report(("ready",))
            if command[0] == "send":
                ctx = TraceContext(
                    run, command[1], "robot", command[2], time.monotonic_ns(), "fixture/robot"
                )
                with context_scope(ctx):
                    transport.publish("identical-payload")
    except Exception as error:
        report(("error", type(error).__name__))
    finally:
        if transport is not None:
            transport.stop()
        shutdown_all_providers()
        pipe.close()


def run_hosted(options: dict[str, Any] | None = None) -> dict[str, Any]:
    """No sessions are listed or selected: the fixture creates and deletes its own."""
    if find_spec("aiortc") is None:
        return {"backend": "cloudflare", "status": "skipped", "reason": "missing aiortc"}
    options = options or {
        "broker_url": os.environ.get("DIMOS_TEST_BROKER_URL"),
        "api_key": os.environ.get("DIMOS_TEST_ROBOT_API_KEY"),
        "operator_token": os.environ.get("DIMOS_TEST_OPERATOR_TOKEN"),
        "dedicated_test_robot": os.environ.get("DIMOS_TEST_DEDICATED_ROBOT") == "1",
    }
    if not options.get("dedicated_test_robot") or not all(
        options.get(k) for k in ("broker_url", "api_key", "operator_token")
    ):
        return {
            "backend": "cloudflare",
            "status": "skipped",
            "reason": "dedicated test robot key, operator token and broker URL required",
        }
    run = uuid4().hex
    publishers: list[str] = []
    deliveries = 0
    for sequences in ([1, 3, 2, 3, 4], [1]):
        with ExitStack() as cleanup:
            parent, child = multiprocessing.get_context("spawn").Pipe()
            process = multiprocessing.get_context("spawn").Process(
                target=_robot,
                args=(
                    child,
                    run,
                    {
                        "broker_url": options["broker_url"],
                        "api_key": options["api_key"],
                        "robot_name": "diagnostics-" + run,
                    },
                ),
            )
            process.start()
            child.close()

            def stop(parent: Connection = parent, process: Any = process) -> None:
                try:
                    parent.send(("stop",))
                except (OSError, EOFError):
                    pass
                process.join(35)
                if process.is_alive():
                    process.terminate()
                    process.join(5)
                parent.close()

            cleanup.callback(stop)

            def receive(kind: str, parent: Connection = parent) -> Any:
                if not parent.poll(60):
                    raise TimeoutError("hosted fixture: " + kind)
                value = parent.recv()
                if value[0] != kind:
                    raise RuntimeError("hosted fixture failed: " + value[0])
                return value

            session_id = receive("session")[1]
            config = OperatorFixtureConfig(
                broker_url=options["broker_url"],
                token=options["operator_token"],
                session_id=session_id,
            )
            operator = traced_link(
                "webrtc",
                run,
                "conformance",
                str,
                config=config,
                publish_channel="state_reliable",
                receive_channel="state_reliable_back",
            ).build()
            cleanup.callback(shutdown_all_providers)
            cleanup.callback(operator.stop)
            received: Queue[Any] = Queue()

            def on_message(value: str, received: Queue[Any] = received) -> None:
                ctx = current_context()
                received.put((value, ctx.record() if ctx else None))

            operator.subscribe(on_message)
            operator.subscribe(on_message)
            parent.send(("ready",))
            operator.wait_ready(30)
            receive("ready")
            operator_id, robot_id = uuid4().hex, uuid4().hex
            publishers.extend([operator_id, robot_id])
            for sequence in sequences:
                ctx = TraceContext(
                    run, operator_id, "operator", sequence, time.monotonic_ns(), "fixture/operator"
                )
                with context_scope(ctx):
                    operator.publish("identical-payload")
                for _ in range(2):
                    result = receive("received")
                    if (
                        result[1] != "identical-payload"
                        or result[2]["sequence"] != sequence
                        or result[2]["publisher"] != operator_id
                    ):
                        raise AssertionError("operator identity changed over hosted transport")
                    deliveries += 1
                parent.send(("send", robot_id, sequence))
                for _ in range(2):
                    value, context = received.get(timeout=30)
                    if (
                        value != "identical-payload"
                        or context["sequence"] != sequence
                        or context["publisher"] != robot_id
                    ):
                        raise AssertionError("robot identity changed over hosted transport")
                    deliveries += 1
    return {
        "backend": "cloudflare",
        "status": "passed",
        "deliveries": deliveries,
        "publisher_restart": True,
        "directional_channels": True,
        "separate_robot_process": True,
        "sequence_order": [1, 3, 2, 3, 4],
        "fanout": "two application subscribers per receiver; broker permits one operator",
        "publisher_incarnations": len(set(publishers)),
    }
