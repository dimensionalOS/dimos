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

"""Explicit traced links over existing typed transports, on dedicated channels."""

from __future__ import annotations

import base64
from collections.abc import Callable, Collection
import json
from queue import Empty, Full, Queue
import threading
import time
from typing import Any, cast
from uuid import uuid4

from dimos.core.stream import Stream, Transport, TransportSpec
from dimos.diagnostics.clock import clock_domain
from dimos.diagnostics.envelope import MAX_PAYLOAD, pack, traced_channel, unpack
from dimos.diagnostics.model import MAX_CONTEXT_BYTES, TraceContext, context_scope, current_context
from dimos.diagnostics.runtime import get_runtime
from dimos.protocol.encode import cbor


class TracedTransport(Transport[Any]):
    """Both endpoints must explicitly select this transport and the same link contract.

    The carrier's message must have a string ``data`` field (std_msgs.String or
    the DDS TraceFrame). Legacy channels are never inspected or modified.
    """

    requires_negotiation = True

    def __init__(
        self, carrier: TransportSpec, wire_type: type, payload_type: type, run_id: str, link_id: str
    ) -> None:
        self.carrier_spec, self.wire_type, self.payload_type = carrier, wire_type, payload_type
        self.run_id, self.link_id = run_id, link_id
        self.channel = traced_channel(run_id, link_id)
        configured_channel = carrier.kwargs.get("topic")
        if configured_channel != self.channel:
            raise ValueError("traced carrier must use the dedicated run/link channel")
        self.carrier = carrier.build()
        self.publisher = uuid4().hex
        self.sequence = 0
        self._sequence_lock = threading.Lock()
        self.encoding = "lcm" if callable(getattr(payload_type, "lcm_decode", None)) else "cbor"
        self.type_id = payload_type.__module__ + "." + payload_type.__qualname__
        self._ready = threading.Event()
        self._stopped = threading.Event()
        self._error: str | None = None
        self._callbacks: list[Callable[[Any], Any]] = []
        self._control_dispose: Callable[[], None] | None = None
        self._thread: threading.Thread | None = None
        self._start_lock = threading.Lock()
        self._peers: set[str] = set()
        self._acks: Queue[str] = Queue(maxsize=32)
        self._negotiated = threading.Condition()
        self._acknowledged: set[str] = set()

    def cbor_encode(self) -> dict[str, Any]:
        return {
            "carrier": self.carrier_spec,
            "wire_type": self.wire_type,
            "payload_type": self.payload_type,
            "run_id": self.run_id,
            "link_id": self.link_id,
        }

    def broadcast(self, selfstream: Stream[Any] | None, value: Any) -> None:
        self.start()
        if self._error or not self._ready.is_set():
            raise RuntimeError(self._error or "traced peer not ready; call wait_ready during setup")
        if not isinstance(value, self.payload_type):
            raise TypeError("traced payload type mismatch")
        with self._sequence_lock:
            self.sequence += 1
            sequence = self.sequence
        ctx = current_context()
        if ctx is None:
            ctx = TraceContext(
                self.run_id,
                self.publisher,
                self.link_id,
                sequence,
                time.monotonic_ns(),
                clock_domain(),
            )
        if ctx.run_id != self.run_id:
            raise ValueError("traced input belongs to another run")
        payload = pack(
            self.link_id,
            self.type_id,
            ctx,
            cast("Any", value).lcm_encode() if self.encoding == "lcm" else cbor.encode(value),
        )
        self.carrier.publish(self.wire_type(data=base64.b64encode(payload).decode("ascii")))

    def subscribe(
        self, callback: Callable[[Any], Any], selfstream: Stream[Any] | None = None
    ) -> Callable[[], None]:
        self.start()
        self._callbacks.append(callback)

        def unsubscribe() -> None:
            if callback in self._callbacks:
                self._callbacks.remove(callback)

        return unsubscribe

    def _receive(self, message: Any) -> None:
        if isinstance(message.data, str) and message.data.startswith("DIMOS_HELLO_V1:"):
            self._negotiate(message.data[len("DIMOS_HELLO_V1:") :])
            return
        if not self._ready.is_set() or self._error:
            return
        try:
            if not isinstance(message.data, str) or len(message.data) > 4 * (
                (MAX_PAYLOAD + MAX_CONTEXT_BYTES + 14) // 3
            ):
                raise ValueError("traced frame exceeds limit")
            ctx, payload = unpack(
                base64.b64decode(message.data, validate=True), self.link_id, self.type_id
            )
            if ctx.run_id != self.run_id:
                raise ValueError("wrong trace run")
            value = (
                cast("Any", self.payload_type).lcm_decode(payload)
                if self.encoding == "lcm"
                else cbor.decode(payload)
            )
            if not isinstance(value, self.payload_type):
                raise TypeError("decoded payload type mismatch")
        except (ValueError, TypeError, KeyError) as error:
            runtime = get_runtime()
            if runtime:
                runtime.event("trace.invalid_context", reason=type(error).__name__)
            raise
        with context_scope(ctx):
            for callback in tuple(self._callbacks):
                callback(value)

    def _contract(self) -> dict[str, str]:
        return {
            "schema": "dimos.trace/v1",
            "run": self.run_id,
            "link": self.link_id,
            "type": self.type_id,
            "encoding": self.encoding,
        }

    def _hello(self, ack: str | None = None) -> None:
        self.carrier.publish(
            self.wire_type(
                data="DIMOS_HELLO_V1:"
                + json.dumps({"endpoint": self.publisher, "contract": self._contract(), "ack": ack})
            )
        )

    def _negotiate(self, data: str) -> None:
        if len(data) > 4096:
            return
        try:
            peer = json.loads(data)
            identity = peer["endpoint"]
            if identity == self.publisher:
                with self._negotiated:
                    self._acknowledged.add(identity)
                    self._negotiated.notify_all()
                return
            if not isinstance(identity, str) or len(identity) > 128:
                return
            if identity not in self._peers and len(self._peers) >= 32:
                return
            if peer.get("contract") != self._contract():
                self._error = "incompatible traced peer contract"
                self._ready.set()  # wake setup waiters so they see the failure
                return
            self._peers.add(identity)
            if peer.get("ack") == self.publisher:
                with self._negotiated:
                    self._acknowledged.add(identity)
                    self._negotiated.notify_all()
                self._ready.set()
            elif peer.get("ack") is None:
                # DDS invokes listeners while holding native receive locks.
                # Sending an acknowledgement here can deadlock a loopback writer.
                # The existing negotiation worker owns all handshake writes.
                try:
                    self._acks.put_nowait(identity)
                except Full:
                    pass  # A subsequent periodic hello retries this bounded setup.
        except (ValueError, KeyError, TypeError):
            self._error = "malformed traced peer contract"
            self._ready.set()

    def wait_ready(self, timeout: float = 5.0, expected_peers: Collection[str] = ()) -> None:
        """Setup barrier, never called on the command path. Both endpoints must start first."""
        self.start()
        deadline = time.monotonic() + timeout
        if not self._ready.wait(timeout):
            raise TimeoutError("traced peer negotiation timed out")
        if self._error:
            raise ValueError(self._error)
        if expected_peers:
            with self._negotiated:
                if not self._negotiated.wait_for(
                    lambda: set(expected_peers).issubset(self._acknowledged),
                    timeout=max(0, deadline - time.monotonic()),
                ):
                    raise TimeoutError("not all required traced peers negotiated")

    def start(self) -> None:
        with self._start_lock:
            if self._stopped.is_set():
                raise RuntimeError("traced endpoint stopped; create a new incarnation")
            if self._thread is not None:
                return
            self.carrier.start()
            self._control_dispose = self.carrier.subscribe(self._receive)

            def announce() -> None:
                while not self._stopped.is_set():
                    try:
                        self._hello()
                        for _ in range(32):
                            try:
                                peer = self._acks.get_nowait()
                            except Empty:
                                break
                            self._hello(peer)
                    except Exception as error:
                        self._error = f"traced setup failed: {type(error).__name__}"
                        self._ready.set()
                        return
                    self._stopped.wait(0.2)

            self._thread = threading.Thread(
                target=announce, name="dimos-trace-negotiation", daemon=True
            )
            self._thread.start()

    def stop(self) -> None:
        if self._stopped.is_set():
            return
        self._stopped.set()
        if self._thread:
            self._thread.join(timeout=2)
        if self._control_dispose:
            self._control_dispose()
        self.carrier.stop()
