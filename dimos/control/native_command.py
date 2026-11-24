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

"""Fence opt-in native command ports without changing their payload types."""

from __future__ import annotations

from contextvars import ContextVar
from dataclasses import dataclass
import threading
import time
from typing import TYPE_CHECKING, Any, TypeVar, cast

from dimos.core.stream import TransportSpec
from dimos.core.transport import pZenohTransport

if TYPE_CHECKING:
    from collections.abc import Callable

    from dimos.control.connection.connection_module import ConnectionConfig
    from dimos.core.coordination.blueprints import Blueprint
    from dimos.core.stream import Out, Stream, Transport

T = TypeVar("T")


@dataclass(frozen=True, slots=True)
class CommandTicket:
    epoch: int
    sequence: int
    deadline: int


_sending: ContextVar[tuple[object, int]] = ContextVar("native_command_deadline", default=(None, 0))
_receiving: ContextVar[CommandTicket] = ContextVar("native_command_ticket")


class NativeCommandTransport(pZenohTransport[T]):
    """Typed ports over one private Zenoh stream carrying data and revocations.

    Both workers must share a monotonic clock (same host). There is one publishing
    coordinator per stream. A newer epoch also fences old grants if a revoke was lost.
    """

    def __init__(self, topic: str, interface: str) -> None:
        super().__init__(topic)
        self.interface = interface
        self._epoch = time.monotonic_ns()
        self._seen = self._written = 0
        self._received = self._epoch  # Never admit data sent before this worker was bound.
        self._admission = threading.Lock()

    def cbor_encode(self) -> dict[str, Any]:
        return {"topic": self.topic, "interface": self.interface}

    def offer(self, port: Out[T], payload: T, deadline: int) -> None:
        token = _sending.set((port, deadline))
        try:
            port.publish(payload)
        finally:
            _sending.reset(token)

    def broadcast(self, stream: Stream[T] | None, msg: T) -> None:
        now = time.monotonic_ns()
        port, deadline = _sending.get()
        if stream is port and deadline > now:
            self._send((self._epoch, now, deadline, msg))

    def revoke(self) -> None:
        self._epoch = time.monotonic_ns()
        self._send((self._epoch, self._epoch, 0, None))

    def _send(self, frame: tuple[int, int, int, T | None]) -> None:
        # The framing is private to the transport. The Out and In both use T.
        super().broadcast(None, cast("T", frame))

    def subscribe(
        self, callback: Callable[[T], None], selfstream: Stream[T] | None = None
    ) -> Callable[[], None]:
        def receive(wire: T) -> None:
            if not isinstance(wire, tuple) or len(wire) != 4:
                return
            epoch, seq, deadline, payload = wire
            if not all(type(n) is int for n in (epoch, seq, deadline)):
                return
            now = time.monotonic_ns()
            if not 0 < epoch <= seq <= now or not (
                (deadline == 0 and payload is None)
                or (deadline > now and selfstream is not None and type(payload) is selfstream.type)
            ):
                return
            with self._admission:
                if epoch < self._seen or seq <= self._received:
                    return
                self._seen, self._received = epoch, seq
            if deadline:
                token = _receiving.set(CommandTicket(epoch, seq, deadline))
                try:
                    callback(cast("T", payload))
                finally:
                    _receiving.reset(token)

        return super().subscribe(receive, selfstream)

    def admit(self, ticket: CommandTicket, timeout_s: float) -> bool:
        with self._admission:
            if (
                ticket.epoch != self._seen
                or ticket.sequence <= self._written
                or time.monotonic_ns() >= min(ticket.deadline, ticket.sequence + timeout_s * 1e9)
            ):
                return False
            self._written = ticket.sequence
            return True


def received_ticket() -> CommandTicket:
    return _receiving.get()


def native_transport(port: Stream[T], interface: str) -> NativeCommandTransport[T]:
    transport = getattr(port, "_transport", None)
    if not isinstance(transport, NativeCommandTransport) or transport.interface != interface:
        raise ValueError(f"{interface}: {port.name!r} needs its wired native command transport")
    return transport


def wire_native_commands(
    blueprint: Blueprint, coordinator: str, connection: str, declaration: ConnectionConfig
) -> Blueprint:
    """Validate the native endpoints of explicit blueprint remaps and frame those streams."""
    atoms = {atom.name: atom for atom in blueprint.active_blueprints}
    transports: dict[tuple[str, type], TransportSpec | Transport[Any]] = {}
    for name in declaration.group_commands:
        interface = f"{declaration.source}/{name}"
        expected = ((coordinator, f"{declaration.source}_{name}"), (connection, name))
        topic = blueprint.remapping_map.get(expected[1], name)
        endpoints = {
            (atom.name, port.name): port
            for atom in atoms.values()
            for port in atom.streams
            if blueprint.remapping_map.get((atom.name, port.name), port.name) == topic
        }
        output, receiver = (endpoints.get(key) for key in expected)
        if (
            not isinstance(topic, str)
            or len(endpoints) != 2
            or output is None
            or receiver is None
            or output.direction != "out"
            or receiver.direction != "in"
            or output.type is not receiver.type
            or not isinstance(receiver.type, type)
            or (topic, receiver.type) in blueprint.transport_map
        ):
            raise ValueError(f"{interface}: remap one matching typed Out to the native In")
        transports[topic, receiver.type] = TransportSpec(NativeCommandTransport, (topic, interface))
    return blueprint.transports(transports)
