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

"""Small provider-owned command lane, independent of bulk puts and encoders.

The Zenoh put must be nonblocking (express, REAL_TIME, congestion DROP). The
robot still needs its own watchdog: this process cannot stop a disconnected
robot or run after SIGKILL. Tickets are provider-clock deadlines, not browser
timestamps; delayed heartbeats/commands cannot extend an expired command.
"""

from collections.abc import Callable
from dataclasses import dataclass, field
import threading
import time
from typing import Literal

from dimos.web.relay_bridge.topic_protocol import (
    MAX_TOPIC_GUARD_BYTES,
    MAX_TOPIC_GUARDS,
    TopicGuardClosed,
    TopicGuardCommandMeta,
    TopicGuardHeartbeat,
    TopicGuardOpenMeta,
    TopicGuardReady,
    TopicGuardResult,
    TopicGuardTicket,
    TopicMsg,
)


@dataclass
class _Guard:
    spec: TopicGuardOpenMeta
    stop: bytes
    heartbeat_deadline: float
    command_deadline: float
    ready: bool = False
    closing: str = ""
    sequence: int = 0
    ticket: int = 0
    issued_at: float = 0
    tickets: dict[int, float] = field(default_factory=dict)
    pending: tuple[int, bytes, float] | None = None


class GuardedPublishers:
    """At most 32 handles, one pending command each, and one watchdog thread.

    State and nonblocking puts are serialized by one condition lock. Stop
    latches before the next command can run. Expired/closed handles never
    reopen; the caller must explicitly create a fresh publisher.
    """

    def __init__(
        self,
        put: Callable[[str, bytes, str], None],
        emit: Callable[[TopicMsg], None],
        *,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._put = put
        self._emit = emit
        self._clock = clock
        self._condition = threading.Condition()
        self._guards: dict[str, _Guard] = {}
        self._closed = False
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        with self._condition:
            if self._thread is not None or self._closed:
                raise RuntimeError("publisher watchdog already started or closed")
            self._thread = threading.Thread(target=self._run, name="topic-watchdog", daemon=True)
            self._thread.start()

    def _closed_message(self, spec: TopicGuardOpenMeta, code: str) -> None:
        self._emit(TopicGuardClosed(handle=spec.handle, gen=spec.gen, code=code))

    def open(self, spec: TopicGuardOpenMeta, stop: bytes) -> None:
        with self._condition:
            if self._closed:
                self._closed_message(spec, "provider_closed")
                return
            if len(stop) > MAX_TOPIC_GUARD_BYTES or not stop:
                self._closed_message(spec, "invalid_guard_stop")
                return
            if spec.handle in self._guards or len(self._guards) >= MAX_TOPIC_GUARDS:
                self._closed_message(spec, "guard_limit")
                return
            conflicts = [
                guard
                for guard in self._guards.values()
                if not guard.closing
                and (guard.spec.group == spec.group or guard.spec.key == spec.key)
                and guard.spec.owner != spec.owner
            ]
            if conflicts and not spec.takeover:
                self._closed_message(spec, "guard_owned")
                return
            for guard in conflicts:
                guard.closing = "taken_over"
            # A duplicate key owned by the same viewer also replaces its old
            # stop before installing the new one; two handles cannot drive it.
            for guard in self._guards.values():
                if guard.spec.key == spec.key and not guard.closing:
                    guard.closing = "replaced"
            now = self._clock()
            deadline = now + float(spec.timeoutMs) / 1000
            self._guards[spec.handle] = _Guard(spec, bytes(stop), deadline, deadline)
            self._condition.notify()

    def _ticket(self, guard: _Guard, now: float) -> int:
        guard.tickets = {ticket: end for ticket, end in guard.tickets.items() if now < end}
        period = float(guard.spec.timeoutMs) / 1000
        if not guard.ticket or now - guard.issued_at >= period / 4:
            guard.ticket += 1
            guard.issued_at = now
            guard.tickets[guard.ticket] = now + period
        return guard.ticket

    def heartbeat(self, msg: TopicGuardHeartbeat) -> None:
        with self._condition:
            guard = self._guards.get(msg.handle)
            now = self._clock()
            if guard is None or guard.spec.gen != msg.gen or not guard.ready or guard.closing:
                return
            if now >= min(guard.heartbeat_deadline, guard.command_deadline):
                guard.closing = "guard_expired"
            elif now < guard.tickets.get(int(msg.ticket), 0):
                ticket = self._ticket(guard, now)
                guard.heartbeat_deadline = guard.tickets[ticket]
                self._emit(TopicGuardTicket(handle=msg.handle, gen=msg.gen, ticket=ticket))
            self._condition.notify()

    def _result(
        self,
        guard: _Guard,
        sequence: int,
        outcome: Literal["accepted", "rejected", "unknown"],
        code: str = "",
    ) -> None:
        self._emit(
            TopicGuardResult(
                handle=guard.spec.handle,
                gen=guard.spec.gen,
                seq=sequence,
                outcome=outcome,
                code=code,
                providerTs=time.time(),
            )
        )

    def command(self, meta: TopicGuardCommandMeta, sequence: int, payload: bytes) -> None:
        with self._condition:
            guard = self._guards.get(meta.handle)
            if guard is None or guard.spec.gen != meta.gen:
                return
            now = self._clock()
            deadline = guard.tickets.get(int(meta.ticket), 0)
            if (
                guard.closing
                or not guard.ready
                or now >= min(deadline, guard.heartbeat_deadline, guard.command_deadline)
                or sequence <= guard.sequence
                or len(payload) > MAX_TOPIC_GUARD_BYTES
            ):
                self._result(guard, sequence, "rejected", "stale_guard_command")
                return
            guard.sequence = sequence
            if guard.pending is not None:
                self._result(guard, guard.pending[0], "rejected", "superseded")
            guard.pending = sequence, bytes(payload), deadline
            self._condition.notify()

    def stop(self, handle: str, generation: int | float, code: str = "released") -> None:
        with self._condition:
            guard = self._guards.get(handle)
            if guard is not None and guard.spec.gen == generation:
                guard.closing = guard.closing or code
                self._condition.notify()

    def poll(self) -> float | None:
        """Process due work; also supports deterministic clock-driven tests."""
        with self._condition:
            now = self._clock()
            for guard in self._guards.values():
                if now >= min(guard.heartbeat_deadline, guard.command_deadline):
                    guard.closing = guard.closing or "guard_expired"
            # Stop every displaced owner before a replacement can be ready.
            for handle, guard in list(self._guards.items()):
                if not guard.closing:
                    continue
                if guard.pending is not None:
                    self._result(guard, guard.pending[0], "rejected", guard.closing)
                try:
                    self._put(guard.spec.key, guard.stop, guard.spec.encoding)
                except Exception:
                    guard.closing = "guard_stop_failed"
                    # Do not acknowledge a replacement if stopping its old
                    # owner had an unknown outcome.
                    for other in self._guards.values():
                        if other.spec.group == guard.spec.group or other.spec.key == guard.spec.key:
                            other.closing = "guard_stop_failed"
                self._closed_message(guard.spec, guard.closing)
                del self._guards[handle]
            for guard in self._guards.values():
                if guard.closing:
                    # Marked while stopping a group mate above: neither armed
                    # nor driven, the next poll stops it.
                    continue
                if not guard.ready:
                    # Establish the safe state before acknowledging ownership.
                    try:
                        self._put(guard.spec.key, guard.stop, guard.spec.encoding)
                    except Exception:
                        guard.closing = "guard_put_failed"
                        continue
                    guard.ready = True
                    ticket = self._ticket(guard, now)
                    self._emit(
                        TopicGuardReady(
                            id=guard.spec.id,
                            handle=guard.spec.handle,
                            gen=guard.spec.gen,
                            ticket=ticket,
                            timeoutMs=guard.spec.timeoutMs,
                        )
                    )
                if guard.pending is None:
                    continue
                sequence, payload, deadline = guard.pending
                guard.pending = None
                if self._clock() >= deadline:
                    self._result(guard, sequence, "rejected", "stale_guard_command")
                    continue
                try:
                    self._put(guard.spec.key, payload, guard.spec.encoding)
                except Exception:
                    self._result(guard, sequence, "unknown", "guard_put_failed")
                    guard.closing = "guard_put_failed"
                else:
                    # A delayed packet cannot buy another full timeout.
                    guard.command_deadline = deadline
                    self._result(guard, sequence, "accepted")
            if any(guard.closing for guard in self._guards.values()):
                return 0.0
            return min(
                (
                    max(0.0, min(g.heartbeat_deadline, g.command_deadline) - self._clock())
                    for g in self._guards.values()
                ),
                default=None,
            )

    def _run(self) -> None:
        with self._condition:
            while not self._closed or self._guards:
                delay = self.poll()
                if self._closed and not self._guards:
                    break
                self._condition.wait(delay)

    def close(self) -> None:
        with self._condition:
            self._closed = True
            for guard in self._guards.values():
                guard.closing = guard.closing or "provider_closed"
            self._condition.notify()
        if self._thread is not None:
            self._thread.join()
        else:
            self.poll()
