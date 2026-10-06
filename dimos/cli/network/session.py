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

from collections.abc import Callable
import importlib.metadata
import json
import struct
import threading
import time
from typing import Any

from packaging.version import Version
import zenoh

from dimos.cli.network.model import RTT, Settings

HEADER = struct.Struct("!IQ")


def zenoh_version() -> str:
    version = importlib.metadata.version("eclipse-zenoh")
    if not Version("1.10.1") <= Version(version) < Version("2"):
        raise ValueError(f"Zenoh {version} unsupported; require >=1.10.1,<2")
    return version


class Receiver:
    """Count exact unique messages, including deadline accounting and arrival gaps."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.reset(0, 1)

    def reset(self, phase: int, duration: float) -> None:
        with self._lock:
            self._phase = phase
            self._duration = duration
            self._seen: set[int] = set()
            self._duplicates = 0
            self._out_of_order = 0
            self._highest = -1
            self._start: float | None = None
            self._previous: float | None = None
            self._gaps: list[float] = []
            self._window_bytes = 0
            self._window_count = 0

    def accept(self, data: bytes, now: float) -> None:
        if len(data) < HEADER.size:
            return
        phase, sequence = HEADER.unpack_from(data)
        with self._lock:
            if phase != self._phase:
                return
            if sequence in self._seen:
                self._duplicates += 1
                return
            self._seen.add(sequence)
            if sequence < self._highest:
                self._out_of_order += 1
            self._highest = max(sequence, self._highest)
            if self._start is None:
                self._start = now
            if now - self._start <= self._duration:
                self._window_bytes += len(data)
                self._window_count += 1
            if self._previous is not None:
                self._gaps.append((now - self._previous) * 1000)
            self._previous = now

    def summary(self, sent: int) -> dict[str, Any]:
        with self._lock:
            received = len(self._seen)
            missing = max(0, sent - received)
            mean = sum(self._gaps) / len(self._gaps) if self._gaps else None
            jitter = (
                (sum((gap - mean) ** 2 for gap in self._gaps) / len(self._gaps)) ** 0.5
                if mean is not None
                else None
            )
            return {
                "unique_received": received,
                "missing": missing,
                "missing_pct": missing * 100 / sent if sent else 0.0,
                "duplicates": self._duplicates,
                "out_of_order": self._out_of_order,
                "window_bytes": self._window_bytes,
                "window_messages": self._window_count,
                "goodput_mbps": self._window_bytes * 8 / self._duration / 1e6,
                "window_seconds": self._duration,
                "mean_gap_ms": mean,
                "gap_stddev_ms": jitter,
                "max_gap_ms": max(self._gaps, default=None),
            }


class Endpoint:
    """An isolated direct TCP Zenoh session; never scouts or joins robot topics."""

    def __init__(self, token: str, role: str, endpoint: str, stop: threading.Event) -> None:
        config = zenoh.Config()
        for key, value in {
            "mode": "peer",
            "scouting/multicast/enabled": False,
            "scouting/gossip/enabled": False,
            "listen/endpoints": [endpoint] if role == "remote" else [],
            "connect/endpoints": [endpoint] if role == "local" else [],
            "connect/timeout_ms": 5000,
        }.items():
            config.insert_json5(key, json.dumps(value))
        self._stop = stop
        self._session = zenoh.open(config)
        self.receiver = Receiver()
        prefix = f"dimos/network-check/{token}"
        other = "remote" if role == "local" else "local"
        self._data = self._session.declare_publisher(
            f"{prefix}/data/{role}",
            reliability=zenoh.Reliability.RELIABLE,
            congestion_control=zenoh.CongestionControl.DROP,
        )
        self._ping = self._session.declare_publisher(f"{prefix}/ping", express=True)
        self._pong = self._session.declare_publisher(f"{prefix}/pong", express=True)
        self._reply = threading.Condition()
        self._pending: dict[int, float] = {}
        self._next_probe = 0
        self._subs = [self._session.declare_subscriber(f"{prefix}/data/{other}", self._receive)]
        self._subs.append(
            self._session.declare_subscriber(
                f"{prefix}/{'ping' if role == 'remote' else 'pong'}",
                self._echo if role == "remote" else self._receive_pong,
            )
        )

    def _receive(self, sample: zenoh.Sample) -> None:
        self.receiver.accept(sample.payload.to_bytes(), time.monotonic())

    def _echo(self, sample: zenoh.Sample) -> None:
        if not self._stop.is_set():
            self._pong.put(sample.payload)

    def _receive_pong(self, sample: zenoh.Sample) -> None:
        data = sample.payload.to_bytes()
        if len(data) != 8:
            return
        seq = struct.unpack("!Q", data)[0]
        with self._reply:
            if seq in self._pending:
                self._pending[seq] = time.perf_counter()
                self._reply.notify_all()

    def probe(self, timeout: float) -> float | None:
        seq = self._next_probe
        self._next_probe += 1
        start = time.perf_counter()
        with self._reply:
            self._pending[seq] = 0.0
        self._ping.put(struct.pack("!Q", seq))
        with self._reply:
            self._reply.wait_for(lambda: self._pending[seq] != 0 or self._stop.is_set(), timeout)
            finished = self._pending.pop(seq)
        return (finished - start) * 1000 if finished and finished - start <= timeout else None

    def probes(
        self,
        duration: float,
        settings: Settings,
        on_tick: Callable[[RTT, float], None] | None = None,
    ) -> RTT:
        result = RTT()
        end = time.monotonic() + duration
        next_at = time.monotonic()
        start = next_at
        last_tick = start
        while time.monotonic() < end and not self._stop.is_set():
            value = self.probe(min(settings.probe_timeout, max(0.001, end - time.monotonic())))
            if value is None:
                result.timeouts += 1
            else:
                result.samples_ms.append(value)
            if on_tick is not None and time.monotonic() - last_tick >= 0.25:
                on_tick(result, time.monotonic() - start)
                last_tick = time.monotonic()
            next_at += 1 / settings.probe_hz
            # No catch-up bursts after a timeout.
            next_at = max(next_at, time.monotonic())
            self._stop.wait(max(0, min(end, next_at) - time.monotonic()))
        return result

    def send(
        self, phase: int, rate: float, duration: float, settings: Settings, budget: int
    ) -> dict[str, Any]:
        interval = settings.payload_bytes * 8 / (rate * 1e6)
        body = bytes(settings.payload_bytes - HEADER.size)
        start = time.monotonic()
        cpu = time.process_time()
        count = 0
        next_at = start + interval
        while time.monotonic() - start < duration and not self._stop.is_set():
            if (count + 1) * settings.payload_bytes > budget:
                break
            self._stop.wait(max(0, min(start + duration, next_at) - time.monotonic()))
            if self._stop.is_set() or time.monotonic() >= start + duration:
                break
            self._data.put(HEADER.pack(phase, count) + body)
            count += 1
            # Keep actual offered rate bounded even if scheduling falls behind.
            next_at = max(next_at + interval, time.monotonic() + interval)
        elapsed = time.monotonic() - start
        return {
            "messages": count,
            "bytes": count * settings.payload_bytes,
            "elapsed_seconds": elapsed,
            "cpu_seconds": time.process_time() - cpu,
            "offered_mbps": count * settings.payload_bytes * 8 / max(elapsed, 0.001) / 1e6,
            "budget_exhausted": (count + 1) * settings.payload_bytes > budget,
        }

    def close(self) -> None:
        self._session.close()


def start_sender(
    target: Callable[[], dict[str, Any]],
) -> tuple[threading.Thread, list[dict[str, Any]], list[Exception]]:
    result: list[dict[str, Any]] = []
    errors: list[Exception] = []

    def run() -> None:
        try:
            result.append(target())
        except (OSError, RuntimeError, zenoh.ZError) as error:
            errors.append(error)

    thread = threading.Thread(target=run, name="network-check-sender")
    thread.start()
    return thread, result, errors
