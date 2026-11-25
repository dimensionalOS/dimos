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

"""Versioned identities and clock bounds; no robot or transport dependencies."""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager
from contextvars import ContextVar
from dataclasses import asdict, dataclass
from functools import cached_property
import hashlib
import json
import math
from typing import Any

SCHEMA = "dimos.trace/v1"
MAX_CONTEXT_BYTES = 16384


@dataclass(frozen=True)
class TraceContext:
    run_id: str
    publisher: str
    stream: str
    sequence: int
    origin_ns: int
    clock_domain: str
    parents: tuple[str, ...] = ()
    lineage: str = "complete"

    @cached_property
    def sample_id(self) -> str:
        identity = json.dumps(
            [self.run_id, self.publisher, self.stream, self.sequence], separators=(",", ":")
        )
        return hashlib.sha256(identity.encode()).hexdigest()

    def record(self) -> dict[str, Any]:
        # These fields are immutable; avoid dataclasses.asdict's recursive deepcopy
        # on every hot-path event. JSON encoding remains on the observer worker.
        return {
            "run_id": self.run_id,
            "publisher": self.publisher,
            "stream": self.stream,
            "sequence": self.sequence,
            "origin_ns": self.origin_ns,
            "clock_domain": self.clock_domain,
            "parents": self.parents,
            "lineage": self.lineage,
        }

    def encode(self) -> bytes:
        data = json.dumps({"schema": SCHEMA, **asdict(self)}, separators=(",", ":")).encode()
        if len(data) > MAX_CONTEXT_BYTES:
            raise ValueError("trace context exceeds limit")
        return data

    @classmethod
    def decode(cls, data: bytes) -> TraceContext:
        if len(data) > MAX_CONTEXT_BYTES:
            raise ValueError("trace context exceeds limit")
        value = json.loads(data)
        if not isinstance(value, dict) or value.pop("schema", None) != SCHEMA:
            raise ValueError("unsupported trace context")
        for name in ("run_id", "publisher", "stream", "clock_domain"):
            if not isinstance(value.get(name), str) or not value[name]:
                raise ValueError(f"invalid {name}")
        for name in ("sequence", "origin_ns"):
            if type(value.get(name)) is not int or value[name] < 0:
                raise ValueError(f"invalid {name}")
        parents = value.get("parents", [])
        if (
            not isinstance(parents, list)
            or len(parents) > 32
            or not all(isinstance(p, str) for p in parents)
        ):
            raise ValueError("invalid parents")
        if value.get("lineage", "complete") not in ("complete", "partial", "unknown"):
            raise ValueError("invalid lineage")
        value["parents"] = tuple(parents)
        try:
            return cls(**value)
        except TypeError as error:
            raise ValueError("invalid trace fields") from error


_current: ContextVar[TraceContext | None] = ContextVar("dimos_trace", default=None)


def current_context() -> TraceContext | None:
    """Retain this immutable context explicitly when caching an input for later work."""
    return _current.get()


@contextmanager
def context_scope(context: TraceContext | None) -> Iterator[None]:
    token = _current.set(context)
    try:
        yield
    finally:
        _current.reset(token)


@dataclass(frozen=True)
class ClockMapping:
    """Remote minus local clock bounds from a four-timestamp exchange (nanoseconds)."""

    local_domain: str
    remote_domain: str
    lower_ns: int
    upper_ns: int
    measured_ns: int
    valid_for_ns: int
    drift_ppm: float = 100.0

    @classmethod
    def exchange(
        cls,
        local: str,
        remote: str,
        a0: int,
        b1: int,
        b2: int,
        a3: int,
        valid_for_ns: int,
        error_ns: int = 0,
        drift_ppm: float = 100.0,
    ) -> ClockMapping:
        if a3 < a0 or b2 < b1 or a3 - a0 < b2 - b1 or error_ns < 0 or valid_for_ns <= 0:
            raise ValueError("invalid clock exchange")
        if not math.isfinite(drift_ppm) or drift_ppm < 0:
            raise ValueError("invalid drift bound")
        return cls(
            local, remote, b2 - a3 - error_ns, b1 - a0 + error_ns, a3, valid_for_ns, drift_ppm
        )

    def latency_bounds(self, sent_ns: int, received_ns: int, now_ns: int) -> tuple[int, int] | None:
        age = now_ns - self.measured_ns
        if age < 0 or age > self.valid_for_ns:
            return None
        drift = math.ceil(age * self.drift_ppm / 1_000_000)
        low = received_ns - sent_ns - self.upper_ns - drift
        high = received_ns - sent_ns - self.lower_ns + drift
        return None if high < 0 else (max(0, low), high)


def wire_event(context: TraceContext | None, **fields: Any) -> dict[str, Any]:
    return {"schema": SCHEMA, "context": context.record() if context else None, **fields}
