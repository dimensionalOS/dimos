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

"""Keep immutable provenance beside data across asynchronous latest-value mailboxes."""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass
import time
from typing import Any

from dimos.diagnostics.model import TraceContext, context_scope, current_context
from dimos.diagnostics.runtime import get_runtime, span


@dataclass(frozen=True)
class QueuedSample:
    value: Any
    context: TraceContext | None
    enqueued_ns: int


def capture(value: Any) -> Any:
    if get_runtime() is None:
        return value
    return QueuedSample(value, current_context(), time.monotonic_ns())


def discard(value: Any, queue: str, reason: str = "superseded") -> None:
    runtime = get_runtime()
    if isinstance(value, QueuedSample) and runtime:
        with context_scope(value.context):
            runtime.event("queue.discard", queue=queue, reason=reason)


@contextmanager
def delivery(value: Any, queue: str) -> Iterator[Any]:
    if not isinstance(value, QueuedSample):
        yield value
        return
    with context_scope(value.context):
        runtime = get_runtime()
        if runtime:
            runtime.event(
                "queue.delay", queue=queue, duration_ns=time.monotonic_ns() - value.enqueued_ns
            )
        with span("handler", queue=queue):
            yield value.value
