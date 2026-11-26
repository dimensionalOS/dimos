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

"""Optional instrumentation; disabled by default and independent of robot modules."""

from __future__ import annotations

from asyncio import CancelledError
import atexit
from collections import Counter
from collections.abc import Callable, Iterator, Sequence
from contextlib import contextmanager
from functools import wraps
from itertools import count
import logging
from multiprocessing.util import Finalize
import os
from pathlib import Path
import socket
import threading
import time
from typing import Any, cast
from uuid import uuid4

from dimos.constants import STATE_DIR
from dimos.diagnostics.clock import clock_domain
from dimos.diagnostics.collector import Collector
from dimos.diagnostics.model import TraceContext, context_scope, current_context, wire_event
from dimos.diagnostics.store import validate_run_id


class Runtime:
    def __init__(
        self, run_id: str, directory: Path, mode: str = "trace", *, collector: str = "python"
    ) -> None:
        validate_run_id(run_id)
        self.pid = os.getpid()
        self.mode = mode
        self.span_ids = count(1)
        self.process = uuid4().hex
        self.run_id = run_id
        self.host = socket.gethostname()
        self.clock_domain = clock_domain()
        self.sequences: Counter[str] = Counter()
        self.lock = threading.Lock()
        self.collector = Collector(
            directory / self.process,
            {
                "schema": "dimos.incident/v1",
                "run_id": run_id,
                "process": self.process,
                "pid": self.pid,
                "host": self.host,
                "clock_domain": self.clock_domain,
                "wall_ns": time.time_ns(),
                "mono_ns": time.monotonic_ns(),
                "coverage": "instrumented boundaries only",
                "collector": collector,
            },
            mode=mode,
            storage_root=directory.parent,
        )
        self.bindings: set[tuple[str, str, str, str]] = set()

    def binding(self, transport: str, channel: str, direction: str, stream: Any = None) -> None:
        owner = getattr(stream, "owner", None)
        module = (
            (
                getattr(getattr(owner, "config", None), "instance_name", None)
                or getattr(type(owner), "name", None)
                or f"{type(owner).__name__}:{id(owner):x}"
            )
            if owner is not None
            else "direct"
        )
        endpoint = f"{module}/{getattr(stream, 'name', channel)}"
        key = (transport, channel, direction, endpoint)
        with self.lock:
            if key in self.bindings or len(self.bindings) >= 4096:
                return
            self.bindings.add(key)
        self.event(
            "topology.edge",
            transport=transport,
            channel=channel,
            direction=direction,
            endpoint=endpoint,
            module=module,
            host=self.host,
            observed=direction == "publish",
        )

    def sample(
        self, stream: str, parents: Sequence[TraceContext] | None = None, lineage: str = "complete"
    ) -> TraceContext:
        if parents is None:
            parent = current_context()
            parents = [parent] if parent else []
        with self.lock:
            if stream not in self.sequences and len(self.sequences) >= 4096:
                # A shared overflow sequence still preserves uniqueness; explicitly
                # lose stream attribution instead of growing without bound.
                stream = "diagnostics/stream-limit"
                lineage = "partial"
            self.sequences[stream] += 1
            seq = self.sequences[stream]
        if len(parents) > 32 or any(p.lineage != "complete" for p in parents):
            lineage = "partial"
        return TraceContext(
            self.run_id,
            self.process,
            stream,
            seq,
            time.monotonic_ns(),
            self.clock_domain,
            tuple(p.sample_id for p in parents[:32]),
            lineage,
        )

    def event(self, event: str, **fields: Any) -> None:
        self.collector.emit(
            wire_event(
                current_context() if self.mode == "trace" else None,
                event=event,
                mono_ns=time.monotonic_ns(),
                clock_domain=self.clock_domain,
                process=self.process,
                **fields,
            )
        )

    def watch(self, name: str, max_age_ms: float, *, active: bool = True) -> None:
        """Observe an expected live stream; deactivate it when the consumer stops."""
        if max_age_ms <= 0:
            raise ValueError("freshness limit must be positive")
        self.event("freshness.observe", watch=name, max_age_ns=int(max_age_ms * 1e6), active=active)

    def close(self) -> None:
        self.collector.close()


_runtime: Runtime | None = None
_mode = os.environ.get("DIMOS_DIAGNOSTICS", "off")
_init_lock = threading.Lock()
_failed_pid: int | None = None


def configure(mode: str, run_id: str, directory: str) -> None:
    """Apply resolved run configuration before workers/native children start."""
    global _mode, _runtime, _failed_pid
    if mode not in ("off", "metrics", "trace"):
        raise ValueError("invalid diagnostics mode")
    if mode != "off":
        validate_run_id(run_id)
    # Resolve on each worker host, then pass an absolute path to native children.
    # Rust PathBuf does not expand '~'; transmitting a resolved Mac home path to
    # a Linux worker would also put that worker's evidence in the wrong place.
    directory = str(Path(directory or STATE_DIR / "diagnostics").expanduser().resolve())
    changed = (
        _mode,
        os.environ.get("DIMOS_DIAGNOSTICS_RUN"),
        os.environ.get("DIMOS_DIAGNOSTICS_DIR"),
    ) != (mode, run_id, directory)
    if changed and _runtime is not None:
        if _runtime.pid == os.getpid():
            _runtime.close()
        _runtime = None
    _mode = mode
    _failed_pid = None
    os.environ.update(
        DIMOS_DIAGNOSTICS=mode, DIMOS_DIAGNOSTICS_RUN=run_id, DIMOS_DIAGNOSTICS_DIR=directory
    )


def get_runtime() -> Runtime | None:
    global _runtime, _failed_pid
    if _mode not in ("trace", "metrics"):
        return None
    if _failed_pid == os.getpid():
        return None
    if _runtime is None or _runtime.pid != os.getpid():
        with _init_lock:
            if _runtime is None or _runtime.pid != os.getpid():
                root = Path(
                    os.environ.get("DIMOS_DIAGNOSTICS_DIR") or STATE_DIR / "diagnostics"
                ).expanduser()
                run = os.environ.get("DIMOS_DIAGNOSTICS_RUN", "unassigned")
                try:
                    _runtime = Runtime(run, root / run, _mode)
                except (OSError, ValueError, TypeError, KeyError) as error:
                    _failed_pid = os.getpid()
                    logging.getLogger(__name__).error(
                        "Diagnostics unavailable: %s", type(error).__name__
                    )
                    return None
                atexit.register(_runtime.close)
                # multiprocessing workers exit through their finalizer registry,
                # not normal Python atexit callbacks.
                Finalize(_runtime, _runtime.close, exitpriority=10)
    return _runtime if not _runtime.collector.stop_event.is_set() else None


@contextmanager
def publication(
    stream: str, parents: Sequence[TraceContext] | None = None, *, lineage: str = "complete"
) -> Iterator[TraceContext | None]:
    runtime = get_runtime()
    if runtime is None or runtime.mode == "metrics":
        yield None
        return
    sample = runtime.sample(stream, parents, lineage)
    with context_scope(sample):
        runtime.event("sample.created")
        yield sample


@contextmanager
def span(stage: str, **fields: Any) -> Iterator[None]:
    runtime = get_runtime()
    if runtime is None:
        yield
        return
    started_ns = time.monotonic_ns()
    span_id = str(next(runtime.span_ids))
    runtime.event(stage + ".start", span_id=span_id, **fields)
    try:
        yield
    except BaseException as error:
        cancelled = isinstance(error, (CancelledError, KeyboardInterrupt, SystemExit))
        runtime.event(
            stage + ".end",
            span_id=span_id,
            outcome="cancelled" if cancelled else "error",
            reason=type(error).__name__,
        )
        if not cancelled:
            runtime.collector.trigger(stage + ":" + type(error).__name__)
        raise
    else:
        runtime.event(stage + ".end", span_id=span_id, outcome="ok")
        if time.monotonic_ns() - started_ns > 100_000_000:
            runtime.collector.trigger(stage + ":duration_over_100ms")


def measured(stage: str) -> Callable[[Callable[..., Any]], Callable[..., Any]]:
    def decorate(fn: Callable[..., Any]) -> Callable[..., Any]:
        @wraps(fn)
        def wrapped(*args: Any, **kwargs: Any) -> Any:
            if get_runtime() is None:
                return fn(*args, **kwargs)
            with span(stage):
                return fn(*args, **kwargs)

        return wrapped

    return decorate


def instrument_transport(cls: type) -> None:
    """Instrument each concrete transport implementation without altering its wire format."""
    broadcast = cls.__dict__.get("broadcast")
    if broadcast is not None:

        @wraps(broadcast)
        def send(self: Any, stream: Any, value: Any) -> Any:
            rt = get_runtime()
            if rt is None:
                return broadcast(self, stream, value)
            channel = str(
                getattr(
                    self, "channel", getattr(self, "topic", getattr(stream, "name", cls.__name__))
                )
            )
            rt.binding(cls.__name__, channel, "publish", stream)
            if current_context() is None:
                with (
                    publication(channel),
                    span("transport.publish", transport=cls.__name__, channel=channel),
                ):
                    return broadcast(self, stream, value)
            with span("transport.publish", transport=cls.__name__, channel=channel):
                return broadcast(self, stream, value)

        cast("Any", cls).broadcast = send
    subscribe = cls.__dict__.get("subscribe")
    if subscribe is not None:

        @wraps(subscribe)
        def receive(self: Any, callback: Callable[..., Any], stream: Any = None) -> Any:
            rt = get_runtime()
            if rt is None:
                return subscribe(self, callback, stream)
            channel = str(
                getattr(
                    self, "channel", getattr(self, "topic", getattr(stream, "name", cls.__name__))
                )
            )
            rt.binding(cls.__name__, channel, "subscribe", stream)

            def delivered(*args: Any, **kwargs: Any) -> Any:
                if get_runtime() is None:
                    return callback(*args, **kwargs)
                incoming = current_context()
                if incoming is None and rt.mode == "trace":
                    incoming = rt.sample(channel, parents=[], lineage="unknown")
                with context_scope(incoming):
                    rt.event("transport.received", transport=cls.__name__, channel=channel)
                    with span("callback", transport=cls.__name__, channel=channel):
                        return callback(*args, **kwargs)

            return subscribe(self, delivered, stream)

        cast("Any", cls).subscribe = receive
