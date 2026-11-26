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

"""Bounded, process-local flight recorder. IO and JSON encoding stay on its worker."""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
import fcntl
import json
import math
from pathlib import Path
from queue import Empty, Full, Queue
import threading
import time
from typing import Any, cast
from uuid import uuid4

from dimos.diagnostics.host import process_snapshot
from dimos.diagnostics.limits import MemoryReservation
from dimos.diagnostics.ring import TraceRing


def bounded_metadata(event: dict[str, Any]) -> dict[str, Any]:
    """Copy only small JSON metadata; never retain application objects/payload buffers."""
    remaining, nodes = 4096, 128

    def copy(value: Any, depth: int = 0) -> Any:
        nonlocal remaining, nodes
        nodes -= 1
        if nodes < 0 or depth > 8:
            raise ValueError("metadata structure exceeds limit")
        if value is None or type(value) is bool:
            return value
        if type(value) is int and value.bit_length() <= 128:
            return value
        if type(value) is float and math.isfinite(value):
            return value
        if isinstance(value, str):
            remaining -= len(value)
            if remaining < 0:
                raise ValueError("metadata text exceeds limit")
            return value
        if isinstance(value, (list, tuple)) and len(value) <= 64:
            return [copy(item, depth + 1) for item in value]
        if isinstance(value, dict) and len(value) <= 64 and all(isinstance(k, str) for k in value):
            return {copy(k, depth + 1): copy(v, depth + 1) for k, v in value.items()}
        raise ValueError("metadata must contain bounded JSON values")

    return cast("dict[str, Any]", copy(event))


@dataclass(frozen=True)
class CaptureLimits:
    ring_bytes: int = 64 * 1024 * 1024
    storage_bytes: int = 1024 * 1024 * 1024
    queue_events: int = 128
    pre_seconds: float = 60.0
    post_seconds: float = 30.0

    def __post_init__(self) -> None:
        if (
            min(self.ring_bytes, self.storage_bytes, self.queue_events) <= 0
            or min(self.pre_seconds, self.post_seconds) < 0
        ):
            raise ValueError("capture limits must be positive; windows nonnegative")


class Collector:
    def __init__(
        self,
        directory: Path,
        manifest: dict[str, Any],
        limits: CaptureLimits = CaptureLimits(),
        mode: str = "trace",
        storage_root: Path | None = None,
    ) -> None:
        self.directory, self.manifest, self.limits, self.mode = directory, manifest, limits, mode
        self.storage_root = storage_root or directory
        # Leave room for native collectors when a blueprint uses one Python worker per module.
        self.reservation = MemoryReservation(self.storage_root, requested=8 * 1024 * 1024)
        if self.reservation.bytes < 4 * 1024 * 1024:
            self.reservation.close()
            raise ValueError("diagnostics host/process memory budget exhausted")
        # Half the reservation covers bounded ingress, caches, histograms and bookkeeping.
        self.ring_limit = min(limits.ring_bytes, self.reservation.bytes // 2)
        self.queue: Queue[dict[str, Any]] = Queue(min(limits.queue_events, 128))
        self.dropped = 0
        self.evicted = 0
        self.errors = 0
        self.counts: Counter[str] = Counter()
        self.last_events: dict[str, int] = {}
        self.active_spans: dict[str, tuple[str, int, str]] = {}
        self.durations: dict[str, dict[str, Any]] = {}
        self.ring = TraceRing(self.ring_limit)
        self.started = time.monotonic()
        self.topology: dict[str, dict[str, Any]] = {}
        self.pending: tuple[float, str] | None = None
        self.watches: dict[str, dict[str, Any]] = {}
        self.last_watch_check = 0.0
        self.last_checkpoint = time.monotonic()
        self.stop_event = threading.Event()
        self.finished = False
        self.host_metrics: dict[str, Any] = {}
        self.thread = threading.Thread(target=self._run, name="dimos-diagnostics", daemon=True)
        self.thread.start()

    def emit(self, event: dict[str, Any]) -> None:
        if self.stop_event.is_set():
            return
        try:
            self.queue.put_nowait(bounded_metadata(event))
        except (Full, ValueError):
            self.dropped += 1

    def trigger(self, reason: str) -> None:
        self.emit(
            {"event": "incident.trigger", "reason": reason[:512], "mono_ns": time.monotonic_ns()}
        )

    def close(self) -> None:
        self.stop_event.set()
        self.thread.join(timeout=5)
        if not self.thread.is_alive():
            self.reservation.close()

    def _health(self) -> dict[str, Any]:
        now = time.monotonic()
        requested_start = (
            self.pending[0] - self.limits.post_seconds - self.limits.pre_seconds
            if self.pending
            else now - self.limits.pre_seconds
        )
        covered_start = max(self.started, requested_start, self.ring.capacity_lost_until)
        return {
            "observer_drops": self.dropped,
            "ring_evictions": self.ring.evictions,
            "io_errors": self.errors,
            "counts": dict(self.counts),
            "last_events": dict(self.last_events),
            "stages": self.durations,
            "mode": self.mode,
            "ring_bytes": self.ring.size,
            "raw_ring_bytes": self.ring.raw_bytes,
            "requested_pre_seconds": self.limits.pre_seconds,
            "requested_post_seconds": self.limits.post_seconds,
            "retained_seconds": max(0, now - covered_start),
            "capture_window_complete": self.mode == "trace"
            and covered_start <= requested_start
            and (self.pending is None or now >= self.pending[0]),
            "capacity_truncated": self.ring.capacity_lost_until > requested_start,
            "memory_reserved_bytes": self.reservation.bytes,
            "ring_limit_bytes": self.ring_limit,
            "coverage_limited_by_host_budget": self.ring_limit == 0,
            "topology": list(self.topology.values()),
            "at_ns": time.time_ns(),
            "state": "stopped"
            if self.finished
            else "stopping"
            if self.stop_event.is_set()
            else "collecting",
            "process_metrics": self.host_metrics,
        }

    def _measure(self, event: dict[str, Any]) -> None:
        name = event.get("event", "")
        if name in self.last_events or len(self.last_events) < 512:
            self.last_events[name] = time.time_ns()
        identity = event.get("span_id")
        if not identity or not isinstance(event.get("mono_ns"), int):
            return
        if name.endswith(".start"):
            if len(self.active_spans) >= 4096:
                self.active_spans.pop(next(iter(self.active_spans)))
                self.dropped += 1
            self.active_spans[identity] = (
                name[:-6],
                event["mono_ns"],
                event.get("clock_domain", ""),
            )
        elif name.endswith(".end"):
            start = self.active_spans.pop(identity, None)
            if start is None or start[2] != event.get("clock_domain", ""):
                return
            duration = event["mono_ns"] - start[1]
            if duration < 0 or (start[0] not in self.durations and len(self.durations) >= 512):
                return
            metrics = self.durations.setdefault(
                start[0], {"n": 0, "sum_ns": 0, "max_ns": 0, "buckets_log2_ns": [0] * 64}
            )
            metrics["n"] += 1
            metrics["sum_ns"] += duration
            metrics["max_ns"] = max(metrics["max_ns"], duration)
            metrics["buckets_log2_ns"][min(duration.bit_length(), 63)] += 1

    def _save(self, reason: str) -> None:
        self.directory.mkdir(parents=True, exist_ok=True)
        path = self.directory / (
            "incident-checkpoint.jsonl"
            if reason == "checkpoint"
            else f"incident-{time.time_ns()}-{uuid4().hex[:8]}.jsonl"
        )
        header = (
            json.dumps(
                {
                    "kind": "manifest",
                    **self.manifest,
                    "captured_ns": time.monotonic_ns(),
                    "reason": reason,
                    "health": self._health(),
                }
            ).encode()
            + b"\n"
        )
        incoming_bytes = len(header) + self.ring.raw_bytes
        if incoming_bytes > self.limits.storage_bytes:
            self.errors += 1
            return
        # All Python collectors on this host use one retention budget. The lock
        # is taken only by the diagnostics worker, never the control thread.
        self.storage_root.mkdir(parents=True, exist_ok=True)
        with (self.storage_root / ".retention.lock").open("a") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            files = sorted(
                self.storage_root.rglob("incident-*.jsonl"), key=lambda p: p.stat().st_mtime_ns
            )
            total = sum(p.stat().st_size for p in files)
            for old in files:
                if total + incoming_bytes <= self.limits.storage_bytes:
                    break
                total -= old.stat().st_size
                old.unlink()
            temporary = path.with_suffix(".tmp")
            with temporary.open("wb") as file:
                file.write(header)
                for row in self.ring.rows():
                    file.write(row)
            temporary.replace(path)

    def _check_freshness(self, at_ns: int, now: float) -> None:
        for name, observed in self.watches.items():
            age = max(0, at_ns - observed["mono_ns"])
            threshold = observed.get("max_age_ns", 250_000_000)
            event = {
                **observed,
                "event": "freshness.age",
                "mono_ns": at_ns,
                "age_ns": age,
                "expired": age > threshold,
            }
            if self.mode == "trace" and self.ring_limit:
                row = json.dumps(event, separators=(",", ":")).encode() + b"\n"
                self.ring.append(now, row)
            if age > threshold and self.pending is None:
                self.pending = (now + self.limits.post_seconds, f"freshness:{name}")

    def _run(self) -> None:
        last_status = 0.0
        while not self.stop_event.is_set() or not self.queue.empty():
            try:
                event = self.queue.get(timeout=0.1)
            except Empty:
                event = None
            now = time.monotonic()
            try:
                if event is not None:
                    name = str(event.get("event", "unknown"))[:128]
                    if name not in self.counts and len(self.counts) >= 1024:
                        name = "other"
                    self.counts[name] += 1
                    self._measure(event)
                    if name in (
                        "topology.edge",
                        "topology.declared",
                        "deployment.build",
                        "network.route",
                    ):
                        # Keep resolved declarations even if their raw ring window expires.
                        key = json.dumps(
                            [
                                event.get(k)
                                for k in (
                                    "event",
                                    "module",
                                    "transport",
                                    "channel",
                                    "direction",
                                    "endpoint",
                                )
                            ]
                        )
                        if key in self.topology or len(self.topology) < 128:
                            self.topology[key] = event
                    if name == "freshness.observe":
                        watch = str(event.get("watch", ""))[:256]
                        if event.get("active", True) and (
                            watch in self.watches or len(self.watches) < 64
                        ):
                            self.watches[watch] = event
                        else:
                            self.watches.pop(watch, None)
                    if name == "incident.trigger" and self.pending is None:
                        self.pending = (
                            now + self.limits.post_seconds,
                            str(event.get("reason", "manual")),
                        )
                    if self.mode == "trace" and self.ring_limit:
                        row = (
                            json.dumps(event, separators=(",", ":"), allow_nan=False).encode()
                            + b"\n"
                        )
                        if len(row) <= 65536:
                            self.ring.append(now, row)
                        else:
                            self.dropped += 1
                if now - self.last_watch_check >= 0.1:
                    self.last_watch_check = now
                    self._check_freshness(time.monotonic_ns(), now)
                cutoff = now - self.limits.pre_seconds
                if self.pending:
                    cutoff = self.pending[0] - self.limits.post_seconds - self.limits.pre_seconds
                self.ring.trim(cutoff)
                if self.mode == "trace" and now - self.last_checkpoint >= 5:
                    self._save("checkpoint")
                    self.last_checkpoint = now
                if self.pending and now >= self.pending[0]:
                    self._save(self.pending[1])
                    self.pending = None
                if now - last_status >= 1:
                    self.host_metrics = process_snapshot()
                    # Keep the resource history alongside the incident; status.json
                    # alone only retains the last sample before a disconnection.
                    self.emit(
                        {
                            "schema": "dimos.trace/v1",
                            "event": "host.process",
                            "process": self.manifest.get("process"),
                            "clock_domain": self.manifest.get("clock_domain"),
                            "mono_ns": time.monotonic_ns(),
                            "metrics": self.host_metrics,
                        }
                    )
                    self.directory.mkdir(parents=True, exist_ok=True)
                    control = self.directory / "control.json"
                    if control.exists():
                        request = json.loads(control.read_text())
                        control.unlink()
                        if request.get("action") == "mark":
                            self.trigger(str(request.get("reason", "manual")))
                        elif request.get("action") == "stop":
                            self.stop_event.set()
                        elif request.get("action") == "flush":
                            self._save("flush")
                    tmp = self.directory / "status.tmp"
                    tmp.write_text(json.dumps({**self.manifest, **self._health()}))
                    tmp.replace(self.directory / "status.json")
                    last_status = now
            except (OSError, ValueError, TypeError):
                self.errors += 1
        try:
            self._save(self.pending[1] if self.pending else "shutdown")
            self.ring.clear()
            self.finished = True
            tmp = self.directory / "status.tmp"
            tmp.write_text(json.dumps({**self.manifest, **self._health()}))
            tmp.replace(self.directory / "status.json")
        except OSError:
            self.errors += 1
