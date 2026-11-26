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

"""Bounded browser ingestion into the same incident store as runtime collectors."""

from __future__ import annotations

import fcntl
import json
import os
from pathlib import Path
import time
from typing import Any
import zlib

from dimos.diagnostics.session import SessionStore
from dimos.diagnostics.store import DiagnosticsStore, validate_run_id

WIRE_LIMIT = 512 * 1024
RAW_LIMIT = 1024 * 1024


def decode_batch(body: bytes, encoding: str | None) -> dict[str, Any]:
    if len(body) > WIRE_LIMIT:
        raise ValueError("batch too large")
    if encoding == "gzip":
        decoder = zlib.decompressobj(16 + zlib.MAX_WBITS)
        body = decoder.decompress(body, RAW_LIMIT + 1)
        if len(body) > RAW_LIMIT or not decoder.eof or decoder.unused_data:
            raise ValueError("invalid or oversized gzip batch")
    elif encoding:
        raise ValueError("unsupported content encoding")
    value: dict[str, Any] = json.loads(body)
    manifest, events = value["manifest"], value["events"]
    if not manifest["process"].startswith("browser-"):
        raise ValueError("browser process namespace required")
    for field in (manifest["run_id"], manifest["process"], value["batchId"]):
        validate_run_id(field)
    if (
        manifest.get("schema") != "dimos.incident/v1"
        or not isinstance(events, list)
        or len(events) > 512
        or not isinstance(manifest.get("clock_domain"), str)
        or len(manifest["clock_domain"]) > 512
    ):
        raise ValueError("invalid batch")
    for event in events:
        if (
            not isinstance(event, dict)
            or event.get("schema") != "dimos.trace/v1"
            or event.get("process") != manifest["process"]
            or event.get("clock_domain") != manifest["clock_domain"]
            or type(event.get("mono_ns")) is not int
            or event["mono_ns"] < 0
            or not isinstance(event.get("event"), str)
            or len(event["event"]) > 256
            or (event.get("context") and event["context"].get("run_id") != manifest["run_id"])
        ):
            raise ValueError("invalid event")
    return value


def ingest(
    root: Path, batch: dict[str, Any], budget: int = 1024**3, max_files: int = 4096
) -> dict[str, Any]:
    store = DiagnosticsStore(root)
    manifest, events, batch_id = batch["manifest"], batch["events"], batch["batchId"]
    root = store.root
    root.mkdir(parents=True, exist_ok=True)
    # Same locks as session control and all host collectors; no app-owned retention policy.
    with (
        (root / ".session.lock").open("a") as session_lock,
        (root / ".retention.lock").open("a") as lock,
    ):
        fcntl.flock(session_lock, fcntl.LOCK_EX)
        active = SessionStore(root).active()
        if active is None or active["run_id"] != manifest["run_id"] or active["mode"] != "trace":
            raise ValueError("browser batch does not belong to the active trace session")
        fcntl.flock(lock, fcntl.LOCK_EX)
        directory = store.run(manifest["run_id"]) / manifest["process"]
        if directory.is_symlink():
            raise ValueError("symlink capture directory refused")
        directory.mkdir(parents=True, exist_ok=True)
        manifest = {**manifest, "kind": "manifest", "collector": "browser"}
        body = (
            "\n".join(
                json.dumps(row, separators=(",", ":"), allow_nan=False)
                for row in [manifest, *events]
            )
            + "\n"
        ).encode()
        destination = directory / f"incident-{batch_id}.jsonl"
        if destination.exists() or destination.is_symlink():
            if destination.is_symlink() or destination.read_bytes() != body:
                raise ValueError("batch identity reused with different content")
            return {"stored": True, "batchId": batch_id}
        if len(body) > budget:
            raise ValueError("batch exceeds storage budget")
        files = sorted(
            (
                p
                for p in root.glob("*/*/incident-*.jsonl")
                if not p.is_symlink() and p.resolve().is_relative_to(root)
            ),
            key=lambda p: p.stat().st_mtime_ns,
        )
        total = sum(p.stat().st_size for p in files)
        evicted = 0
        for old in files:
            if total + len(body) <= budget and len(files) - evicted < max_files:
                break
            total -= old.stat().st_size
            old.unlink()
            evicted += 1
        temp = destination.with_suffix(".tmp")
        if temp.is_symlink():
            raise ValueError("invalid capture destination")
        with temp.open("wb") as output:
            output.write(body)
            output.flush()
            os.fsync(output.fileno())
        temp.replace(destination)
        now = time.time_ns()
        # Readiness uses server receipt times, never subtracts a browser wall clock.
        previous = directory / "status.json"
        last_events = (
            json.loads(previous.read_text()).get("last_events", {}) if previous.exists() else {}
        )
        for event in events:
            if event["event"] in last_events or len(last_events) < 512:
                last_events[event["event"]] = now
        status = {
            **manifest,
            "at_ns": now,
            "state": "collecting",
            "observer_drops": manifest.get("health", {}).get("observer_drops", 0),
            "last_events": last_events,
            "storage_evictions_this_batch": evicted,
        }
        temp_status = directory / "status.tmp"
        temp_status.write_text(json.dumps(status))
        temp_status.replace(directory / "status.json")
        return {"stored": True, "batchId": batch_id}
