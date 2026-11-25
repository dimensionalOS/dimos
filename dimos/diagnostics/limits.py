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

"""Shared host budget reservations, also consumed by native collectors."""

from __future__ import annotations

import fcntl
import json
import os
from pathlib import Path
from typing import Any
from uuid import uuid4

HOST_BYTES = 256 * 1024 * 1024
PROCESS_BYTES = 64 * 1024 * 1024


class MemoryReservation:
    def __init__(self, root: Path, requested: int = PROCESS_BYTES) -> None:
        self.root, self.key, self.bytes = root, uuid4().hex, 0
        root.mkdir(parents=True, exist_ok=True)
        with (root / ".memory.lock").open("a") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            entries = self._read()
            active = {}
            for key, value in entries.items():
                try:
                    os.kill(value["pid"], 0)
                    active[key] = value
                except ProcessLookupError:
                    pass
                except PermissionError:
                    active[key] = value
            available = max(0, HOST_BYTES - sum(v["bytes"] for v in active.values()))
            process_available = max(
                0,
                PROCESS_BYTES - sum(v["bytes"] for v in active.values() if v["pid"] == os.getpid()),
            )
            self.bytes = min(requested, process_available, available)
            active[self.key] = {"pid": os.getpid(), "bytes": self.bytes}
            self._write(active)

    def _read(self) -> dict[str, Any]:
        path = self.root / ".memory.json"
        try:
            if path.stat().st_size > 1024 * 1024:
                raise ValueError("invalid diagnostics reservation file")
            entries = json.loads(path.read_text())
            if not isinstance(entries, dict) or any(
                not isinstance(value, dict)
                or type(value.get("pid")) is not int
                or value["pid"] <= 0
                or type(value.get("bytes")) is not int
                or not 0 <= value["bytes"] <= HOST_BYTES
                for value in entries.values()
            ):
                raise ValueError("invalid diagnostics reservations")
            return entries
        except FileNotFoundError:
            return {}

    def _write(self, entries: dict[str, Any]) -> None:
        path = self.root / ".memory.json"
        temp = path.with_suffix(".tmp")
        temp.write_text(json.dumps(entries))
        temp.replace(path)

    def close(self) -> None:
        with (self.root / ".memory.lock").open("a") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            entries = self._read()
            entries.pop(self.key, None)
            self._write(entries)
