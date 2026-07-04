#!/usr/bin/env python3
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

from __future__ import annotations

from dataclasses import dataclass
from datetime import UTC, datetime
import json
import os
from pathlib import Path
import re
import subprocess
from typing import Any

from dimos.core.coordination.process_lifecycle import DIMOS_RUN_ID_ENV

EVOLUTION_EVENT_SCHEMA = "dimos.evolution_event.v1"
EVOLUTION_LEDGER_DIR_ENV = "DIMOS_EVOLUTION_LEDGER_DIR"


@dataclass(frozen=True)
class EvolutionEvent:
    """One low-volume, reviewable event in the local evolution ledger."""

    timestamp: float
    event_type: str
    task: str
    run_id: str
    payload: dict[str, Any]
    commit_requested: bool
    commit_sha: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema": EVOLUTION_EVENT_SCHEMA,
            "timestamp": round(self.timestamp, 3),
            "event_type": self.event_type,
            "task": self.task,
            "run_id": self.run_id,
            "payload": self.payload,
            "git": {
                "commit_requested": self.commit_requested,
                "commit_sha": self.commit_sha,
            },
        }


def build_evolution_event(
    event_type: str,
    task: str,
    payload: dict[str, Any],
    commit_requested: bool,
    timestamp: float | None = None,
    run_id: str | None = None,
) -> EvolutionEvent:
    """Validate and build an evolution event without writing it."""
    event_type = event_type.strip()
    if not event_type:
        raise ValueError("event_type is required")
    if not isinstance(payload, dict):
        raise ValueError("payload must be a JSON object")

    return EvolutionEvent(
        timestamp=timestamp if timestamp is not None else datetime.now(UTC).timestamp(),
        event_type=event_type,
        task=task.strip(),
        run_id=(run_id if run_id is not None else os.environ.get(DIMOS_RUN_ID_ENV, "")).strip(),
        payload=payload,
        commit_requested=commit_requested,
    )


def resolve_evolution_ledger_dir() -> Path:
    """Return the configured ledger directory, defaulting to repo-root `.dimos/evolution`."""
    configured = os.environ.get(EVOLUTION_LEDGER_DIR_ENV, "").strip()
    if configured:
        return Path(configured).expanduser().resolve()
    return _current_git_root() / ".dimos" / "evolution"


def write_evolution_event_file(
    ledger_dir: Path,
    event: EvolutionEvent,
) -> Path:
    """Write an event JSON file atomically and return its path."""
    event_day = datetime.fromtimestamp(event.timestamp, UTC)
    event_dir = (
        ledger_dir
        / "events"
        / f"{event_day.year:04d}"
        / f"{event_day.month:02d}"
        / (f"{event_day.day:02d}")
    )
    event_dir.mkdir(parents=True, exist_ok=True)
    (ledger_dir / "proposals").mkdir(parents=True, exist_ok=True)
    _ensure_readme(ledger_dir)

    path = _unique_event_path(event_dir, event_day, event.event_type)
    payload = json.dumps(event.to_dict(), indent=2, sort_keys=True) + "\n"
    tmp_path = path.with_name(f".{path.name}.tmp-{os.getpid()}")
    tmp_path.write_text(payload, encoding="utf-8")
    tmp_path.replace(path)
    return path


def parse_payload_json(payload_json: str) -> dict[str, Any]:
    """Parse the primitive MCP JSON argument into a JSON object payload."""
    try:
        payload = json.loads(payload_json or "{}")
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid payload_json: {exc.msg}") from exc
    if not isinstance(payload, dict):
        raise ValueError("payload_json must decode to a JSON object")
    return payload


def _ensure_readme(ledger_dir: Path) -> None:
    readme = ledger_dir / "README.md"
    if readme.exists():
        return
    readme.write_text(
        "# DimOS Evolution Ledger\n\n"
        "This directory stores low-volume agent evolution events and human-reviewable "
        "proposals. It should not contain raw images, embeddings, database files, or "
        "large robot logs.\n",
        encoding="utf-8",
    )


def _current_git_root() -> Path:
    result = subprocess.run(
        ["git", "rev-parse", "--show-toplevel"],
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode == 0:
        return Path(result.stdout.strip()).resolve()
    return Path.cwd().resolve()


def _unique_event_path(event_dir: Path, event_day: datetime, event_type: str) -> Path:
    safe_type = re.sub(r"[^A-Za-z0-9_.-]+", "_", event_type).strip("._-") or "event"
    stem = f"{event_day.strftime('%Y%m%dT%H%M%S%fZ')}-{safe_type}"
    path = event_dir / f"{stem}.json"
    suffix = 1
    while path.exists():
        path = event_dir / f"{stem}-{suffix}.json"
        suffix += 1
    return path


__all__ = [
    "EVOLUTION_EVENT_SCHEMA",
    "EVOLUTION_LEDGER_DIR_ENV",
    "EvolutionEvent",
    "build_evolution_event",
    "parse_payload_json",
    "resolve_evolution_ledger_dir",
    "write_evolution_event_file",
]
