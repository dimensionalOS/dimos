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

from collections.abc import Iterable, Iterator
import json
from operator import itemgetter
from pathlib import Path
import re
from typing import TYPE_CHECKING, Any

from dimos.constants import LOG_DIR
from experimental.gateway import constants

if TYPE_CHECKING:
    from dimos.core.run_registry import RunEntry

LEVEL_RANK = {"debug": 10, "warning": 30, "warn": 30, "error": 40, "critical": 50}


def logs_dirs(dimos_dir: Path) -> list[Path]:
    return list(dict.fromkeys([LOG_DIR, dimos_dir / LOG_DIR.name]))


def _text(value: Any) -> str:
    if value is None:
        return ""
    return value if isinstance(value, str) else json.dumps(value, separators=(",", ":"))


def parse_line(line: str) -> dict[str, Any] | None:
    if not line.strip():
        return None
    try:
        data = json.loads(line)
    except ValueError:
        data = None
    if not isinstance(data, dict):
        return {
            "timestamp": "",
            "level": "raw",
            "logger": "",
            "event": line,
            "extra": {},
            "raw": line,
        }
    return {
        "timestamp": _text(data.get("timestamp")),
        "level": _text(data.get("level")).lower() or "info",
        "logger": _text(data.get("logger")),
        "event": _text(data.get("event")),
        "extra": {
            key: value
            for key, value in data.items()
            if key not in ("timestamp", "level", "logger", "event")
        },
        "raw": line,
    }


def matches(record: dict[str, Any], query: str | None, min_level: str | None) -> bool:
    if min_level and LEVEL_RANK.get(record["level"], 20) < LEVEL_RANK.get(min_level, 20):
        return False
    return not query or query.lower() in record["raw"].lower()


def _run_logs(dimos_dir: Path, registered: Iterable[RunEntry]) -> Iterator[tuple[str, Path]]:
    """Yield the registry's log location first when a run also appears under a checkout."""
    for run in registered:
        file = Path(run.log_dir) / "main.jsonl"
        if file.is_file():
            yield run.run_id, file
    for root in sorted(logs_dirs(dimos_dir), reverse=True):
        for file in root.glob("*/main.jsonl"):
            if file.is_file():
                yield file.parent.name, file


def find(dimos_dir: Path, run_id: str) -> tuple[str, Path] | None:
    from dimos.core.run_registry import list_runs

    files = _run_logs(dimos_dir, list_runs(alive_only=False))
    if run_id == "latest":
        return max(files, key=itemgetter(0), default=None)
    return next((entry for entry in files if entry[0] == run_id), None)


def history(dimos_dir: Path) -> list[dict[str, Any]]:
    """Every run with a log, newest first, with its blueprint and start when the run registry knows them."""
    from dimos.core.run_registry import list_runs

    known = {run.run_id: run for run in list_runs(alive_only=False)}
    alive = {run.run_id for run in list_runs(alive_only=True)}
    files: dict[str, Path] = {}
    for run_id, file in _run_logs(dimos_dir, known.values()):
        files.setdefault(run_id, file)
    return [
        {
            "runId": run_id,
            # a stopped run leaves the registry: its blueprint is in its id, its start in its log
            "blueprint": known[run_id].blueprint if run_id in known else blueprint_of(run_id),
            "startedAt": known[run_id].started_at if run_id in known else first_time(files[run_id]),
            "endedAt": None if run_id in alive else last_time(files[run_id]),
            "running": run_id in alive,
        }
        for run_id in sorted(files, reverse=True)
    ]


def blueprint_of(run_id: str) -> str | None:
    """The blueprint in a run id: `<YYYYMMDD-HHMMSS>[-<4 hex>]-<blueprint>`."""
    found = re.fullmatch(r"\d{8}-\d{6}-(?:[0-9a-f]{4}-)?(.+)", run_id)
    return found.group(1) if found else None


def first_time(file: Path | None) -> str | None:
    """The timestamp of a log's first record."""
    if file is None:
        return None
    try:
        with file.open("rb") as handle:
            lines = (
                handle.read(constants.LOG_TIMESTAMP_PROBE_BYTES)
                .decode("utf-8", "replace")
                .splitlines()
            )
    except OSError:
        return None
    for line in lines:
        record = parse_line(line)
        if record is not None and record["timestamp"]:
            return str(record["timestamp"])
    return None


def last_time(file: Path | None) -> str | None:
    """The timestamp of a log's last record."""
    if file is None:
        return None
    try:
        with file.open("rb") as handle:
            handle.seek(max(0, handle.seek(0, 2) - constants.LOG_TIMESTAMP_PROBE_BYTES))
            lines = handle.read().decode("utf-8", "replace").splitlines()
    except OSError:
        return None
    for line in reversed(lines):
        record = parse_line(line)
        if record is not None and record["timestamp"]:
            return str(record["timestamp"])
    return None


def read(
    file: Path | None,
    run_id: str,
    after: int | None,
    limit: int | None,
    query: str | None = None,
    min_level: str | None = None,
) -> dict[str, Any]:
    empty = {"runId": run_id, "records": [], "offset": after or 0, "loggers": []}
    if file is None:
        return empty
    try:
        with file.open("rb") as handle:
            size = handle.seek(0, 2)
            start = after if after is not None and after <= size else 0
            start = max(start, size - constants.LOG_MAX_READ_BYTES)
            handle.seek(start)
            buffer = handle.read(size - start)
    except OSError:
        return empty
    consumed = buffer.rfind(b"\n") + 1
    lines = buffer[:consumed].decode("utf-8", "replace").splitlines()
    if start > 0 and after is None and lines:
        lines.pop(0)
    every = [record for record in map(parse_line, lines) if record is not None]
    records = [record for record in every if matches(record, query, min_level)]
    if after is None and limit is not None:
        records = records[-limit:]
    loggers = sorted({record["logger"] for record in every if record["logger"]})
    return {"runId": run_id, "records": records, "offset": start + consumed, "loggers": loggers}
