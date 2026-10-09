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

"""A run's structured log (`<logs>/<run_id>/main.jsonl`), read straight off disk, for paging and tailing."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any

from dimos.constants import LOG_DIR

MAX_READ = 4 * 1024 * 1024
LEVEL_RANK = {"debug": 10, "warning": 30, "warn": 30, "error": 40, "critical": 50}


def level_rank(level: str) -> int:
    return LEVEL_RANK.get(level, 20)


def logs_dirs(dimos_dir: Path) -> list[Path]:
    """Where runs log (`<dir>/<run_id>/main.jsonl`): dimos's LOG_DIR, and the served checkout's own when the gateway
    runs from another install."""
    return list(dict.fromkeys([LOG_DIR, dimos_dir / LOG_DIR.name]))


def _text(value: Any) -> str:
    if value is None:
        return ""
    return value if isinstance(value, str) else json.dumps(value, separators=(",", ":"))


def parse_line(line: str) -> dict[str, Any] | None:
    """One line of main.jsonl; a non-JSON line becomes a level "raw" record."""
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
    extra = {k: v for k, v in data.items() if k not in ("timestamp", "level", "logger", "event")}
    level = _text(data.get("level")).lower()
    return {
        "timestamp": _text(data.get("timestamp")),
        "level": level or "info",
        "logger": _text(data.get("logger")),
        "event": _text(data.get("event")),
        "extra": extra,
        "raw": line,
    }


@dataclass
class Filter:
    query: str | None = None
    min_level: str | None = None

    def matches(self, record: dict[str, Any]) -> bool:
        if self.min_level and level_rank(record["level"]) < level_rank(self.min_level):
            return False
        if self.query:
            return self.query.lower() in record["raw"].lower()
        return True


def is_problem(record: dict[str, Any]) -> bool:
    return level_rank(record["level"]) >= 30 and record["level"] != "raw"


def log_runs(dimos_dir: Path) -> list[tuple[str, Path]]:
    """(run_id, main.jsonl), newest first."""
    runs = []
    for root in logs_dirs(dimos_dir):
        if root.is_dir():
            runs += [
                (d.name, d / "main.jsonl") for d in root.iterdir() if (d / "main.jsonl").exists()
            ]
    return sorted(runs, reverse=True)


def read(
    dimos_dir: Path, run_id: str | None, after: int | None, limit: int, filter: Filter
) -> dict[str, Any]:
    """A run's records (`latest` or none: the newest run)."""
    from dimos.core.run_registry import list_runs

    runs = log_runs(dimos_dir)
    if run_id and run_id != "latest":
        # the registry knows a run's log folder; older runs are only found in the log folders
        entry = next((e for e in list_runs(alive_only=False) if e.run_id == run_id), None)
        registered = Path(entry.log_dir) / "main.jsonl" if entry else None
        target = (
            (run_id, registered)
            if registered and registered.exists()
            else next((run for run in runs if run[0] == run_id), None)
        )
    else:
        target = runs[0] if runs else None
    if target is None:
        return {"runId": run_id, "records": [], "offset": 0, "loggers": []}
    return read_file(target[1], target[0], after, limit, filter)


def read_file(
    file: Path, run_id: str, after: int | None, limit: int | None, filter: Filter
) -> dict[str, Any]:
    """Records after byte `after` (tailing), or the last `limit` when `after` is None; `offset` is where to go on."""
    empty = {"runId": run_id, "records": [], "offset": after or 0, "loggers": []}
    try:
        with file.open("rb") as handle:
            size = handle.seek(0, 2)
            # an offset past the end means the file was replaced: start over
            start = after if after is not None and after <= size else max(0, size - MAX_READ)
            start = max(start, size - MAX_READ)
            handle.seek(start)
            buffer = handle.read(size - start)
    except OSError:
        return empty
    # a trailing partial line waits for the next poll
    consumed = buffer.rfind(b"\n") + 1
    lines = buffer[:consumed].decode("utf-8", "replace").splitlines()
    # starting mid-file, the first line is probably cut
    if start > 0 and after is None and lines:
        lines.pop(0)
    parsed = [parse_line(line) for line in lines]
    every = [record for record in parsed if record is not None]
    loggers = sorted({record["logger"] for record in every if record["logger"]})
    records = [record for record in every if filter.matches(record)]
    if after is None and limit is not None and len(records) > limit:
        records = records[-limit:]
    return {"runId": run_id, "records": records, "offset": start + consumed, "loggers": loggers}
