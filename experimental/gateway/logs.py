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

import json
from pathlib import Path
from typing import Any

from dimos.constants import LOG_DIR

MAX_READ = 4 * 1024 * 1024
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
            k: v for k, v in data.items() if k not in ("timestamp", "level", "logger", "event")
        },
        "raw": line,
    }


def matches(record: dict[str, Any], query: str | None, min_level: str | None) -> bool:
    if min_level and LEVEL_RANK.get(record["level"], 20) < LEVEL_RANK.get(min_level, 20):
        return False
    return not query or query.lower() in record["raw"].lower()


def find(dimos_dir: Path, run_id: str) -> Path | None:
    from dimos.core.run_registry import list_runs

    if run_id != "latest":
        entry = next((e for e in list_runs(alive_only=False) if e.run_id == run_id), None)
        if entry is not None and (Path(entry.log_dir) / "main.jsonl").exists():
            return Path(entry.log_dir) / "main.jsonl"
    found = sorted(
        (path.name, path / "main.jsonl")
        for root in logs_dirs(dimos_dir)
        if root.is_dir()
        for path in root.iterdir()
        if (path / "main.jsonl").exists() and (run_id == "latest" or path.name == run_id)
    )
    return found[-1][1] if found else None


def read(
    file: Path | None,
    run_id: str,
    after: int | None,
    limit: int | None,
    query: str | None = None,
    min_level: str | None = None,
) -> dict[str, Any]:
    if file is not None and run_id == "latest":
        run_id = file.parent.name
    empty = {"runId": run_id, "records": [], "offset": after or 0, "loggers": []}
    if file is None:
        return empty
    try:
        with file.open("rb") as handle:
            size = handle.seek(0, 2)
            start = after if after is not None and after <= size else 0
            start = max(start, size - MAX_READ)
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
