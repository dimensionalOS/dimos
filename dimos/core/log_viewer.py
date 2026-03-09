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

"""Log viewer for per-run DimOS logs."""

from __future__ import annotations

import json
from pathlib import Path
import time
from typing import TYPE_CHECKING

from dimos.core.run_registry import get_most_recent, list_runs

if TYPE_CHECKING:
    from collections.abc import Iterator

_SKIP_KEYS = frozenset({"timestamp", "level", "logger", "event", "func_name", "lineno"})
_COLORS = {"err": "\033[31m", "war": "\033[33m", "deb": "\033[2m"}
_RESET = "\033[0m"


def resolve_log_path(run_id: str = "") -> Path | None:
    """Find the log file: specific run → alive run → most recent."""
    if run_id:
        for entry in list_runs(alive_only=False):
            if entry.run_id == run_id:
                path = Path(entry.log_dir) / "main.jsonl"
                return path if path.exists() else None
        return None

    for alive_only in (True, False):
        result = get_most_recent(alive_only=alive_only)
        if result is not None:
            path = Path(result.log_dir) / "main.jsonl"
            if path.exists():
                return path
    return None


def format_line(raw: str, json_output: bool = False) -> str:
    """Parse a JSONL line into human-readable format.

    Default format: ``HH:MM:SS [lvl] logger_name       event extras``
    With *json_output*: returns the raw line stripped.
    """
    if json_output:
        return raw.rstrip()
    try:
        d: dict[str, object] = json.loads(raw)
    except json.JSONDecodeError:
        return raw.rstrip()

    ts = str(d.get("timestamp", ""))
    time_str = ts[11:19] if len(ts) > 19 else ts

    level = str(d.get("level", "?"))[:3]
    logger = Path(str(d.get("logger", "?"))).name
    event = str(d.get("event", ""))

    extras = " ".join(f"{k}={v}" for k, v in d.items() if k not in _SKIP_KEYS)
    color = _COLORS.get(level, "")

    line = f"{time_str} {color}[{level}]{_RESET} {logger:17} {event}"
    if extras:
        line += f"  {extras}"
    return line


def read_log(path: Path, count: int | None = 50) -> list[str]:
    """Read last *count* lines from a log file.  ``None`` returns all."""
    with open(path) as f:
        lines = f.readlines()
    if count is not None:
        lines = lines[-count:]
    return lines


def follow_log(path: Path) -> Iterator[str]:
    """Yield new lines as they appear (``tail -f`` style)."""
    with open(path) as f:
        f.seek(0, 2)  # seek to end
        while True:
            line = f.readline()
            if line:
                yield line
            else:
                time.sleep(0.1)
