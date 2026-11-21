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

from collections import deque
from datetime import datetime
import json
from pathlib import Path
import time
from typing import TYPE_CHECKING

from dimos.constants import LOG_DIR
from dimos.core.run_registry import get_most_recent, list_runs

if TYPE_CHECKING:
    from collections.abc import Callable, Iterator

_STANDARD_KEYS = {"timestamp", "level", "logger", "event", "func_name", "lineno"}
_LEVEL_COLORS = {"err": "\033[31m", "war": "\033[33m", "deb": "\033[2m"}
_RESET = "\033[0m"


def resolve_log_path(run_id: str = "") -> Path | None:
    """Find the log file: specific run → alive run → most recent."""
    # A run leaves the registry when it exits, but its log stays in LOG_DIR.
    if run_id:
        for entry in list_runs(alive_only=False):
            if entry.run_id == run_id:
                return _log_path_if_exists(entry.log_dir)
        return _log_path_if_exists(LOG_DIR / run_id)

    # Prefer alive run, fall back to most recent stopped run.
    alive = get_most_recent(alive_only=True)
    if alive is not None:
        return _log_path_if_exists(alive.log_dir)
    # Timestamped run IDs sort by start time, matching the registry's ordering.
    return max(LOG_DIR.glob("*/main.jsonl"), key=lambda p: p.parent.name, default=None)


def format_line(raw: str, *, json_output: bool = False) -> str:
    """Format a JSONL log line for display.

    Default: ``HH:MM:SS [lvl] logger           event  k=v …`` in local time.
    """
    if json_output:
        return raw.rstrip()
    try:
        rec: dict[str, object] = json.loads(raw)
    except json.JSONDecodeError:
        return raw.rstrip()

    ts = str(rec.get("timestamp", ""))
    try:
        # The record stamp is UTC. Print local time, like the console does.
        hms = datetime.fromisoformat(ts.replace("Z", "+00:00")).astimezone().strftime("%H:%M:%S")
    except ValueError:
        hms = ts
    level = str(rec.get("level", "?"))[:3]
    logger = Path(str(rec.get("logger", "?"))).name
    event = str(rec.get("event", ""))
    color = _LEVEL_COLORS.get(level, "")

    extras = " ".join(f"{k}={v}" for k, v in rec.items() if k not in _STANDARD_KEYS)
    line = f"{hms} {color}[{level}]{_RESET} {logger:17} {event}"
    if extras:
        line += f"  {extras}"
    return line


def read_log(path: Path, count: int | None = 50) -> list[str]:
    """Read last *count* lines from a log file (``None`` = all)."""
    if count is None:
        return path.read_text().splitlines(keepends=True)
    # Only keep the tail — avoids loading the full file into a list.
    tail: deque[str] = deque(maxlen=count)
    with open(path) as f:
        for line in f:
            tail.append(line)
    return list(tail)


def follow_log(
    path: Path, stop: Callable[[], bool] | None = None, *, count: int | None = 50
) -> Iterator[str]:
    """Yield the last *count* lines, then new lines (``None`` = all history).

    *stop* is an optional callable; when it returns ``True`` the
    generator exits cleanly.
    """
    with open(path) as f:
        history: deque[str] = deque(maxlen=count)
        for line in f:
            if stop is not None and stop():
                return
            history.append(line)
        for line in history:
            if stop is not None and stop():
                return
            yield line
        while stop is None or not stop():
            line = f.readline()
            if line:
                yield line
            else:
                time.sleep(0.1)


def _log_path_if_exists(log_dir: str | Path) -> Path | None:
    path = Path(log_dir) / "main.jsonl"
    return path if path.exists() else None
