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
import json
from pathlib import Path
import time
from typing import TYPE_CHECKING

from dimos.core.instance_registry import get, get_sole_running, latest_run_dir, list_running

if TYPE_CHECKING:
    from collections.abc import Callable, Iterator

_STANDARD_KEYS = {"timestamp", "level", "logger", "event", "func_name", "lineno"}
_LEVEL_COLORS = {"err": "\033[31m", "war": "\033[33m", "deb": "\033[2m"}
_RESET = "\033[0m"


def resolve_log_path(name: str = "", run_datetime: str = "") -> Path | None:
    """Find the log file for a given instance name and optional run datetime.

    Resolution order:
    1. If name + run_datetime given: look in that specific run dir
    2. If name given: use the running instance's run_dir, or latest run dir
    3. If no name: use sole running instance, or scan all for latest
    """
    if name and run_datetime:
        from dimos.core.instance_registry import _instances_dir
        run_dir = _instances_dir() / name / "runs" / run_datetime
        return _log_path_if_exists(str(run_dir))

    if name:
        # Check if it's running
        info = get(name)
        if info is not None:
            return _log_path_if_exists(info.run_dir)
        # Not running — get latest run dir
        rd = latest_run_dir(name)
        if rd is not None:
            return _log_path_if_exists(str(rd))
        return None

    # No name specified — try sole running instance
    running = list_running()
    if len(running) == 1:
        return _log_path_if_exists(running[0].run_dir)

    # Try to find the most recent run across all instances
    from dimos.core.instance_registry import _instances_dir
    base = _instances_dir()
    if not base.exists():
        return None

    latest: Path | None = None
    for child in base.iterdir():
        runs_dir = child / "runs"
        if not runs_dir.exists():
            continue
        for rd in sorted(runs_dir.iterdir(), reverse=True):
            p = rd / "main.jsonl"
            if p.exists():
                if latest is None or rd.name > latest.parent.name:
                    latest = p
                break  # only check most recent per instance

    return latest


def format_line(raw: str, *, json_output: bool = False) -> str:
    """Format a JSONL log line for display."""
    if json_output:
        return raw.rstrip()
    try:
        rec: dict[str, object] = json.loads(raw)
    except json.JSONDecodeError:
        return raw.rstrip()

    ts = str(rec.get("timestamp", ""))
    hms = ts[11:19] if len(ts) >= 19 else ts
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
    tail: deque[str] = deque(maxlen=count)
    with open(path) as f:
        for line in f:
            tail.append(line)
    return list(tail)


def follow_log(path: Path, stop: Callable[[], bool] | None = None) -> Iterator[str]:
    """Yield new lines as they appear (``tail -f`` style)."""
    with open(path) as f:
        f.seek(0, 2)
        while stop is None or not stop():
            line = f.readline()
            if line:
                yield line
            else:
                time.sleep(0.1)


def _log_path_if_exists(log_dir: str) -> Path | None:
    path = Path(log_dir) / "main.jsonl"
    return path if path.exists() else None
