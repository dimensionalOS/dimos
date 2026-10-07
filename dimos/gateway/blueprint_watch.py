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

"""Watches where dimos's blueprint list comes from: the checkout's blueprint sources (dimos/robot/**, whose
all_blueprints.py is the registry) and the venv's site-packages (an external blueprint arrives as a package whose
*.dist-info names it). A change, settled for `settle` s, re-lists the blueprints in a child process (this process
keeps its first import of the registry, so it can't re-read it) and, only when the list differs, sends
`{"type": "blueprints", "added", "removed"}` and asks discovery to re-check its key.

It polls file times every `poll` s rather than taking a file-watching dependency: dimos/robot is a few hundred files
(about 2 ms a pass) and site-packages is read one level deep.
"""

from __future__ import annotations

import asyncio
from collections.abc import Awaitable, Callable
import os
from pathlib import Path
from typing import Any

from dimos.gateway.discovery import site_dirs
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

POLL_S = 1.0
SETTLE_S = 3.0
# at least this long between re-listings (each imports the registry in a child): changes in between wait for the next
MIN_GAP_S = 10.0


def relevant(name: str) -> bool:
    """A file or folder whose change can touch the blueprint list: Python sources and package metadata."""
    return name.endswith((".py", ".pth")) or ".dist-info" in name


def snapshot(dimos_dir: Path) -> frozenset[tuple[str, int, int]]:
    """Every relevant path under dimos/robot (recursive) and in site-packages (one level), with its mtime and size."""
    found: set[tuple[str, int, int]] = set()
    sources = dimos_dir / "dimos" / "robot"
    for root, dirs, files in os.walk(sources):
        dirs[:] = [name for name in dirs if name != "__pycache__"]
        for name in files:
            if relevant(name):
                path = os.path.join(root, name)
                try:
                    stat = os.stat(path)
                except OSError:
                    continue
                found.add((path, stat.st_mtime_ns, stat.st_size))
    for packages in site_dirs(dimos_dir):
        try:
            entries = list(os.scandir(packages))
        except OSError:
            continue
        for entry in entries:
            if relevant(entry.name):
                try:
                    found.add((entry.path, entry.stat().st_mtime_ns, 0))
                except OSError:
                    continue
    return frozenset(found)


def diff(before: list[str], after: list[str]) -> tuple[list[str], list[str]]:
    """(added, removed), each sorted."""
    return sorted(set(after) - set(before)), sorted(set(before) - set(after))


class BlueprintWatch:
    """`list_blueprints()` re-lists in a child: [{name, kind}]. `changed(listed, added, removed)` hears a different
    list; `touched()` hears every settled change (discovery re-checks its key then)."""

    def __init__(
        self,
        dimos_dir: Path,
        list_blueprints: Callable[[], Awaitable[list[dict[str, Any]]]],
        changed: Callable[[list[dict[str, Any]], list[str], list[str]], None],
        touched: Callable[[], None],
        poll: float = POLL_S,
        settle: float = SETTLE_S,
        min_gap: float = MIN_GAP_S,
    ) -> None:
        self.dimos_dir = dimos_dir
        self.list_blueprints = list_blueprints
        self.changed = changed
        self.touched = touched
        self.poll = poll
        self.settle = settle
        self.min_gap = min_gap
        self.names: list[str] | None = None
        # the latest listing from a child: fresher than this process's import of the registry
        self.listed: list[dict[str, Any]] | None = None

    async def relist(self) -> None:
        try:
            listed = await self.list_blueprints()
        except Exception as error:
            logger.warning("re-listing the blueprints failed", error=str(error))
            return
        names = [entry["name"] for entry in listed]
        self.listed = listed
        if self.names is not None:
            added, removed = diff(self.names, names)
            if added or removed:
                self.changed(listed, added, removed)
        self.names = names

    async def run(self) -> None:
        """Forever: list once, then re-list after each settled change."""
        await self.relist()
        seen = await asyncio.to_thread(snapshot, self.dimos_dir)
        last = asyncio.get_running_loop().time()
        while True:
            await asyncio.sleep(self.poll)
            now = await asyncio.to_thread(snapshot, self.dimos_dir)
            if now == seen:
                continue
            # debounce: a burst (a checkout, a pip install flooding site-packages) re-lists once, after it has been
            # quiet for `settle` s; throttle: never sooner than `min_gap` s after the last re-listing
            while True:
                await asyncio.sleep(self.settle)
                later = await asyncio.to_thread(snapshot, self.dimos_dir)
                if later == now:
                    break
                now = later
            wait = last + self.min_gap - asyncio.get_running_loop().time()
            if wait > 0:
                await asyncio.sleep(wait)
                now = await asyncio.to_thread(snapshot, self.dimos_dir)
            seen = now
            self.touched()
            await self.relist()
            last = asyncio.get_running_loop().time()
