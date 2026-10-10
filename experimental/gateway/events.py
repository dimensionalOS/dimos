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

import asyncio
from collections.abc import Awaitable, Callable
import json
import os
from pathlib import Path
import re
from typing import Any

from dimos.utils.logging_config import setup_logger
from experimental.gateway import launches

logger = setup_logger()

NAMESPACE_ENV = "DIMOS_ZENOH_NAMESPACE"


def resolve_namespace(given: str | None) -> str | None:
    namespace = given or os.environ.get(NAMESPACE_ENV)
    if not namespace:
        return None
    if any(chunk == "" for chunk in namespace.split("/")) or re.search(r"[*?#$]", namespace):
        raise ValueError(f"zenoh namespace {namespace!r} has an empty chunk or a wildcard")
    return namespace


def resolve_connect(given: str | None) -> list[str]:
    from dimos.core.global_config import global_config

    value = given if given is not None else global_config.zenoh_connect
    return [item.strip() for item in str(value or "").split(",") if item.strip()]


def zenoh_session(connect: list[str]) -> Any:
    from dimos.protocol.service.zenohservice import ZenohConfig, default_session_pool

    return default_session_pool.acquire(ZenohConfig(connect=connect))


def zenoh_put(connect: list[str]) -> Callable[[str, bytes], None]:
    import zenoh

    session = zenoh_session(connect)

    def put(key: str, payload: bytes) -> None:
        session.put(key, payload, encoding=zenoh.Encoding.APPLICATION_JSON)

    return put


class Bus:
    def __init__(
        self, namespace: str | None = None, put: Callable[[str, bytes], None] | None = None
    ) -> None:
        self.namespace = namespace
        self.put = put
        self.launch_key: tuple[Any, ...] | None = None

    def send(self, event: dict[str, Any]) -> None:
        if self.put is None or self.namespace is None:
            return
        try:
            self.put(f"{self.namespace}/dimos/events/{event['type']}", json.dumps(event).encode())
        except Exception:
            logger.exception("publishing an event failed", event_type=event.get("type"))

    def launch(self, launch: dict[str, Any] | None) -> None:
        key = (launch["blueprint"], launch["phase"], launch["runId"]) if launch else None
        if key != self.launch_key:
            self.launch_key = key
            self.send({"type": "launch", "launch": launch})


async def watch_launch(bus: Bus, interval: float = 1.0) -> None:
    failure = ""
    while True:
        await asyncio.sleep(interval)
        try:
            bus.launch(await asyncio.to_thread(launches.current_launch))
            failure = ""
        except Exception as error:
            if repr(error) != failure:
                failure = repr(error)
                logger.exception("reading the launch failed")


def snapshot(dimos_dir: Path, site_dirs: list[Path]) -> frozenset[tuple[str, int]]:
    found: set[tuple[str, int]] = set()
    for root, dirs, files in os.walk(dimos_dir / "dimos" / "robot"):
        dirs[:] = [name for name in dirs if name != "__pycache__"]
        for name in files:
            if name.endswith(".py"):
                path = os.path.join(root, name)
                try:
                    found.add((path, os.stat(path).st_mtime_ns))
                except OSError:
                    continue
    for packages in site_dirs:
        try:
            found |= {(e.path, 0) for e in os.scandir(packages) if ".dist-info" in e.name}
        except OSError:
            continue
    return frozenset(found)


async def watch_blueprints(
    dimos_dir: Path,
    site_dirs: list[Path],
    relist: Callable[[], Awaitable[list[str]]],
    changed: Callable[[list[str], list[str]], None],
    poll: float = 2.0,
    settle: float = 3.0,
) -> None:
    names = await relist()
    seen = await asyncio.to_thread(snapshot, dimos_dir, site_dirs)
    while True:
        await asyncio.sleep(poll)
        now = await asyncio.to_thread(snapshot, dimos_dir, site_dirs)
        if now == seen:
            continue
        while True:
            await asyncio.sleep(settle)
            later = await asyncio.to_thread(snapshot, dimos_dir, site_dirs)
            if later == now:
                break
            now = later
        seen = now
        try:
            listed = await relist()
        except Exception as error:
            logger.warning("re-listing the blueprints failed", error=str(error))
            continue
        added, removed = sorted(set(listed) - set(names)), sorted(set(names) - set(listed))
        names = listed
        if added or removed:
            changed(added, removed)
