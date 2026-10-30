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
from contextlib import suppress
import json
from pathlib import Path
import re
from typing import TYPE_CHECKING, Any

import watchfiles

from dimos.core import run_registry
from dimos.utils.logging_config import setup_logger
from experimental.gateway import constants, launches, store

if TYPE_CHECKING:
    from experimental.gateway.process_stats import ProcessStats

logger = setup_logger()


def resolve_namespace(given: str | None) -> str | None:
    namespace = given or store.setting("zenoh", "namespace")
    if not namespace:
        return None
    if any(chunk == "" for chunk in namespace.split("/")) or re.search(r"[*?#$]", namespace):
        raise ValueError(f"zenoh namespace {namespace!r} has an empty chunk or a wildcard")
    return namespace


def resolve_connect(given: str | None) -> list[str]:
    from dimos.core.global_config import global_config

    configured = store.setting("zenoh", "connect")
    value = given if given is not None else configured or global_config.zenoh_connect
    return [item.strip() for item in str(value or "").split(",") if item.strip()]


def zenoh_put(connect: list[str]) -> Callable[[str, bytes], None]:
    import zenoh

    from dimos.protocol.service.zenohservice import ZenohConfig, default_session_pool

    session = default_session_pool.acquire(ZenohConfig(connect=connect))

    def put(key: str, payload: bytes) -> None:
        try:
            session.put(key, payload, encoding=zenoh.Encoding.APPLICATION_JSON)
        except zenoh.ZError:
            logger.exception("publishing an event failed", key=key)

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
        self.put(f"{self.namespace}/dimos/events/{event['type']}", json.dumps(event).encode())

    def launch(self, launch: dict[str, Any] | None) -> None:
        key = (launch["blueprint"], launch["phase"], launch["runId"]) if launch else None
        if key != self.launch_key:
            self.launch_key = key
            self.send({"type": "launch", "launch": launch})


async def set_on_changes(
    wake: asyncio.Event,
    paths: list[Path],
    watch_filter: Callable[[watchfiles.Change, str], bool],
    recursive: bool = True,
    debounce_ms: int = constants.FILE_EVENT_DEBOUNCE_MS,
) -> None:
    """Sets `wake` after each batch of file changes under the paths that exist."""
    existing = [path for path in paths if path.is_dir()]
    if not existing:
        return
    try:
        async for _ in watchfiles.awatch(
            *existing, watch_filter=watch_filter, recursive=recursive, debounce=debounce_ms
        ):
            wake.set()
    except Exception:
        logger.exception("watching files failed", paths=[str(path) for path in existing])


async def wait_or_timeout(wake: asyncio.Event, timeout_s: float) -> None:
    # python 3.10's wait_for raises asyncio.TimeoutError, not the builtin
    with suppress(asyncio.TimeoutError, TimeoutError):
        await asyncio.wait_for(wake.wait(), timeout_s)
    wake.clear()


# the stats subscription ends with the run
async def watch_launch(
    bus: Bus,
    stats: ProcessStats | None = None,
    stale_check_s: float = constants.STALE_RUN_CHECK_S,
    debounce_ms: int = constants.FILE_EVENT_DEBOUNCE_MS,
) -> None:
    """Publishes the launch on registry or launch.json changes, the child exiting, or a crash check."""
    registry_dir, gateway_dir = Path(run_registry.REGISTRY_DIR), store.gateway_dir()
    for directory in (registry_dir, gateway_dir):
        directory.mkdir(parents=True, exist_ok=True)
    # the OS reports resolved paths (macOS: /private/var for /var)
    registry_dir = registry_dir.resolve()
    launch_file_name = launches.launch_file().name

    def is_launch_state(_change: watchfiles.Change, path: str) -> bool:
        changed = Path(path)
        return changed.parent == registry_dir or changed.name == launch_file_name

    wake = asyncio.Event()
    loop = asyncio.get_running_loop()

    def child_exited() -> None:
        with suppress(RuntimeError):
            loop.call_soon_threadsafe(wake.set)

    launches.exit_listeners.append(child_exited)
    file_watch = asyncio.create_task(
        set_on_changes(
            wake,
            [registry_dir, gateway_dir],
            is_launch_state,
            recursive=False,
            debounce_ms=debounce_ms,
        )
    )
    failure = ""
    try:
        while True:
            try:
                bus.launch(await asyncio.to_thread(launches.current_launch))
                if stats is not None:
                    await asyncio.to_thread(stats.stop_if_nothing_runs)
                failure = ""
            except Exception as error:
                if repr(error) != failure:
                    failure = repr(error)
                    logger.exception("reading the launch failed")
            await wait_or_timeout(wake, stale_check_s)
    finally:
        launches.exit_listeners.remove(child_exited)
        file_watch.cancel()


def is_blueprint_source(_change: watchfiles.Change, path: str) -> bool:
    """A .py file or a directory (a deleted one has no suffix either), outside __pycache__."""
    changed = Path(path)
    return "__pycache__" not in changed.parts and changed.suffix in ("", ".py")


def is_installed_package(_change: watchfiles.Change, path: str) -> bool:
    return any(".dist-info" in part for part in Path(path).parts)


async def watch_blueprints(
    dimos_dir: Path,
    site_dirs: list[Path],
    relist: Callable[[], Awaitable[list[str]]],
    changed: Callable[[list[str], list[str]], None],
    debounce_ms: int = constants.FILE_EVENT_DEBOUNCE_MS,
) -> None:
    """Re-lists the blueprints after dimos/robot's sources or site-packages' installs change."""
    wake = asyncio.Event()
    # site-packages is watched one level deep: a recursive watch of a venv can exhaust inotify's watches
    watchers = [
        asyncio.create_task(
            set_on_changes(
                wake, [dimos_dir / "dimos" / "robot"], is_blueprint_source, debounce_ms=debounce_ms
            )
        ),
        asyncio.create_task(
            set_on_changes(
                wake, site_dirs, is_installed_package, recursive=False, debounce_ms=debounce_ms
            )
        ),
    ]
    try:
        names = await relist()
        while True:
            await wake.wait()
            wake.clear()
            try:
                listed = await relist()
            except Exception:
                logger.exception("re-listing the blueprints failed")
                continue
            added, removed = sorted(set(listed) - set(names)), sorted(set(names) - set(listed))
            names = listed
            if added or removed:
                changed(added, removed)
    finally:
        for watcher in watchers:
            watcher.cancel()
