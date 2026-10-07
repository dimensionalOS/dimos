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

"""The gateway's events, each a JSON object with a `type`, on zenoh at `<ns>/dimos/events/<type>` (zenoh_events.py)
and, deprecated for one release, on the SSE stream /dimos/events (one per `data:` line). Its types:

{"type": "launch", "launch": <launch or null>}       the launch's phase changed (and first, on connect)
{"type": "log", "runId", "record"}                   a warning+ record in the launched run's main.jsonl
{"type": "upload", "upload"}                         an upload changed
{"type": "upload-removed", "id"}
{"type": "uploads", "waitingForLogin", "cleared"?}   the upload queue as a whole
{"type": "cloud-login", "login"}                     the device login's state
"""

from __future__ import annotations

import asyncio
from collections.abc import AsyncIterator, Callable
import json
from pathlib import Path
from typing import Any

from dimos.gateway import logs, runs
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

KEEP_ALIVE_S = 15.0


class Bus:
    """Fan-out to every sink (the zenoh publisher) and open event stream; a stream that can't keep up loses events,
    never blocks the gateway."""

    def __init__(self) -> None:
        self.queues: set[asyncio.Queue[dict[str, Any]]] = set()
        self.sinks: list[Callable[[dict[str, Any]], None]] = []
        # (key under `<ns>/dimos/`, payload): what isn't an event, e.g. a job's lines on `jobs/<job>`
        self.publishers: list[Callable[[str, dict[str, Any]], None]] = []
        # the (blueprint, phase, runId) last sent as a `launch` event: the route that changed it and the watcher that
        # notices it a second later send it once between them
        self.launch_key: tuple[Any, ...] | None = None

    def launch(self, launch: dict[str, Any] | None) -> None:
        """A `launch` event, when (blueprint, phase, runId) changed since the last one."""
        key = (launch["blueprint"], launch["phase"], launch["runId"]) if launch else None
        if key != self.launch_key:
            self.launch_key = key
            self.send({"type": "launch", "launch": launch})

    def publish(self, key: str, payload: dict[str, Any]) -> None:
        for publisher in self.publishers:
            try:
                publisher(key, payload)
            except Exception:
                logger.exception("a publisher failed", key=key)

    def send(self, event: dict[str, Any]) -> None:
        for sink in self.sinks:
            try:
                sink(event)
            except Exception:
                logger.exception("an event sink failed", event_type=event.get("type"))
        for queue in list(self.queues):
            try:
                queue.put_nowait(event)
            except asyncio.QueueFull:
                pass

    async def stream(
        self, first: Callable[[], dict[str, Any]], keep_alive: float = KEEP_ALIVE_S
    ) -> AsyncIterator[str]:
        """SSE text: `first()` (taken once subscribed, so nothing is missed in between), then every event."""
        queue: asyncio.Queue[dict[str, Any]] = asyncio.Queue(maxsize=1024)
        self.queues.add(queue)
        try:
            yield f"data: {json.dumps(first())}\n\n"
            while True:
                try:
                    event = await asyncio.wait_for(queue.get(), keep_alive)
                except asyncio.TimeoutError:
                    yield ":\n\n"
                    continue
                yield f"data: {json.dumps(event)}\n\n"
        finally:
            self.queues.discard(queue)


async def watch_launch(bus: Bus, interval: float = 1.0) -> None:
    """Turns the launch's phase changes and its run's new warnings and errors into events, for as long as it runs."""
    tailing: tuple[Path, int] | None = None
    last_failure = ""
    while True:
        await asyncio.sleep(interval)
        try:
            launch = await asyncio.to_thread(runs.current_launch)
        except Exception as error:
            # it retries every second: log each new failure once, not every second
            if repr(error) != last_failure:
                last_failure = repr(error)
                logger.exception("reading the launch failed; no launch events until it works")
            continue
        last_failure = ""
        bus.launch(launch)
        log_dir = launch["logDir"] if launch else None
        file = Path(log_dir) / "main.jsonl" if log_dir else None
        if file is None:
            tailing = None
        elif tailing and tailing[0] == file:
            page = logs.read_file(file, "", tailing[1], None, logs.Filter())
            tailing = (file, page["offset"])
            for record in page["records"]:
                if logs.is_problem(record):
                    bus.send(
                        {
                            "type": "log",
                            "runId": launch["runId"] if launch else None,
                            "record": record,
                        }
                    )
        else:
            # start at the end: only new problems are events
            tailing = (file, file.stat().st_size if file.exists() else 0)
