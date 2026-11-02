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
from collections.abc import Callable
from dataclasses import dataclass, field
import os
from pathlib import Path
import sysconfig
import time
from typing import Annotated, Any

from fastapi import Depends, Query, Request
from pydantic import BeforeValidator

from dimos.utils.logging_config import setup_logger
from experimental.gateway import constants, events, introspect, store
from experimental.gateway.discovery import Jobs, Scanner
from experimental.gateway.errors import ApiError as ApiError
from experimental.gateway.launches import RunSource
from experimental.gateway.live import Live
from experimental.gateway.process_stats import ProcessStats

logger = setup_logger()

API_VERSION = "8.0.0"


@dataclass
class State:
    dimos_dir: Path
    bus: events.Bus
    scanner: Scanner
    jobs: Jobs = field(default_factory=Jobs)
    live: Live = field(default_factory=Live)
    process_stats: ProcessStats = field(default_factory=ProcessStats)
    cache: introspect.Cache = field(default_factory=introspect.Cache)
    listed: list[dict[str, Any]] | None = None
    exit: Callable[[], None] = lambda: os._exit(0)
    started_at: int = field(default_factory=lambda: int(time.time()))
    background: list[asyncio.Task[Any]] = field(default_factory=list)

    @staticmethod
    def introspect_key(args: list[str], stdin: Any = None, source: RunSource | None = None) -> str:
        """The cache key of one introspection, for introspected() and for forgetting its answer."""
        return " ".join(args) + (repr(stdin) if stdin is not None else "") + repr(source or "")

    async def introspected(
        self, args: list[str], stdin: Any = None, source: RunSource | None = None
    ) -> Any:
        """Introspect the gateway's checkout, or the one a run's dimos came from."""
        key = self.introspect_key(args, stdin, source)
        dimos_dir, python = (source.checkout, source.python) if source else (self.dimos_dir, None)
        try:
            return await self.cache.get(
                key,
                constants.INTROSPECT_CACHE_S,
                lambda: introspect.run_child(dimos_dir, args, stdin, python=python),
            )
        except introspect.IntrospectError as error:
            # introspect.py also runs alone in other checkouts, so its error can't import errors.py's StatusError
            raise ApiError(500, str(error))

    async def relist(self) -> list[str]:
        try:
            self.listed = (await introspect.run_child(self.dimos_dir, ["list"]))["blueprints"]
        except introspect.IntrospectError as error:
            # a checkout mid-edit can fail to list: keep the last listing, and say why
            logger.warning("listing the blueprints failed", error=str(error))
        return [entry["name"] for entry in self.listed or []]

    def blueprints_changed(self, added: list[str], removed: list[str]) -> None:
        self.cache = introspect.Cache()
        self.bus.send({"type": "blueprints", "added": added, "removed": removed})


def site_dirs(dimos_dir: Path) -> list[Path]:
    found = sorted((store.venv_dir(dimos_dir) / "lib").glob("python*/site-packages"))
    return found or [Path(sysconfig.get_paths()["purelib"])]


def new_state(dimos_dir: Path, bus: events.Bus | None = None) -> State:
    bus = bus or events.Bus()
    jobs = Jobs()
    return State(dimos_dir=dimos_dir, bus=bus, scanner=Scanner(dimos_dir, jobs), jobs=jobs)


def gateway_state(request: Request) -> State:
    state: State = request.app.state.gateway
    return state


GatewayState = Annotated[State, Depends(gateway_state)]

FreshQuery = Annotated[bool, BeforeValidator(lambda value: True if value == "" else value), Query()]
