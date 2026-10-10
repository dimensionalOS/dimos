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

from experimental.gateway import events, introspect, store
from experimental.gateway.discovery import Jobs, Scanner
from experimental.gateway.skills import Skills
from experimental.gateway.topic_rates import TopicWatch
from experimental.gateway.uploads import Uploads

API_VERSION = "2.0.0"
INTROSPECT_TTL_S = 600.0


class ApiError(Exception):
    def __init__(self, status: int, message: str) -> None:
        super().__init__(message)
        self.status = status


@dataclass
class State:
    dimos_dir: Path
    bus: events.Bus
    uploads: Uploads
    scanner: Scanner
    jobs: Jobs = field(default_factory=Jobs)
    skills: Skills = field(default_factory=Skills)
    cache: introspect.Cache = field(default_factory=introspect.Cache)
    listed: list[dict[str, Any]] | None = None
    topics: TopicWatch | None = None
    exit: Callable[[], None] = lambda: os._exit(0)
    started_at: int = field(default_factory=lambda: int(time.time()))
    background: list[asyncio.Task[Any]] = field(default_factory=list)

    async def introspected(self, args: list[str], stdin: Any = None) -> Any:
        key = " ".join(args) + (repr(stdin) if stdin is not None else "")

        async def compute() -> Any:
            return await introspect.run_child(self.dimos_dir, args, stdin)

        try:
            return await self.cache.get(key, INTROSPECT_TTL_S, compute)
        except introspect.IntrospectError as error:
            raise ApiError(500, str(error))

    async def relist(self) -> list[str]:
        try:
            self.listed = (await introspect.run_child(self.dimos_dir, ["list"]))["blueprints"]
        except introspect.IntrospectError:
            pass
        return [entry["name"] for entry in self.listed or []]

    def blueprints_changed(self, added: list[str], removed: list[str]) -> None:
        self.cache = introspect.Cache()
        self.scanner.invalidate()
        self.bus.send({"type": "blueprints", "added": added, "removed": removed})


def site_dirs(dimos_dir: Path) -> list[Path]:
    found = sorted((store.venv_dir(dimos_dir) / "lib").glob("python*/site-packages"))
    return found or [Path(sysconfig.get_paths()["purelib"])]


def new_state(dimos_dir: Path, bus: events.Bus | None = None) -> State:
    bus = bus or events.Bus()
    uploads = Uploads(bus, store.gateway_dir() / "uploads.json")
    return State(dimos_dir=dimos_dir, bus=bus, uploads=uploads, scanner=Scanner(dimos_dir))


def gateway_state(request: Request) -> State:
    state: State = request.app.state.gateway
    return state


GatewayState = Annotated[State, Depends(gateway_state)]

FreshQuery = Annotated[bool, BeforeValidator(lambda value: True if value == "" else value), Query()]
