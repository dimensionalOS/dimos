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
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from experimental.gateway.utils import (
    blueprints,
    config,
    events,
)
from experimental.gateway.utils.blueprint_watch import BlueprintWatch
from experimental.gateway.utils.discovery import Discovery
from experimental.gateway.utils.jobs import Jobs
from experimental.gateway.utils.topic_rates import TopicWatch
from experimental.gateway.utils.uploads import Uploads


@dataclass
class ServerState:
    dimos_dir: Path
    bus: events.Bus
    uploads: Uploads
    cache: blueprints.Cache = field(default_factory=blueprints.Cache)
    background: list[asyncio.Task[None]] = field(default_factory=list)
    exit: Any = None
    discovery: Discovery | None = None
    jobs: Jobs | None = None
    zenoh_namespace: str | None = None
    topics: TopicWatch | None = None
    zenoh_connect: list[str] = field(default_factory=list)
    watch: BlueprintWatch | None = None


def default_state(dimos_dir: Path) -> ServerState:
    bus = events.Bus()
    config.state_file("uploaded.json")
    uploads = Uploads(
        dimos_dir, bus, config.state_file("uploads.json"), config.logs_dir() / "uploads.log"
    )
    return ServerState(dimos_dir=dimos_dir, bus=bus, uploads=uploads)
