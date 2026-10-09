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
from typing import Any

from fastapi import FastAPI

from experimental.gateway.server.openapi import route_doc
from experimental.gateway.server.state import ServerState
from experimental.gateway.utils import models


def register(app: FastAPI, state: ServerState) -> None:
    s = state

    @app.get(
        "/dimos/robots",
        response_model=models.Robots,
        **route_doc(
            "blueprints",
            "Every robot dimos supports and its blueprints: title, description, the settings to decide before running each (recommended_config), starter picks, recommended blueprints, hidden ones",
            "The checkout's experimental/gateway/annotations.json (dimos.yaml's `robots:`, also readable per tag without a server) with its defaults applied: each blueprint's recommended_config, every setting resolved: one config value (`key`, `scope`: global keys are `--key value` before `run`, module keys after the blueprint name) or a pick (`kind` pick: each choice `set`s several values, e.g. Robot / Replay / Simulator), either with `choices` (an enum) and `when` (shown only while those values hold); robots' `type` is a key of `types`, the order a launcher groups them in. `registered` and `unlisted` compare it with the blueprint registry. CI keeps the file in step with the code (experimental/gateway/robots.py). Read from disk on every call; no side effects.",
            agent=True,
            answer="`{ about, types, robots: { [id]: { name, description, type, manufacturer, dirs, recommended, blueprints: { [name]: { title, description, starter, hidden, recommended_config, robot, registered } } } }, excluded, unlisted }`",
        ),
    )
    async def robot_list() -> dict[str, Any]:
        from experimental.gateway.utils import robots

        def compute() -> dict[str, Any]:
            from dimos.robot.all_blueprints import all_blueprints

            path = s.dimos_dir / "experimental" / "gateway" / "annotations.json"
            return robots.resolved(
                robots.load(path if path.exists() else robots.ROBOTS_FILE), all_blueprints
            )

        return await asyncio.to_thread(compute)
