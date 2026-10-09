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
        "/dimos/topics/rates",
        response_model=models.TopicRates,
        **route_doc(
            "server",
            "Every topic on the bus the gateway has heard since it started: rate, throughput, messages, last heard",
            "The gateway listens to zenoh `dimos/**` from its start, so a topic published once (at a blueprint's startup) is listed too, and a topic stays listed after it goes quiet (0 Hz, `lastSeen` seconds ago). Rates are over the last 2 s. Not listed: RPC calls (zenoh queries) and LCM-only traffic. `up` is false (with `error`) when the gateway has no zenoh session. No side effects.",
            answer="`{ up, error, topics: [{ topic, type, hz, bps, messages, lastSeen, declared }] }`",
        ),
    )
    async def topic_rates() -> dict[str, Any]:
        if s.topics is None:
            return {"up": False, "error": "this gateway isn't listening to the bus", "topics": []}
        return await asyncio.to_thread(s.topics.snapshot)
