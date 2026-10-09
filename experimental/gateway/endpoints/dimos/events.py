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

from fastapi import FastAPI
from fastapi.responses import StreamingResponse

from experimental.gateway.server.openapi import route_doc
from experimental.gateway.server.state import ServerState
from experimental.gateway.utils import runs
from experimental.gateway.utils.http import EventStreamResponse


def register(app: FastAPI, state: ServerState) -> None:
    s = state

    @app.get(
        "/dimos/events",
        deprecated=True,
        response_class=EventStreamResponse,
        **route_doc(
            "events",
            "The dimos gateway's live events (SSE): launch phases, warning+ log records, uploads, the cloud login",
            "Deprecated and internal, kept for one release: the same events are published on zenoh at `<ns>/dimos/events/<type>` (Desktop's docs/events.md), so listen there. The stream starts with the current `launch` event, then sends every event (a client that can't keep up loses events). Each event type's payload is a component schema (LaunchEvent, LogEvent, UploadEvent, UploadsEvent, UploadRemovedEvent, CloudLoginEvent; DimosEvent is any of them).",
            ok={
                "description": "Server-sent events, one `data: <DimosEvent JSON>` each; `:` keep-alives every 15 s",
                "content": {
                    "text/event-stream": {"schema": {"$ref": "#/components/schemas/DimosEvent"}}
                },
            },
        ),
    )
    async def event_stream() -> StreamingResponse:
        stream = s.bus.stream(lambda: {"type": "launch", "launch": runs.current_launch()})
        return EventStreamResponse(
            stream, headers={"Cache-Control": "no-cache", "Deprecation": "true"}
        )
