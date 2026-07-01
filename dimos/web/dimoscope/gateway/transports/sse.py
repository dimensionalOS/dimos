#!/usr/bin/env python3
# SSE (Server-Sent Events) bench transport — base64([f64 send-ms][LC02]) lines over text/event-stream,
# one long-lived GET per client. READ-ONLY. Taps the shared Bus directly. Mounts at /sse.
from __future__ import annotations

import asyncio
import base64
import json

from starlette.requests import Request
from starlette.responses import StreamingResponse

from ..bus import Bus, Sample
from ._common import frame, subs_from_query, wants


class _SseClient:
    __slots__ = ("q", "subs")

    def __init__(self, subs: set[str]) -> None:
        self.q: asyncio.Queue = asyncio.Queue(maxsize=4096)
        self.subs = subs


class SsePlane:
    def __init__(self, bus: Bus) -> None:
        self.bus = bus
        self.clients: set[_SseClient] = set()
        bus.subscribe(self._on_sample)

    def _on_sample(self, s: Sample) -> None:
        if not self.clients:
            return
        line: bytes | None = None
        for c in self.clients:
            if not wants(c.subs, s.topic):
                continue
            if line is None:
                line = b"data: " + base64.b64encode(frame(s)) + b"\n\n"
            try:
                c.q.put_nowait(line)
            except asyncio.QueueFull:
                pass

    async def handle(self, request: Request) -> StreamingResponse:
        subs = subs_from_query(request.query_params.get("topics"))
        client = _SseClient(subs)
        self.clients.add(client)
        hello = (
            "event: hello\ndata: "
            + json.dumps({"topics": self.bus.topic_list(), "label": "dimoscope/SSE"})
            + "\n\n"
        ).encode()

        async def gen():
            yield hello
            try:
                while True:
                    yield await client.q.get()
            finally:
                self.clients.discard(client)

        return StreamingResponse(
            gen(),
            media_type="text/event-stream",
            headers={"cache-control": "no-cache", "access-control-allow-origin": "*"},
        )
