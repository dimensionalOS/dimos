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
import io
import itertools
import json
from pathlib import Path
from typing import Any

from dimos.utils.logging_config import setup_logger
from experimental.gateway.errors import ApiError
from experimental.gateway.events import Bus

logger = setup_logger()


class AgentClient:
    """One `dimos agent sdk serve` process: HTTP calls and the agent's events share its connection."""

    def __init__(self, bus: Bus) -> None:
        self.bus = bus
        self.checkout: Path | None = None
        self.proc: asyncio.subprocess.Process | None = None
        self.reader: asyncio.Task[None] | None = None
        self.pending: dict[int, asyncio.Future[dict[str, Any]]] = {}
        self.ids = itertools.count()
        self.lock = asyncio.Lock()

    def running(self, checkout: Path) -> bool:
        return self.proc is not None and self.proc.returncode is None and self.checkout == checkout

    async def start(self, checkout: Path) -> None:
        async with self.lock:
            if self.running(checkout):
                return
            await self.close()
            script = checkout / "dimos/agents/dist/cli.js"
            if not script.is_file():
                raise ApiError(
                    503, "The DimOS Agent client is missing. Build it with dimos agent version"
                )
            try:
                self.proc = await asyncio.create_subprocess_exec(
                    "node",
                    str(script.resolve()),
                    "sdk",
                    "serve",
                    cwd=checkout,
                    stdin=asyncio.subprocess.PIPE,
                    stdout=asyncio.subprocess.PIPE,
                )
            except OSError as exc:
                raise ApiError(503, f"Could not start the DimOS Agent client: {exc}") from exc
            self.checkout = checkout
            self.reader = asyncio.create_task(self.read(self.proc))

    async def read(self, proc: asyncio.subprocess.Process) -> None:
        assert proc.stdout is not None
        # Lines can carry base64 images, so split them here instead of under StreamReader's line limit.
        line = bytearray()
        while chunk := await proc.stdout.read(io.DEFAULT_BUFFER_SIZE):
            first, *rest = chunk.split(b"\n")
            line += first
            for piece in rest:
                self.dispatch(json.loads(line))
                line = bytearray(piece)
        code = await proc.wait()
        if self.proc is proc:
            logger.warning(
                "DimOS Agent client exited; the next /agent request restarts it", code=code
            )
        for future in self.pending.values():
            if not future.done():
                future.set_exception(ApiError(503, "The DimOS Agent client exited"))

    def dispatch(self, message: dict[str, Any]) -> None:
        if "reply" not in message:
            self.bus.send(message)
            return
        future = self.pending.get(message.pop("reply"))
        if future is not None and not future.done():
            future.set_result(message)

    async def call(
        self, checkout: Path, operation: str, args: Any = None, conversation: int | None = None
    ) -> Any:
        await self.start(checkout)
        assert self.proc is not None and self.proc.stdin is not None
        request = next(self.ids)
        future = asyncio.get_running_loop().create_future()
        self.pending[request] = future
        try:
            line = {"id": request, "operation": operation, "args": args or {}}
            if conversation is not None:
                line["conversation"] = conversation
            self.proc.stdin.write(json.dumps(line).encode() + b"\n")
            await self.proc.stdin.drain()
            reply = await future
        finally:
            del self.pending[request]
        if "error" in reply:
            raise ApiError(reply["status"], reply["error"])
        return reply["value"]

    async def close(self) -> None:
        proc, reader, self.proc, self.reader = self.proc, self.reader, None, None
        if proc is None:
            return
        if proc.returncode is None and proc.stdin is not None:
            proc.stdin.close()
        await proc.wait()
        if reader is not None:
            await reader
