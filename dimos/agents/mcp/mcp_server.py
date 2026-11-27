# Copyright 2025-2026 Dimensional Inc.
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
import base64
from collections.abc import AsyncIterator, Callable
from concurrent.futures import ThreadPoolExecutor
from contextlib import suppress
import inspect
import json
import os
import sys
import time
from typing import TYPE_CHECKING, Annotated, Any
import uuid

from aiohttp import web
from pydantic import Field, WithJsonSchema

from dimos.agents.annotation import skill
from dimos.agents.capabilities import CapabilityRegistry
from dimos.agents.mcp import tool_stream
from dimos.agents.mcp.constants import DEFAULT_MCP_PORT, mcp_url
from dimos.agents.skill_result import SkillResult
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.rpc_client import RpcCall, RPCClient
from dimos.core.transport_factory import make_transport
from dimos.utils.logging_config import setup_logger
from dimos.utils.safe_thread_map import safe_thread_map

if sys.version_info >= (3, 11):
    import aiohttp_mcp_server as mcp
    from aiohttp_sse import sse_response

if TYPE_CHECKING:
    from dimos.core.module import SkillInfo

logger = setup_logger()


_SSE_KEEPALIVE_INTERVAL = 20.0  # seconds

# How long a `tools/call` waits for a capability held by a short, self-completing
# (instant) skill before refusing. Well under the MCP client's 120s HTTP timeout.
# Background holders run until stopped, so they are never waited on (see
# `_can_wait` in `McpServer._call_skill`).
DEFAULT_CAP_ACQUIRE_TIMEOUT = 30.0  # seconds

_SHUTDOWN_TIMEOUT = 2.0  # seconds


def _inline_refs(schema: Any, defs: dict[str, Any], inside: frozenset[str] = frozenset()) -> Any:
    """`schema` with each `$ref` replaced by the definition it names.

    A definition that refers to itself is left open there: it accepts anything.
    """
    if isinstance(schema, list):
        return [_inline_refs(item, defs, inside) for item in schema]
    if not isinstance(schema, dict):
        return schema
    inlined = {k: _inline_refs(v, defs, inside) for k, v in schema.items() if k != "$ref"}
    ref = schema.get("$ref")
    if ref is not None and ref not in inside:
        definition = defs[ref.removeprefix("#/$defs/")]
        inlined = {**_inline_refs(definition, defs, inside | {ref}), **inlined}
    return inlined


def _content(blocks: list[dict[str, Any]]) -> list[mcp.Content]:
    """`agent_encode()` blocks as MCP content: text, or an image out of its data URL."""
    content: list[mcp.Content] = []
    for block in blocks:
        if block["type"] == "text":
            content.append(mcp.TextContent(block["text"]))
        else:
            mime_type, data = block["image_url"]["url"].removeprefix("data:").split(";base64,")
            content.append(mcp.ImageContent(base64.b64decode(data), mime_type))
    return content


class McpServerConfig(ModuleConfig):
    host: str = Field(default_factory=lambda m: m["g"].listen_host)
    port: int = Field(default=DEFAULT_MCP_PORT, ge=1, le=65535)


class McpServer(Module):
    config: McpServerConfig
    _tool_stream_cleanup: Callable[[], None] | None = None

    @classmethod
    def check_dependencies(cls, **kwargs: Any) -> None:
        if sys.version_info < (3, 11):
            raise ImportError(
                "McpServer needs Python 3.11 or newer: aiohttp-mcp-server does not support 3.10."
            )

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._skills: list[SkillInfo] = []
        self._skills_by_name: dict[str, SkillInfo] = {}
        self._cap_registry = CapabilityRegistry()
        self._cap_acquire_timeout = DEFAULT_CAP_ACQUIRE_TIMEOUT
        self._event_queues: set[asyncio.Queue[dict[str, Any] | None]] = set()
        self._mcp_tools = mcp.ToolRegistry(ttl=0, rate_limiter=None, dynamic=True)
        self._executor = ThreadPoolExecutor(thread_name_prefix="McpServer")

    async def main(self) -> AsyncIterator[None]:
        app = web.Application()
        mcp.setup(app, name="dimensional", version="1.0.0", tools=self._mcp_tools)
        # TODO(sam): Remove /mcp compatibility hack.
        app.router.add_get("/mcp", self._event_stream)
        runner = web.AppRunner(app, access_log=None, shutdown_timeout=_SHUTDOWN_TIMEOUT)
        await runner.setup()
        await web.TCPSite(runner, self.config.host, self.config.port).start()
        yield
        for queue in self._event_queues:
            queue.put_nowait(None)
        await runner.cleanup()
        self._executor.shutdown(wait=False, cancel_futures=True)

    @rpc
    def start(self) -> None:
        super().start()
        self._tool_stream_cleanup = tool_stream.subscribe(self._on_tool_stream)

    @rpc
    def stop(self) -> None:
        if self._tool_stream_cleanup is not None:
            self._tool_stream_cleanup()
            self._tool_stream_cleanup = None
        super().stop()

    @rpc
    async def on_system_modules(self, modules: list[RPCClient]) -> None:
        # TODO: this is a bit hacky
        assert self.rpc is not None
        # One RPC per module, in parallel: a worker still warming up blocks
        # its call on the langchain import.
        per_module = await asyncio.get_event_loop().run_in_executor(
            self._executor, safe_thread_map, modules, lambda module: module.get_skills() or []
        )
        self._skills = [skill_info for skills in per_module for skill_info in skills]
        self._skills_by_name = {s.func_name: s for s in self._skills}
        rpc_calls = {
            skill_info.func_name: RpcCall(
                None, self.rpc, skill_info.func_name, module.remote_name, []
            )
            for module, skills in zip(modules, per_module, strict=True)
            for skill_info in skills
        }
        self._mcp_tools.replace(
            *(self._tool(self._skills_by_name[name], call) for name, call in rpc_calls.items())
        )

    def _tool(self, skill_info: SkillInfo, rpc_call: RpcCall) -> mcp.Tool:
        """The MCP tool that calls a skill in its module.

        A tool's schema comes from its handler's signature. The skill's own signature
        stays in its module's worker (see `RPCClient.cbor_encode`), so the handler is
        given one built from the skill's schema.
        """
        schema = json.loads(skill_info.args_schema)
        required = schema.get("required", ())

        async def handler(request: mcp.MCPRequest, /, **arguments: Any) -> object:
            return await self._call_skill(skill_info, rpc_call, dict(request.meta), arguments)

        parameters = [inspect.Parameter("request", inspect.Parameter.POSITIONAL_ONLY)]
        annotations: dict[str, Any] = {"request": mcp.MCPRequest}
        properties = _inline_refs(schema.get("properties", {}), schema.get("$defs", {}))
        for name, prop in properties.items():
            default = inspect.Parameter.empty if name in required else ...
            parameters.append(
                inspect.Parameter(name, inspect.Parameter.KEYWORD_ONLY, default=default)
            )
            annotations[name] = Annotated[Any, WithJsonSchema(prop)]
        handler.__signature__ = inspect.Signature(parameters)  # type: ignore[attr-defined]
        handler.__annotations__ = annotations
        return mcp.Tool(handler, name=skill_info.func_name, description=schema.get("description"))

    async def _call_skill(
        self, skill_info: SkillInfo, rpc_call: RpcCall, meta: dict[str, Any], args: dict[str, Any]
    ) -> str | list[mcp.Content] | mcp.ToolResult[Any]:
        name = skill_info.func_name
        progress_token = meta.get("progressToken")
        uses = list(skill_info.uses)
        cap_registry = self._cap_registry

        acquire_token = uuid.uuid4().hex
        if uses:

            def _can_wait(holder: str) -> bool:
                info = self._skills_by_name.get(holder)
                return (info.lifecycle if info is not None else "instant") != "background"

            # Run the (possibly blocking) acquire off the event loop so waiting for a
            # busy capability doesn't stall the server.
            conflict = await asyncio.get_event_loop().run_in_executor(
                self._executor,
                lambda: cap_registry.acquire(
                    uses,
                    tool_name=name,
                    token=acquire_token,
                    timeout=self._cap_acquire_timeout,
                    can_wait=_can_wait,
                ),
            )
            if conflict is not None:
                cap, holder = conflict
                logger.info(
                    "MCP tool refused (capability busy)",
                    tool=name,
                    cap=cap,
                    holder=holder,
                    snapshot=cap_registry.snapshot(),
                )
                holder_info = self._skills_by_name.get(holder)
                holder_lifecycle = holder_info.lifecycle if holder_info is not None else "instant"
                if holder_lifecycle == "background":
                    advice = "Call the appropriate stop tool first, then retry."
                else:
                    advice = "It is taking longer than expected; wait a moment and then retry."
                return f"Cannot start '{name}': capability '{cap}' is held by '{holder}'. {advice}"

        logger.info("MCP tool call", tool=name, args=args, progress_token=progress_token)
        t0 = time.monotonic()

        # _mcp_context is a reserved kwarg consumed by the `@skill` wrapper; it never
        # reaches the user-visible skill signature.
        call_kwargs = dict(args)
        mcp_context: dict[str, Any] = {"meta": meta}
        if progress_token is not None:
            mcp_context["progress_token"] = progress_token
        if uses:
            mcp_context["acquire_token"] = acquire_token
        call_kwargs["_mcp_context"] = mcp_context

        caps_held = bool(uses)
        try:
            try:
                result = await asyncio.get_event_loop().run_in_executor(
                    self._executor, lambda: rpc_call(**call_kwargs)
                )
            except Exception as e:
                logger.exception(
                    "MCP tool error", tool=name, duration=f"{time.monotonic() - t0:.3f}s"
                )
                raise mcp.ToolError(f"Error running tool '{name}': {type(e).__name__}: {e}") from e

            if skill_info.lifecycle == "background":
                # Hand ownership of the caps off to the tool-stream lifecycle.
                caps_held = False
        finally:
            if caps_held:
                cap_registry.release_by_token(acquire_token)

        duration = f"{time.monotonic() - t0:.3f}s"
        response = str(result)[:200]

        if isinstance(result, SkillResult):
            content = _content(result.agent_encode())
            structured = result.to_json()
            logger.info("MCP tool done", tool=name, duration=duration, response=response)
            try:
                # JSON has no NaN or Infinity, so they cannot go in structuredContent; the
                # text content tolerates them, so the call still answers with it alone.
                json.dumps(structured, allow_nan=False)
            except ValueError as e:
                logger.warning("MCP tool result has no strict JSON form", tool=name, error=str(e))
                return content
            return mcp.ToolResult(content, structured)

        if hasattr(result, "agent_encode"):
            logger.info("MCP tool done", tool=name, duration=duration, response=response)
            return _content(result.agent_encode())

        logger.info("MCP tool done", tool=name, duration=duration, response=response)
        return str(result)

    def _on_tool_stream(self, msg: dict[str, Any]) -> None:
        """Tool-stream subscriber callback: forward a frame to every active SSE client.

        Also releases capabilities held by a background skill when its tool-stream
        closes (signaled by a ``dimos/tool_stopped`` frame).
        """
        if msg.get("method") == tool_stream.TOOL_STREAM_STOPPED_METHOD:
            params = msg.get("params") or {}
            token = params.get("token")
            if token:
                released = self._cap_registry.release_by_token(token)
                if released:
                    logger.info(
                        "Capabilities released on tool-stream stop",
                        holder=params.get("tool_name"),
                        token=token,
                        released=released,
                    )
        loop = self._loop
        if loop is None or not loop.is_running():
            return
        loop.call_soon_threadsafe(self._fan_out, msg)

    def _fan_out(self, msg: dict[str, Any]) -> None:
        for queue in self._event_queues:
            queue.put_nowait(msg)

    async def _event_stream(self, request: web.Request) -> web.StreamResponse:
        """Persistent server-to-client SSE channel for tool-stream frames.

        MCP 2026-07-28 has no such channel, so this is a route of our own beside the
        MCP endpoint. Every tool-stream update is fanned out here, so the subscription
        is live for the full client session and independent of any particular
        ``tools/call`` request.
        """
        queue: asyncio.Queue[dict[str, Any] | None] = asyncio.Queue()
        self._event_queues.add(queue)
        try:
            async with sse_response(request, ping_interval=_SSE_KEEPALIVE_INTERVAL) as response:
                while response.is_connected():
                    # A wait that times out checks the connection again, and a send
                    # that fails has closed it.
                    with suppress(TimeoutError, ConnectionResetError):
                        msg = await asyncio.wait_for(queue.get(), _SSE_KEEPALIVE_INTERVAL)
                        if msg is None:
                            break
                        await response.send(json.dumps(msg), event="message")
        finally:
            self._event_queues.discard(queue)
        return response

    @skill
    def server_status(self) -> str:
        """Get MCP server status: main process PID, deployed modules, and skill count."""
        from dimos.core.run_registry import get_most_recent

        skills = self._skills
        modules = list(dict.fromkeys(s.class_name for s in skills))
        entry = get_most_recent()
        pid = entry.pid if entry else os.getpid()
        return json.dumps(
            {
                "pid": pid,
                "modules": modules,
                "skills": [s.func_name for s in skills],
            }
        )

    @skill
    def list_modules(self) -> str:
        """List deployed modules and their skills."""
        skills = self._skills
        modules: dict[str, list[str]] = {}
        for s in skills:
            modules.setdefault(s.class_name, []).append(s.func_name)
        return json.dumps({"modules": modules})

    @skill
    def agent_send(self, message: str) -> str:
        """Send a message to the running DimOS agent over the active transport."""
        if not message:
            raise ValueError("Message cannot be empty")

        transport = make_transport("/human_input")
        try:
            transport.start()
            transport.publish(message)
            return f"Message sent to agent: {message[:100]}"
        finally:
            transport.stop()

    @rpc
    def mcp_url(self) -> str:
        """The HTTP endpoint for clients on this host."""
        return mcp_url(self.config.host, self.config.port)
