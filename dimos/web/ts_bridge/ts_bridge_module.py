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

"""TypeScript API bridge.

Mirrors the rerun bridge: subscribes to every topic on the configured pubsubs
and exposes them over a websocket + HTTP server for the `@dimos/client` browser
library, and hosts `@web_module` static bundles.

Wire format: the bridge is a byte pipe. It forwards each message in the bus's
own encoding (`pubsub.encode`) as a binary frame; the client decodes with the
matching codec (`@dimos/msgs` for LCM). The bridge never transcodes — there is
no bespoke server-side serialization, and nothing is LCM-specific here.
"""

from __future__ import annotations

import asyncio
from collections import deque
from collections.abc import AsyncIterator, Callable
from dataclasses import dataclass, field, replace
from functools import lru_cache, partial
import json
import mimetypes
from pathlib import Path
import threading
import time
from typing import Any, Literal, Protocol, runtime_checkable

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, Response
from reactivex.disposable import Disposable
import uvicorn

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.porcelain.remote_module_source import RemoteModuleSource
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.protocol.pubsub.patterns import Glob, pattern_matches
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

WS_PATH = "/ws"
CLIENT_DIR = Path(__file__).parent / "client"


@runtime_checkable
class EncodingPubSub(Protocol):
    """A pubsub the bridge can listen on and ask to encode messages to bytes.

    Params are positional-only so any pubsub matches regardless of arg names.
    """

    def subscribe_all(self, callback: Callable[[Any, Any], Any], /) -> Callable[[], None]: ...

    def encode(self, message: Any, topic: Any, /) -> bytes: ...


def _default_pubsubs() -> list[EncodingPubSub]:
    return [LCM()]


def topic_name(topic: Any) -> str:
    """Stream name for a topic, dropping the LCM `#type` suffix."""
    name = getattr(topic, "name", None) or getattr(topic, "topic", None) or str(topic)
    return str(name).split("#")[0]


def binary_frame(stream: str, payload: bytes) -> bytes:
    """Wire format for a message frame: uint16 name length, name, encoded payload."""
    name = stream.encode("utf-8")
    return len(name).to_bytes(2, "big") + name + payload


def media_type_for(route: str, data: bytes) -> str:
    """Content type for a bundled file: by extension, falling back to byte sniffing.

    Sniffing lets an extensionless `icon` entry be any image — including a
    high-res PNG or a resolution-independent SVG (`.ico` itself caps at 256x256).
    """
    guessed = mimetypes.guess_type(route)[0]
    if guessed:
        return guessed
    if data[:8] == b"\x89PNG\r\n\x1a\n":
        return "image/png"
    if data[:3] == b"\xff\xd8\xff":
        return "image/jpeg"
    if data[:4] == b"\x00\x00\x01\x00":
        return "image/x-icon"
    head = data[:512].lstrip()
    if head[:4] == b"<svg" or (head[:5] == b"<?xml" and b"<svg" in head):
        return "image/svg+xml"
    return "application/octet-stream"


def merge_tf(frames: dict[tuple[str, str], Any], message: TFMessage) -> TFMessage:
    """Accumulate a (possibly partial) TF message into the full tree.

    `/tf` publishers each send only their own transforms, so coalescing whole
    messages would drop frames. Merging by (parent, child) keeps the latest of
    every frame, so the complete tree always reaches the web.
    """
    for transform in message.transforms:
        frames[(transform.frame_id, transform.child_frame_id)] = transform
    return TFMessage(*frames.values())


def _json_safe(value: Any) -> Any:
    try:
        json.dumps(value)
        return value
    except TypeError:
        return str(value)


# --- filtering / exposure --------------------------------------------------


@lru_cache(maxsize=1024)
def _glob(pattern: str) -> Glob:
    return Glob(pattern)


def glob_match(pattern: str, name: str) -> bool:
    """Match a glob pattern string (e.g. `/sensor/*`, `Module.*`) against a name.

    Plain strings match exactly; `*`/`**`/`?` glob. Compiled globs are cached.
    """
    return pattern_matches(_glob(pattern), name)


def passes_filter(name: str, whitelist: list[str], blacklist: list[str]) -> bool:
    """Glob whitelist/blacklist gate. Empty lists allow everything."""
    if whitelist:
        return any(glob_match(pattern, name) for pattern in whitelist)
    if blacklist:
        return not any(glob_match(pattern, name) for pattern in blacklist)
    return True


def require_one(whitelist: list[str], blacklist: list[str], what: str) -> None:
    if whitelist and blacklist:
        raise ValueError(f"Only one of {what} whitelist or blacklist may be set, not both")


# Framework-internal RPCs that are never reachable from web clients, regardless
# of the operator's allow/deny lists (lifecycle, stream wiring, registration).
INTERNAL_RPCS = frozenset(
    {
        "start",
        "stop",
        "build",
        "set_transport",
        "connect_stream",
        "peek_stream",
        "web_register",
        "web_init",
        "log_blueprint_graph",
    }
)


def rpc_exposed(module: str, method: str, whitelist: list[str], blacklist: list[str]) -> bool:
    """Whether a web client may call `module.method`.

    Internal/private methods are always hidden. Patterns match a whole module
    (`"GO2Connection"`), an exact method (`"GO2Connection.standup"`), or a glob
    (`"GO2Connection.*"`).
    """
    if method in INTERNAL_RPCS or method.startswith("_"):
        return False

    def matches(pattern: str) -> bool:
        return pattern == module or glob_match(pattern, f"{module}.{method}")

    if whitelist:
        return any(matches(pattern) for pattern in whitelist)
    if blacklist:
        return not any(matches(pattern) for pattern in blacklist)
    return True


# --- QoS -------------------------------------------------------------------


@dataclass(frozen=True)
class QosProfile:
    """ROS2-flavoured QoS for one stream.

    - reliability: best_effort coalesces to the newest value (low latency, lossy);
      reliable buffers up to `depth` messages in order.
    - depth: outbox buffer size (history). Defaults to 1 (best_effort) or 10 (reliable).
    - durability: transient_local replays the last cached value to late joiners.
    - rate: max messages/second (None = unlimited).
    """

    reliability: Literal["best_effort", "reliable"] = "best_effort"
    durability: Literal["volatile", "transient_local"] = "volatile"
    depth: int = 1
    rate: float | None = None


DEFAULT_QOS = QosProfile()


def profile_from_dict(raw: dict[str, Any]) -> QosProfile:
    reliability = raw.get("reliability", "best_effort")
    depth = raw.get("depth")
    if depth is None:
        depth = 10 if reliability == "reliable" else 1
    return QosProfile(
        reliability=reliability,
        durability=raw.get("durability", "volatile"),
        depth=max(1, int(depth)),
        rate=raw.get("rate"),
    )


class ClientSubscription:
    """A connection's whitelist/blacklist + per-stream QoS, resolved lazily."""

    def __init__(
        self,
        whitelist: list[str],
        blacklist: list[str],
        qos: dict[str, dict[str, Any]],
        rate_limit: dict[str, float],
    ) -> None:
        require_one(whitelist, blacklist, "subscription")
        self.whitelist = whitelist
        self.blacklist = blacklist
        self._qos = {pattern: profile_from_dict(value) for pattern, value in qos.items()}
        self._rate_limit = rate_limit
        self._resolved: dict[str, QosProfile] = {}
        self._last_sent: dict[str, float] = {}

    def allows(self, stream: str) -> bool:
        return passes_filter(stream, self.whitelist, self.blacklist)

    def update_qos(self, qos: dict[str, dict[str, Any]]) -> None:
        """Set/replace QoS for the given stream patterns at runtime."""
        for pattern, value in qos.items():
            self._qos[pattern] = profile_from_dict(value)
        self._resolved.clear()

    def qos_for(self, stream: str) -> QosProfile:
        cached = self._resolved.get(stream)
        if cached is not None:
            return cached
        profile = DEFAULT_QOS
        for pattern, candidate in self._qos.items():
            if glob_match(pattern, stream):
                profile = candidate
                break
        if profile.rate is None:  # rateLimit is sugar for qos.rate
            for pattern, hz in self._rate_limit.items():
                if glob_match(pattern, stream):
                    profile = replace(profile, rate=hz)
                    break
        self._resolved[stream] = profile
        return profile

    def rate_ok(self, stream: str, now: float) -> bool:
        rate = self.qos_for(stream).rate
        if rate is None or rate <= 0:
            return True
        last = self._last_sent.get(stream)
        if last is not None and now - last < 1.0 / rate:
            return False
        self._last_sent[stream] = now
        return True


# --- per-connection state --------------------------------------------------


class _Connection:
    """One websocket client: filter + QoS and a per-stream outbox of encoded bytes."""

    def __init__(self, websocket: WebSocket, loop: asyncio.AbstractEventLoop) -> None:
        self.websocket = websocket
        self._loop = loop
        self.subscription: ClientSubscription | None = None
        self._pending: dict[str, deque[bytes]] = {}
        self._wake = asyncio.Event()

    def wants(self, stream: str, now: float) -> bool:
        """LCM-thread streaming decision (filter + rate)."""
        sub = self.subscription
        if sub is None:
            return False
        return sub.allows(stream) and sub.rate_ok(stream, now)

    def offer(self, stream: str, payload: bytes) -> None:
        """Hand an encoded message to this connection (from the LCM thread)."""
        self._loop.call_soon_threadsafe(self._accept, stream, payload)

    def _accept(self, stream: str, payload: bytes) -> None:
        sub = self.subscription
        if sub is None or not sub.allows(stream):
            return
        depth = sub.qos_for(stream).depth
        outbox = self._pending.get(stream)
        if outbox is None or outbox.maxlen != depth:
            outbox = deque(outbox or (), maxlen=depth)
            self._pending[stream] = outbox
        outbox.append(payload)
        self._wake.set()

    async def sender(self) -> None:
        """Drain buffered messages to the websocket as binary frames."""
        while True:
            await self._wake.wait()
            self._wake.clear()
            batch = self._pending
            self._pending = {}
            for stream, payloads in batch.items():
                for payload in payloads:
                    await self.websocket.send_bytes(binary_frame(stream, payload))


class DimosWebsocketConfig(ModuleConfig):
    pubsubs: list[EncodingPubSub] = field(default_factory=_default_pubsubs)
    port: int | None = None
    host: str | None = None
    # Backend exposure control (safety): what web clients are *ever* allowed to
    # see/call, independent of each client's own filter. Empty = allow all.
    whitelist: list[str] = field(default_factory=list)
    blacklist: list[str] = field(default_factory=list)
    rpc_whitelist: list[str] = field(default_factory=list)
    rpc_blacklist: list[str] = field(default_factory=list)


class DimosWebsocket(Module):
    """Bridge dimos streams + RPCs to browser clients over one websocket/HTTP server."""

    config: DimosWebsocketConfig
    dedicated_worker = True

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        require_one(self.config.whitelist, self.config.blacklist, "topic")
        require_one(self.config.rpc_whitelist, self.config.rpc_blacklist, "rpc")
        self._connections: set[_Connection] = set()
        self._connections_lock = threading.Lock()
        self._latest: dict[str, bytes] = {}  # newest encoded bytes per stream (transient_local)
        self._tf_frames: dict[str, dict[tuple[str, str], Any]] = {}  # per-stream TF tree
        self._module_source: RemoteModuleSource | None = None
        self._web_bundles: dict[str, dict[str, bytes]] = {}  # module name -> {route: bytes}

    def _topic_allowed(self, stream: str) -> bool:
        return passes_filter(stream, self.config.whitelist, self.config.blacklist)

    def _rpc_allowed(self, module: str, method: str) -> bool:
        return rpc_exposed(module, method, self.config.rpc_whitelist, self.config.rpc_blacklist)

    @property
    def host(self) -> str:
        return self.config.host or self.config.g.ts_api_host or self.config.g.listen_host

    @property
    def port(self) -> int:
        return self.config.port or self.config.g.ts_api_port

    async def main(self) -> AsyncIterator[None]:
        server = uvicorn.Server(
            uvicorn.Config(self._build_app(), host=self.host, port=self.port, log_level="error")
        )
        serve_task = asyncio.create_task(server.serve())

        for pubsub in self.config.pubsubs:
            if hasattr(pubsub, "start"):
                pubsub.start()  # type: ignore[attr-defined]
            unsub = pubsub.subscribe_all(partial(self._on_message, pubsub=pubsub))
            self.register_disposable(Disposable(unsub))
            if hasattr(pubsub, "stop"):
                self.register_disposable(Disposable(pubsub.stop))  # type: ignore[attr-defined]

        logger.info(f"DimosWebsocket listening on http://{self.host}:{self.port}")

        yield

        server.should_exit = True
        await serve_task
        if self._module_source is not None:
            self._module_source.close()

    def _encode(self, message: Any, topic: Any, pubsub: EncodingPubSub, stream: str) -> bytes:
        """Encode a message to the bus's wire bytes, aggregating TF into a full tree."""
        if isinstance(message, TFMessage):
            message = merge_tf(self._tf_frames.setdefault(stream, {}), message)
        return pubsub.encode(message, topic)

    def _on_message(self, message: Any, topic: Any, pubsub: EncodingPubSub) -> None:
        """LCM-thread callback: encode once, fan out to interested connections."""
        stream = topic_name(topic)
        if not self._topic_allowed(stream):
            return
        payload = self._encode(message, topic, pubsub, stream)
        self._latest[stream] = payload

        now = time.monotonic()
        with self._connections_lock:
            interested = [conn for conn in self._connections if conn.wants(stream, now)]
        for conn in interested:
            conn.offer(stream, payload)

    def _build_app(self) -> FastAPI:
        app = FastAPI()
        app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_methods=["*"],
            allow_headers=["*"],
        )

        @app.get("/dimos.json")
        async def dimos_json() -> dict[str, Any]:
            return {"host": self.host, "port": self.port, "wsPath": WS_PATH}

        @app.get("/config")
        async def config() -> dict[str, Any]:
            return self._jsonable_config()

        @app.get("/dimos.js")
        async def client() -> FileResponse:
            return FileResponse(CLIENT_DIR / "dimos.js", media_type="text/javascript")

        @app.websocket(WS_PATH)
        async def websocket_endpoint(websocket: WebSocket) -> None:
            await self._handle_websocket(websocket)

        # @web_module bundles — declared last so they don't shadow the routes above.
        @app.get("/{name}")
        async def web_root(name: str) -> Response:
            return self._serve_web_file(name, "index.html")

        @app.get("/{name}/{route:path}")
        async def web_file(name: str, route: str) -> Response:
            return self._serve_web_file(name, route or "index.html")

        return app

    def _serve_web_file(self, name: str, route: str) -> Response:
        bundle = self._web_bundles.get(name)
        if bundle is None or route not in bundle:
            return Response(status_code=404)
        data = bundle[route]
        return Response(data, media_type=media_type_for(route, data))

    @rpc
    def web_register(self, name: str, files: dict[str, bytes]) -> None:
        """Register a module's static bundle (served under /<name>). Called from @web_module."""
        self._web_bundles[name] = files
        logger.info(f"ts_bridge serving /{name} ({len(files)} files)")

    def _jsonable_config(self) -> dict[str, Any]:
        return self.config.g.model_dump(mode="json")

    def _list_modules(self) -> dict[str, list[str]]:
        # Streaming works without a Coordinator RPC service; RPC relay needs one
        # (present under `dimos run`). Degrade to no modules if it's unreachable.
        try:
            source = self._ensure_source()
            descriptors = source._coord.call("list_modules")
        except Exception:
            logger.debug("ts_bridge: coordinator RPC unavailable, RPC relay disabled")
            return {}
        modules: dict[str, list[str]] = {}
        for descriptor in descriptors:
            methods = [
                name
                for name in descriptor.rpc_names
                if self._rpc_allowed(descriptor.class_name, name)
            ]
            if methods:
                modules[descriptor.class_name] = methods
        return modules

    def _ensure_source(self) -> RemoteModuleSource:
        if self._module_source is None:
            self._module_source = RemoteModuleSource()
        return self._module_source

    async def _handle_websocket(self, websocket: WebSocket) -> None:
        await websocket.accept()
        assert self._loop is not None
        connection = _Connection(websocket, self._loop)
        sender_task = asyncio.create_task(connection.sender())
        try:
            while True:
                request = await websocket.receive_json()
                await self._handle_request(connection, request)
        except WebSocketDisconnect:
            pass
        except Exception:
            logger.debug("ts_bridge websocket error", exc_info=True)
        finally:
            sender_task.cancel()
            with self._connections_lock:
                self._connections.discard(connection)

    async def _handle_request(self, connection: _Connection, request: dict[str, Any]) -> None:
        kind = request.get("type")

        if kind == "hello":
            connection.subscription = ClientSubscription(
                whitelist=request.get("whitelist") or [],
                blacklist=request.get("blacklist") or [],
                qos=request.get("qos") or {},
                rate_limit=request.get("rateLimit") or {},
            )
            modules = await asyncio.to_thread(self._list_modules)
            # Send ready before registering so no stream frames precede it.
            await connection.websocket.send_json(
                {"type": "ready", "config": self._jsonable_config(), "modules": modules}
            )
            with self._connections_lock:
                self._connections.add(connection)
            self._deliver_transient_local(connection)

        elif kind == "set_qos":
            if connection.subscription is not None:
                connection.subscription.update_qos(request.get("qos") or {})

        elif kind == "rpc":
            await self._handle_rpc(connection, request)

    def _deliver_transient_local(self, connection: _Connection) -> None:
        """Replay the cached newest message for transient_local streams on join."""
        subscription = connection.subscription
        assert subscription is not None
        for stream, payload in list(self._latest.items()):
            if (
                subscription.allows(stream)
                and subscription.qos_for(stream).durability == "transient_local"
            ):
                connection.offer(stream, payload)

    async def _handle_rpc(self, connection: _Connection, request: dict[str, Any]) -> None:
        request_id = request.get("id")
        try:
            result = await asyncio.to_thread(
                self._call_rpc,
                request["module"],
                request["method"],
                request.get("args") or [],
                request.get("kwargs") or {},
            )
            await connection.websocket.send_json(
                {"type": "rpc_result", "id": request_id, "result": _json_safe(result)}
            )
        except Exception as error:
            await connection.websocket.send_json(
                {"type": "rpc_result", "id": request_id, "error": str(error)}
            )

    def _call_rpc(self, module: str, method: str, args: list[Any], kwargs: dict[str, Any]) -> Any:
        if not self._rpc_allowed(module, method):
            raise PermissionError(f"RPC {module}.{method} is not exposed")
        source = self._ensure_source()
        proxy = source.get_module(module)
        bound = getattr(proxy, method)
        return bound(*args, **kwargs)


dimos_websocket = DimosWebsocket.blueprint()
