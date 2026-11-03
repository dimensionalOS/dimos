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

"""Robot-side descriptor adapter. Not included in the standalone runtime."""

import asyncio
from collections.abc import AsyncIterator, Sequence
from contextlib import suppress
import secrets
import threading
from typing import Any

from pydantic import ValidationError
import zenoh

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.core import rpc
from dimos.core.introspection.module.info import StreamDescriptor
from dimos.core.module import Module
from dimos.core.rpc_client import RPCClient
from dimos.core.transport import ZenohTransport, pZenohTransport
from dimos.protocol.service.zenohservice import ZenohService
from dimos.utils.logging_config import setup_logger
from dimos.web.relay_bridge.topic_metadata import (
    MAX_METADATA_BYTES,
    MAX_METADATA_ENTRIES,
    METADATA_PREFIX,
    METADATA_REFRESH_SECONDS,
    DescriptorSnapshot,
)
from dimos.web.relay_bridge.topic_protocol import TopicDescriptor

logger = setup_logger()


def describe_stream(stream: StreamDescriptor) -> TopicDescriptor | None:
    """Use exact bound transport keys; never build a transport or guess a domain."""
    transport = stream.transport.cls
    if not issubclass(transport, (ZenohTransport, pZenohTransport)):
        return None
    try:
        expression = zenoh.KeyExpr(stream.channel)
        if "*" in str(expression):
            return None
        cbor = issubclass(transport, pZenohTransport)
        return TopicDescriptor(
            key=stream.channel,
            encoding="application/cbor" if cbor else "application/x-lcm",
            type=stream.type_name if cbor else getattr(stream.type, "msg_name", stream.type_name),
            availability="known",
        )
    except (zenoh.ZError, ValidationError):
        return None


class DescriptorPublisher:
    """Coalesces lifecycle notifications; at most one descriptor RPC worker.

    No RPC runs on the module's RPC-serving thread. Own proxies are skipped,
    each call has a one-second deadline, and stop prevents further calls.
    """

    def __init__(self, own_name: str) -> None:
        self._own_name = own_name
        self._owner = secrets.token_hex(16)
        self._key = METADATA_PREFIX + self._owner
        self._lock = threading.Lock()
        self._modules: tuple[RPCClient, ...] = ()
        self._revision = 0
        self._changed = asyncio.Event()
        self._stopped = threading.Event()
        self._loop: asyncio.AbstractEventLoop | None = None
        self._payload = b""

    def update(self, modules: Sequence[RPCClient]) -> None:
        with self._lock:
            self._modules = tuple(modules[:256])
            self._revision += 1
        if self._loop is not None:
            self._loop.call_soon_threadsafe(self._changed.set)

    def _collect(self, modules: tuple[RPCClient, ...], revision: int) -> tuple[bytes, bool]:
        entries: dict[str, TopicDescriptor] = {}
        failed = False
        for module in modules:
            if self._stopped.is_set():
                break
            if module.remote_name == self._own_name or "get_stream_descriptors" not in module.rpcs:
                continue
            try:
                streams, release = module.rpc.call_sync(
                    f"{module.remote_name}/get_stream_descriptors", ((), {}), rpc_timeout=1
                )
                try:
                    for stream in streams:
                        descriptor = describe_stream(stream)
                        if descriptor is not None:
                            entries[descriptor.key] = descriptor
                        if len(entries) >= MAX_METADATA_ENTRIES:
                            break
                finally:
                    release()
            except Exception:
                # A module may be unloading while this snapshot is collected.
                failed = True
            if len(entries) >= MAX_METADATA_ENTRIES:
                break
        snapshot = DescriptorSnapshot(
            owner=self._owner, revision=revision, entries=list(entries.values())
        )
        payload = snapshot.model_dump_json(exclude_none=True).encode()
        while len(payload) > MAX_METADATA_BYTES:
            snapshot.entries = snapshot.entries[: len(snapshot.entries) // 2]
            payload = snapshot.model_dump_json(exclude_none=True).encode()
        return payload, failed

    def _query(self, query: zenoh.Query) -> None:
        if self._payload and not self._stopped.is_set():
            query.reply(self._key, self._payload, encoding=zenoh.Encoding.APPLICATION_JSON)

    async def run(self, session: zenoh.Session) -> None:
        self._stopped.clear()
        self._loop = asyncio.get_running_loop()
        self._changed.set()
        # Declarations only register local handlers; no remote RPC is waited on.
        queryable = session.declare_queryable(self._key, self._query)
        token = None
        retry = False
        try:
            token = session.liveliness().declare_token(self._key)
            while True:
                if self._changed.is_set() or retry:
                    self._changed.clear()
                    with self._lock:
                        if retry:
                            self._revision += 1
                        modules, revision = self._modules, self._revision
                    collecting = asyncio.create_task(
                        asyncio.to_thread(self._collect, modules, revision)
                    )
                    try:
                        payload, retry = await asyncio.shield(collecting)
                    except asyncio.CancelledError:
                        self._stopped.set()
                        await collecting  # at most the current one-second RPC
                        raise
                    with self._lock:
                        if revision != self._revision:
                            continue  # a newer load/unload supersedes this result
                    self._payload = payload
                if self._payload:
                    try:
                        await asyncio.to_thread(
                            session.put,
                            self._key,
                            self._payload,
                            encoding=zenoh.Encoding.APPLICATION_JSON,
                        )
                    except zenoh.ZError:
                        logger.warning("topic metadata put failed; retrying")
                with suppress(asyncio.TimeoutError):
                    await asyncio.wait_for(self._changed.wait(), METADATA_REFRESH_SECONDS)
        finally:
            self._stopped.set()
            self._loop = None
            try:
                if token is not None:
                    token.undeclare()
            finally:
                queryable.undeclare()


class TopicMetadataModule(Module):
    """Announce wired Zenoh descriptors without a Cockpit or relay connection."""

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        name = self.config.rpc_name or self.config.instance_name or type(self).__name__
        self._publisher = DescriptorPublisher(name)

    @rpc
    def on_system_modules(self, modules: list[RPCClient]) -> None:
        """Schedule a refresh after modules load, unload or reconnect."""
        self._publisher.update(modules)

    async def main(self) -> AsyncIterator[None]:
        service = ZenohService()
        await asyncio.to_thread(service.start)
        task = asyncio.create_task(self._publisher.run(service.session))
        try:
            yield
        finally:
            task.cancel()
            with suppress(asyncio.CancelledError):
                await task
            service.stop()  # the existing session pool owns the shared socket


def with_topic_metadata(blueprint: Blueprint) -> Blueprint:
    if any(
        isinstance(atom.module, type) and issubclass(atom.module, TopicMetadataModule)
        for atom in blueprint.blueprints
    ):
        return blueprint
    return autoconnect(blueprint, TopicMetadataModule.blueprint())
