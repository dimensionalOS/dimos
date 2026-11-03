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

"""Pieces shared by the topic end-to-end tests: the provider and wire helpers."""

import asyncio
from collections.abc import AsyncIterator
from contextlib import asynccontextmanager, suppress
from typing import Any, TypeVar

import zenoh

from dimos.utils.asyncio_timeout import timeout
from dimos.web.relay_bridge.protocol import Msg
from dimos.web.relay_bridge.relay_process import RelayReadyInfo
from dimos.web.relay_bridge.topic_catalog import TopicCatalog
from dimos.web.relay_bridge.topic_metadata import MetadataReceiver
from dimos.web.relay_bridge.topic_profiles import MEDIA_PROFILES, PROFILE_OPTIONS
from dimos.web.relay_bridge.topic_protocol import TopicCapabilities
from dimos.web.relay_bridge.topic_put import TopicPuts
from dimos.web.relay_bridge.topic_service import TopicProvider, TopicServiceConfig
from dimos.web.relay_bridge.wt_client import RelayClient

MessageT = TypeVar("MessageT", bound=Msg)


async def next_message(client: RelayClient, kind: type[MessageT]) -> MessageT:
    async with timeout(5):
        while True:
            msg = await client._session.control_msgs.get()
            if isinstance(msg, kind):
                return msg


def route_variants(provider: TopicProvider) -> dict[str, dict[str, dict[str, Any]]]:
    """The open variants per provider route with their settings in force."""
    return {route: service.variants for route, service in provider._routes.items()}


@asynccontextmanager
async def provider_connection(
    relay: RelayReadyInfo, session: zenoh.Session, *, write: bool = True
) -> AsyncIterator[TopicProvider]:
    """Serve ``session`` as the ``workspace`` source over a real relay connection."""
    config = TopicServiceConfig(
        source="workspace",
        relay_url=f"http://127.0.0.1:{relay.http_port}",
        write=write,
        guarded_publishers=True,
    )
    async with await RelayClient.connect(relay.wt_url, "robot") as client:
        await client.hello(
            topic_source="workspace",
            topic_capabilities=TopicCapabilities(
                read=True,
                write=config.write,
                profiles=["raw", *MEDIA_PROFILES],
                guardedPublishers=config.write,
                profileOptions=PROFILE_OPTIONS,
            ),
        )
        catalog = TopicCatalog()
        metadata = MetadataReceiver(session, catalog)
        await metadata.start()
        puts = TopicPuts(session)
        provider = TopicProvider(config, session, client, puts, catalog)
        task = asyncio.create_task(provider.run())
        try:
            yield provider
        finally:
            task.cancel()
            with suppress(asyncio.CancelledError):
                await task
            await metadata.close()
            await puts.wait_idle()
