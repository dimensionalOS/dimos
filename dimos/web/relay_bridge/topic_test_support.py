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
from collections.abc import AsyncIterator, Iterator
from contextlib import asynccontextmanager, contextmanager, suppress
import json
from pathlib import Path
import threading
import time
from typing import Any, TypeVar

from dimos_lcm.sensor_msgs import Image
import numpy as np
import zenoh

from dimos.utils.asyncio_timeout import timeout
from dimos.web.relay_bridge.protocol import Msg
from dimos.web.relay_bridge.relay_process import RelayReadyInfo
from dimos.web.relay_bridge.topic_catalog import TopicCatalog
from dimos.web.relay_bridge.topic_metadata import MetadataReceiver
from dimos.web.relay_bridge.topic_profiles import PROFILE_OPTIONS
from dimos.web.relay_bridge.topic_protocol import TopicCapabilities
from dimos.web.relay_bridge.topic_put import TopicPuts
from dimos.web.relay_bridge.topic_service import (
    TopicProvider,
    TopicServiceConfig,
    VideoSettings,
    capabilities,
    video_encoder,
)
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
    relay: RelayReadyInfo, session: zenoh.Session, *, write: bool = True, video: bool = False
) -> AsyncIterator[TopicProvider]:
    """Serve ``session`` as the ``workspace`` source over a real relay connection."""
    config = TopicServiceConfig(
        source="workspace",
        relay_url=f"http://127.0.0.1:{relay.http_port}",
        write=write,
        guarded_publishers=True,
        video=VideoSettings(enabled=video),
    )
    codec = video_encoder(config)
    async with await RelayClient.connect(relay.wt_url, "robot") as client:
        await client.hello(
            topic_source="workspace",
            topic_capabilities=TopicCapabilities(
                read=True,
                write=config.write,
                profiles=capabilities(config, video=codec is not None).profiles,
                guardedPublishers=config.write,
                profileOptions=PROFILE_OPTIONS,
            ),
        )
        catalog = TopicCatalog()
        metadata = MetadataReceiver(session, catalog)
        await metadata.start()
        puts = TopicPuts(session)
        provider = TopicProvider(config, session, client, puts, catalog, codec)
        task = asyncio.create_task(provider.run())
        try:
            yield provider
        finally:
            task.cancel()
            with suppress(asyncio.CancelledError):
                await task
            await metadata.close()
            await puts.wait_idle()


def camera_frame(seq: int, width: int = 320, height: int = 240) -> bytes:
    """An rgb8 Image whose picture moves with ``seq``, so an encoder has work."""
    vectors = json.loads(
        (Path(__file__).parents[3] / "web/shared/fixtures/topic_media.json").read_text()
    )
    vector = next(v for v in vectors if v["name"] == "color_rgb_boxed")
    message = Image.lcm_decode(bytes.fromhex(vector["lcm"]))
    message.header.seq = seq
    shape = (height, width)
    rows = np.broadcast_to((np.arange(height)[:, None] * 2 + seq * 7) % 256, shape)
    cols = np.broadcast_to((np.arange(width)[None, :] * 3 + seq * 5) % 256, shape)
    picture = np.stack([(rows + cols) % 256, rows, cols], axis=2).astype(np.uint8)
    message.height, message.width, message.encoding = height, width, "rgb8"
    message.is_bigendian, message.step = False, width * 3
    message.data = picture.tobytes()
    message.data_length = len(message.data)
    return bytes(message.lcm_encode())


@contextmanager
def publish_camera(
    source: zenoh.Session,
    key: str,
    count: int,
    hz: float = 30,
    width: int = 320,
    height: int = 240,
) -> Iterator[threading.Thread]:
    """Publish up to ``count`` synthetic frames at ``hz`` from a thread that
    stops with the block, so a failing test never leaves it putting into a
    closing Zenoh session."""
    stop = threading.Event()

    def run() -> None:
        # Absolute schedule: building a frame never lowers the rate.
        period = 1 / hz
        next_at = time.monotonic()
        for seq in range(count):
            if stop.is_set():
                return
            source.put(key, camera_frame(seq, width, height))
            next_at += period
            stop.wait(max(0.0, next_at - time.monotonic()))

    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    try:
        yield thread
    finally:
        stop.set()
        thread.join(5)
