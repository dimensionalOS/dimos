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

"""A real coordinator's descriptors and CBOR data reach a topic viewer across reloads."""

import asyncio
from collections.abc import Iterator
from dataclasses import dataclass
import socket
from typing import Any

import pytest
import zenoh

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import Out, TransportSpec
from dimos.core.transport import pZenohTransport
from dimos.protocol.encode import cbor
from dimos.protocol.service.zenoh_wire import make_zenoh_config
from dimos.utils.asyncio_timeout import timeout
from dimos.web.relay_bridge.protocol import DataFrame, Msg
from dimos.web.relay_bridge.relay_process import RelayProcess, RelayReadyInfo
from dimos.web.relay_bridge.topic_descriptors import TopicMetadataModule
from dimos.web.relay_bridge.topic_protocol import (
    TopicActivate,
    TopicError,
    TopicList,
    TopicPage,
    TopicReady,
    TopicSources,
    TopicSub,
)
from dimos.web.relay_bridge.topic_test_support import provider_connection
from dimos.web.relay_bridge.wt_client import RelayClient

pytestmark = pytest.mark.self_hosted
PREFIX = "dimos-domains/custom/remapped/"
KEYS = {PREFIX + "temperature", PREFIX + "values"}


@dataclass
class Reading:
    value: float
    unit: str


class CustomReadingModule(Module):
    reading: Out[Reading]
    values: Out[dict[str, Any]]

    @rpc
    def emit(self, value: float) -> None:
        """Publish only on request, so subscription readiness precedes any data."""
        self.reading.publish(Reading(value, "C"))
        self.values.publish({"temperature": value, "valid": True, "labels": ["custom", None]})


@pytest.fixture(scope="module")
def relay(tmp_path_factory: pytest.TempPathFactory) -> Iterator[RelayReadyInfo]:
    process = RelayProcess()
    try:
        yield process.start()
    finally:
        process.stop()


def free_port() -> int:
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


async def test_custom_module_reload_keeps_descriptors_data_and_source(
    relay: RelayReadyInfo,
) -> None:
    endpoint = f"tcp/127.0.0.1:{free_port()}"
    # A router carries coordinator and worker RPC between separately connected
    # clients without scouting unrelated stacks on the host.
    config = make_zenoh_config(
        {"mode": "router", "listen": [endpoint], "multicast": False, "gossip": False}
    )
    seen: list[Msg | DataFrame] = []
    coordinator: ModuleCoordinator | None = None
    with zenoh.open(config) as session:
        async with (
            provider_connection(relay, session, write=False),
            await RelayClient.connect(relay.wt_url, "viewer") as viewer,
        ):

            async def expect(kind: type[Any]) -> Any:
                async with timeout(5):
                    while True:
                        msg = await viewer._session.control_msgs.get()
                        seen.append(msg)
                        if isinstance(msg, kind):
                            return msg

            async def listed(availability: str) -> None:
                async with timeout(15):
                    while True:
                        viewer.send_control(
                            TopicList(
                                id="custom", source="workspace", prefix=PREFIX, cursor="", limit=32
                            )
                        )
                        page: TopicPage = await expect(TopicPage)
                        if {entry.key for entry in page.entries} == KEYS and all(
                            entry.availability == availability for entry in page.entries
                        ):
                            return
                        await asyncio.sleep(0.05)

            await viewer.hello(topics=True)
            viewer.send_control(
                TopicSub(
                    id="custom",
                    source="workspace",
                    keyExpr=PREFIX + "**",
                    delivery="reliable",
                    profile="raw",
                    maxHz=1000,
                )
            )
            ready: TopicReady = await expect(TopicReady)
            viewer.send_control(TopicActivate(route=ready.route, gen=ready.gen))
            frames = viewer.frames()
            try:
                metadata = autoconnect(TopicMetadataModule.blueprint()).global_config(
                    n_workers=1,
                    viewer="none",
                    build_native=False,
                    transport="zenoh",
                    zenoh_connect=endpoint,
                    zenoh_mode="client",
                    zenoh_scouting=False,
                    zenoh_multicast=False,
                    zenoh_gossip=False,
                    serve_coordinator_rpc=False,
                )
                coordinator = await asyncio.to_thread(ModuleCoordinator.build, metadata)
                custom = CustomReadingModule.blueprint().transports(
                    {
                        ("reading", Reading): TransportSpec(
                            pZenohTransport, kwargs={"topic": PREFIX + "temperature"}
                        ),
                        ("values", dict[str, Any]): TransportSpec(
                            pZenohTransport, kwargs={"topic": PREFIX + "values"}
                        ),
                    }
                )
                for value in (42.5, 43.5):
                    await asyncio.to_thread(coordinator.load_blueprint, custom)
                    await listed("known")
                    instance = coordinator.get_instance(CustomReadingModule)
                    await asyncio.to_thread(instance.emit, value)
                    async with timeout(5):
                        arrived = [await anext(frames) for _ in KEYS]
                    received: dict[str, DataFrame] = {}
                    for frame in arrived:
                        assert frame.header.meta is not None
                        received[frame.header.meta["key"]] = frame
                    assert set(received) == KEYS, [
                        (frame.header.seq, frame.header.meta, frame.payload) for frame in arrived
                    ]
                    reading = cbor.decode(received[PREFIX + "temperature"].payload)
                    assert type(reading).__name__ == "Reading"
                    assert vars(reading) == {"value": value, "unit": "C"}
                    assert cbor.decode(received[PREFIX + "values"].payload) == {
                        "temperature": value,
                        "valid": True,
                        "labels": ["custom", None],
                    }
                    await asyncio.to_thread(coordinator.unload_module, CustomReadingModule)
                    await listed("observed")
            finally:
                if coordinator is not None:
                    await asyncio.to_thread(coordinator.stop)
            # The coordinator and its worker shared the bus with the provider.
            # Its route stayed open under one source generation throughout.
            while not viewer._session.control_msgs.empty():
                seen.append(viewer._session.control_msgs.get_nowait())
            assert not [msg for msg in seen if isinstance(msg, TopicError)]
            assert len({s.gen for m in seen if isinstance(m, TopicSources) for s in m.sources}) == 1
    await asyncio.get_running_loop().shutdown_default_executor()
