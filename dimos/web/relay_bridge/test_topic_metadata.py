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

import asyncio
from collections.abc import Callable
from contextlib import suppress
from dataclasses import dataclass

import pytest
import zenoh

from dimos.core.introspection.module.info import StreamDescriptor
from dimos.core.rpc_client import RPCClient
from dimos.core.stream import TransportSpec
from dimos.core.transport import LCMTransport, pZenohTransport
from dimos.protocol.service.zenoh_wire import make_zenoh_config
from dimos.utils.asyncio_timeout import timeout
from dimos.web.relay_bridge.topic_catalog import TopicCatalog
from dimos.web.relay_bridge.topic_descriptors import DescriptorPublisher, describe_stream
from dimos.web.relay_bridge.topic_metadata import (
    METADATA_PREFIX,
    DescriptorSnapshot,
    MetadataReceiver,
)


@dataclass
class Reading:
    value: float


@pytest.fixture
def session():
    config = make_zenoh_config({"listen": ["tcp/127.0.0.1:0"], "multicast": False, "gossip": False})
    with zenoh.open(config) as opened:
        yield opened


@pytest.fixture
async def receiver(session):
    catalog = TopicCatalog()
    receiver = MetadataReceiver(session, catalog)
    await receiver.start()
    try:
        yield catalog, receiver
    finally:
        await receiver.close()


@pytest.fixture
def stream():
    return StreamDescriptor(
        module="custom",
        name="temperature",
        direction="out",
        type_name=f"{Reading.__module__}.Reading",
        channel="dimos-domains/example/remapped/temperature",
        type=Reading,
        transport=TransportSpec(pZenohTransport, kwargs={"topic": "unrelated-config-alias"}),
    )


async def until(condition: Callable[[], bool]) -> None:
    async with timeout(5):
        while not condition():
            await asyncio.sleep(0.001)


def test_descriptor_uses_the_bound_key_and_excludes_non_zenoh_transports(stream):
    descriptor = describe_stream(stream)
    assert descriptor is not None
    assert descriptor.model_dump() == {
        "key": "dimos-domains/example/remapped/temperature",
        "encoding": "application/cbor",
        "type": f"{Reading.__module__}.Reading",
        "availability": "known",
    }
    assert describe_stream(stream._replace(transport=TransportSpec(LCMTransport))) is None


async def test_lifecycle_refresh_skips_own_rpc_and_does_not_require_a_payload_subscriber(
    session, receiver, stream, mocker
):
    catalog, _receiver = receiver
    remote_rpc = mocker.Mock()
    release = mocker.Mock()
    remote_rpc.call_sync.return_value = ([stream], release)
    own_rpc = mocker.Mock()
    own = RPCClient(
        None, None, "metadata", ["get_stream_descriptors"], rpc=own_rpc, rpc_address="run/metadata"
    )
    other = RPCClient(
        None, None, "custom", ["get_stream_descriptors"], rpc=remote_rpc, rpc_address="run/custom"
    )
    publisher = DescriptorPublisher("run/metadata")
    publisher.update([own, other])
    remote_rpc.call_sync.assert_not_called()
    task = asyncio.create_task(publisher.run(session))
    try:
        await until(lambda: bool(catalog.page("", "", 32)[0]))
        assert catalog.page("", "", 32)[0] == [describe_stream(stream)]
        own_rpc.call_sync.assert_not_called()
        remote_rpc.call_sync.assert_called_once_with(
            "run/custom/get_stream_descriptors", ((), {}), rpc_timeout=1
        )
        release.assert_called_once_with()

        # A module unload is a new complete descriptor snapshot, not a restart.
        publisher.update([own])
        await until(lambda: not catalog.page("", "", 32)[0])
        publisher.update([own, other])
        await until(lambda: bool(catalog.page("", "", 32)[0]))
        assert catalog.page("", "", 32)[0] == [describe_stream(stream)]
    finally:
        task.cancel()
        with suppress(asyncio.CancelledError):
            await task
    await until(lambda: not catalog.page("", "", 32)[0])
    assert catalog.page("", "", 32) == ([], "")


async def test_quiet_external_liveliness_is_distinct_from_observed_data(session, receiver):
    catalog, _receiver = receiver
    token = session.liveliness().declare_token("external/quiet")
    try:
        await until(lambda: bool(catalog.page("external/", "", 32)[0]))
        assert catalog.page("external/", "", 32)[0][0].availability == "advertised"
    finally:
        token.undeclare()
    await until(lambda: not catalog.page("external/", "", 32)[0])
    assert catalog.page("external/", "", 32) == ([], "")


async def test_metadata_owner_must_match_its_key_and_boolean_version_is_invalid(
    session, receiver, stream
):
    catalog, inbox = receiver
    descriptor = describe_stream(stream)
    assert descriptor is not None
    snapshot = DescriptorSnapshot(owner="one", revision=1, entries=[descriptor])
    wrong_version = snapshot.model_dump_json().replace('"v":1', '"v":true')
    session.put(METADATA_PREFIX + "one", wrong_version)
    await until(lambda: inbox.discarded == 1)
    assert catalog.page("", "", 32) == ([], "")
    session.put(METADATA_PREFIX + "two", snapshot.model_dump_json())
    # A valid following snapshot is a processing barrier for both messages.
    barrier = descriptor.model_copy(update={"key": "barrier"})
    session.put(
        METADATA_PREFIX + "three",
        DescriptorSnapshot(owner="three", revision=1, entries=[barrier]).model_dump_json(),
    )
    await until(lambda: bool(catalog.page("barrier", "", 32)[0]))
    assert catalog.page("", "", 32) == ([barrier], "")
