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

"""Real relay fixture for the Node adapter tests, controlled by JSON lines."""

import asyncio
import base64
from contextlib import suppress
import json
import socket
import sys

import zenoh

from dimos.protocol.service.zenoh_wire import make_zenoh_config
from dimos.protocol.service.zenohservice import ZenohConfig
from dimos.web.relay_bridge.relay_process import RelayProcess
from dimos.web.relay_bridge.topic_metadata import METADATA_PREFIX, DescriptorSnapshot
from dimos.web.relay_bridge.topic_protocol import TopicDescriptor
from dimos.web.relay_bridge.topic_service import TopicServiceConfig, serve


async def main() -> None:
    relay = RelayProcess()
    stop = asyncio.Event()
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        bus = f"tcp/127.0.0.1:{probe.getsockname()[1]}"
    wire = ZenohConfig(
        mode="peer", listen=[bus], connect=[], multicast=False, gossip=False
    ).to_wire()
    publisher = zenoh.open(make_zenoh_config(wire))
    metadata_key = METADATA_PREFIX + "adapter-test"
    snapshot = DescriptorSnapshot(owner="adapter-test", revision=0, entries=[])
    payload = snapshot.model_dump_json().encode()
    queryable = publisher.declare_queryable(
        metadata_key, lambda query: query.reply(metadata_key, payload)
    )
    provider = None
    try:
        info = await asyncio.to_thread(relay.start)
        config = TopicServiceConfig(
            source="adapter-test",
            relay_url=info.open_url,
            zenoh={**wire, "listen": ["tcp/127.0.0.1:0"], "connect": [bus]},
        )
        provider = asyncio.create_task(serve(config, stop))
        print(json.dumps({"url": info.open_url, "source": config.source}), flush=True)
        while line := await asyncio.to_thread(sys.stdin.readline):
            command = json.loads(line)
            if command["kind"] == "descriptors":
                snapshot = DescriptorSnapshot(
                    owner="adapter-test",
                    revision=snapshot.revision + 1,
                    entries=[TopicDescriptor.model_validate(entry) for entry in command["entries"]],
                )
                payload = snapshot.model_dump_json().encode()
                publisher.put(metadata_key, payload)
            elif command["kind"] == "frame":
                publisher.put(
                    command["key"], base64.b64decode(command["bytes"]), encoding=command["encoding"]
                )
            elif command["kind"] in ("disconnect", "restart"):
                stop.set()
                await provider
                if command["kind"] == "restart":
                    await asyncio.to_thread(relay.stop)
                    relay = RelayProcess(port=info.http_port)
                    info = await asyncio.to_thread(relay.start)
                stop = asyncio.Event()
                provider = asyncio.create_task(serve(config, stop))
            print(json.dumps({"done": True}), flush=True)
    finally:
        stop.set()
        try:
            if provider is not None:
                with suppress(asyncio.CancelledError):
                    await asyncio.wait_for(provider, 10)
        finally:
            queryable.undeclare()
            publisher.close()
            await asyncio.to_thread(relay.stop)


asyncio.run(main())
