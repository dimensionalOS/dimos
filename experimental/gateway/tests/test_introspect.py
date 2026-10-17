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

from dimos.core.transport import LCMTransport, ZenohTransport
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.protocol.pubsub.impl.zenohpubsub import Topic as ZenohTopic
from experimental.gateway.introspect import Cache, blueprint, spec_topic


def test_spec_topic_reads_lcm_and_zenoh_specs() -> None:
    assert spec_topic(LCMTransport.spec("/cmd_vel", Twist)) == "/cmd_vel"
    assert (
        spec_topic(ZenohTransport.spec(ZenohTopic("dimos/head_left_color", Twist)))
        == "/head_left_color"
    )


def test_remapped_streams_show_their_wired_topic() -> None:
    streams = {
        (m["name"], s["name"]): s for m in blueprint("r1pro-nav")["modules"] for s in m["streams"]
    }
    left = streams["headleftcamera", "jpeg_out"]
    assert (left["wire"], left["topic"]) == ("head_left_color", "/head_left_color")
    assert streams["headrightcamera", "jpeg_out"]["topic"] == "/head_right_color"


def test_the_cache_shares_a_computation_and_drops_expired_ones() -> None:
    cache, computed = Cache(), []

    async def compute() -> int:
        computed.append(1)
        await asyncio.sleep(0.01)
        return len(computed)

    async def scenario() -> None:
        assert await asyncio.gather(*(cache.get("a", 60, compute) for _ in range(3))) == [1, 1, 1]
        await cache.get("b", 0, compute)
        await cache.get("c", 60, compute)

    asyncio.run(scenario())
    assert len(computed) == 3
    assert set(cache.entries) == {"a", "c"}
