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

from queue import Queue

import pytest

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.module import Module, StreamGroup
from dimos.core.stream import Out
from dimos.core.transport_factory import make_transport


class FanInModule(Module):
    """One handler for two same-typed streams; echoes which one it came from."""

    tagged: Out[int]

    async def handle_sensors(self, index: int, value: int) -> None:
        self.tagged.publish(index * 100 + value)


@pytest.fixture
def start_fan_in_module(each_transport):
    blueprint = FanInModule.blueprint(stream_groups={"sensors": StreamGroup(names=["s0", "s1"])})
    coordinator = ModuleCoordinator.build(blueprint)
    yield
    coordinator.stop()


@pytest.fixture
def group_transports(each_transport):
    transports = [make_transport("s0"), make_transport("s1"), make_transport("tagged")]
    for transport in transports:
        transport.start()
    yield transports
    for transport in transports:
        transport.stop()


def test_stream_group_tags_each_message_with_its_stream(start_fan_in_module, group_transports):
    s0, s1, tagged = group_transports
    queue: Queue[int] = Queue()
    tagged.subscribe(queue.put)

    s0.publish(7)
    assert queue.get(timeout=1.0) == 7

    s1.publish(7)
    assert queue.get(timeout=1.0) == 107
