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
from dimos.core.module import Module, TopicMetadata
from dimos.core.stream import In, Out
from dimos.core.transport_factory import make_transport


class FanInModule(Module):
    """One handler for two same-typed streams; echoes which one it came from."""

    sensors: In[int]
    tagged: Out[int]
    named: Out[str]
    scaled: Out[float]

    async def handle_sensors(self, value: int, meta: TopicMetadata) -> None:
        self.tagged.publish(meta.index * 100 + value)
        self.named.publish(meta.name)
        self.scaled.publish(value * meta.info.get("scale", 1.0))


@pytest.fixture
def start_fan_in_module(each_transport):
    blueprint = FanInModule.blueprint().remappings(
        [(FanInModule, "sensors", {"s0": {"scale": 0.5}, "s1": {}})]
    )
    coordinator = ModuleCoordinator.build(blueprint)
    yield
    coordinator.stop()


@pytest.fixture
def group_transports(each_transport):
    transports = [
        make_transport("s0"),
        make_transport("s1"),
        make_transport("tagged"),
        make_transport("named"),
        make_transport("scaled"),
    ]
    for transport in transports:
        transport.start()
    yield transports
    for transport in transports:
        transport.stop()


def test_topic_funnel_tags_each_message_with_its_stream(start_fan_in_module, group_transports):
    s0, s1, tagged, named, scaled = group_transports
    queue: Queue[int] = Queue()
    names: Queue[str] = Queue()
    scales: Queue[float] = Queue()
    tagged.subscribe(queue.put)
    named.subscribe(names.put)
    scaled.subscribe(scales.put)

    s0.publish(7)
    assert queue.get(timeout=1.0) == 7
    assert names.get(timeout=1.0) == "s0"
    assert scales.get(timeout=1.0) == 3.5

    s1.publish(7)
    assert queue.get(timeout=1.0) == 107
    assert names.get(timeout=1.0) == "s1"
    assert scales.get(timeout=1.0) == 7.0


@pytest.fixture
def start_remapped_fan_in_module(each_transport):
    blueprint = FanInModule.blueprint().remappings(
        [(FanInModule, "sensors", ["s0", "s1"]), (FanInModule, "s0", "alt0")]
    )
    coordinator = ModuleCoordinator.build(blueprint)
    yield
    coordinator.stop()


def test_topic_funnel_entries_follow_remappings(start_remapped_fan_in_module, each_transport):
    """A remapped entry keeps its index and declared name, but listens on the
    remapped stream."""
    alt0, tagged, named = make_transport("alt0"), make_transport("tagged"), make_transport("named")
    for transport in (alt0, tagged, named):
        transport.start()
    try:
        queue: Queue[int] = Queue()
        names: Queue[str] = Queue()
        tagged.subscribe(queue.put)
        named.subscribe(names.put)
        alt0.publish(7)
        assert queue.get(timeout=1.0) == 7
        assert names.get(timeout=1.0) == "s0"
    finally:
        for transport in (alt0, tagged, named):
            transport.stop()


def test_namespace_prefixes_topic_funnel_entries():
    blueprint = (
        FanInModule.blueprint()
        .remappings([(FanInModule, "sensors", ["s0", "s1"])])
        .namespace("bot")
    )
    atom = blueprint.blueprints[0]
    assert blueprint.remapping_map[atom.name, "s0"] == "bot/s0"
    assert blueprint.remapping_map[atom.name, "s1"] == "bot/s1"


def test_a_fanned_in_port_no_longer_connects_under_its_own_name():
    """The port stands in for its entries rather than being a stream of its own."""
    blueprint = FanInModule.blueprint().remappings([(FanInModule, "sensors", ["s0", "s1"])])
    names = {stream.name for stream in blueprint.blueprints[0].streams}
    assert "sensors" not in names
    assert {"s0", "s1"} <= names


def test_fanning_in_an_undeclared_port_is_an_error():
    with pytest.raises(ValueError, match="no such In/IO stream"):
        FanInModule.blueprint().remappings([(FanInModule, "nope", ["s0"])])


def test_a_funnel_entry_cannot_collide_with_a_declared_stream():
    with pytest.raises(ValueError, match="collides"):
        FanInModule(topic_funnels={"sensors": {"names": ["tagged"]}})


def test_a_funnel_entry_rejects_a_backend_topic_string():
    """Entries are stream names, so a leading slash is a transport leaking in."""
    with pytest.raises(ValueError, match="not topics"):
        FanInModule.blueprint().remappings([(FanInModule, "sensors", ["/s0"])])


class PlainFanInModule(Module):
    """A funnel handler that doesn't ask for metadata just gets the message."""

    sensors: In[int]
    echoed: Out[int]

    async def handle_sensors(self, value: int) -> None:
        self.echoed.publish(value)


def test_a_funnel_handler_without_meta_gets_just_the_message(each_transport):
    blueprint = PlainFanInModule.blueprint().remappings(
        [(PlainFanInModule, "sensors", ["s0", "s1"])]
    )
    coordinator = ModuleCoordinator.build(blueprint)
    s1, echoed = make_transport("s1"), make_transport("echoed")
    for transport in (s1, echoed):
        transport.start()
    try:
        queue: Queue[int] = Queue()
        echoed.subscribe(queue.put)
        s1.publish(7)
        assert queue.get(timeout=1.0) == 7
    finally:
        for transport in (s1, echoed):
            transport.stop()
        coordinator.stop()
