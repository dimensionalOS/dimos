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

from collections.abc import Iterator
from dataclasses import dataclass
from types import new_class
from typing import Any, Generic, TypedDict, TypeVar

from langchain_core.messages import AIMessage, HumanMessage
import numpy as np
from numpy.typing import NDArray
import pytest
from reactivex.disposable import Disposable

from dimos.agents.agent import Agent
from dimos.agents.base_agent import BaseAgent
from dimos.agents.typesafe.types import SystemOneRequest
from dimos.core.coordination.blueprints import Blueprint, BlueprintAtom, autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.global_config import GlobalConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.protocol.encode import cbor
from dimos.utils.testing.waiting import wait_until

T = TypeVar("T")


@dataclass(frozen=True)
class ImageQuery:
    text: str
    images: tuple[Image, ...] = ()


@dataclass(frozen=True)
class RobotQuery:
    state: JointState
    odom: Odometry
    command: str


class Reading(TypedDict):
    sensor: str
    value: float


class TextAgent(BaseAgent[str, str]):
    @rpc
    def infer(self, x: str) -> str:
        return x.upper()


class LoudTextAgent(TextAgent):
    pass


class CaptionAgent(BaseAgent[ImageQuery, str]):
    @rpc
    def infer(self, x: ImageQuery) -> str:
        shapes = ", ".join(f"{image.width}x{image.height}" for image in x.images)
        return f"{x.text}: {shapes}"


class RobotAgent(BaseAgent[RobotQuery, dict[str, float]]):
    @rpc
    def infer(self, x: RobotQuery) -> dict[str, float]:
        return {
            "joints": float(len(x.state.name)),
            "x": x.odom.pose.position.x,
            "words": float(len(x.command.split())),
        }


class StreamedAgent(BaseAgent[RobotQuery, dict[str, float]]):
    """Reads its own stream and answers on its own stream, through infer."""

    query: In[RobotQuery]
    result: Out[dict[str, float]]

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(
            Disposable(self.query.subscribe(lambda x: self.result.publish(self.infer(x))))
        )

    @rpc
    def infer(self, x: RobotQuery) -> dict[str, float]:
        return {
            "joints": float(len(x.state.name)),
            "x": x.odom.pose.position.x,
            "words": float(len(x.command.split())),
        }


class DictAgent(BaseAgent[dict[str, float], list[str]]):
    @rpc
    def infer(self, x: dict[str, float]) -> list[str]:
        return sorted(x)


class ReadingAgent(BaseAgent[Reading, float]):
    pass


class Embedder(BaseAgent[tuple[str | Image, ...], NDArray[np.float32]]):
    pass


class OneInput(BaseAgent[T, str], Generic[T]):
    pass


class BytesAgent(OneInput[bytes]):
    pass


def _image() -> Image:
    return Image(data=np.zeros((4, 6, 3), dtype=np.uint8), format=ImageFormat.RGB)


def _robot_query() -> RobotQuery:
    return RobotQuery(
        state=JointState(name=["hip", "knee"], position=[0.1, 0.2]),
        odom=Odometry(pose=Pose(1.5, 0.0, 0.0)),
        command="walk to the door",
    )


@pytest.mark.parametrize(
    "agent, input_type, output_type",
    [
        (TextAgent, str, str),
        (LoudTextAgent, str, str),
        (CaptionAgent, ImageQuery, str),
        (RobotAgent, RobotQuery, dict[str, float]),
        (DictAgent, dict[str, float], list[str]),
        (ReadingAgent, Reading, float),
        (BytesAgent, bytes, str),
    ],
)
def test_an_agent_carries_the_types_it_names(
    agent: type[BaseAgent[Any, Any]], input_type: object, output_type: object
) -> None:
    assert (agent.input_type, agent.output_type) == (input_type, output_type)


def test_agents_with_different_types_are_told_apart() -> None:
    assert isinstance(CaptionAgent.input_type, type) and isinstance(TextAgent.input_type, type)
    assert CaptionAgent.input_type is not TextAgent.input_type
    assert Embedder.output_type == NDArray[np.float32]


def test_an_agent_without_types_is_refused_when_defined() -> None:
    with pytest.raises(TypeError, match="must name its input and output types"):
        type("Untyped", (BaseAgent,), {})


_ANY_AGENT: Any = BaseAgent


class Handle:
    """A plain class: nothing says how to send it."""


@pytest.mark.parametrize("bad", [Handle, list[Handle], dict[str, Handle | None]])
def test_an_agent_whose_types_cannot_be_sent_is_refused_when_defined(bad: Any) -> None:
    for pair in ((bad, str), (str, bad)):
        with pytest.raises(TypeError, match="cannot be sent over RPC or a transport"):
            new_class("Unsendable", (_ANY_AGENT[pair],))


@pytest.mark.parametrize(
    "agent, value",
    [
        (TextAgent, "hello"),
        (CaptionAgent, ImageQuery("two views", (_image(), _image()))),
        (RobotAgent, _robot_query()),
        (DictAgent, {"a": 1.0}),
        (ReadingAgent, Reading(sensor="lidar", value=0.5)),
        (Embedder, ("a door", _image())),
    ],
)
def test_an_input_survives_the_wire(agent: type[BaseAgent[Any, Any]], value: Any) -> None:
    back = cbor.decode(cbor.encode(value))
    assert type(back) is type(value)
    assert agent.accepts(back)


def test_an_agent_that_leaves_a_type_open_is_refused_when_defined() -> None:
    with pytest.raises(TypeError, match="unbound"):
        type("Open", (OneInput,), {})


@pytest.mark.parametrize(
    "agent, value, accepted",
    [
        (TextAgent, "hello", True),
        (TextAgent, 3, False),
        (CaptionAgent, ImageQuery("what is this?"), True),
        (CaptionAgent, "what is this?", False),
        (RobotAgent, None, False),
        (DictAgent, {"a": 1.0}, True),
        (DictAgent, {"a": "one"}, False),
        (DictAgent, {1: 1.0}, False),
        (ReadingAgent, {"sensor": "lidar", "value": 0.5}, True),
        (ReadingAgent, {"sensor": "lidar"}, False),
        (BytesAgent, b"raw", True),
        (BytesAgent, "raw", False),
    ],
)
def test_accepts_checks_a_value_against_the_input_type(
    agent: type[BaseAgent[Any, Any]], value: object, accepted: bool
) -> None:
    assert agent.accepts(value) is accepted


def test_accepts_takes_a_multi_part_input() -> None:
    assert RobotAgent.accepts(_robot_query())
    assert CaptionAgent.accepts(ImageQuery("two views", (_image(), _image())))
    assert Embedder.accepts(("a door", _image()))
    assert not Embedder.accepts(("a door", 3))


def test_infer_is_left_to_the_subclass() -> None:
    agent = ReadingAgent()
    try:
        with pytest.raises(NotImplementedError):
            agent.infer({"sensor": "lidar", "value": 0.5})
    finally:
        agent.stop()


@pytest.fixture
def coordinator() -> Iterator[ModuleCoordinator]:
    mc = ModuleCoordinator(g=GlobalConfig(n_workers=1, viewer="none"))
    mc.start()
    try:
        yield mc
    finally:
        mc.stop()


@pytest.mark.parametrize(
    "agent, value, answer",
    [
        (TextAgent, "hello", "HELLO"),
        (CaptionAgent, ImageQuery("two views", (_image(), _image())), "two views: 6x4, 6x4"),
        (RobotAgent, _robot_query(), {"joints": 2.0, "x": 1.5, "words": 4.0}),
        (DictAgent, {"b": 2.0, "a": 1.0}, ["a", "b"]),
    ],
)
def test_infer_runs_over_rpc(
    coordinator: ModuleCoordinator, agent: type[BaseAgent[Any, Any]], value: Any, answer: Any
) -> None:
    coordinator.load_module(agent)
    assert coordinator.get_instance(agent).infer(value) == answer


def test_base_agent_gives_an_agent_no_input_or_output_stream() -> None:
    streams = {s.name for s in BlueprintAtom.create(RobotAgent, {}).streams}
    assert streams == {"agent", "agent_idle"}


def test_the_messages_an_agent_takes_and_returns_survive_the_wire() -> None:
    human = HumanMessage(
        content=[{"type": "text", "text": "what is this?"}, *_image().agent_encode()]
    )
    answer = AIMessage(content="a door", tool_calls=[{"name": "observe", "args": {}, "id": "c1"}])
    request = SystemOneRequest(
        state={"goal": "chair"},
        questions={"seen": {"type": "noul", "instructions": "Is it visible?"}},
    )

    for value in (human, answer):
        back = cbor.decode(cbor.encode(value))
        assert type(back) is type(value) and back.content == value.content
    assert cbor.decode(cbor.encode(answer)).tool_calls == answer.tool_calls
    assert cbor.decode(cbor.encode(request)) == request
    assert Agent.accepts(cbor.decode(cbor.encode(human)))


def _built(*blueprints: Blueprint) -> ModuleCoordinator:
    return ModuleCoordinator.build(autoconnect(*blueprints).global_config(viewer="none"))


def test_an_agent_feeds_its_own_stream_to_infer() -> None:
    coordinator = _built(StreamedAgent.blueprint())
    try:
        query = coordinator.transports["query", RobotQuery]
        result = coordinator.transports["result", dict[str, float]]
        answers: list[dict[str, float]] = []
        result.subscribe(answers.append)

        def answered() -> bool:
            query.publish(_robot_query())
            return bool(answers)

        wait_until(answered, timeout=20, interval=0.5)
        assert answers[0] == {"joints": 2.0, "x": 1.5, "words": 4.0}
        assert coordinator.get_instance(StreamedAgent).infer(_robot_query()) == answers[0]
    finally:
        coordinator.stop()


def test_agents_of_different_classes_share_a_blueprint() -> None:
    coordinator = _built(TextAgent.blueprint(), CaptionAgent.blueprint(), StreamedAgent.blueprint())
    try:
        agents = coordinator.get_instances(BaseAgent)
        assert sorted(agents) == ["captionagent", "streamedagent", "textagent"]
        assert agents["textagent"].infer("hello") == "HELLO"
        assert coordinator.get_instances(TextAgent).keys() == {"textagent"}
    finally:
        coordinator.stop()


def test_agents_of_one_class_are_separated_by_namespaces() -> None:
    coordinator = _built(
        StreamedAgent.blueprint().namespace("left"), StreamedAgent.blueprint().namespace("right")
    )
    try:
        agents = coordinator.get_instances(StreamedAgent)
        assert sorted(agents) == ["left/streamedagent", "right/streamedagent"]
        topics = {name: agent.query.transport.topic for name, agent in agents.items()}
        assert topics["left/streamedagent"] != topics["right/streamedagent"]
        for agent in agents.values():
            assert agent.infer(_robot_query())["words"] == 4.0
    finally:
        coordinator.stop()


def test_an_agent_used_as_a_plain_class_answers_infer() -> None:
    """No coordinator, no worker process, no transport."""
    agent = RobotAgent()
    try:
        assert agent.infer(_robot_query()) == {"joints": 2.0, "x": 1.5, "words": 4.0}
        assert RobotAgent.accepts(_robot_query())
    finally:
        agent.stop()
