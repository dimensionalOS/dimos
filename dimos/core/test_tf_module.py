# Copyright 2025-2026 Dimensional Inc.
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

from collections.abc import Callable, Iterator
from typing import Any

import pytest

from dimos.core.module import Module
from dimos.core.stream import IO, Transport
from dimos.core.tf_module import TfModule, TfModuleConfig
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.tf2_msgs.TFMessage import TFMessage


class OdomConfig(TfModuleConfig):
    tf_frames_local: dict[str, str] = {"body": "base_link"}
    tf_frames_global: dict[str, str] = {"world": "world"}
    tf_edges: list[tuple[str, str]] = [("world", "body")]


class Odom(TfModule):
    config: OdomConfig


@pytest.fixture
def odom() -> Iterator[Odom]:
    module = Odom()
    module._tf = module._create_tfbuffer(None)
    yield module
    module._tf = None
    module.stop()


def test_declared_frames_publish_and_get(odom: Odom) -> None:
    frames = odom.config
    odom.tfbuffer.publish(
        Transform(frame_id=frames.tf_frames_global["world"], child_frame_id="base_link", ts=1.0)
    )
    assert odom.tfbuffer.get("world", "base_link", time_point=1.0) is not None


def test_frames_are_configurable() -> None:
    module = Odom(tf_frames_local={"body": "robot0/base_link"})
    module._tf = module._create_tfbuffer(None)
    try:
        module.tfbuffer.publish(
            Transform(frame_id="world", child_frame_id="robot0/base_link", ts=1.0)
        )
        with pytest.raises(ValueError, match="Odom publishes undeclared tf edge"):
            module.tfbuffer.publish(Transform(frame_id="world", child_frame_id="base_link", ts=1.0))
    finally:
        module._tf = None
        module.stop()


def test_undeclared_edge_is_refused(odom: Odom) -> None:
    with pytest.raises(ValueError, match="Odom publishes undeclared tf edge"):
        odom.tfbuffer.publish(Transform(frame_id="base_link", child_frame_id="world", ts=1.0))


def test_skip_tf_checks_publishes_any_edge() -> None:
    module = Odom(skip_tf_checks=True)
    module._tf = module._create_tfbuffer(None)
    try:
        module.tfbuffer.publish(Transform(frame_id="base_link", child_frame_id="lidar", ts=1.0))
    finally:
        module._tf = None
        module.stop()


def test_edges_must_use_declared_roles() -> None:
    with pytest.raises(ValueError, match=r"undeclared tf roles \['lidar'\]"):
        OdomConfig(tf_edges=[("body", "lidar")])
    with pytest.raises(ValueError, match=r"in both"):
        OdomConfig(tf_frames_local={"world": "x"})


def test_frame_reads_local_and_global_roles(odom: Odom) -> None:
    assert (odom.frame.body, odom.frame.world) == ("base_link", "world")
    with pytest.raises(AttributeError, match="lidar"):
        odom.frame.lidar  # noqa: B018


class _LoopbackTransport(Transport[TFMessage]):
    def __init__(self) -> None:
        self.callbacks: list[Callable[[TFMessage], Any]] = []

    def subscribe(
        self, callback: Callable[[TFMessage], Any], selfstream: Any = None
    ) -> Callable[[], None]:
        self.callbacks.append(callback)
        return lambda: self.callbacks.remove(callback)

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def broadcast(self, selfstream: Any, msg: TFMessage) -> None:
        for callback in self.callbacks:
            callback(msg)


def _wired(**kwargs: Any) -> tuple[Odom, list[TFMessage]]:
    module = Odom(**kwargs)
    transport = _LoopbackTransport()
    sent: list[TFMessage] = []
    transport.subscribe(sent.append)
    module.set_transport("tf", transport)
    # The buffer subscribes on first touch, so build it before anything is published.
    assert module.tfbuffer is not None
    return module, sent


@pytest.fixture
def wired() -> Iterator[tuple[Odom, list[TFMessage]]]:
    module, sent = _wired()
    yield module, sent
    module.stop()


def test_tf_port_keeps_its_name_and_kind(odom: Odom) -> None:
    assert type(odom.tf) is IO
    assert odom.tf.name == "tf" and odom.ios["tf"] is odom.tf


def test_tf_port_publishes_declared_edge(wired: tuple[Odom, list[TFMessage]]) -> None:
    module, sent = wired
    module.tf.publish(TFMessage(Transform(frame_id="world", child_frame_id="base_link", ts=1.0)))
    assert len(sent) == 1


def test_tf_port_refuses_undeclared_edge(wired: tuple[Odom, list[TFMessage]]) -> None:
    module, sent = wired
    with pytest.raises(ValueError, match="Odom publishes undeclared tf edge"):
        module.tf.publish(
            TFMessage(Transform(frame_id="base_link", child_frame_id="world", ts=1.0))
        )
    assert sent == []


def test_tfbuffer_get_reads_declared_frames(wired: tuple[Odom, list[TFMessage]]) -> None:
    module, _ = wired
    module.tf.publish(TFMessage(Transform(frame_id="world", child_frame_id="base_link", ts=1.0)))
    assert module.tfbuffer.get("world", "base_link", time_point=1.0) is not None


def test_tfbuffer_get_refuses_undeclared_frame(wired: tuple[Odom, list[TFMessage]]) -> None:
    module, _ = wired
    with pytest.raises(ValueError, match="Odom looks up undeclared tf frame 'lidar'"):
        module.tfbuffer.get("world", "lidar")


def test_tf_port_skip_tf_checks_is_unchecked() -> None:
    module, sent = _wired(skip_tf_checks=True)
    try:
        module.tf.publish(
            TFMessage(Transform(frame_id="base_link", child_frame_id="lidar", ts=1.0))
        )
        assert len(sent) == 1
        assert module.tfbuffer.get("base_link", "lidar", time_point=1.0) is not None
    finally:
        module.stop()


def test_plain_module_tf_port_is_plain_io() -> None:
    class Plain(Module):
        tf: IO[TFMessage]

    module = Plain()
    try:
        assert type(module.tf) is IO
    finally:
        module.stop()
