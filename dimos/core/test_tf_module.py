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

from collections.abc import Iterator

import pytest

from dimos.core.tf_module import TfModule, TfModuleConfig
from dimos.msgs.geometry_msgs.Transform import Transform


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
        with pytest.raises(ValueError, match="undeclared tf edges"):
            module.tfbuffer.publish(Transform(frame_id="world", child_frame_id="base_link", ts=1.0))
    finally:
        module._tf = None
        module.stop()


def test_undeclared_edge_is_refused(odom: Odom) -> None:
    with pytest.raises(ValueError, match="undeclared tf edges"):
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
