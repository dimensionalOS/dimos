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

from __future__ import annotations

from collections.abc import Callable, Iterator
import math
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import RobotModel
from dimos.robot.robot_tf_publisher import RobotTfPublisher

# One revolute joint about z, child offset 1m along x, so a 90 degree command
# has an answer anyone can check by hand.
_URDF = """<?xml version="1.0"?>
<robot name="one_joint">
  <link name="base"/>
  <link name="tip"/>
  <joint name="shoulder" type="revolute">
    <parent link="base"/><child link="tip"/>
    <origin xyz="1 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.15" upper="3.15" effort="10" velocity="1"/>
  </joint>
</robot>
"""


@pytest.fixture
def make_publisher(tmp_path: Path) -> Iterator[Callable[..., RobotTfPublisher]]:
    urdf = tmp_path / "robot.urdf"
    urdf.write_text(_URDF)
    modules: list[RobotTfPublisher] = []

    def make(**overrides: Any) -> RobotTfPublisher:
        model: dict[str, Any] = {
            "name": "robot",
            "model": RobotModel.from_file(urdf),
            "joint_names": ["shoulder"],
            "base_link": "base",
        }
        model.update(overrides.pop("robot_model", {}))
        module = RobotTfPublisher(robot_model=RobotModelConfig(**model), **overrides)
        modules.append(module)
        return module

    yield make
    for module in modules:
        module.dispose()


def _published(module: RobotTfPublisher) -> dict[tuple[str, str], Any]:
    sent: list[Any] = []
    module.tf.publish = sent.append  # type: ignore[method-assign]
    module.on_joint_state(JointState(name=["shoulder"], position=[0.0], ts=1.0))
    return {(t.frame_id, t.child_frame_id): t for t in sent[0].transforms}


def test_a_joint_state_yields_every_model_edge_plus_the_base_placement(
    make_publisher: Callable[..., RobotTfPublisher],
) -> None:
    module = make_publisher()

    edges = _published(module)

    assert ("world", "base") in edges, "base_pose places the robot in the fixed frame"
    assert ("base", "tip") in edges, "and every URDF joint becomes an edge"


def test_forward_kinematics_actually_moves_the_child(
    make_publisher: Callable[..., RobotTfPublisher],
) -> None:
    module = make_publisher()
    sent: list[Any] = []
    module.tf.publish = sent.append  # type: ignore[method-assign]

    module.on_joint_state(JointState(name=["shoulder"], position=[math.pi / 2], ts=1.0))

    tip = next(t for t in sent[0].transforms if t.child_frame_id == "tip")
    # The offset is on the joint origin, so the tip stays at x=1 and only its
    # orientation turns. What must change is the rotation, not the translation.
    np.testing.assert_allclose(
        [tip.translation.x, tip.translation.y, tip.translation.z], [1.0, 0.0, 0.0], atol=1e-9
    )
    assert tip.rotation.z == pytest.approx(math.sin(math.pi / 4), abs=1e-9)


def test_the_base_edge_carries_the_configured_placement(
    make_publisher: Callable[..., RobotTfPublisher],
) -> None:
    module = make_publisher(
        robot_model={"base_pose": PoseStamped(position=Vector3(0.0, 2.0, 0.0))},
        fixed_frame="map",
    )

    edges = _published(module)

    base = edges[("map", "base")]
    assert base.translation.y == pytest.approx(2.0)


def test_an_incomplete_joint_state_publishes_nothing(
    make_publisher: Callable[..., RobotTfPublisher],
) -> None:
    # A partial state would leave some links at a stale pose, and the self
    # filter would trust it and cut the wrong volume out of the cloud.
    module = make_publisher()
    sent: list[Any] = []
    module.tf.publish = sent.append  # type: ignore[method-assign]

    assert module.on_joint_state(JointState(name=["elbow"], position=[0.0], ts=1.0)) is False
    assert sent == []


def test_coordinator_joint_names_are_mapped_into_the_model(
    make_publisher: Callable[..., RobotTfPublisher],
) -> None:
    module = make_publisher(robot_model={"joint_name_mapping": {"arm/j1": "shoulder"}})
    sent: list[Any] = []
    module.tf.publish = sent.append  # type: ignore[method-assign]

    # The coordinator speaks "arm/j1"; the URDF only knows "shoulder".
    assert module.on_joint_state(JointState(name=["arm/j1"], position=[0.0], ts=1.0)) is True
    assert sent


def test_lifecycle_methods_stay_rpc() -> None:
    # An overridden start without @rpc drops the RPC registration, and the
    # coordinator then pickles the bound method and kills the whole worker.
    assert RobotTfPublisher.start.__rpc__  # type: ignore[attr-defined]
