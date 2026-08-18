# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

from __future__ import annotations

from collections.abc import Callable, Iterator
from pathlib import Path
from typing import Any

import pytest

from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.robot_tf_publisher import RobotTfPublisher


@pytest.fixture
def robot_model(tmp_path: Path) -> RobotModelConfig:
    urdf = tmp_path / "robot.urdf"
    urdf.write_text(
        """<robot name="test">
  <link name="base"/>
  <link name="link"/>
  <joint name="joint" type="revolute">
    <parent link="base"/><child link="link"/>
    <origin xyz="0 0 1" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="1" velocity="1"/>
  </joint>
</robot>"""
    )
    return RobotModelConfig(
        name="robot",
        model_path=urdf,
        base_pose=PoseStamped(position=[1.0, 2.0, 3.0]),
        joint_names=["joint"],
        base_link="base",
    )


@pytest.fixture
def make_publisher() -> Iterator[Callable[..., RobotTfPublisher]]:
    modules: list[RobotTfPublisher] = []

    def make(**kwargs: object) -> RobotTfPublisher:
        module = RobotTfPublisher(**kwargs)
        modules.append(module)
        return module

    yield make
    for module in modules:
        module.dispose()


def test_joint_state_publishes_complete_parent_child_tree(
    robot_model: RobotModelConfig,
    make_publisher: Callable[..., RobotTfPublisher],
    mocker: Any,
) -> None:
    module = make_publisher(robot_model=robot_model, fixed_frame="map")
    publish = mocker.patch.object(module.tf, "publish")

    assert module.on_joint_state(JointState(ts=12.5, name=["joint"], position=[0.5])) is True

    base, link = publish.call_args.args[0].transforms
    assert base.frame_id == "map"
    assert base.child_frame_id == "base"
    assert list(base.translation) == pytest.approx([1.0, 2.0, 3.0])
    assert base.ts == 12.5
    assert link.frame_id == "base"
    assert link.child_frame_id == "link"
    assert list(link.translation) == pytest.approx([0.0, 0.0, 1.0])
    assert link.ts == 12.5


def test_incomplete_joint_state_publishes_nothing(
    robot_model: RobotModelConfig,
    make_publisher: Callable[..., RobotTfPublisher],
    mocker: Any,
) -> None:
    module = make_publisher(robot_model=robot_model)
    publish = mocker.patch.object(module.tf, "publish")

    assert module.on_joint_state(JointState(ts=12.5, name=[], position=[])) is False
    publish.assert_not_called()


def test_stream_contract_uses_joint_state_and_tf_message() -> None:
    assert RobotTfPublisher.__annotations__["coordinator_joint_state"]
    assert RobotTfPublisher.__annotations__["tf"]
    assert TFMessage
