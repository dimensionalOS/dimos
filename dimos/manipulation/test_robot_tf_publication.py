# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

from typing import Any, cast
from unittest.mock import MagicMock, patch

from dimos.manipulation.manipulation_module import ManipulationModule, ManipulationModuleConfig
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState


def test_joint_state_tf_publication_emits_complete_parent_child_tree() -> None:
    module = object.__new__(ManipulationModule)
    state = cast("dict[str, Any]", module.__dict__)
    state["config"] = ManipulationModuleConfig(planning_world_frame="world")
    state["_robot_tf_edges"] = {"arm": [("base", "link1"), ("link1", "link2")]}
    monitor = MagicMock()
    poses = {
        "base": PoseStamped(frame_id="world", position=Vector3(1.0, 0.0, 0.0)),
        "link1": PoseStamped(frame_id="world", position=Vector3(1.0, 2.0, 0.0)),
        "link2": PoseStamped(frame_id="world", position=Vector3(1.0, 2.0, 3.0)),
    }
    joint_state = JointState(name=["joint1"], position=[0.25], ts=42.0)
    monitor.get_link_pose.side_effect = lambda _robot_id, link, _joint_state: poses[link]
    state["_world_monitor"] = monitor
    tf = MagicMock()
    state["_tf"] = tf
    robot = RobotModelConfig(name="arm", model_path="unused.urdf", joint_names=[], base_link="base")

    module._publish_robot_tf("arm", "robot-id", robot, joint_state)

    transforms = tf.publish.call_args.args
    assert [(item.frame_id, item.child_frame_id) for item in transforms] == [
        ("world", "base"),
        ("base", "link1"),
        ("link1", "link2"),
    ]
    assert all(item.ts == 42.0 for item in transforms)
    assert transforms[1].translation.y == 2.0
    assert transforms[2].translation.z == 3.0
    assert all(call.args[2] is joint_state for call in monitor.get_link_pose.call_args_list)


def test_stop_disposes_input_subscriptions_before_state_monitors() -> None:
    module = object.__new__(ManipulationModule)
    state = cast("dict[str, Any]", module.__dict__)
    events: list[str] = []
    joint_subscription = MagicMock()
    joint_subscription.dispose.side_effect = lambda: events.append("joint subscription")
    voxel_subscription = MagicMock()
    voxel_subscription.dispose.side_effect = lambda: events.append("voxel subscription")
    monitor = MagicMock()
    monitor.stop_all_monitors.side_effect = lambda: events.append("state monitors")
    state["_joint_state_subscription"] = joint_subscription
    state["_planning_voxel_subscription"] = voxel_subscription
    state["_world_monitor"] = monitor

    with patch(
        "dimos.manipulation.manipulation_module.Module.stop",
        side_effect=lambda: events.append("module"),
    ):
        module.stop()

    assert events == [
        "joint subscription",
        "voxel subscription",
        "state monitors",
        "module",
    ]
