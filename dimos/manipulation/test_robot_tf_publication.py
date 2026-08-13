# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

from typing import Any, cast
from unittest.mock import MagicMock

from dimos.manipulation.manipulation_module import ManipulationModule, ManipulationModuleConfig
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3


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
    monitor.get_link_pose.side_effect = lambda _robot_id, link: poses[link]
    state["_world_monitor"] = monitor
    tf = MagicMock()
    state["_tf"] = tf
    robot = RobotModelConfig(name="arm", model_path="unused.urdf", joint_names=[], base_link="base")

    module._publish_robot_tf("arm", "robot-id", robot, 42.0)

    transforms = tf.publish.call_args.args
    assert [(item.frame_id, item.child_frame_id) for item in transforms] == [
        ("world", "base"),
        ("base", "link1"),
        ("link1", "link2"),
    ]
    assert all(item.ts == 42.0 for item in transforms)
    assert transforms[1].translation.y == 2.0
    assert transforms[2].translation.z == 3.0
