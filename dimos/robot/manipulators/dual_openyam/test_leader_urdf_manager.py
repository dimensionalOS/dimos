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

import numpy as np

from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.protocol.tf.tf import TF
from dimos.robot.manipulators.dual_openyam.config import dual_openyam_model_config
from dimos.robot.manipulators.dual_openyam.leader_urdf_manager import DualOpenYamLeaderUrdfManager


def test_leader_copy_follows_commands_and_ignores_a_paused_one() -> None:
    manager = DualOpenYamLeaderUrdfManager(
        model=dual_openyam_model_config(robot="arm").model, publish_joint_tf=False
    )
    manager.__dict__["_tf"] = TF()

    manager.add_leader_joint_state(
        JointState(ts=1.0, name=["arm/left_joint1"], position=[0.5], velocity=[])
    )

    # The leader link sits where the follower's link would at that joint position.
    follower = {
        (t.frame_id, t.child_frame_id): t
        for t in manager.joint_transforms(JointState(name=["arm/left_joint1"], position=[0.5]))
    }
    leader = {(t.frame_id, t.child_frame_id): t for t in manager.leader_transforms(1.0)}
    expected = follower[("left_base", "left_link1")]
    actual = leader[("leader_left_base", "leader_left_link1")]
    assert np.allclose(actual.to_matrix(), expected.to_matrix())
    assert "<group>leader</group>" in manager._description.xml

    # Following paused: names but no positions. The copy keeps its pose.
    manager.add_leader_joint_state(JointState(ts=2.0, name=["arm/left_joint1"], position=[]))
    after = {(t.frame_id, t.child_frame_id): t for t in manager.leader_transforms(2.0)}
    assert np.allclose(
        after[("leader_left_base", "leader_left_link1")].to_matrix(), expected.to_matrix()
    )

    manager.__dict__["_tf"] = None
    manager.dispose()
