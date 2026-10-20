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

"""The coordinator publishes tasks' measured frame poses on tf."""

from __future__ import annotations

import threading
from unittest.mock import MagicMock

import pytest

from dimos.control.components import HardwareComponent, HardwareType, make_joints
from dimos.control.coordinator import ControlCoordinator
from dimos.control.hardware_interface import ConnectedHardware
from dimos.control.task import BaseControlTask, CoordinatorState, ResourceClaim
from dimos.control.tick_loop import TickLoop
from dimos.hardware.manipulators.spec import ManipulatorAdapter
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.manipulators.common.coordinators import ArmTwistCoordinator

JOINTS = make_joints("arm", 2)


class _PoseTask(BaseControlTask):
    """Reports link_tcp at x = the first measured joint, like FK would."""

    _name = "ik"

    def claim(self) -> ResourceClaim:
        return ResourceClaim(joints=frozenset())

    def is_active(self) -> bool:
        return False

    def compute(self, state: CoordinatorState) -> None:
        return None

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        pass

    def measured_frame_poses(self, state: CoordinatorState) -> dict[str, PoseStamped]:
        x = state.joints.get_position(JOINTS[0])
        return {"link_tcp": PoseStamped(frame_id="link_base", position=(x, 0.0, 0.5))}


def test_measured_frame_poses_are_published_as_world_tf_at_a_limited_rate() -> None:
    adapter = MagicMock(spec=ManipulatorAdapter)
    adapter.read_joint_positions.return_value = [0.25, 0.5]
    adapter.read_joint_velocities.return_value = [0.0, 0.0]
    adapter.read_joint_efforts.return_value = [0.0, 0.0]
    component = HardwareComponent(
        hardware_id="arm", hardware_type=HardwareType.MANIPULATOR, joints=JOINTS
    )
    joint_states = MagicMock()
    published: list[TFMessage] = []
    loop = TickLoop(
        tick_rate=100.0,
        hardware={"arm": ConnectedHardware(adapter, component)},
        hardware_lock=threading.Lock(),
        tasks={"ik": _PoseTask()},
        task_lock=threading.Lock(),
        joint_to_hardware={},
        publish_callback=joint_states,
        publish_tf_callback=published.append,
    )
    for _ in range(5):
        loop._tick()  # all within one 30 Hz period

    (message,) = published
    (transform,) = message.transforms
    assert (transform.frame_id, transform.child_frame_id) == ("world", "link_tcp")
    assert transform.translation.to_numpy().tolist() == [0.25, 0.0, 0.5]
    assert transform.ts == joint_states.call_args_list[0].args[0].ts


def test_only_coordinators_declaring_tf_can_publish_frame_poses() -> None:
    plain = ControlCoordinator(publish_frame_poses=True)
    arm = ArmTwistCoordinator(publish_frame_poses=True)
    try:
        with pytest.raises(ValueError, match="add `tf: Out\\[TFMessage\\]`"):
            plain._frame_pose_port()
        assert arm._frame_pose_port() is arm.tf
        assert "tf" not in plain.outputs
    finally:
        plain.stop()
        arm.stop()
