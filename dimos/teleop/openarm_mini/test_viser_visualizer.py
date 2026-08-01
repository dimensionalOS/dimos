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

from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.openarm.config import openarm_model_config
from dimos.teleop.openarm_mini.viser_visualizer import (
    OpenArmJointStateViserModule,
    missing_joint_names,
    ordered_joint_state,
)


class _FakeRuntime:
    def __init__(self) -> None:
        self.closed = False

    def close(self) -> None:
        self.closed = True


class _FakeScene:
    def __init__(self) -> None:
        self.updates: list[tuple[str, JointState]] = []
        self.closed = False

    def update_current_robot(self, robot_id: str, joint_state: JointState | None) -> None:
        assert joint_state is not None
        self.updates.append((robot_id, joint_state))

    def close(self) -> None:
        self.closed = True


def _shuffled_left_command() -> JointState:
    return JointState(
        {
            "name": [
                "openarm_left_joint3",
                "openarm_left_joint1",
                "openarm_left_joint2",
                "openarm_left_joint5",
                "openarm_left_joint4",
                "openarm_left_joint7",
                "openarm_left_joint6",
                "gripper",
            ],
            "position": [3.0, 1.0, 2.0, 5.0, 4.0, 7.0, 6.0, 99.0],
        }
    )


def test_ordered_joint_state_reorders_by_required_model_joint_names() -> None:
    model = openarm_model_config("left")

    ordered = ordered_joint_state(_shuffled_left_command(), model.joint_names)

    assert ordered is not None
    assert ordered.name == model.joint_names
    assert ordered.position == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]


def test_ordered_joint_state_returns_none_and_reports_missing_required_joints() -> None:
    model = openarm_model_config("left")
    command = JointState(
        {
            "name": model.joint_names[:-1],
            "position": [0.0] * 6,
        }
    )

    assert ordered_joint_state(command, model.joint_names) is None
    assert missing_joint_names(command, model.joint_names) == ("openarm_left_joint7",)


def test_viser_module_renders_ordered_joint_state_without_follower_hardware() -> None:
    model = openarm_model_config("left")
    module = OpenArmJointStateViserModule(model, robot_id="left-vis")
    runtime = _FakeRuntime()
    scene = _FakeScene()
    module._runtime = runtime  # type: ignore[assignment]
    module._scene = scene  # type: ignore[assignment]

    rendered = module.render_joint_command(_shuffled_left_command())
    module.stop()

    assert rendered is True
    assert scene.updates[0][0] == "left-vis"
    assert scene.updates[0][1].name == model.joint_names
    assert scene.updates[0][1].position == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]
    assert scene.closed is True
    assert runtime.closed is True


def test_viser_module_skips_incomplete_joint_state() -> None:
    model = openarm_model_config("left")
    module = OpenArmJointStateViserModule(model, robot_id="left-vis")
    runtime = _FakeRuntime()
    scene = _FakeScene()
    module._runtime = runtime  # type: ignore[assignment]
    module._scene = scene  # type: ignore[assignment]
    command = JointState({"name": model.joint_names[:-1], "position": [0.0] * 6})

    rendered = module.render_joint_command(command)
    module.stop()

    assert rendered is False
    assert scene.updates == []
