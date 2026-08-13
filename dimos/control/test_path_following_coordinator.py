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

"""A Path published on the coordinator's port must reach the follower via its
card and arm it — the wiring the go2 controller blueprints depend on."""

from __future__ import annotations

from typing import Any

import pytest

from dimos.control.benchmarking.paths import straight_rotate
from dimos.control.components import HardwareComponent, HardwareType, make_twist_base_joints
from dimos.control.coordinator import TaskConfig
from dimos.control.path_following_coordinator import PathFollowingCoordinator
from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.registry import control_task_registry
from dimos.msgs.std_msgs.Float32 import Float32

JOINTS = make_twist_base_joints("go2")


@pytest.fixture
def coordinator(mocker) -> Any:
    mocker.patch("dimos.control.coordinator.TickLoop")
    coord = PathFollowingCoordinator(
        publish_joint_state=False,
        hardware=[
            HardwareComponent(
                hardware_id="go2",
                hardware_type=HardwareType.BASE,
                joints=JOINTS,
                adapter_type="mock_twist_base",
            )
        ],
        tasks=[
            TaskConfig(
                name="follower",
                type="holonomic_pose_follower",
                joint_names=JOINTS,
                priority=10,
                params={"speed": 0.5},
            )
        ],
    )
    taps: dict[str, list] = {}
    for stream, port in coord.inputs.items():
        cbs: list = []
        taps[stream] = cbs
        mocker.patch.object(
            port, "subscribe", side_effect=lambda cb, _c=cbs: (_c.append(cb), mocker.Mock())[1]
        )
    coord.start()
    yield coord, taps
    coord.stop()


def _emit(taps: dict[str, list], stream: str, msg: Any) -> None:
    assert taps[stream], f"nothing subscribed to {stream!r} — card routing did not wire it"
    for cb in taps[stream]:
        cb(msg)


def test_published_path_reaches_the_follower_and_arms_it(coordinator):
    coord, taps = coordinator
    follower = coord.get_task("follower")

    _emit(taps, "path", straight_rotate(length=2.0))
    state = CoordinatorState(
        joints=JointStateSnapshot(
            joint_positions=dict.fromkeys(JOINTS, 0.0),
            joint_velocities=dict.fromkeys(JOINTS, 0.0),
        ),
        t_now=0.0,
        dt=0.1,
    )
    follower.compute(state)
    assert follower.get_state() == "tracking"


def test_published_speed_retunes_the_follower(coordinator):
    coord, taps = coordinator
    _emit(taps, "speed", Float32(data=0.8))
    assert coord.get_task("follower")._config.speed == pytest.approx(0.8)


def test_follower_cards_name_the_handlers_the_tasks_implement():
    for task_type in ("holonomic_pose_follower", "rpp_path_follower", "path_follower"):
        streams = {
            b.stream: b.handler for b in control_task_registry.bindings_for(task_type).consumes
        }
        assert streams == {"path": "on_path", "speed": "on_speed"}, f"{task_type}: {streams}"
