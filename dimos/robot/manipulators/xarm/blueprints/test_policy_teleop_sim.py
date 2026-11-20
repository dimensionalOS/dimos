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

from pytest_mock import MockerFixture

from dimos.core.transport_factory import rpc_backend
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.manipulators.xarm.blueprints.policy_teleop_sim import CubeHoverPolicy


async def test_hover_uses_latest_cube_position_and_preserves_observation_age(
    mocker: MockerFixture,
) -> None:
    for method in ("start", "serve_module_rpc", "stop"):
        mocker.patch.object(rpc_backend(), method)
    policy = CubeHoverPolicy()
    commands: list[TwistStamped] = []
    policy.policy_command.subscribe(commands.append)
    tcp = Transform(ts=10.1, child_frame_id="link_tcp", translation=Vector3(0.4, 0, 0.25))
    try:
        await policy.handle_current_pose(tcp)
        assert commands == []
        cube = Transform(ts=10.0, child_frame_id="entity:cube", translation=Vector3(0.4, 0, 0.15))
        await policy.handle_body_poses(TFMessage(cube))
        await policy.handle_current_pose(tcp)
        assert commands[-1].is_zero()
        cube = Transform(
            ts=10.05, child_frame_id="entity:cube", translation=Vector3(0.4, 0.1, 0.15)
        )
        await policy.handle_body_poses(TFMessage(cube))
        await policy.handle_current_pose(tcp)
        assert commands[-1].linear.to_list() == [0, policy.config.max_speed_m_s, 0]
        assert commands[-1].angular == Vector3()
        assert commands[-1].ts == cube.ts
    finally:
        policy.stop()
