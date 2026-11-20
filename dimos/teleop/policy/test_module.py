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

from collections.abc import AsyncIterator

import pytest
from pytest_mock import MockerFixture

from dimos.core.transport_factory import rpc_backend
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.teleop.policy.module import PolicyTeleopModule


@pytest.fixture
async def blender(mocker: MockerFixture) -> AsyncIterator[PolicyTeleopModule]:
    backend = rpc_backend()
    for method in ("start", "serve_module_rpc", "stop"):
        mocker.patch.object(backend, method)
    module = PolicyTeleopModule(timeout_s=0.25)
    try:
        yield module
    finally:
        module.stop()


async def test_policy_only_moves_without_teleop_then_stops_on_missing_feedback(blender):
    published = []
    blender.ee_twist_command.subscribe(published.append)
    await blender.handle_current_pose(Transform(ts=10.0, child_frame_id="link_tcp"))
    cmd = TwistStamped(ts=10.0, frame_id="world", linear=[0.04, 0, 0])
    await blender.handle_policy_command(cmd)
    blender._tick(10.1)
    await blender.handle_policy_command(TwistStamped(ts=10.3, frame_id="world", linear=cmd.linear))
    blender._tick(10.3)
    assert published[0] == cmd
    assert published[1].is_zero()


@pytest.mark.parametrize(
    "stamp, frame, x",
    [
        (9.0, "world", 0.05),
        (11.0, "world", 0.05),
        (10.0, "base", 0.05),
        (10.0, "world", float("nan")),
    ],
)
async def test_unusable_policy_sample_outputs_zero(blender, stamp, frame, x):
    published = []
    blender.ee_twist_command.subscribe(published.append)
    await blender.handle_current_pose(Transform(ts=10.0, child_frame_id="link_tcp"))
    await blender.handle_policy_command(TwistStamped(ts=stamp, frame_id=frame, linear=[x, 0, 0]))
    blender._tick(10.1)
    assert len(published) == 1
    assert published[0].is_zero()
    assert published[0].frame_id == "world"


async def test_shared_axes_require_teleop_and_apply_pose_weights(blender):
    blender.config.takeover_fn = lambda pose: (0, 0.5, 1, 1, 0.5, 0)
    published = []
    blender.ee_twist_command.subscribe(published.append)
    await blender.handle_current_pose(Transform(ts=10.0, child_frame_id="link_tcp"))
    await blender.handle_policy_command(
        TwistStamped(ts=10.0, frame_id="world", linear=[1, 1, 1], angular=[1, 1, 1])
    )
    blender._tick(10.1)
    await blender.handle_teleop_command(
        TwistStamped(ts=10.0, frame_id="world", linear=[3, 3, 3], angular=[3, 3, 3])
    )
    blender._tick(10.1)
    assert published[0].is_zero()
    assert published[1].linear.to_list() == [3, 2, 1]
    assert published[1].angular.to_list() == [1, 2, 3]
