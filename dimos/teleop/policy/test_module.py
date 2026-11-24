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

from collections.abc import Iterator
from contextlib import ExitStack
import threading
import time

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.teleop.policy.adapters import TcpPoseAdapter
from dimos.teleop.policy.module import PolicyTeleopConfig
from dimos.utils.testing.waiting import wait_until


@pytest.fixture
async def blender(modules):
    return modules()


@pytest.fixture
def published(blender) -> Iterator[list[TwistStamped]]:
    commands = []
    unsubscribe = blender.ee_twist_command.subscribe(commands.append)
    try:
        yield commands
    finally:
        unsubscribe()


async def feed(module, stamp=10.0):
    await module.handle_current_pose(Transform(ts=stamp, child_frame_id="link_tcp"))
    await module.handle_teleop_command(
        TwistStamped(ts=stamp, frame_id="world", linear=[3] * 3, angular=[3] * 3)
    )
    await module.handle_policy_command(
        TwistStamped(ts=stamp, frame_id="world", linear=[1] * 3, angular=[1] * 3)
    )


@pytest.mark.parametrize(
    "weight, expected",
    [(0, [3] * 6), (1, [1] * 6), (0.5, [2] * 6), ((0, 0.5, 1, 1, 0.5, 0), [3, 2, 1, 1, 2, 3])],
)
async def test_module_blends_axes_and_stamps_output(blender, published, weight, expected):
    blender.config.takeover_fn = lambda pose: weight
    await feed(blender)
    blender._tick(10.1)
    command = published[-1]
    assert command.linear.to_list() + command.angular.to_list() == expected
    assert (command.ts, command.frame_id) == (10.1, "world")
    assert blender.get_fault() is None


@pytest.mark.parametrize("weight", [0, 1])
async def test_unused_command_source_can_be_absent(blender, published, weight):
    blender.config.takeover_fn = lambda pose: weight
    await blender.handle_current_pose(Transform(ts=10.0, child_frame_id="link_tcp"))
    handler = blender.handle_policy_command if weight else blender.handle_teleop_command
    await handler(TwistStamped(ts=10, frame_id="world", linear=[0.04, 0, 0]))
    blender._tick(10.1)
    assert published[-1].linear.x == 0.04
    assert blender.get_fault() is None


@pytest.mark.parametrize("source", ["current_pose", "teleop_command", "policy_command"])
@pytest.mark.parametrize("stamp", [9.0, 11.0, np.nan, np.inf])
async def test_stale_future_or_nonfinite_timestamp_holds(blender, published, source, stamp):
    blender.config.takeover_fn = lambda pose: 0.5
    await feed(blender)
    sample = (
        Transform(ts=stamp, child_frame_id="link_tcp")
        if source == "current_pose"
        else TwistStamped(ts=stamp, frame_id="world")
    )
    await getattr(blender, f"handle_{source}")(sample)
    blender._tick(10.1)
    assert published[-1].is_zero()
    assert blender.get_fault() is not None


@pytest.mark.parametrize("source", ["current_pose", "teleop_command", "policy_command"])
async def test_wrong_frame_holds(blender, published, source):
    blender.config.takeover_fn = lambda pose: 0.5
    await feed(blender)
    sample = (
        Transform(ts=10, frame_id="tool", child_frame_id="link_tcp")
        if source == "current_pose"
        else TwistStamped(ts=10, frame_id="tool", linear=[1, 0, 0])
    )
    await getattr(blender, f"handle_{source}")(sample)
    blender._tick(10.1)
    assert published[-1].is_zero()
    assert blender.get_fault() is not None


@pytest.mark.parametrize(
    "pose",
    [
        Transform(ts=10, child_frame_id="other_tcp"),
        Transform(ts=10, child_frame_id="link_tcp", translation=Vector3(np.nan, 0, 0)),
        Transform(ts=10, child_frame_id="link_tcp", rotation=Quaternion(0, 0, 0, 0)),
        Transform(ts=10, child_frame_id="link_tcp", rotation=Quaternion(0, 0, 0, np.inf)),
    ],
)
async def test_invalid_pose_holds(blender, published, pose):
    await feed(blender)
    await blender.handle_current_pose(pose)
    blender._tick(10.1)
    assert published[-1].is_zero()
    assert blender.get_fault() is not None


@pytest.mark.parametrize("source", ["policy_command", "teleop_command"])
async def test_nonfinite_command_holds(blender, published, source):
    blender.config.takeover_fn = lambda pose: 0.5
    await feed(blender)
    await getattr(blender, f"handle_{source}")(
        TwistStamped(ts=10, frame_id="world", angular=[0, 0, np.nan])
    )
    blender._tick(10.1)
    assert published[-1].is_zero()
    assert "nonfinite" in blender.get_fault()


@pytest.mark.parametrize("weights", [np.nan, np.inf, -1, 2, (0, 1)])
async def test_bad_callable_return_holds(blender, published, weights):
    blender.config.takeover_fn = lambda pose: weights
    await feed(blender)
    blender._tick(10.1)
    assert published[-1].is_zero()
    assert "weights" in blender.get_fault()


async def test_callable_failure_is_reported_and_next_valid_tick_recovers(
    blender, published, mocker
):
    blender.config.takeover_fn = mocker.Mock(side_effect=RuntimeError("broken takeover"))
    await feed(blender)
    blender._tick(10.1)
    assert published[-1].is_zero()
    assert blender.get_fault() == "RuntimeError: broken takeover"
    blender.config.takeover_fn = lambda pose: pose.translation.z
    await blender.handle_current_pose(
        Transform(ts=10, child_frame_id="link_tcp", translation=Vector3(z=0.5))
    )
    blender._tick(10.1)
    assert published[-1].linear.to_list() == [2, 2, 2]
    assert blender.get_fault() is None


async def test_missing_inputs_and_stale_feedback_stop_motion(blender, published):
    blender._tick(10)
    assert published[-1].is_zero() and blender.get_fault() is not None
    await feed(blender)
    blender._tick(10.1)
    assert not published[-1].is_zero()
    await blender.handle_policy_command(TwistStamped(ts=10.3, frame_id="world", linear=[1, 0, 0]))
    blender._tick(10.3)
    assert published[-1].is_zero() and "pose" in blender.get_fault()


def test_lifecycle_binds_inputs_publishes_periodically_and_cleans_up(modules, mocker):
    module = modules(control_hz=100, timeout_s=0.1)
    publisher = mocker.spy(module.ee_twist_command, "publish")
    moving, holding = threading.Event(), threading.Event()

    def observe(command):
        if command.is_zero():
            holding.set()
        else:
            moving.set()

    with ExitStack() as stack:
        stack.callback(module.ee_twist_command.subscribe(observe))
        module.start()
        wait_until(lambda: module.get_fault() is not None, timeout=2)
        now = time.time()
        module.current_pose.transport.publish(Transform(ts=now, child_frame_id="link_tcp"))
        module.policy_command.transport.publish(
            TwistStamped(ts=now, frame_id="world", linear=[0.01, 0, 0])
        )
        assert moving.wait(2)
        holding.clear()
        assert holding.wait(2)
        assert "stale" in module.get_fault()
        module.stop()
        assert publisher.call_args.args[0].is_zero()
        assert module._loop_thread is None
        assert module._main_gen is None
        module.current_pose.transport.subscribe.return_value.assert_called_once_with()


async def test_tf_adapter_selects_tcp_without_refreshing_timestamp(modules):
    adapter = modules(TcpPoseAdapter)
    received = []
    unsubscribe = adapter.current_pose.subscribe(received.append)
    try:
        tcp = Transform(ts=10, child_frame_id="link_tcp")
        await adapter.handle_tf(
            TFMessage(
                Transform(child_frame_id="other"),
                Transform(frame_id="base", child_frame_id="link_tcp"),
                tcp,
            )
        )
        assert received == [tcp]
        assert received[0].ts == 10
    finally:
        unsubscribe()


@pytest.mark.parametrize(
    "field, value",
    [
        ("control_hz", 0),
        ("control_hz", np.inf),
        ("timeout_s", -1),
        ("timeout_s", np.nan),
        ("world_frame", ""),
        ("tcp_frame", ""),
    ],
)
def test_invalid_configuration_is_rejected(field, value):
    with pytest.raises(ValueError):
        PolicyTeleopConfig(**{field: value})
