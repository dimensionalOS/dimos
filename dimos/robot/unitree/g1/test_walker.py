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

import asyncio
import time

import numpy as np
from numpy.typing import NDArray
import pytest

from dimos.control.tasks.g1_groot_wbc_task.g1_groot_wbc_task import g1_joints
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.unitree.g1.walker import G1Walker, GrootPolicy, wall_ticks
from dimos.utils.data import LfsPath

ROBOT = "office/g1"


def _standing(walker: G1Walker, pose: NDArray[np.float32]) -> None:
    """Readings of a G1 standing still, upright, in ``pose``."""
    zeros = [0.0] * len(g1_joints)
    asyncio.run(
        walker.handle_joint_state(
            JointState(name=list(g1_joints), position=pose.tolist(), velocity=zeros, effort=zeros)
        )
    )
    asyncio.run(
        walker.handle_imu(
            Imu(
                angular_velocity=Vector3(0.0, 0.0, 0.0),
                linear_acceleration=Vector3(0.0, 0.0, 9.81),
                orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
            )
        )
    )


def test_it_commands_nothing_until_it_has_both_joint_states_and_an_imu_reading() -> None:
    walker = G1Walker(instance_name=f"{ROBOT}/G1Walker")
    sent: list[JointState] = []
    try:
        walker._policy = object()  # type: ignore[assignment]
        walker.position_command.subscribe(sent.append)
        walker.act()
    finally:
        walker.stop()
    assert sent == []


@pytest.mark.self_hosted  # GR00T's ONNX policies are in Git LFS
def test_standing_still_it_commands_every_joint_near_its_default_pose_on_wall_time() -> None:
    policy = GrootPolicy(LfsPath("groot"))
    walker = G1Walker(instance_name=f"{ROBOT}/G1Walker")
    sent: list[JointState] = []
    try:
        walker._policy = policy
        _standing(walker, policy.default_pose)
        walker.position_command.subscribe(sent.append)
        before = time.time()
        for _ in range(3):
            walker.act()
        after = time.time()
    finally:
        walker.stop()
    assert len(sent) == 3
    assert all(before <= command.ts <= after for command in sent)
    assert all(command.name == g1_joints for command in sent)
    assert sent[-1].position == pytest.approx(list(policy.default_pose), abs=0.3)


def test_wall_ticks_keep_their_rate_skip_what_a_late_one_missed_and_let_other_tasks_run() -> None:
    rate, count = 100.0, 30
    ticked: list[float] = []

    async def tick() -> None:
        async for stamp in wall_ticks(rate):
            ticked.append(stamp)
            if len(ticked) == count // 2:
                # Busy for five periods: those ticks are skipped, not caught up on.
                time.sleep(5 / rate)
            if len(ticked) == count:
                return

    asyncio.run(tick())
    gaps = np.diff(ticked)
    assert gaps.min() > 0.5 / rate, gaps
    # Every tick on time but the late one: no drift from each sleep's overrun.
    on_time = np.delete(gaps, count // 2 - 1)
    assert np.median(on_time) == pytest.approx(1 / rate, rel=0.05)

    async def behind() -> int:
        handled = 0

        async def handler() -> None:
            nonlocal handled
            while True:
                await asyncio.sleep(0)
                handled += 1

        handling = asyncio.ensure_future(handler())
        ticks = 0
        async for _ in wall_ticks(10_000):
            # Work that never yields, longer than a period.
            time.sleep(0.01)
            ticks += 1
            if ticks == 10:
                break
        handling.cancel()
        return handled

    assert asyncio.run(behind()) >= 5, "always behind, its ticks starve the loop's other tasks"
