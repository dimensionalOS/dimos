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
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.unitree.g1.sim2 import usd_joint
from dimos.robot.unitree.g1.sim2_walker import GROOT_PERIOD, G1SimWalker, GrootPolicy
from dimos.sim2.world_observations import WorldMirror, WorldWriter
from dimos.utils.data import LfsPath

ROBOT = "office/g1"
STANDING = (0.0, 0.0, 0.793, 0.0, 0.0, 0.0, 1.0)


def _standing_world(default_pose: NDArray[np.float32]) -> WorldMirror:
    """A world holding one G1 standing still in its policy's default pose."""
    writer, mirror = WorldWriter(), WorldMirror()
    writer.add(ROBOT)
    for name, position in zip(g1_joints, default_pose, strict=True):
        writer.add(f"{ROBOT}/{usd_joint(name)}")
        writer.joint(f"{ROBOT}/{usd_joint(name)}", position, 0.0, 0.0)
    writer.body_state(ROBOT, STANDING, [0.0] * 6, [0.0] * 6)
    mirror.apply(writer.end_frame(0.0))
    return mirror


@pytest.mark.self_hosted  # GR00T's ONNX policies are in Git LFS
def test_the_walker_commands_every_joint_stamped_with_wall_time() -> None:
    policy = GrootPolicy(LfsPath("groot"))
    walker = G1SimWalker(instance_name=f"{ROBOT}/G1SimWalker")
    sent: list[JointState] = []
    try:
        walker._policy, walker._world = policy, _standing_world(policy.default_pose)
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
    # Standing still in its default pose, GR00T keeps every joint near it.
    assert sent[-1].position == pytest.approx(list(policy.default_pose), abs=0.3)


def test_the_walker_steps_at_its_policy_rate_of_wall_time(monkeypatch: pytest.MonkeyPatch) -> None:
    walker = G1SimWalker(instance_name=f"{ROBOT}/G1SimWalker")
    acted: list[float] = []
    seconds = 10 * GROOT_PERIOD

    async def walk_for_a_while() -> None:
        walking = asyncio.ensure_future(walker._walk())
        await asyncio.sleep(seconds)
        walking.cancel()

    try:
        walker._world = _standing_world(np.zeros(len(g1_joints), dtype=np.float32))
        monkeypatch.setattr(walker, "act", lambda: acted.append(time.time()))
        asyncio.run(walk_for_a_while())
    finally:
        walker.stop()
    # One step per period from the first, give or take the one the cancel lands on.
    assert abs(len(acted) - (seconds / GROOT_PERIOD + 1)) <= 1, len(acted)
    assert np.diff(acted).min() >= GROOT_PERIOD * 0.5, np.diff(acted)
