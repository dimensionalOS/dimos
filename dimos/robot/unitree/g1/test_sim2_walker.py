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

import numpy as np
from numpy.typing import NDArray
import pytest

from dimos.control.tasks.g1_groot_wbc_task.g1_groot_wbc_task import g1_joints
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.unitree.g1.sim2 import usd_joint
from dimos.robot.unitree.g1.sim2_walker import GROOT_PERIOD, G1SimWalker, GrootPolicy
from dimos.sim2.world_observations import ACT, WorldMirror, WorldWriter
from dimos.utils.data import LfsPath

ROBOT = "office/g1"
STANDING = (0.0, 0.0, 0.793, 0.0, 0.0, 0.0, 1.0)


class _World:
    """A world holding one G1 standing still in its policy's default pose."""

    def __init__(self, default_pose: NDArray[np.float32]) -> None:
        self.writer = WorldWriter()
        self.mirror = WorldMirror()
        self.writer.add(ROBOT)
        for name, position in zip(g1_joints, default_pose, strict=True):
            self.writer.add(f"{ROBOT}/{usd_joint(name)}")
            self.writer.joint(f"{ROBOT}/{usd_joint(name)}", position, 0.0, 0.0)
        self.writer.body_state(ROBOT, STANDING, [0.0] * 6, [0.0] * 6)

    def tick(self, sim_time: float, act: bool) -> None:
        if act:
            self.writer.set(ROBOT, ACT, True)
        self.mirror.apply(self.writer.end_frame(sim_time))


@pytest.mark.self_hosted  # GR00T's ONNX policies are in Git LFS
def test_the_walker_sends_one_command_per_control_tick_stamped_in_sim_time() -> None:
    policy = GrootPolicy(LfsPath("groot"))
    world = _World(policy.default_pose)
    walker = G1SimWalker(instance_name=f"{ROBOT}/G1SimWalker")
    sent: list[JointState] = []
    try:
        walker._policy, walker._world = policy, world.mirror
        walker.position_command.subscribe(sent.append)
        ticks = [0.0, GROOT_PERIOD, 2 * GROOT_PERIOD]
        stamps: list[float] = []
        for sim_time in ticks:
            world.tick(sim_time - GROOT_PERIOD / 2, act=False)
            world.tick(sim_time, act=True)
            stamps.append(world.mirror.stamp())
            walker.act()
            # The world says ACT again while it waits; that is the same tick.
            world.tick(sim_time, act=True)
            walker.act()
    finally:
        walker.stop()
    assert [command.ts for command in sent] == pytest.approx(stamps)
    assert sent[1].ts - sent[0].ts == pytest.approx(GROOT_PERIOD, abs=1e-6)
    assert all(command.name == g1_joints for command in sent)
    # Standing still in its default pose, GR00T keeps every joint near it.
    assert sent[-1].position == pytest.approx(list(policy.default_pose), abs=0.3)
