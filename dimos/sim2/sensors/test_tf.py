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

"""Simulation transforms use physical poses and acquisition timestamps."""

import numpy as np
import pytest

from dimos.protocol.tf.tf import TF
from dimos.sim2.sensors.tf import SimTfModule


@pytest.fixture
def sim_tf(mocker):
    module = SimTfModule(hz=50)
    try:
        mocker.patch.object(module, "_tf", TF())
        publish = mocker.patch.object(module.tfbuffer, "publish")
        yield module, publish
    finally:
        module.stop()


async def test_physical_pose_edges_reconstruct_world_poses_at_capture_time(sim_tf, poses):
    module, publish = sim_tf
    sample = poses(time_ns=2_000_000_000)
    await module.handle_observations(sample)
    edges = list(publish.call_args.args)
    absolute = {"world": np.eye(4)}
    while edges:
        ready = [edge for edge in edges if edge.frame_id in absolute]
        assert ready, "TF tree is disconnected"
        for edge in ready:
            assert edge.ts == 2.0
            absolute[edge.child_frame_id] = absolute[edge.frame_id] @ edge.to_matrix()
            edges.remove(edge)
    for body in sample.bodies:
        p = body.pose.position
        np.testing.assert_allclose(absolute[body.body_id][:3, 3], [p.x, p.y, p.z], atol=1e-8)
    assert len(absolute) > 3


async def test_rate_and_epoch_changes_do_not_replay_old_transforms(sim_tf, poses):
    module, publish = sim_tf
    for time_ns, epoch in [(0, 0), (1_000_000, 0), (20_000_000, 0), (0, 1), (40_000_000, 0)]:
        await module.handle_observations(poses(time_ns=time_ns, epoch=epoch))
    assert [call.args[0].ts for call in publish.call_args_list] == [0.0, 0.02, 0.0]
    sample = poses(time_ns=20_000_000, epoch=1)
    sample.header.clock_id = "different-clock"
    with pytest.raises(ValueError, match="clock changed"):
        await module.handle_observations(sample)
    assert publish.call_count == 3
