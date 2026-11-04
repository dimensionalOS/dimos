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

from dimos_lcm.control_msgs import JointTarget
import numpy as np
import pytest

from dimos.sim2.geometry.poses import poses_by_id
from dimos.sim2.sensors.camera.config import CameraConfig
from dimos.sim2.sensors.camera.renderers.mujoco import MujocoRenderer


@pytest.fixture
def renderer(world):
    renderer = MujocoRenderer()
    renderer.load(world, CameraConfig("left/camera"))
    yield renderer
    renderer.close()


def test_camera_renders_recorded_body_poses_without_physics_state(backend, poses, renderer):
    sample = poses()
    renderer.set_poses(sample)
    rgb, depth = renderer.render(poses_by_id(sample)["left/camera"])
    assert rgb.std() > 5
    assert depth.shape == (240, 320) and np.isfinite(depth).all()
    assert renderer.geometry.model.nq == renderer.geometry.model.nv == 0
    backend.apply_action([JointTarget(joint_id="left/slide_x", position=0.2, kp=200, kd=20)], [])
    for _ in range(300):
        backend.step()
    moved = poses()
    renderer.set_poses(moved)
    after, _ = renderer.render(poses_by_id(moved)["left/camera"])
    assert np.mean(np.abs(after.astype(float) - rgb)) > 2
    renderer.set_poses(sample)
    replay, _ = renderer.render(poses_by_id(sample)["left/camera"])
    np.testing.assert_array_equal(replay, rgb)
