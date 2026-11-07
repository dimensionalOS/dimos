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

"""Real offscreen projection/camera-switch checks; requires a graphics context."""

from dimos_lcm.common_msgs import SampleHeader
from dimos_lcm.geometry_msgs import Point, Pose, Quaternion
from dimos_lcm.sim_msgs import BodyState, WorldState
import numpy as np
import pytest

from dimos.robot.assets.model import RobotModel
from dimos.sim2.description import Instance, SceneConfig
from dimos.sim2.sensors.camera.config import CameraConfig
from dimos.sim2.sensors.camera.renderers.mujoco import MujocoRenderer

pytestmark = pytest.mark.self_hosted


@pytest.fixture
def renderer(tmp_path):
    path = tmp_path / "target.urdf"
    path.write_text(
        '<robot name="target"><link name="optical"><visual><origin xyz="0.2 -0.1 2"/><geometry><box size="1 1 0.2"/></geometry></visual></link></robot>'
    )
    scene = SceneConfig((Instance("target", RobotModel.from_file(path)),), ground_plane=False)
    camera = CameraConfig("optical", width=200, height=150, fx=100, fy=120, cx=75, cy=55)
    renderer = MujocoRenderer()
    renderer.load(scene, camera)
    renderer.set_poses(
        WorldState(
            header=SampleHeader(clock_id="test"),
            model_revision=scene.revision,
            bodies_length=1,
            bodies=[BodyState(body_id="optical", pose=Pose(orientation=Quaternion(w=1)))],
        )
    )
    try:
        yield renderer
    finally:
        renderer.close()


def test_asymmetric_intrinsics_and_switching_resolutions_keep_registered_depth(renderer):
    pose = Pose(orientation=Quaternion(w=1))
    first_rgb, first_depth = renderer.render(pose)
    yy, xx = np.where(np.isfinite(first_depth))
    assert first_depth[55, 75] == pytest.approx(1.9, abs=1e-4)
    assert xx.mean() == pytest.approx(75 + 100 * 0.2 / 1.9, abs=1)
    assert yy.mean() == pytest.approx(55 - 120 * 0.1 / 1.9, abs=1)
    assert (xx.max() - xx.min() + 1) == pytest.approx(100 / 1.9, abs=1)
    assert (yy.max() - yy.min() + 1) == pytest.approx(120 / 1.9, abs=1)
    other = CameraConfig("optical", width=160, height=100, fx=80, fy=100, cx=80, cy=50)
    rgb, depth = renderer.render_camera(
        other, Pose(position=Point(x=0.2, y=-0.1), orientation=Quaternion(w=1))
    )
    assert rgb.shape == (100, 160, 3)
    assert depth[50, 80] == pytest.approx(1.9, abs=1e-4)
    replay_rgb, replay_depth = renderer.render(pose)
    np.testing.assert_array_equal(replay_rgb, first_rgb)
    np.testing.assert_array_equal(replay_depth, first_depth)
