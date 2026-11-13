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

from dimos_lcm.common_msgs import SampleHeader
from dimos_lcm.geometry_msgs import Pose, Quaternion
from dimos_lcm.sim_msgs import BodyState, WorldState
import numpy as np
import pytest

from dimos.robot.galaxea.r1pro.sim.camera_module import R1ProSimCameras
from dimos.robot.galaxea.r1pro.sim.cameras import head_camera, wrist_camera
from dimos.sim2.description import SceneConfig
from dimos.sim2.sensors.camera.renderers.unreal.renderer import UnrealRenderer


@pytest.fixture
def rig(mocker):
    renderer = mocker.Mock(spec=UnrealRenderer)
    cameras = (head_camera(), wrist_camera("left"), wrist_camera("right"))
    renderer.bodies = {c.frame_id for c in cameras}
    renderer.render_camera.return_value = (
        np.zeros((1, 4, 3), dtype=np.uint8),
        np.array([[0.06, 0.1, 0.5, 0.51]], dtype=np.float32),
    )
    factory = mocker.Mock(return_value=renderer)
    module = R1ProSimCameras(
        scene=SceneConfig(()), head=cameras[0], left=cameras[1], right=cameras[2], renderer=factory
    )
    ports = [
        (module.color_image, module.depth_image, module.camera_info),
        (module.left_wrist, module.left_wrist_depth, module.left_wrist_info),
        (module.right_wrist, module.right_wrist_depth, module.right_wrist_info),
    ]
    for group in ports:
        for port in group:
            mocker.patch.object(port, "publish")
    try:
        module.open_sensor(module.config.scene)
        yield module, renderer, ports, cameras
    finally:
        module.stop()
        module.close_sensor()


def sample(cameras, stamp):
    return WorldState(
        header=SampleHeader(clock_id="test", time_ns=stamp),
        bodies_length=3,
        bodies=[
            BodyState(body_id=c.frame_id, pose=Pose(orientation=Quaternion(w=1))) for c in cameras
        ],
    )


def test_shared_renderer_fairly_captures_fresh_views_with_own_calibration(rig):
    module, renderer, ports, cameras = rig
    module.acquire(sample(cameras, 1), latest=False)
    renderer.render_camera.assert_not_called()
    for i in range(6):
        module.acquire(sample(cameras, (i + 1) * 100_000_000))
    assert renderer.load.call_count == 1
    assert [call.args[0].frame_id for call in renderer.render_camera.call_args_list] == [
        c.frame_id for c in cameras
    ] * 2
    for i, (color, depth, info) in enumerate(ports):
        rgb, d, k = (
            color.publish.call_args.args[0],
            depth.publish.call_args.args[0],
            info.publish.call_args.args[0],
        )
        assert rgb.frame_id == d.frame_id == k.frame_id == cameras[i].frame_id
        assert rgb.ts == d.ts == k.ts == (i + 4) / 10
        assert k.K[0] == cameras[i].fx
    np.testing.assert_allclose(ports[0][1].publish.call_args.args[0].data, [[0.06, 0.1, 0.5, 0.51]])
    for i in (1, 2):
        np.testing.assert_allclose(ports[i][1].publish.call_args.args[0].data, [[0, 0.1, 0.5, 0]])
    assert module.statistics() == {
        "acquisitions": 6,
        "workspace": 2,
        "left": 2,
        "right": 2,
        "dropped_inputs": 0,
        "gaps": 0,
        "error": "",
    }
