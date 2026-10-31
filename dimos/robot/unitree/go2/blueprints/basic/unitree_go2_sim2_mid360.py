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

"""The sim2 Go2 with a Mid-360 on the real rig's mount, its fitted pattern and mount occlusion."""

import math

from scipy.spatial.transform import Rotation

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.protocol.tf.static_tf_publisher import StaticTfPublisher, frames_to_edge_transforms
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import rerun_config
from dimos.robot.unitree.go2.go2_mid360_static_transforms import CAMERA_XYZ, FRAMES
from dimos.robot.unitree.go2.sim2 import Go2Sim
from dimos.sim2.camera import SimCamera
from dimos.sim2.livox import LivoxSim
from dimos.sim2.module import Simulation
from dimos.sim2.spec import Mount
from dimos.visualization.vis_module import vis_module

_EDGES = {edge.child_frame_id: edge for edge in frames_to_edge_transforms(FRAMES)}
_MID360 = _EDGES["front_camera"].to_matrix() @ _EDGES["mid360_link"].to_matrix()


class Go2Mid360Mount(StaticTfPublisher):
    """base_link -> front_camera -> mid360_link; Go2Sim already publishes the camera frames."""

    def transforms(self) -> list[Transform]:
        return [_EDGES["front_camera"], _EDGES["mid360_link"]]


unitree_go2_sim2_mid360 = autoconnect(
    Simulation.blueprint(),
    vis_module(viewer_backend=global_config.viewer, rerun_config=rerun_config),
    Go2Sim.blueprint(),
    Go2Mid360Mount.blueprint(),
    LivoxSim.blueprint(
        robot_id="go2",
        mount=Mount(
            "base",
            tuple(_MID360[:3, 3]),
            tuple(Rotation.from_matrix(_MID360[:3, :3]).as_euler("xyz")),
        ),
        go2_occlusion=True,
    ),
    # MuJoCo cameras look down their -z; this one looks along the body's +x.
    SimCamera.blueprint(
        robot_id="go2", mount=Mount("base", CAMERA_XYZ, (math.pi / 2, 0.0, -math.pi / 2))
    ),
).global_config(n_workers=4, robot_model="unitree_go2")
