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

"""Both OpenArm leaders in the SF555 office with head and wrist RGB-D views."""

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.galaxea.r1pro.blueprints.teleop.r1pro_joylo import (
    _operator_blueprint,
    _robot_blueprint,
    _transports,
)
from dimos.robot.galaxea.r1pro.sim.cameras import head_camera, wrist_camera
from dimos.robot.galaxea.r1pro.sim.office import SCENE, office_renderer
from dimos.robot.usd_manager import UsdManager
from dimos.sim2.sensors.camera.module import SimCameraModule
from dimos.sim2.sensors.camera.render_module import CameraRenderModule

r1pro_openarm_joylo_office = (
    autoconnect(
        _robot_blueprint(scene=SCENE),
        _operator_blueprint(),
        SimCameraModule.blueprint(instance_name="head_camera", sensor=head_camera()),
        *(
            SimCameraModule.blueprint(
                instance_name=f"{side}_wrist_camera", sensor=wrist_camera(side)
            ).remappings(
                [
                    (SimCameraModule, "color_image", f"{side}_wrist"),
                    (SimCameraModule, "depth_image", f"{side}_wrist_depth"),
                    (SimCameraModule, "camera_info", f"{side}_wrist_info"),
                ]
            )
            for side in ("left", "right")
        ),
        CameraRenderModule.blueprint(
            scene_manager=UsdManager,
            cameras=("head_camera", "left_wrist_camera", "right_wrist_camera"),
            renderer=office_renderer,
        ),
    )
    .transports(_transports)
    .global_config(viewer="none", transport="zenoh", n_workers=7)
)
