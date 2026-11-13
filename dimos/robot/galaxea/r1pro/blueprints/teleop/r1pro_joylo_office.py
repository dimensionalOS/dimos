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
from dimos.robot.galaxea.r1pro.sim.camera_module import R1ProSimCameras
from dimos.robot.galaxea.r1pro.sim.cameras import head_camera, wrist_camera
from dimos.robot.galaxea.r1pro.sim.office import MODELS, SCENE, office_renderer

r1pro_openarm_joylo_office = (
    autoconnect(
        _robot_blueprint(scene=SCENE, models=MODELS),
        _operator_blueprint(),
        R1ProSimCameras.blueprint(
            scene=SCENE,
            head=head_camera(),
            left=wrist_camera("left"),
            right=wrist_camera("right"),
            renderer=office_renderer,
        ),
    )
    .transports(_transports)
    .global_config(viewer="none", transport="zenoh", n_workers=7)
)
