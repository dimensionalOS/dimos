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

"""The Go2 in sim2's DimSim backend: a kinematic base the size of the Go2 pushing the props, with its head L1 and front camera."""

from pathlib import Path

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.assets.model import RobotModel
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_sim2_l1 import (
    _GO2_DESCRIPTION,
    Go2UrdfRoot,
)
from dimos.robot.unitree.go2.go2_mid360_static_transforms import CAMERA_XYZ, OPTICAL_RPY
from dimos.robot.urdf_manager import UrdfManager
from dimos.sim2.camera import SimCamera
from dimos.sim2.kinematic_base import SimKinematicBase
from dimos.sim2.l1 import SimL1
from dimos.sim2.module import Simulation
from dimos.sim2.spec import Mount
from dimos.utils.data import get_data

unitree_go2_dimsim = autoconnect(
    Simulation.blueprint(
        backend="dimsim", scene=Path(__file__).parents[5] / "sim2" / "logistics_props.xml"
    ),
    SimKinematicBase.blueprint(
        robot_id="go2", model=get_data("go2_sim") / "unitree_go2" / "go2.xml", root_body="base"
    ),
    Go2UrdfRoot.blueprint(),
    UrdfManager.blueprint(
        model=RobotModel.from_file(
            _GO2_DESCRIPTION / "urdf" / "go2_description.urdf",
            package_paths={"go2_description": _GO2_DESCRIPTION},
        )
    ),
    SimL1.blueprint(),
    SimCamera.blueprint(robot_id="go2", mount=Mount("base", CAMERA_XYZ, OPTICAL_RPY)),
).global_config(n_workers=6, robot_model="unitree_go2")
