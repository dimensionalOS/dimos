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

"""G1 simulation with native Unreal RGB-D on the standard Desktop topics.

Run navigation with ``--simulation mujoco``; control-only blueprints also support
``--simulation genesis``. Set ``--unrealcameramodule.executable``
or ``UNREALCAMERAMODULE__EXECUTABLE`` to the packaged renderer executable.
The observer camera looks toward the robot's starting position from the room.
"""

import math

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_basic import unitree_g1_control
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_groot_wbc import (
    g1_groot_wbc_simulation_profile,
)
from dimos.sim2.sensors.camera.spec import Camera
from dimos.sim2.sensors.camera.unreal.module import UnrealCameraModule
from dimos.sim2.sensors.spec import Mount

_camera = UnrealCameraModule.blueprint(
    sensor=Camera(
        "unreal",
        Mount("world", xyz=(2.4, -2.4, 1.8), rpy=(1.2707753655437184, 0.0, math.pi / 4)),
    ),
    rpc_timeouts={"start": 210.0},
).global_config(viewer="none")


def _require_simulation() -> str | None:
    if global_config.simulation not in ("mujoco", "genesis"):
        return "Select --simulation mujoco or --simulation genesis for the Unreal physics scene."
    return None


unitree_g1_groot_wbc_unreal = autoconnect(g1_groot_wbc_simulation_profile(), _camera)

unitree_g1_groot_control_unreal = autoconnect(
    g1_groot_wbc_simulation_profile(navigation=False), _camera
)

unitree_g1_control_unreal = autoconnect(unitree_g1_control, _camera).requirements(
    _require_simulation
)
