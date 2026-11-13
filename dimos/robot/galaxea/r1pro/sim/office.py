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

"""R1 Pro at the downstairs engineering tables, with two free water bottles.

All poses are in the source office's metric Z-up frame. Bottle positions are
the measured V24 assembly bounds centres. Robot spawn is an explicit staging
choice at the east end of the table by bottle B, facing west. Camera optics are ideal
approximations on the unchanged 2025 vendor sensor frames.
"""

from dataclasses import replace
import json
import math
import os
from pathlib import Path

from dimos.robot.assets.model import RobotModel
from dimos.robot.galaxea.r1pro.sim.config import SCENE as ROBOT_SCENE
from dimos.sim2.description import Instance, SceneConfig
from dimos.sim2.scenes.sf555.pickables import ASSETS
from dimos.sim2.sensors.camera.renderers.unreal.renderer import UnrealRenderer

MODELS = {
    "r1pro": RobotModel.from_file(ASSETS / "r1pro.urdf"),
    "office": RobotModel.from_file(ASSETS / "office.urdf"),
    "bottle_a": RobotModel.from_file(ASSETS / "bottle_a.urdf"),
    "bottle_b": RobotModel.from_file(ASSETS / "bottle_b.urdf"),
}
SCENE = SceneConfig(
    instances=(
        replace(ROBOT_SCENE.instances[0], xyz=(18.0, 8.41, 0.005), rpy=(0, 0, math.pi)),
        Instance("office"),
        Instance("bottle_a", xyz=(15.720000267, 8.409999847, 0.877500033), floating=True),
        Instance("bottle_b", xyz=(17.109998703, 8.409999847, 0.871500025), floating=True),
    ),
    ground_plane=False,
)


def office_renderer() -> UnrealRenderer:
    """Open one native renderer for the whole head/wrist rig, only at run time."""
    return UnrealRenderer(
        executable=Path(
            os.environ.get(
                "DIMOS_UNREAL_EXECUTABLE",
                "/Users/Shared/Epic Games/UE_5.7/Engine/Binaries/Mac/UnrealEditor.app/Contents/MacOS/UnrealEditor",
            )
        ),
        project=Path(
            os.environ.get(
                "DIMOS_UNREAL_PROJECT", str(ASSETS / "DimosRenderer/DimosRenderer.uproject")
            )
        ),
        level="/Game/SF555/Maps/OfficeJoylo",
        authored_instances=("office",),
        native_visuals=json.loads((ASSETS / "native-visuals.json").read_text()),
        log_path=ASSETS / "r1pro-cameras.log",
        startup_timeout=600,
        capture_timeout=30,
    )
