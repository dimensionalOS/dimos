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

"""G1 in the saved SF555 office: MuJoCo physics, independent native Unreal RGB-D.

Prepare the local office assets first; see docs/development/sim2-sf555.md.
The scene uses the source's metric, Z-up frame. The default spawn is an open
part of the main hall, facing +X. Furniture and doors are static in this demo.
"""

from dataclasses import replace
from functools import partial
import os
from pathlib import Path

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.assets.model import UsdModel
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_sim2 import (
    CAMERA,
    ENCODERS,
    IMU,
    LIDAR,
)
from dimos.robot.unitree.g1.sim2_policy import JOINTS, G1PolicyModule
from dimos.robot.usd_manager import UsdManager
from dimos.sim2.module import SimModule
from dimos.sim2.physics.mujoco import MujocoBackend
from dimos.sim2.sensors.camera.module import SimCameraModule
from dimos.sim2.sensors.camera.render_module import CameraRenderModule
from dimos.sim2.sensors.camera.renderers.unreal.renderer import UnrealRenderer
from dimos.sim2.sensors.encoder.module import SimJointEncoderModule
from dimos.sim2.sensors.imu.module import SimImuModule
from dimos.sim2.sensors.lidar.module import SimLidarModule
from dimos.sim2.sensors.lidar.raycasters.mujoco import MujocoRaycaster

ASSETS = Path(os.environ.get("DIMOS_SF555_ASSETS", str(Path.home() / ".cache/dimos/sf555")))
PROJECT = Path(
    os.environ.get("DIMOS_UNREAL_PROJECT", str(ASSETS / "DimosRenderer/DimosRenderer.uproject"))
)
EXECUTABLE = Path(
    os.environ.get(
        "DIMOS_UNREAL_EXECUTABLE",
        "/Users/Shared/Epic Games/UE_5.7/Engine/Binaries/Mac/UnrealEditor.app/Contents/MacOS/UnrealEditor",
    )
)
# Prepared explicitly; the simulation and every sensor consume the same USD scene.
SCENE = ASSETS / "g1_scene.usda"

unitree_g1_sim2_sf555 = autoconnect(
    UsdManager.blueprint(model=UsdModel.from_file(SCENE)),
    SimModule.blueprint(
        scene_manager=UsdManager,
        backend=MujocoBackend,
        observation_hz=200,
        joint_ids=JOINTS,
    ),
    SimImuModule.blueprint(sensor=IMU),
    SimJointEncoderModule.blueprint(sensor=ENCODERS),
    SimCameraModule.blueprint(
        sensor=replace(CAMERA, width=640, height=480, fx=376.72, fy=376.72, cx=320, cy=240),
    ),
    CameraRenderModule.blueprint(
        scene_manager=UsdManager,
        cameras=("SimCameraModule",),
        renderer=partial(
            UnrealRenderer,
            executable=EXECUTABLE,
            project=PROJECT,
            level="/Game/SF555/Maps/Office",
            authored_bodies=("sf555/world",),
            log_path=ASSETS / "unreal-camera.log",
            startup_timeout=600,
            capture_timeout=30,
        ),
    ),
    SimLidarModule.blueprint(
        scene_manager=UsdManager,
        sensor=replace(LIDAR, excluded_bodies=("g1/head_link",)),
        raycaster=MujocoRaycaster,
    ),
    G1PolicyModule.blueprint(scene_manager=UsdManager),
).global_config(viewer="none", transport="zenoh", n_workers=7)
