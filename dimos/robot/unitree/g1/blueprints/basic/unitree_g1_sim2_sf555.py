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
from dimos.robot.assets.model import RobotModel
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_sim2 import (
    CAMERA,
    ENCODERS,
    G1_MODEL,
    IMU,
    LIDAR,
    SCENE as G1_SCENE,
)
from dimos.robot.unitree.g1.sim2_policy import CONTROL, JOINTS, G1PolicyModule
from dimos.robot.urdf_manager import UrdfManager
from dimos.sim2.description import Instance, SceneConfig
from dimos.sim2.module import SimModule
from dimos.sim2.physics.mujoco import MujocoBackend
from dimos.sim2.sensors.camera.module import SimCameraModule
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
MODELS = {"g1": G1_MODEL, "office": RobotModel.from_file(ASSETS / "office.urdf")}
SCENE = SceneConfig(
    instances=(replace(G1_SCENE.instances[0], xyz=(16, 1, 0.793)), Instance("office")),
    ground_plane=False,
)

unitree_g1_sim2_sf555 = autoconnect(
    UrdfManager.blueprint(models=MODELS),
    SimModule.blueprint(
        scene=SCENE,
        backend=MujocoBackend,
        observation_hz=200,
        controllers=(CONTROL,),
        joint_ids=JOINTS,
    ),
    SimImuModule.blueprint(sensor=IMU),
    SimJointEncoderModule.blueprint(sensor=ENCODERS),
    SimCameraModule.blueprint(
        scene=SCENE,
        sensor=replace(CAMERA, width=640, height=480, fx=376.72, fy=376.72, cx=320, cy=240),
        renderer=partial(
            UnrealRenderer,
            executable=EXECUTABLE,
            project=PROJECT,
            level="/Game/SF555/Maps/Office",
            authored_instances=("office",),
            log_path=ASSETS / "unreal-camera.log",
            startup_timeout=600,
            capture_timeout=30,
        ),
    ),
    SimLidarModule.blueprint(scene=SCENE, sensor=LIDAR, raycaster=MujocoRaycaster),
    G1PolicyModule.blueprint(),
).global_config(viewer="none", transport="zenoh", n_workers=7)
