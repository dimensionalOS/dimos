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

from dimos.core.coordination.blueprints import autoconnect
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
from dimos.sim2.scenes.sf555.config import ASSETS as ASSETS, OFFICE_MODEL, OFFICE_RENDERER
from dimos.sim2.sensors.camera.module import SimCameraModule
from dimos.sim2.sensors.encoder.module import SimJointEncoderModule
from dimos.sim2.sensors.imu.module import SimImuModule
from dimos.sim2.sensors.lidar.module import SimLidarModule
from dimos.sim2.sensors.lidar.raycasters.mujoco import MujocoRaycaster

MODELS = {"g1": G1_MODEL, "office": OFFICE_MODEL}
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
        renderer=OFFICE_RENDERER,
    ),
    SimLidarModule.blueprint(scene=SCENE, sensor=LIDAR, raycaster=MujocoRaycaster),
    G1PolicyModule.blueprint(),
).global_config(viewer="none", transport="zenoh", n_workers=7)
