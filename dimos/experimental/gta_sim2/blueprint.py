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

"""Construct an isolated G1 deployment against a prepared, bounded map patch."""

from collections.abc import Callable
from functools import partial
from pathlib import Path

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.coordination.blueprints import MetaBlueprint, autoconnect
from dimos.robot.assets.model import UsdModel
from dimos.robot.unitree.g1.sim2_policy import JOINTS, G1PolicyModule
from dimos.robot.unitree.g1.sim2_scene import TORSO, G1Desk
from dimos.sim2.description import joint_paths
from dimos.sim2.module import SimModule
from dimos.sim2.physics.mujoco import MujocoBackend
from dimos.sim2.sensors.camera.config import CameraConfig, CameraRenderer
from dimos.sim2.sensors.camera.module import SimCameraModule
from dimos.sim2.sensors.camera.render_module import CameraRenderModule
from dimos.sim2.sensors.camera.renderers.mujoco import MujocoRenderer
from dimos.sim2.sensors.encoder.module import EncoderConfig, SimJointEncoderModule
from dimos.sim2.sensors.imu.module import ImuConfig, SimImuModule
from dimos.sim2.sensors.lidar.config import LidarConfig
from dimos.sim2.sensors.lidar.mid360 import Mid360Returns, Mid360Scan
from dimos.sim2.sensors.lidar.module import SimLidarModule
from dimos.sim2.sensors.lidar.raycasters.mujoco import MujocoRaycaster
from dimos.utils.data import LfsPath


def gta_g1_blueprint(
    scene_path: Path, renderer: Callable[[], CameraRenderer] = MujocoRenderer
) -> MetaBlueprint:
    """Keep the stock policy, mounts, clocks and sensor contracts; change the world."""
    scene = UsdModel.from_file(scene_path)
    paths = joint_paths(scene)
    ids = tuple(paths[name] for name in JOINTS)
    camera = SimCameraModule.attach(
        G1Desk.mounts.camera_mount, instance_name="camera", sensor=CameraConfig("camera")
    )
    return autoconnect(
        G1Desk.blueprint(instance_name="scene", usd=scene),
        SimImuModule.attach(
            G1Desk.mounts.imu_mount,
            instance_name="imu",
            sensor=ImuConfig("imu", orientation="ideal"),
        ).remappings([(SimImuModule, "imu", "imu")]),
        camera,
        CameraRenderModule.blueprint(
            renderer=renderer, cameras=tuple(atom.name for atom in camera.active_blueprints)
        ),
        SimLidarModule.attach(
            G1Desk.mounts.lidar_mount,
            instance_name="lidar",
            raycaster=MujocoRaycaster,
            usd=UsdModel.from_file(DIMOS_PROJECT_ROOT / "assets/g1_desk/mid360.usda"),
            sensor=LidarConfig(
                "mid360/lidar_link",
                min_range=0.16,
                pattern=partial(Mid360Scan, LfsPath("mid360_pattern/fourier.npz"), downsample=10),
                response=Mid360Returns,
                excluded_bodies=(
                    TORSO + "/head_body",
                    TORSO + "/lidar_mount/mid360/lidar_link/lidar_link_body",
                ),
            ),
        ).remappings(
            [(SimLidarModule, "lidar", "lidar_raw"), (SimLidarModule, "pointcloud", "pointcloud")]
        ),
        SimModule.blueprint(backend=MujocoBackend, observation_hz=200, joint_ids=ids),
        SimJointEncoderModule.blueprint(sensor=EncoderConfig(ids, hz=50)),
        G1PolicyModule.blueprint(scene=scene),
    ).global_config(viewer="none", transport="zenoh", n_workers=7)
