# Copyright 2025-2026 Dimensional Inc.
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

"""Stock G1 sensors and mapping, consumed by Desktop over the selected transport."""

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.hardware.sensors.camera.webcam import Webcam
from dimos.hardware.sensors.camera.zed import compat as zed
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.voxels.module import VoxelGridMapper
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.experimental.frontier_exploration.wavefront_frontier_goal_selector import (
    WavefrontFrontierExplorer,
)


def _create_webcam() -> Webcam:
    return Webcam(
        camera_index=0, fps=15, stereo_slice="left", camera_info=zed.CameraInfo.SingleWebcam
    )


# Stock simulation uses the camera authored in its model and an ideal scan.
# These are sensor workers attached to the collected world, not a second sim.
if global_config.simulation:
    # Simulation packages are optional for stock hardware deployments.
    from dimos.sim2.sensors.camera.module import SimRGBDCameraModule
    from dimos.sim2.sensors.camera.spec import Camera
    from dimos.sim2.sensors.lidar.models.fibonacci import Fibonacci
    from dimos.sim2.sensors.lidar.module import LidarModule
    from dimos.sim2.sensors.lidar.spec import Lidar
    from dimos.sim2.sensors.spec import Mount

    _sensors = autoconnect(
        SimRGBDCameraModule.blueprint(sensor=Camera("head", "head_camera")),
        LidarModule.blueprint(
            sensor=Lidar(
                "lidar",
                Mount("torso_link", xyz=(0.05, 0.0, 0.42)),
                Fibonacci,
                model_kwargs={"ray_count": 4096},
            ),
        ).remappings([(LidarModule, "pointcloud", "lidar")]),
    )
else:
    _sensors = CameraModule.blueprint(
        transform=Transform(
            translation=Vector3(0.05, 0.0, 0.6),
            rotation=Quaternion.from_euler(Vector3(0.0, 0.2, 0.0)),
            frame_id="sensor",
            child_frame_id="camera_link",
        ),
        hardware=_create_webcam,
    )


def _require_sensor_backend() -> str | None:
    if global_config.simulation == "genesis":
        return (
            "Genesis does not yet implement the stock camera/lidar workers; "
            "run unitree-g1-control, or use --simulation mujoco for navigation."
        )
    return None


unitree_g1_primitive_no_nav = (
    autoconnect(
        _sensors,
        VoxelGridMapper.blueprint(),
        CostMapper.blueprint(),
        WavefrontFrontierExplorer.blueprint(),
    )
    .global_config(n_workers=4, robot_model="unitree_g1", viewer="none")
    .requirements(_require_sensor_backend)
)
