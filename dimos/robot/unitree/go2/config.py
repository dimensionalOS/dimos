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

from importlib import resources
from pathlib import Path

from dimos.constants import (
    DEFAULT_ROBOT_FRAME,
    DEFAULT_WORLD_FRAME,
)
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.robot.config import RobotConfig

_FRONT_CAMERA_720_YAML = resources.files("dimos.robot.unitree.go2").joinpath(
    "front_camera_720.yaml"
)


def camera_info_static() -> CameraInfo:
    with resources.as_file(_FRONT_CAMERA_720_YAML) as yaml_path:
        return CameraInfo.from_yaml(str(yaml_path))


Go2Config = RobotConfig(
    name="unitree_go2",
    model_path=Path(__file__).parent / "go2.urdf",
)


def odom_to_tf(odom: Odometry) -> list[Transform]:
    """[world→base_link, base_link→camera_link, camera_link→camera_optical] for tests."""
    base_link = Transform(
        translation=odom.position,
        rotation=odom.orientation,
        frame_id=odom.frame_id or DEFAULT_WORLD_FRAME,
        child_frame_id=DEFAULT_ROBOT_FRAME,
        ts=odom.ts,
    )
    statics = [
        Transform(
            translation=t.translation,
            rotation=t.rotation,
            frame_id=t.frame_id,
            child_frame_id=t.child_frame_id,
            ts=odom.ts,
        )
        for t in (
            Go2Config.static_transforms["camera_link"],
            Go2Config.static_transforms["camera_optical"],
            Go2Config.static_transforms["lidar_link"],
        )
    ]
    return [base_link, *statics]
