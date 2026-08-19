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

"""Static front-camera calibration for the Unitree Go2."""

from importlib import resources

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo

_FRONT_CAMERA_720_YAML = resources.files("dimos.robot.unitree.go2").joinpath(
    "front_camera_720.yaml"
)


def camera_info_static() -> CameraInfo:
    """Load the calibrated front-camera intrinsics."""
    with resources.as_file(_FRONT_CAMERA_720_YAML) as yaml_path:
        return CameraInfo.from_yaml(str(yaml_path))


# Static camera mount chain: base_link -> camera_link -> camera_optical.
BASE_TO_OPTICAL: Transform = Transform(
    translation=Vector3(0.3, 0.0, 0.0),
    rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
    frame_id="base_link",
    child_frame_id="camera_link",
) + Transform(
    translation=Vector3(0.0, 0.0, 0.0),
    rotation=Quaternion(-0.5, 0.5, -0.5, 0.5),
    frame_id="camera_link",
    child_frame_id="camera_optical",
)
