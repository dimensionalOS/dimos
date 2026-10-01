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

"""The packaged Go2 front-camera calibration, independent of its robot connection."""

from importlib import resources

from dimos_generated.sensor_msgs.msg import CameraInfo
from dimos_generated.std_msgs.msg import Header

from dimos.msgs.camera_info import camera_info_from_yaml


def front_camera_calibration() -> CameraInfo:
    resource = resources.files("dimos.robot.unitree.go2").joinpath("front_camera_720.yaml")
    with resources.as_file(resource) as path:
        return camera_info_from_yaml(path, header=Header(frame_id="camera_optical"))
