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

"""Packaged Go2 camera calibration, independent of robot connections."""

from importlib import resources

from dimos.message_utils.camera_info import from_yaml
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.sensor_msgs.msg import CameraInfo
from dimos.messages.std_msgs.msg import Header


def front_camera_calibration() -> CameraInfo:
    """Load a fresh 720p front-camera calibration in the camera optical frame."""
    source = resources.files(__package__).joinpath("front_camera_720.yaml")
    with resources.as_file(source) as path:
        return from_yaml(
            path, header=Header(stamp=Time(sec=0, nanosec=0), frame_id="camera_optical")
        )
