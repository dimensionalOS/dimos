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

from dimos.robot.unitree.go2.calibration import BASE_TO_OPTICAL, camera_info_static


def test_front_camera_calibration() -> None:
    camera_info = camera_info_static()

    assert camera_info_static() is not camera_info
    assert camera_info.frame_id == "camera_optical"
    assert (camera_info.width, camera_info.height) == (1280, 720)
    assert BASE_TO_OPTICAL.frame_id == "base_link"
    assert BASE_TO_OPTICAL.child_frame_id == "camera_optical"
