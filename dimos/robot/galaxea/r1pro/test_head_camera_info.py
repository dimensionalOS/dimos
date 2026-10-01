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

from typing import get_type_hints

from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.robot.galaxea.r1pro.blueprints.basic.r1pro_coordinator import r1pro_control
from dimos.robot.galaxea.r1pro.connection import R1ProConnection


def test_both_head_cameras_publish_their_camera_info() -> None:
    hints = get_type_hints(R1ProConnection)
    for stream in ("head_left_info", "head_right_info"):
        assert hints[stream] == Out[CameraInfo], stream


def test_the_coordinator_carries_the_camera_info_off_the_robot() -> None:
    transports = r1pro_control().transport_map
    assert ("head_left_info", CameraInfo) in transports
    assert ("head_right_info", CameraInfo) in transports
