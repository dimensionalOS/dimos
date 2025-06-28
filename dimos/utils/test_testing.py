# Copyright 2025 Dimensional Inc.
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

import hashlib
import os
import subprocess

from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.utils import testing


def test_sensor_replay():
    counter = 0
    for message in testing.SensorReplay(name="office_lidar").iterate():
        counter += 1
        assert isinstance(message, dict)
    assert counter == 500


def test_sensor_replay_cast():
    counter = 0
    for message in testing.SensorReplay(
        name="office_lidar", autocast=LidarMessage.from_msg
    ).iterate():
        counter += 1
        assert isinstance(message, LidarMessage)
    assert counter == 500
