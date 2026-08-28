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

import pytest

from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import Go2Memory
from dimos.robot.unitree.go2.go2_mid360_recorder import Go2Mid360Recorder


@pytest.mark.parametrize("recorder", [Go2Memory, Go2Mid360Recorder])
def test_recorder_captures_camera_info(recorder: type[Go2Memory | Go2Mid360Recorder]) -> None:
    atom = recorder.blueprint().blueprints[0]

    camera_info = next(stream for stream in atom.streams if stream.name == "camera_info")

    assert camera_info.direction == "in"
    assert camera_info.type is CameraInfo
