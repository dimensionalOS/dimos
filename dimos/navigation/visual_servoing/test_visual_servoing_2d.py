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

from dimos_generated.geometry_msgs.msg import Twist
from dimos_generated.sensor_msgs.msg import CameraInfo
import pytest

from dimos.navigation.visual_servoing.visual_servoing_2d import VisualServoing2D


@pytest.mark.parametrize(
    "bbox, linear, angular",
    [
        ((270, 0, 370, 100), 0.5, 0),
        ((245, 0, 395, 100), 0, 0),
        ((120, 0, 520, 100), -0.3, 0),
        ((370, 0, 470, 100), 0.36, -0.2),
        ((320, 0, 320, 100), 0, 0),
    ],
)
def test_generated_camera_to_twist(bbox, linear, angular):
    camera = CameraInfo(width=640, height=480, k=[500, 0, 320, 0, 500, 240, 0, 0, 1])
    controller = VisualServoing2D(CameraInfo.decode(camera.encode()))
    output = controller.compute_twist(bbox, 640)
    decoded = Twist.decode(output.encode())
    assert decoded.linear.x == pytest.approx(linear)
    assert decoded.angular.z == pytest.approx(angular)
    assert decoded.linear.y == decoded.linear.z == 0
    assert decoded.angular.x == decoded.angular.y == 0
