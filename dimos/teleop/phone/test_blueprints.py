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

from dimos.core.transport import JpegLcmTransport, LCMTransport
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Image import Image
from dimos.teleop.phone.blueprints import teleop_phone_go2


def test_teleop_phone_go2_routes_browser_keyboard_to_go2_cmd_vel() -> None:
    transport = teleop_phone_go2.transport_map[("tele_cmd_vel", Twist)]

    assert type(transport) is LCMTransport
    assert transport.topic.topic == "/cmd_vel"
    assert transport.topic.lcm_type is Twist


def test_teleop_phone_go2_exposes_camera_to_rerun_over_jpeg_lcm() -> None:
    transport = teleop_phone_go2.transport_map[("color_image", Image)]

    assert isinstance(transport, JpegLcmTransport)
    assert transport.topic.topic == "/color_image"
    assert transport.topic.lcm_type is Image
