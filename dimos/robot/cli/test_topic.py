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

from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.robot.cli.topic import _decode_typed_lcm_message


def test_decode_typed_lcm_message_resolves_message_submodule() -> None:
    msg = CameraInfo(
        width=1920,
        height=1080,
        distortion_model="plumb_bob",
        frame_id="camera_optical",
    )

    decoded = _decode_typed_lcm_message(
        "/camera_info#sensor_msgs.CameraInfo",
        msg.lcm_encode(),
    )

    assert isinstance(decoded, CameraInfo)
    assert decoded.width == 1920
    assert decoded.height == 1080
    assert decoded.frame_id == "camera_optical"
    assert decoded.distortion_model == "plumb_bob"
