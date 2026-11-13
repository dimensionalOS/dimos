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

"""R1 headset adapters preserve encoded video and shared physical speeds."""

import json

import pytest

from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.msgs.std_msgs.String import String
from dimos.robot.galaxea.r1pro._connection_test_fakes import LocalTransport
from dimos.robot.galaxea.r1pro.motion import motion_groups
from dimos.robot.galaxea.r1pro.webxr import R1ProWebXR, R1ProWebXRJoystick


@pytest.mark.asyncio
async def test_headset_forwards_existing_jpeg_bytes(mocker):
    module = R1ProWebXR()
    forward = mocker.patch("dimos.robot.galaxea.r1pro.webxr.push_jpeg_bytes")
    encoded = b"\xff\xd8existing-compressed-frame\xff\xd9"
    try:
        await module.handle_color_image(CompressedImage(data=encoded, format="jpeg"))
        forward.assert_called_once_with(module, encoded)
    finally:
        module.stop()


@pytest.fixture
def joystick(mocker):
    mapper = R1ProWebXRJoystick()
    settings = {name: group.model_dump() for name, group in motion_groups().items()}
    limits = mocker.Mock()
    limits.get_motion_limits.return_value = settings
    mocker.patch.object(mapper, "_motion_limits", limits, create=True)
    for name in {**mapper.inputs, **mapper.outputs}:
        mapper.set_transport(name, LocalTransport())
    try:
        mapper.start()
        yield mapper, settings
    finally:
        mapper.stop()


@pytest.mark.asyncio
async def test_live_speed_settings_scale_headset_sticks(joystick):
    mapper, settings = joystick
    mapper._on_left_joy(Joy(axes=[0.0, -0.5, 0.0, 0.0], buttons=[0] * 6))
    mapper._on_right_joy(Joy(axes=[-0.5, 0.0, 0.0, 0.0], buttons=[0, 0, 0, 1, 0, 0]))
    first = mapper.base_twist.transport.sent[-1]
    assert (first.linear.x, first.angular.z) == (0.15, 0.3)
    settings["base_speed"]["rate"] = 0.1
    settings["yaw_speed"]["rate"] = 0.2
    await mapper.handle_motion_settings(String(json.dumps(settings)))
    mapper._on_right_joy(Joy(axes=[-0.5, 0.0, 0.0, 0.0], buttons=[0, 0, 0, 1, 0, 0]))
    latest = mapper.base_twist.transport.sent[-1]
    assert (latest.linear.x, latest.angular.z) == (0.05, 0.1)
