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

import pytest

from dimos.protocol.rpc.zenohrpc import ZenohRPC
from dimos.robot.galaxea.r1pro.blueprints.basic.r1pro_sew_fake import R1FakeWebXR
from dimos.teleop.webxr.controller_types import WebXRControllerState
from dimos.teleop.webxr.extensions import ArmTeleopModule


@pytest.fixture
def module(mocker):
    # Only the RPC boundary is replaced; normal initialization and cleanup run.
    mocker.patch.object(ZenohRPC, "start")
    mocker.patch.object(ZenohRPC, "serve_module_rpc")
    mocker.patch.object(ZenohRPC, "stop")
    module = R1FakeWebXR(rpc_transport=ZenohRPC)
    try:
        yield module
    finally:
        module.stop()


@pytest.mark.parametrize("missing", ["left", "right", "both"])
def test_missing_controllers_cannot_publish_synthetic_release(mocker, module, missing):
    publish = mocker.patch.object(ArmTeleopModule, "_publish_button_state")
    left = None if missing in ("left", "both") else WebXRControllerState()
    right = None if missing in ("right", "both") else WebXRControllerState(is_left=False)

    module._publish_button_state(left, right)

    publish.assert_not_called()


def test_physical_controller_release_is_forwarded(mocker, module):
    publish = mocker.patch.object(ArmTeleopModule, "_publish_button_state")
    left = WebXRControllerState()
    right = WebXRControllerState(is_left=False)

    module._publish_button_state(left, right)

    publish.assert_called_once_with(left, right)
