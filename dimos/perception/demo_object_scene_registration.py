#!/usr/bin/env python3
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

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.hardware.sensors.camera.zed.compat import ZEDCamera
from dimos.perception.detection.detectors.yoloe import YoloePromptMode
from dimos.perception.object_scene_registration import ObjectSceneRegistrationModule
from dimos.robot.foxglove_bridge import FoxgloveBridge

camera_choice = "zed"

if camera_choice == "realsense":
    camera_module = RealSenseCamera.blueprint(enable_pointcloud=False)
elif camera_choice == "zed":
    camera_module = ZEDCamera.blueprint(enable_pointcloud=False)
else:
    raise ValueError(f"Invalid camera choice: {camera_choice}")

demo_object_scene_registration = autoconnect(
    camera_module,
    ObjectSceneRegistrationModule.blueprint(target_frame="world", prompt_mode=YoloePromptMode.LRPC),
    FoxgloveBridge.blueprint(),
    McpServer.blueprint(),
    McpClient.blueprint(),
).global_config(viewer="foxglove")
