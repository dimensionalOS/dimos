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

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.hardware.sensors.camera.webcam import Webcam
from dimos.hardware.sensors.camera.zed import compat as zed

demo_agent = autoconnect(McpServer.blueprint(), McpClient.blueprint())


def _create_webcam() -> Webcam:
    return Webcam(
        camera_index=0,
        fps=15,
        camera_info=zed.CameraInfo.SingleWebcam,
    )


demo_agent_camera = autoconnect(
    McpServer.blueprint(),
    McpClient.blueprint(),
    CameraModule.blueprint(
        hardware=_create_webcam,
    ),
)
