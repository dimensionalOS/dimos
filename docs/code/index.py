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
"""Code examples embedded in the documentation index page."""

import threading
import time

import numpy as np

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.robot.unitree.go2.connection import GO2Connection


class RobotConnection(Module):
    cmd_vel: In[Twist]
    color_image: Out[Image]

    @rpc
    def start(self) -> None:
        threading.Thread(target=self._image_loop, daemon=True).start()

    def _image_loop(self) -> None:
        while True:
            img = Image.from_numpy(
                np.zeros((120, 160, 3), np.uint8),
                format=ImageFormat.RGB,
                frame_id="camera_optical",
            )
            self.color_image.publish(img)
            time.sleep(0.2)


class Listener(Module):
    color_image: In[Image]

    @rpc
    def start(self) -> None:
        self.color_image.subscribe(lambda img: print(f"image {img.width}x{img.height}"))


def run_connection() -> None:
    ModuleCoordinator.build(autoconnect(RobotConnection.blueprint(), Listener.blueprint())).loop()


def run_agentic_blueprint() -> None:
    blueprint = autoconnect(
        GO2Connection.blueprint(),
        McpServer.blueprint(),
        McpClient.blueprint(),
    ).transports({("color_image", Image): LCMTransport("/color_image", Image)})
    ModuleCoordinator.build(blueprint).loop()
