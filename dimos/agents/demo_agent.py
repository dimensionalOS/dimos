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

from dimos.agents.agent import Agent
from dimos.core.blueprints import autoconnect
from dimos.hardware.sensors.camera import zed
from dimos.hardware.sensors.camera.module import camera_module
from dimos.hardware.sensors.camera.webcam import Webcam

demo_agent = autoconnect(Agent.blueprint())

demo_agent_camera = autoconnect(
    Agent.blueprint(),
    camera_module(
        hardware=lambda: Webcam(
            camera_index=0,
            fps=15,
            camera_info=zed.CameraInfo.SingleWebcam,
        ),
    ),
)
