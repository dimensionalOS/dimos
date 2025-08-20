#!/usr/bin/env python3

# Copyright 2025 Dimensional Inc.
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

"""Robot configuration types and enums for flexible robot instantiation."""

from typing import Optional, Literal
from pydantic import BaseModel, model_validator


ConnectionType = Literal["webrtc", "fake", "mujoco"]


RobotModel = Literal["unitree_go1", "unitree_go2", "unitree_g1"]


MujocoScene = Literal["empty", "office1"]


class RobotConfig(BaseModel):
    connection_type: Optional[ConnectionType] = "webrtc"
    robot_model: Optional[RobotModel] = "unitree_go2"
    robot_ip: Optional[str] = None
    scene: Optional[MujocoScene] = None
    output_dir: Optional[str] = None
    websocket_port: int = 7779

    class Config:
        extra = "forbid"

    @model_validator(mode="after")
    def validate_configuration_after(self):
        self.connection_type = self.connection_type or "webrtc"
        self.robot_model = self.robot_model or "unitree_go2"

        if self.connection_type == "webrtc" and not self.robot_ip:
            # Auto-switch to fake if no IP provided for WebRTC
            self.connection_type = "fake"

        if self.connection_type == "mujoco" and not self.scene:
            self.scene = "office1"

        if self.connection_type == "mujoco" and self.robot_model == "unitree_go2":
            self.robot_model = "unitree_go1"

        return self
