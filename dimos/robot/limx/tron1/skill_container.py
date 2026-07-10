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

from __future__ import annotations

from typing import Any

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.limx.tron1.connection_spec import TRON1ConnectionSpec
from dimos.robot.limx.tron1 import protocol


class TRON1SkillContainer(Module):
    _connection: TRON1ConnectionSpec

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()

    @skill
    def move(self, x: float, y: float = 0.0, yaw: float = 0.0, duration: float = 0.0) -> str:
        """Move TRON1 using velocity control.

        Args:
            x: Forward velocity (m/s).
            y: Left/right velocity (m/s).
            yaw: Rotational velocity (rad/s).
            duration: How long to move (seconds). Use 0 for continuous if supported.
        """
        twist = Twist(linear=Vector3(x, y, 0.0), angular=Vector3(0.0, 0.0, yaw))
        self._connection.move(twist, duration=duration)
        return f"Started moving with velocity=({x}, {y}, {yaw}) for {duration} seconds"

    @skill
    def stand(self) -> str:
        """Enter standing mode."""
        self._connection.publish_request("request_stand_mode", protocol.build_request_stand_mode())
        return "Requested stand mode"

    @skill
    def sitdown(self) -> str:
        """Sit down the robot."""
        self._connection.publish_request("request_sitdown", protocol.build_request_sitdown())
        return "Requested sitdown"

    @skill
    def recover(self) -> str:
        """Trigger fall recovery."""
        self._connection.publish_request("request_recover", protocol.build_request_recover())
        return "Requested recovery"

    @skill
    def emergency_stop(self) -> str:
        """Emergency stop the robot immediately."""
        self._connection.publish_request("request_emgy_stop", protocol.build_request_emgy_stop())
        return "Requested emergency stop"

    @skill
    def set_base_height(self, height: float) -> str:
        """Set robot base height.

        Args:
            height: Target height in meters.
        """
        self._connection.publish_request("request_base_height", protocol.build_request_base_height(height))
        return f"Requested base height: {height}"

    @skill
    def set_light_effect(self, effect: str) -> str:
        """Set robot light effect.

        Args:
            effect: Effect name or encoded mode string.
        """
        self._connection.publish_request("request_light_effect", protocol.build_request_light_effect(effect))
        return f"Requested light effect: {effect}"

    @skill
    def publish_raw(self, name: str, data: dict[str, Any]) -> str:
        """Send a raw TRON1 request by name.

        Args:
            name: Request name, e.g. 'request_twist'.
            data: Raw request payload as a JSON object.
        """
        self._connection.publish_request(name, data)
        return f"Requested {name}"

