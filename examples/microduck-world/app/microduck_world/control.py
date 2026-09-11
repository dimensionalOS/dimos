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

"""Human-only simulation controls alongside the existing Microduck cockpit router."""

import json

from dimos.core.stream import Out
from dimos.robot.pollen.microduck.control_module import DuckControlModule


class WorldControl(DuckControlModule):
    respawn_request: Out[bool]
    ball_drop_request: Out[str]

    def _on_ui_command(self, raw: str) -> None:
        try:
            command = json.loads(raw)
        except (ValueError, TypeError):
            super()._on_ui_command(raw)
            return
        if isinstance(command, dict) and command.get("name") == "drop_ball":
            args = command.get("args", {})
            from microduck_world.football import BALL_NAMES

            if isinstance(args, dict) and set(args) == {"ball"} and args["ball"] in BALL_NAMES:
                self.ball_drop_request.publish(args["ball"])
            return
        if not isinstance(command, dict) or command.get("name") != "respawn":
            super()._on_ui_command(raw)
            return
        if command.get("args", {}) != {}:
            return
        # Switching mode also stops the project's frontier explorer.
        self._set_mode("teleop")
        with self._lock:
            self._teleop_active = False
            self._last_teleop_time = self._clock()
        self._cancel_nav()
        self.respawn_request.publish(True)
