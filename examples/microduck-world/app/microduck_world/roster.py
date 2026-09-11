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

"""Shared player roster, also consumed by the lobby and browser."""

import json

from microduck_world.scene import PROJECT_ROOT

SETTINGS = json.loads((PROJECT_ROOT / "assets/scenes/apartment/multiplayer.json").read_text())
ROSTER = SETTINGS["robots"]
ROBOT_IDS = tuple(ROSTER)

if len(ROBOT_IDS) != 6 or any(
    sum(player["team"] == team for player in ROSTER.values()) != 3 for team in ("red", "blue")
):
    raise ValueError("The football roster requires three red and three blue players")


def prefix_for(robot: str) -> str:
    """The engine's first body remains unprefixed; player permissions are equal."""
    return "" if robot == ROBOT_IDS[0] else robot + "_"
