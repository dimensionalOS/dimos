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

"""Route overlay converters for the Rerun bridge visual_override config.

Module-level functions only: the bridge config is pickled to the viewer's
worker. Entry points: `route_state_overlay` for `world/route_state` and
`route_path_overlay` for `world/route_path`.
"""

from __future__ import annotations

import json
from typing import TYPE_CHECKING

from dimos_lcm.std_msgs import String

from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.route_follower.route import WaypointState

if TYPE_CHECKING:
    from rerun._baseclasses import Archetype

COLOR_PENDING = (128, 128, 128)
COLOR_CURRENT = (255, 255, 0)
COLOR_REACHED = (0, 255, 0)
COLOR_FAILED = (255, 0, 0)
ROUTE_PATH_COLOR = (255, 128, 0)

_STATE_COLORS: dict[str, tuple[int, int, int]] = {
    WaypointState.PENDING.value: COLOR_PENDING,
    WaypointState.CURRENT.value: COLOR_CURRENT,
    WaypointState.REACHED.value: COLOR_REACHED,
    WaypointState.FAILED.value: COLOR_FAILED,
}


def route_state_overlay(msg: String) -> Archetype:
    """Numbered waypoint markers coloured by state, at `world/route_state`."""
    import rerun as rr

    waypoints = json.loads(msg.data)["waypoints"]
    return rr.Points3D(
        positions=[[w["x"], w["y"], w["z"]] for w in waypoints],
        labels=[str(w["index"]) for w in waypoints],
        colors=[_STATE_COLORS[w["state"]] for w in waypoints],
    )


def route_path_overlay(msg: Path) -> Archetype:
    """The route line, in a colour distinct from the planner's green path."""
    return msg.to_rerun(color=ROUTE_PATH_COLOR)
