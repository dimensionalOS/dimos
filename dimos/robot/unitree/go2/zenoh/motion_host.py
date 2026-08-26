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

"""The baked motion host's stdin config, generated -- never hand-edited.

    python -m dimos.robot.unitree.go2.zenoh.motion_host > motion-host.json

Every module block is its python config class at defaults, plus the few values
this deployment tunes; the rust side has no defaults of its own, so this is the
only place they come from. One JSON line: the host reads exactly one.
"""

from __future__ import annotations

from dataclasses import replace
import json

from dimos.cli.bake.cli import emit_config
from dimos.cli.bake.discovery import discover_modules, select_modules
from dimos.cli.bake.graph import build_graph
from dimos.navigation.motion.adapter.follower_native import TrajectoryFollowerNativeConfig
from dimos.navigation.motion.adapter.planner_native import MotionPlannerNativeConfig
from dimos.navigation.motion.embodiment.go2 import GO2
from dimos.protocol.service.zenohservice import ZenohConfig
from dimos.robot.unitree.go2.zenoh.blueprints import MOTION_BODY_DILATE_M

HOST = "motion-host"
# The bake list; `dimos bake` must be given the same one or the graph stamp differs.
MODULES = ("motion_planner", "trajectory_follower", "cmd_vel_mux", "go2_tf")
# The deployment's ceiling over GO2's measured cruise; dial here, not in the law.
MAX_SPEED = 0.7

# go2web is the zenoh ROUTER on 7447; the host is its CLIENT over loopback and
# listens on nothing. Without this block the host opens zenoh's defaults -- a
# peer with multicast scouting -- which a router does not forward to.
SESSION = ZenohConfig(
    mode="client",
    connect=["tcp/127.0.0.1:7447"],
    listen=[],
    multicast=False,
    scouting_interface="lo",
    gossip=True,
    connect_timeout=3.0,
)


def motion_host_config() -> dict[str, object]:
    """The full stdin blob for a host baked from `MODULES`."""
    selected = select_modules(discover_modules(), MODULES)
    body = replace(GO2, max_speed=MAX_SPEED)
    tuned = {
        "motion_planner": MotionPlannerNativeConfig(
            embodiment=body, body_dilate_m=MOTION_BODY_DILATE_M
        ).to_config_dict(),
        "trajectory_follower": TrajectoryFollowerNativeConfig(
            embodiment=body, body_dilate_m=MOTION_BODY_DILATE_M
        ).to_config_dict(),
    }
    blob = emit_config(build_graph(HOST, selected), selected, tuned)
    blob["session"] = SESSION.to_wire()
    return blob


if __name__ == "__main__":
    print(json.dumps(motion_host_config()))
