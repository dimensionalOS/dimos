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

"""Namespaced 3-drone PX4 SITL swarm with MCP (no LLM).

Each vehicle runs its own ``Px4DroneModule`` inside its own namespace, so the
per-drone skills, topics, TF frames and config keys are all separated::

    drone1/px4dronemodule/takeoff      RPC
    /drone1/odom                       topic
    -o drone1/px4dronemodule.connection_string=...

One shared ``SwarmCoordinator`` sits outside the namespaces and owns the
fleet-level surface (state, guardrails, formations, sweeps). Telemetry reaches
it on the exposed ``drone_state`` stream; its commands go back out on the
exposed ``swarm_cmd`` stream.

Launch the sim first, then drive the whole demo from the CLI — no OpenAI key
needed:

    ./dimos/simulation/px4_hil/sim.sh start 3 0    # in-repo MuJoCo+PX4 simulator
    CI=1 dimos run drone-px4-sitl-fleet-mcp --daemon

    dimos mcp modules                      # three drones + the coordinator
    dimos mcp call fleet_state
    dimos mcp call takeoff_all --arg altitude=5
    dimos mcp call grid_sweep --arg corner_b_north=40 --arg corner_b_east=30
    dimos mcp call count_within --arg drone=drone1 --arg radius_m=100
    dimos mcp call line_formation
    dimos mcp call rtl_all

Per-drone commands address the instance directly:

    dimos mcp call drone2/px4dronemodule/takeoff --arg altitude=4
"""

from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.drone.px4_drone_module import Px4DroneModule
from dimos.robot.drone.px4_sitl_fleet_config import get_px4_sitl_fleet_configs
from dimos.robot.drone.px4_swarm_coordinator import SwarmCoordinator

# Streams that must stay global so they cross the namespace boundary: telemetry
# out of each drone, and fleet commands back in.
FLEET_BUS = {"drone_state", "swarm_cmd"}

_CONFIGS = get_px4_sitl_fleet_configs()


def px4_sitl_swarm(
    configs=_CONFIGS,
    min_separation_m: float = 2.0,
    max_altitude_m: float | None = None,
):
    """Compose N namespaced PX4 drones plus one shared coordinator."""
    return autoconnect(
        SwarmCoordinator.blueprint(
            min_separation_m=min_separation_m,
            max_altitude_m=max_altitude_m,
            expected_drones=",".join(f"drone{i + 1}" for i in range(len(configs))),
        ),
        *[
            Px4DroneModule.blueprint(
                connection_string=c.connection_string,
                instance=c.instance,
                sys_id=c.sys_id,
                max_altitude_m=max_altitude_m,
            ).namespace(f"drone{i + 1}", expose=FLEET_BUS)
            for i, c in enumerate(configs)
        ],
    )


drone_px4_sitl_fleet_mcp = autoconnect(
    px4_sitl_swarm(),
    McpServer.blueprint(),
).global_config(n_workers=max(2, 2 * len(_CONFIGS)))

__all__ = ["FLEET_BUS", "drone_px4_sitl_fleet_mcp", "px4_sitl_swarm"]
