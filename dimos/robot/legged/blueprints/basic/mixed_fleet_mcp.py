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

"""Mixed fleet: N namespaced PX4 drones and M namespaced legged robots, one world.

Both robot classes are composed the same way -- one module instance per vehicle
under its own namespace, publishing onto one shared fleet bus. The coordinator
does not know or care which is which; it reads ``robot_class`` off the state
messages. That uniformity is the point of the namespace architecture, and it is
what lets an operator say "anything that flies" or "anything on the ground".

    drone1/px4dronemodule/takeoff     aircraft
    dog1/leggedsimmodule/stand        ground robot
    fleet_state                       both, one report

Bring-up (the simulator must be listening before PX4 starts):

    python -m dimos.simulation.px4_hil.fleet_bridge --drones 3 --dogs 1
    # once per drone:
    cd ~/PX4-Autopilot/build/px4_sitl_default && PX4_SIM_MODEL=none_iris ./bin/px4 -i 0 -d
    python dimos/simulation/px4_hil/sim_params.py --count 3
    CI=1 dimos run mixed-fleet-mcp --daemon

Then::

    dimos mcp call fleet_state
    dimos mcp call takeoff_all --arg altitude=6
    dimos mcp call dog1/leggedsimmodule/crouch
    dimos mcp call rtl_all
"""

import os

from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.drone.blueprints.basic.drone_px4_sitl_fleet_mcp import FLEET_BUS
from dimos.robot.drone.px4_drone_module import Px4DroneModule
from dimos.robot.drone.px4_sitl_fleet_config import get_px4_sitl_fleet_configs
from dimos.robot.drone.px4_swarm_coordinator import SwarmCoordinator
from dimos.robot.legged.legged_sim_module import LeggedSimModule
from dimos.simulation.px4_hil.fleet_bridge import LEGGED_PORT_BASE

# Fleet size comes from the environment so one blueprint covers every mix
# without editing code. The MuJoCo bridge is the authority on what actually
# exists in the world -- these must match what you passed to
# `fleet_bridge --drones N --dogs M`, or DimOS will wait for vehicles that
# are not there.
#
#     SIM_DRONES=2 SIM_DOGS=0 dimos run mixed-fleet-mcp
DEFAULT_DRONE_COUNT = 3
DEFAULT_DOG_COUNT = 1


def _count(var: str, default: int) -> int:
    raw = os.environ.get(var)
    if raw is None or not raw.strip():
        return default
    try:
        value = int(raw)
    except ValueError as e:
        raise ValueError(f"{var} must be an integer, got {raw!r}") from e
    if value < 0:
        raise ValueError(f"{var} must be >= 0, got {value}")
    return value


_N_DRONES = _count("SIM_DRONES", DEFAULT_DRONE_COUNT)
_N_DOGS = _count("SIM_DOGS", DEFAULT_DOG_COUNT)

# fleet_size=0 is legitimate (a dogs-only world), but the config helper rejects
# it, so ask for nothing rather than asking for zero.
_DRONES = get_px4_sitl_fleet_configs(fleet_size=_N_DRONES) if _N_DRONES else []


def mixed_fleet(
    drone_configs=_DRONES,
    n_dogs: int = _N_DOGS,
    min_separation_m: float = 2.0,
    # 40 m default: below the 45 m PX4 geofence ceiling, so a too-high command
    # gets the coordinator's polite refusal, never the fence. Pass None to
    # remove the fleet cap (each drone still enforces its own).
    max_altitude_m: float | None = 40.0,
):
    """Compose namespaced drones and legged robots around one shared coordinator."""
    drone_keys = [f"drone{i + 1}" for i in range(len(drone_configs))]
    dog_keys = [f"dog{i + 1}" for i in range(n_dogs)]
    return autoconnect(
        SwarmCoordinator.blueprint(
            min_separation_m=min_separation_m,
            max_altitude_m=max_altitude_m,
            expected_drones=",".join(drone_keys + dog_keys),
        ),
        *[
            Px4DroneModule.blueprint(
                connection_string=c.connection_string,
                instance=c.instance,
                sys_id=c.sys_id,
                max_altitude_m=max_altitude_m,
            ).namespace(key, expose=FLEET_BUS)
            for key, c in zip(drone_keys, drone_configs, strict=True)
        ],
        *[
            LeggedSimModule.blueprint(
                endpoint=f"udp:127.0.0.1:{LEGGED_PORT_BASE + i}",
            ).namespace(key, expose=FLEET_BUS)
            for i, key in enumerate(dog_keys)
        ],
    )


mixed_fleet_mcp = autoconnect(
    mixed_fleet(),
    McpServer.blueprint(),
).global_config(n_workers=max(3, 2 * (len(_DRONES) + _N_DOGS)))

__all__ = ["DEFAULT_DOG_COUNT", "mixed_fleet", "mixed_fleet_mcp"]
