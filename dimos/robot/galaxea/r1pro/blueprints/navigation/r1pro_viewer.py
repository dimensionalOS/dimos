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


"""The rerun half of r1pro-nav-lio alone, for a laptop: ``dimos run r1pro-viewer --robot-ip <R1>``.

The R1 runs r1pro-nav-lio with ``--viewer none`` behind a zenoh router on its port 7447; this joins that router
as a client (a router forwards to clients only, never between peers), so rendering costs the R1 nothing.
"""

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.robot.galaxea.r1pro.blueprints.navigation.r1pro_nav_lio import _MAX_HZ, _rerun_config
from dimos.visualization.vis_module import vis_module

r1pro_viewer = autoconnect(
    vis_module(
        viewer_backend=global_config.viewer,
        rerun_config={
            **_rerun_config,
            # One subscription per name: an unlisted topic never crosses the link. The local map is left out:
            # it is the heaviest stream, and the raw clouds plus the global map show the same space.
            "topics": ["tf", "lidar", "global_map", "planner_path", "goal"],
            # Here the clouds are the live view, not the onboard viewer's occasional glance.
            "max_hz": {**_MAX_HZ, "world/lidar": 10.0},
        },
    ),
).global_config(
    transport="zenoh",
    zenoh_mode="client",
    # The robot's stack owns the bus-wide `Coordinator` name; this one only watches.
    serve_coordinator_rpc=False,
    n_workers=3,
)
