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

"""Go2 navigation driving the simulated legged Go2 to a clicked goal."""

from __future__ import annotations

from functools import partial
from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.robot.unitree.go2.blueprints.basic.go2_sim import go2_sim, rerun_blueprint, rerun_config
from dimos.robot.unitree.go2.blueprints.navigation.go2_nav import (
    _go2_nav,
    go2_nav_overrides,
    go2_nav_static,
)
from dimos.visualization.vis_module import vis_module

_nav_rerun_config: dict[str, Any] = {
    **rerun_config,
    "blueprint": partial(rerun_blueprint, hidden=("world/nodes", "world/node_edges")),
    "static": {**rerun_config["static"], **go2_nav_static()},
    "visual_override": {**rerun_config["visual_override"], **go2_nav_overrides()},
}

# The nav MovementManager dedupes onto the sim's. One worker per module.
go2_sim_nav = autoconnect(
    go2_sim,
    _go2_nav,
    vis_module(viewer_backend=global_config.viewer, rerun_config=_nav_rerun_config),
).global_config(n_workers=9)
