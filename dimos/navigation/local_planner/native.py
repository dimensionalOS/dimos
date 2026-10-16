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

"""LocalPlannerNative: the SE(2) local planner over the live local map, in rust."""

from __future__ import annotations

from typing import TYPE_CHECKING

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import IO, In, Out
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.navigation import spec
from dimos.navigation.embodiment.base import Embodiment
from dimos.navigation.embodiment.go2 import GO2


class LocalPlannerNativeConfig(NativeModuleConfig):
    source_dir: str | None = "dimos/navigation/local_planner/rust"
    executable: str = "result/bin/local_planner"
    build_command: str | None = "nix build -L path:."
    stdin_config: bool = True
    cli_exclude: frozenset[str] = frozenset({"embodiment"})

    # Every field crosses to rust verbatim and rust has no defaults, so a field missing
    # here fails startup.
    embodiment: Embodiment = GO2
    # Grows (negative: shrinks) every planning box per side; the hard margin stays the embodiment's precision floor.
    body_dilate_m: float = 0.0
    # Price multiplier per meter over lattice cells the cloud saw no floor under; <= 1 turns it off.
    unseen_cost: float = 5.0
    # Extra price per meter where the body steps on a low object, at steppable height. Scales with height.
    step_cost: float = 5.0
    # Speed ceiling (m/s) over waypoints that step on a low object.
    step_speed: float = 0.5
    # Waypoint spacing of the published route (m).
    resolution: float = 0.1
    replan_hz: float = 5.0
    goal_lookahead_m: float = 5.0  # carrot arc along the global path
    # The pose is the world_frame to base_frame edge on tf, read each tick.
    world_frame: str = "odom"
    base_frame: str = "base_link"
    # Replan only when the map or the carrot (by replan_carrot_m) changed; False replans every tick.
    replan_on_change: bool = True
    replan_carrot_m: float = 0.3
    # A carrot jump this far is a new task: the warm start and hysteresis are reset.
    reset_carrot_m: float = 1.0
    # What counts as an obstacle; "body_band" reads the cloud against the surface the feet stand on.
    obstacle_model: str = "body_band"
    # Hold once the local map is this old: a dropped link must not leave us replanning on a frozen world.
    max_map_age_s: float = 5.0


class LocalPlannerNative(NativeModule, spec.MapLocalPlanner):
    """Receding-horizon local planning over the live local map, in rust."""

    config: LocalPlannerNativeConfig

    local_map: In[PointCloud2]
    planner_path: In[Path]
    # IO, not In: rust `#[tf]` subscribes and publishes, and refuses to start on a mismatch.
    tf: IO[TFMessage]

    path: Out[Path]
    # The floor voxels the ground plane was fit to, moved onto it. Empty while the body height stands in.
    ground_support: Out[PointCloud2]


if TYPE_CHECKING:
    LocalPlannerNative()
