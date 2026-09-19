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

"""LocalPlannerNative: the rust twin of :mod:`.module`, same ports, wire and defaults.

:mod:`.module` stays the reference. The one config field that does not cross is
``planner``: the deployed module is the rust target planner.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import IO, In, Out
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.navigation import spec
from dimos.navigation.embodiment.base import Embodiment
from dimos.navigation.local_planner.module import LocalPlannerConfig
from dimos.navigation.local_planner.search.base import RESOLUTION


def _default(field: str) -> Any:
    return LocalPlannerConfig.model_fields[field].default


class LocalPlannerNativeConfig(NativeModuleConfig):
    cwd: str | None = "rust"
    executable: str = "target/release/local_planner"
    build_command: str | None = "cargo build --release --features module"
    stdin_config: bool = True
    cli_exclude: frozenset[str] = frozenset({"embodiment"})

    # Every field crosses to rust verbatim and rust has no defaults, so a field missing
    # here fails startup. Defaults are read off the python config (test_planner.py).
    embodiment: Embodiment = _default("embodiment")
    body_dilate_m: float = _default("body_dilate_m")
    unseen_cost: float = _default("unseen_cost")
    resolution: float = RESOLUTION
    pointcloud_resolution: float = _default("pointcloud_resolution")
    replan_hz: float = _default("replan_hz")
    goal_lookahead_m: float = _default("goal_lookahead_m")
    world_frame: str = _default("world_frame")
    base_frame: str = _default("base_frame")
    replan_on_change: bool = _default("replan_on_change")
    replan_carrot_m: float = _default("replan_carrot_m")
    reset_carrot_m: float = _default("reset_carrot_m")
    obstacle_model: str = _default("obstacle_model")
    max_map_age_s: float = _default("max_map_age_s")


class LocalPlannerNative(NativeModule, spec.MapLocalPlanner):
    """Receding-horizon local planning over the live local map, in rust."""

    config: LocalPlannerNativeConfig

    local_map: In[PointCloud2]
    planner_path: In[Path]
    # IO, not In: rust `#[tf]` subscribes and publishes, and refuses to start on a mismatch.
    tf: IO[TFMessage]

    path: Out[Path]


if TYPE_CHECKING:
    LocalPlannerNative()
