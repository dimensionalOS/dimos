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

"""MotionPlannerNative: the rust twin of :mod:`.planner`.

Same ports, same wire, same defaults -- and no python in the replan tick, so
it can run on the robot beside the follower. :mod:`.planner` stays the
reference implementation; this exists because the deployment plan moves the
time-critical half of the stack onto the Go2.

The one config field that does NOT cross is ``planner``: the deployed module
IS the rust target planner, and a wrapper that wants a different one is not
this module.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import IO, In, Out
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.navigation.motion.adapter.planner import MotionPlannerConfig
from dimos.navigation.motion.embodiment.base import Embodiment
from dimos.navigation.motion.planner.planners.base import RESOLUTION


def _default(field: str) -> Any:
    """The python planner's default for a field the native twin shares."""
    return MotionPlannerConfig.model_fields[field].default


class MotionPlannerNativeConfig(NativeModuleConfig):
    cwd: str | None = "rust"
    executable: str = "target/release/motion_planner"
    build_command: str | None = "cargo build --release"
    stdin_config: bool = True
    cli_exclude: frozenset[str] = frozenset({"embodiment"})

    # Every field below crosses to the rust struct verbatim, and every one of
    # them must: a native config has no rust-side defaults, so a field added
    # there and not here fails startup with `missing [...]`. Defaults are the
    # python module's own, read off its config so the twins cannot drift
    # (test_planner.py asserts it).
    embodiment: Embodiment = _default("embodiment")
    body_dilate_m: float = _default("body_dilate_m")
    unseen_cost: float = _default("unseen_cost")
    # planners/base.py RESOLUTION; here it crosses explicitly, because the rust
    # planner has no python default to read.
    resolution: float = RESOLUTION
    replan_hz: float = _default("replan_hz")
    goal_lookahead_m: float = _default("goal_lookahead_m")
    world_frame: str = _default("world_frame")
    base_frame: str = _default("base_frame")
    replan_on_change: bool = _default("replan_on_change")
    replan_carrot_m: float = _default("replan_carrot_m")
    reset_carrot_m: float = _default("reset_carrot_m")
    obstacle_model: str = _default("obstacle_model")
    max_map_age_s: float = _default("max_map_age_s")


class MotionPlannerNative(NativeModule):
    """Receding-horizon local planning over the live local map, in rust."""

    config: MotionPlannerNativeConfig

    local_map: In[PointCloud2]
    planner_path: In[Path]
    # IO, not In: `#[tf]` both subscribes and publishes, and the rust side
    # refuses to start unless the topic map matches the ports it claims.
    tf: IO[TFMessage]

    path: Out[Path]


if TYPE_CHECKING:
    MotionPlannerNative()
