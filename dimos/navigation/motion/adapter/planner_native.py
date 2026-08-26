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

from typing import TYPE_CHECKING

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import IO, In, Out
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.navigation.motion.embodiment.base import Embodiment
from dimos.navigation.motion.embodiment.go2 import GO2


class MotionPlannerNativeConfig(NativeModuleConfig):
    cwd: str | None = "rust"
    executable: str = "target/release/motion_planner"
    build_command: str | None = "cargo build --release"
    stdin_config: bool = True
    cli_exclude: frozenset[str] = frozenset({"embodiment"})

    # Every field below crosses to the rust struct verbatim, and every one of
    # them must: a native config has no rust-side defaults, so a field added
    # there and not here fails startup with `missing [...]`.
    embodiment: Embodiment = GO2
    # Every planning box grown by this much PER SIDE; negative shrinks it.
    # Both modules must carry the SAME value.
    body_dilate_m: float = 0.0
    # planners/base.py RESOLUTION; here it crosses explicitly, because the rust
    # planner has no python default to read.
    resolution: float = 0.1
    replan_hz: float = 5.0
    goal_lookahead_m: float = 5.0
    world_frame: str = "odom"
    base_frame: str = "base_link"
    replan_on_change: bool = True
    replan_carrot_m: float = 0.2
    reset_carrot_m: float = 1.0
    obstacle_model: str = "body_band"
    max_map_age_s: float = 5.0
    viz_publish_hz: float = 2.0


class MotionPlannerNative(NativeModule):
    """Receding-horizon local planning over the live local map, in rust."""

    config: MotionPlannerNativeConfig

    local_map: In[PointCloud2]
    odometry: In[Odometry]
    planner_path: In[Path]
    # IO, not In: `#[tf]` both subscribes and publishes, and the rust side
    # refuses to start unless the topic map matches the ports it claims.
    tf: IO[TFMessage]

    path: Out[Path]
    plan_body: Out[Path]  # the same plan, for the viewer's body boxes


if TYPE_CHECKING:
    MotionPlannerNative()
