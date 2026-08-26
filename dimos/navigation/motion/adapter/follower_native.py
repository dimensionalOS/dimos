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

"""TrajectoryFollowerNative: the rust twin of :mod:`.follower`.

Same ports, same laws (`dimos_motion2_tc`, parity-locked to the python), and
a control tick with no python in it -- which is the point: on the robot the
follower runs off a locally-held path at a steady 10 Hz instead of receiving
``cmd_vel`` in bursts whenever the link hiccups. :mod:`.follower` stays the
reference implementation.

Two fields do NOT cross. ``controller``, because the track already names the
law and a native module has one law per track by construction; and
``stall_report_s``, because the rust side reports through throttled tracing
lines rather than a StallReporter.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import IO, In, Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Bool import Bool
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.navigation.motion.embodiment.base import Embodiment
from dimos.navigation.motion.embodiment.go2 import GO2


class TrajectoryFollowerNativeConfig(NativeModuleConfig):
    cwd: str | None = "rust"
    executable: str = "target/release/trajectory_follower"
    build_command: str | None = "cargo build --release"
    stdin_config: bool = True
    # argv is ignored by the rust side, which reads stdin; keeping a thirty-field
    # object out of it leaves `ps` readable.
    cli_exclude: frozenset[str] = frozenset({"embodiment"})

    # A TRACK, never a law (control/tracks.py). The rust maps track to law.
    track: str = "hinted"
    control_frequency: float = 10.0
    goal_tolerance: float = 0.20
    # Names the body rather than a number, so the half-width the governor reads
    # is the one the planner priced the plan with.
    embodiment: Embodiment = GO2
    # Every planning box grown by this much PER SIDE; negative shrinks it.
    # Both modules must carry the SAME value.
    body_dilate_m: float = 0.0
    base_frame: str = "base_link"
    # The planner's model, because the room hint has to be measured off the
    # slice the plan was priced in (motion/obstacles.py).
    obstacle_model: str = "body_band"
    idle_speed: float = 0.02
    # The deadman, as in follower.py: measured from ARRIVAL, guards a planner
    # that stopped speaking (dead, or alive and failing every tick). Must clear
    # the replan cadence (plans arrive per MAP, gaps to ~1.3 s observed).
    max_path_age_s: float = 2.5


class TrajectoryFollowerNative(NativeModule):
    """Track the planned path; stop and latch goal_reached on arrival."""

    config: TrajectoryFollowerNativeConfig

    path: In[Path]
    odometry: In[Odometry]
    local_map: In[PointCloud2]
    stop_movement: In[Bool]
    # IO, not In: `#[tf]` both subscribes and publishes, and the rust side
    # refuses to start unless the topic map matches the ports it claims.
    tf: IO[TFMessage]

    nav_cmd_vel: Out[Twist]
    goal_reached: Out[Bool]


if TYPE_CHECKING:
    TrajectoryFollowerNative()
