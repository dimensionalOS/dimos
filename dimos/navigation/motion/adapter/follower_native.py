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

from typing import TYPE_CHECKING, Any

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import IO, In, Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Bool import Bool
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.navigation.motion.adapter.follower import TrajectoryFollowerConfig
from dimos.navigation.motion.embodiment.base import Embodiment


def _default(field: str) -> Any:
    """The python follower's default for a field the native twin shares."""
    return TrajectoryFollowerConfig.model_fields[field].default


class TrajectoryFollowerNativeConfig(NativeModuleConfig):
    cwd: str | None = "rust"
    executable: str = "target/release/trajectory_follower"
    build_command: str | None = "cargo build --release"
    stdin_config: bool = True
    # argv is ignored by the rust side, which reads stdin; keeping a thirty-field
    # object out of it leaves `ps` readable.
    cli_exclude: frozenset[str] = frozenset({"embodiment"})

    # Every field below crosses to the rust struct verbatim, and every one of
    # them must: a native config has no rust-side defaults. Defaults are the
    # python follower's own, read off its config so the twins cannot drift
    # (test_follower.py asserts it).
    track: str = _default("track")
    control_frequency: float = _default("control_frequency")
    goal_tolerance: float = _default("goal_tolerance")
    embodiment: Embodiment = _default("embodiment")
    body_dilate_m: float = _default("body_dilate_m")
    base_frame: str = _default("base_frame")
    obstacle_model: str = _default("obstacle_model")
    idle_speed: float = _default("idle_speed")
    max_path_age_s: float = _default("max_path_age_s")


class TrajectoryFollowerNative(NativeModule):
    """Track the planned path; stop and latch goal_reached on arrival."""

    config: TrajectoryFollowerNativeConfig

    path: In[Path]
    odometry: In[Odometry]
    local_map: In[PointCloud2]
    stop_movement: In[Bool]
    tf: IO[TFMessage]  # IO, not In: see planner_native.py

    nav_cmd_vel: Out[Twist]
    goal_reached: Out[Bool]


if TYPE_CHECKING:
    TrajectoryFollowerNative()
