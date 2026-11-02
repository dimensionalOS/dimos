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

"""The trajectory follower, running the hinted law off a locally held path at a steady rate."""

from __future__ import annotations

from typing import TYPE_CHECKING

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import IO, In, Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.std_msgs.Bool import Bool
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.navigation import spec
from dimos.navigation.embodiment.base import Embodiment


class TrajectoryFollowerNativeConfig(NativeModuleConfig):
    source_dir: str | None = "dimos/navigation/trajectory_follower/fancy/rust"
    executable: str = "result/bin/trajectory_follower"
    build_command: str | None = "nix build -L path:."
    stdin_config: bool = True
    # argv is ignored by the rust side, which reads stdin; keeping a thirty-field
    # object out of it leaves `ps` readable.
    cli_exclude: frozenset[str] = frozenset({"embodiment"})

    # Every field below crosses to the Rust struct verbatim, since a native
    # config has no Rust-side defaults.
    control_frequency: float = 10.0
    goal_tolerance: float = 0.20  # planar distance that counts as arrival in meters
    embodiment: Embodiment
    # The pose is the path.frame_id to base_frame edge on tf, read each tick.
    base_frame: str = "base_link"
    # The deadman zeroes the twist once the held path is this old. It must clear the ~1 Hz replan cadence.
    max_path_age_s: float = 2.5


class TrajectoryFollowerNative(NativeModule, spec.TrajectoryFollower):
    """Track the planned path; stop and latch goal_reached on arrival."""

    config: TrajectoryFollowerNativeConfig

    path: In[Path]
    tf: IO[TFMessage]  # IO, not In: see planner_native.py

    nav_cmd_vel: Out[Twist]
    goal_reached: Out[Bool]


if TYPE_CHECKING:
    TrajectoryFollowerNative()
