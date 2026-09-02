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

"""Rust teleop/nav velocity mux — the robot-side half of MovementManager."""

from __future__ import annotations

from typing import TYPE_CHECKING

from pydantic import Field

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.std_msgs.Bool import Bool


class CmdVelMuxNativeConfig(NativeModuleConfig):
    cwd: str | None = "rust"
    # The crate is a workspace member, so cargo builds into the repo-root target dir.
    executable: str = str(DIMOS_PROJECT_ROOT / "target" / "release" / "cmd_vel_mux")
    build_command: str | None = "cargo build --release"
    stdin_config: bool = True

    # How long a teleop command keeps nav preempted.
    tele_cooldown_sec: float = 1.0
    # Per-axis multipliers on teleop twists only; nav is forwarded raw. The
    # python module typed this as a Twist, which is not a config scalar.
    tele_scale_linear: list[float] = Field(default_factory=lambda: [1.0, 1.0, 1.0])
    tele_scale_angular: list[float] = Field(default_factory=lambda: [1.0, 1.0, 1.0])
    # The deadman: nav_cmd_vel unheard for this long holds cmd_vel at zero.
    # 0.5 s is five missed ticks of a 10 Hz follower. Tune on the robot.
    nav_stale_s: float = 0.5


class CmdVelMuxNative(NativeModule):
    """Teleop preempts nav on cmd_vel, and a watchdog zeros it when nav goes quiet.

    Pairs with MovementManager, which keeps the click relay and the NaN goal
    cancel. Both subscribe tele_cmd_vel so one keystroke reaches both halves.
    """

    config: CmdVelMuxNativeConfig

    nav_cmd_vel: In[Twist]
    tele_cmd_vel: In[Twist]

    cmd_vel: Out[Twist]
    stop_movement: Out[Bool]


if TYPE_CHECKING:
    CmdVelMuxNative()
