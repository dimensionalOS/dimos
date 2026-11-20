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

"""dimos ``cmd_vel`` to the M20's vendor walking controller (rl_deploy) on its own ``rt/REAL_STEER`` DDS topic.

The vendor chain is NAV_CMD (the drdds-zenoh-bridge, m/s) -> basic_server -> REAL_STEER -> rl_deploy; this native
module writes REAL_STEER itself, in pure Rust (rustdds), so neither basic_server nor the bridge has to run. It needs
rl_deploy running and runs on AOS beside it. The limits and gating are the bridge's m20 mode: each axis clamped, only
while rl_deploy's MOTION_INFO is fresh and in RL control (17) and HES_STATUS is clear, zero otherwise, a final zero
once cmd_vel stops (rl_deploy itself zeroes a REAL_STEER older than 150 ms). It sends no gait or stand command.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from pydantic import Field

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.std_msgs.Bool import Bool
from dimos.msgs.std_msgs.Int32 import Int32
from dimos.msgs.std_msgs.UInt32 import UInt32


class M20CmdVelConfig(NativeModuleConfig):
    source_dir: str | None = "dimos/robot/deeprobotics/m20/cmd_vel/rust"
    executable: str = "result/bin/m20_cmd_vel"
    build_command: str | None = "nix build -L path:."
    stdin_config: bool = True

    domain_id: int = Field(default=0, ge=0, le=232)
    steer_topic: str = "rt/REAL_STEER"
    steer_type: str = "drdds::msg::dds_::Steer_"
    motion_info_topic: str = "rt/MOTION_INFO"
    motion_info_type: str = "drdds::msg::dds_::MotionInfo_"
    hes_status_topic: str = "rt/HES_STATUS"
    hes_status_type: str = "drdds::msg::dds_::StdMsgInt32_"
    # the vendor bridge's clamps: m/s forward, m/s left, rad/s counterclockwise
    max_linear_x: float = Field(default=2.0, ge=0.0)
    max_linear_y: float = Field(default=1.0, ge=0.0)
    max_angular_z: float = Field(default=2.0, ge=0.0)
    # seconds without a cmd_vel before the final zero, and without MOTION_INFO before rl_deploy counts as not ready
    command_timeout_s: float = Field(default=0.4, gt=0.0)
    state_timeout_s: float = Field(default=2.5, gt=0.0)
    # rl_deploy's MOTION_INFO state that takes velocity (RL control)
    rl_control_state: int = 17
    # write nothing while another REAL_STEER writer (basic_server) is up, so two never interleave
    exclusive: bool = True
    # seconds teardown waits for rl_deploy to acknowledge the final zero
    release_ack_wait_s: float = Field(default=0.2, ge=0.0)


class M20CmdVel(NativeModule):
    """cmd_vel -> rl_deploy's REAL_STEER, limited and gated as the vendor bridge; rl_deploy's state back out."""

    config: M20CmdVelConfig

    cmd_vel: In[Twist]
    command_ready: Out[Bool]
    motion_state: Out[Int32]
    gait_state: Out[UInt32]


if TYPE_CHECKING:
    M20CmdVel()
