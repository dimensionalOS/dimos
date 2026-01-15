# Copyright 2025-2026 Dimensional Inc.
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

"""Pre-built blueprints for VR teleoperation."""

from dimos_lcm.geometry_msgs import Transform as LCMTransform

from dimos.core.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs import PoseStamped, Twist
from dimos.msgs.std_msgs import Bool, Float32
from dimos.teleop.devices.vr_teleop_module import vr_teleop_module

quest3_teleop = autoconnect(
    vr_teleop_module(
        output_types=[PoseStamped, Twist],
        input_labels=["left_vr", "right_vr"],
        visualize_in_rerun=True,
    ),
).transports(
    {
        ("vr_left_transform", LCMTransform): LCMTransport("/vr_left_transform", LCMTransform),
        ("vr_right_transform", LCMTransform): LCMTransport("/vr_right_transform", LCMTransform),
        ("vr_trigger_0", Float32): LCMTransport("/vr_trigger_0", Float32),
        ("vr_trigger_1", Float32): LCMTransport("/vr_trigger_1", Float32),
        ("teleop_enable", Bool): LCMTransport("/vr_teleop_enable", Bool),
    }
)

__all__ = [
    "quest3_teleop",
]
