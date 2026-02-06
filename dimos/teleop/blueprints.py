#!/usr/bin/env python3
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

"""Teleop blueprints for testing and deployment."""

from dimos.core.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.teleop.quest.quest_extensions import arm_teleop_module, visualizing_teleop_module
from dimos.teleop.quest.quest_types import QuestButtons

# -----------------------------------------------------------------------------
# Quest Teleop Blueprints
# -----------------------------------------------------------------------------

# Arm teleop with toggle-based engage
arm_teleop = autoconnect(
    arm_teleop_module(),
).transports(
    {
        ("left_controller_output", PoseStamped): LCMTransport("/teleop/left_delta", PoseStamped),
        ("right_controller_output", PoseStamped): LCMTransport("/teleop/right_delta", PoseStamped),
        ("buttons", QuestButtons): LCMTransport("/teleop/buttons", QuestButtons),
    }
)

# Arm teleop with Rerun visualization
arm_teleop_visualizing = autoconnect(
    visualizing_teleop_module(),
).transports(
    {
        ("left_controller_output", PoseStamped): LCMTransport("/teleop/left_delta", PoseStamped),
        ("right_controller_output", PoseStamped): LCMTransport("/teleop/right_delta", PoseStamped),
        ("buttons", QuestButtons): LCMTransport("/teleop/buttons", QuestButtons),
    }
)


__all__ = ["arm_teleop", "arm_teleop_visualizing"]
