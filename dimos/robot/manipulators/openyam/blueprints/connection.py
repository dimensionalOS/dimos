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

"""OpenYam trajectories and normalized grippers through a connection module.

No CAN selector runs the in-memory adapter. Supply the connection module's
CAN selector(s) explicitly for hardware; --simulation always uses the mock.
"""

from dimos.robot.manipulators.common.damiao_blueprint import damiao_connection_control
from dimos.robot.manipulators.openyam.config import OPENYAM_ARM_JOINTS
from dimos.robot.manipulators.openyam.connection import (
    OpenYamConnection,
    yam_connection_model,
)

openyam_connection_control = damiao_connection_control(
    OpenYamConnection,
    yam_connection_model(robot="arm"),
    list(OPENYAM_ARM_JOINTS),
    ("gripper",),
).remappings([])
