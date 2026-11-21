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

"""Joint messages <-> the plain {joint: value} mappings RobotModel takes.

SKETCH: signatures only.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
from typing import Literal

from dimos_lcm.control_msgs import DynamicJointState

from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.protocol.robot.model import JointValues, RobotModel

Quantity = Literal["position", "velocity", "effort"]

# Hardware joint name -> description joint name. A mapping, or a function for patterns.
NameMap = Mapping[str, str] | Callable[[str], str]


def joint_values(
    msg: JointState | DynamicJointState,
    quantity: Quantity = "position",
    names: NameMap | None = None,
) -> dict[str, float]:
    """One quantity per joint; a DynamicJointState reads it from the interface of that name."""
    raise NotImplementedError


def joint_state(
    robot: RobotModel,
    positions: JointValues,
    velocities: JointValues | None = None,
    efforts: JointValues | None = None,
    ts: float | None = None,
) -> JointState:
    """A JointState over the robot's movable joints, in description order."""
    raise NotImplementedError
