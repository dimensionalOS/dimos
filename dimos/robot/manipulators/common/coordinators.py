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

"""Coordinators named for the command a single arm accepts.

Pose and twist commands are point-to-point: each consuming task instance
reads its own port. A single-arm deployment needs one port named like the
card's input, which is all these add — no control behavior. Multi-arm
deployments declare a port per arm and bind each task with
``TaskConfig.stream_bind`` instead.
"""

from __future__ import annotations

from dimos.control.coordinator import ControlCoordinator
from dimos.core.stream import In
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped


class ArmPoseCoordinator(ControlCoordinator):
    """Arm driven by target poses; the cartesian_ik / teleop_ik cards name-match."""

    cartesian_command: In[PoseStamped]


class ArmTwistCoordinator(ControlCoordinator):
    """Arm driven by EEF twists; the eef_twist card name-matches."""

    ee_twist_command: In[TwistStamped]


class ArmPoseTwistCoordinator(ArmPoseCoordinator):
    """Arm accepting both, e.g. VR poses preempting a browser twist jog."""

    ee_twist_command: In[TwistStamped]
