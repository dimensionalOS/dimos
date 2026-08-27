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

"""The G1 actuator side: joint order, the GR00T gains, and the torque limits.

The chain itself (PD demand -> clip -> envelope -> lag) is
:mod:`dimos.simulation.sysid.plant`; the G1 has no measured envelope yet.
"""

from __future__ import annotations

import numpy as np

from dimos.control.components import _HUMANOID_29DOF_JOINTS
from dimos.control.tasks.g1_groot_wbc_task.g1_groot_wbc_task import G1_GROOT_KD, G1_GROOT_KP

# DDS / MJCF actuator order: legs (6 + 6), waist (3), arms (7 + 7).
JOINT_NAMES: tuple[str, ...] = tuple(_HUMANOID_29DOF_JOINTS)
NUM_JOINTS = len(JOINT_NAMES)

# Leg dofs in qvel/dof indexing: 6 free-joint dofs, then the twelve leg joints.
LEG_DOFS = slice(6, 18)

# The gains the GR00T policy was trained against and the recording was driven with.
KP = np.asarray(G1_GROOT_KP, dtype=np.float64)
KD = np.asarray(G1_GROOT_KD, dtype=np.float64)

# jnt_actfrcrange from g1_29dof.xml, N.m, actuator order; held against the
# compiled model by test_model. Clipped explicitly before the lag so the lag
# sees the clipped demand, as on the Go2.
TORQUE_LIMITS = np.array(
    [88.0, 88.0, 88.0, 139.0, 50.0, 50.0] * 2
    + [88.0, 50.0, 50.0]
    + [25.0, 25.0, 25.0, 25.0, 25.0, 5.0, 5.0] * 2
)
