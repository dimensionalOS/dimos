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

"""AttitudeSample: one ``gimbal_attitude`` message for tests and simulators.

The message is a JointState with ``gimbal_roll``, ``gimbal_pitch``, ``gimbal_yaw`` in
radians and the GIMBAL_DEVICE_FLAGS bits in ``effort``.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from dimos.hardware.gimbal.siyi.frame import GimbalDeviceFlags
from dimos.msgs.sensor_msgs.JointState import JointState

GIMBAL_JOINTS = ("gimbal_roll", "gimbal_pitch", "gimbal_yaw")
A8_FOLLOW_FLAGS = GimbalDeviceFlags.YAW_LOCK | GimbalDeviceFlags.YAW_IN_VEHICLE_FRAME


@dataclass(frozen=True)
class AttitudeSample:
    t: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    flags: int = A8_FOLLOW_FLAGS
    failure_flags: int = 0

    def joint_state(self, frame_id: str = "gimbal_base") -> JointState:
        return JointState(
            ts=self.t,
            frame_id=frame_id,
            name=list(GIMBAL_JOINTS),
            position=[
                math.radians(self.roll_deg),
                math.radians(self.pitch_deg),
                math.radians(self.yaw_deg),
            ],
            velocity=[],
            effort=[float(self.flags), float(self.failure_flags), 0.0],
        )
