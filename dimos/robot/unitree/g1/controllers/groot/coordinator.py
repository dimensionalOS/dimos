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

"""GR00T's existing coordinator host, wired to the robot's ordinary motor ports."""

import time

from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray


class G1GrootCoordinator(TeleopControlCoordinator):
    g1_motor_command: Out[MotorCommandArray]
    g1_motor_states: In[JointState]
    g1_imu: In[Imu]
    g1_joints: Out[JointState]

    @rpc
    def set_velocity_command(self, vx: float, vy: float, yaw_rate: float) -> None:
        """Walking intent from G1Connection, delivered to the GR00T task."""
        if "groot_wbc" not in self.list_tasks():
            raise RuntimeError("this G1 stack has no walking controller")
        self.task_invoke(
            "groot_wbc",
            "set_velocity_command",
            {"vx": vx, "vy": vy, "yaw_rate": yaw_rate, "t_now": time.perf_counter()},
        )
