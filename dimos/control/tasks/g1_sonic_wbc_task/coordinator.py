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

"""Connect SONIC task faults to the final G1 command publisher."""

import asyncio

from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.g1_sonic_wbc_task.g1_sonic_wbc_task import G1SonicWBCTask
from dimos.core.core import rpc
from dimos.core.global_config import global_config
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.String import String


class SonicCoordinator(ControlCoordinator):
    g1_joints: Out[JointState]
    sonic_fault: Out[String]
    g1_fault: In[String]

    def _setup_from_config(self) -> None:
        super()._setup_from_config()
        for task in self._tasks.values():
            if isinstance(task, G1SonicWBCTask) and not global_config.simulation:
                task.set_fault_publisher(self._publish_sonic_fault)

    def _publish_sonic_fault(self, reason: str) -> None:
        self.sonic_fault.publish(String(reason))

    @rpc
    def stop(self) -> None:
        super().stop()
        with self._task_lock:
            for task in self._tasks.values():
                if isinstance(task, G1SonicWBCTask):
                    task.stop()

    async def handle_g1_fault(self, msg: String) -> None:
        await asyncio.to_thread(self._apply_g1_fault, msg.data)

    def _apply_g1_fault(self, reason: str) -> None:
        with self._task_lock:
            for task in self._tasks.values():
                if isinstance(task, G1SonicWBCTask):
                    task.on_hardware_fault(reason)
