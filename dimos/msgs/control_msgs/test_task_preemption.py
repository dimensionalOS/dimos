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

from dimos.msgs.control_msgs.TaskPreemption import TaskPreemption


def test_task_preemption_round_trips_over_wire() -> None:
    message = TaskPreemption(
        timestamp=123.5,
        preempted_task="policy_rollout",
        preempting_task="teleop_openyam",
        joints=["arm/joint1", "arm/joint2"],
    )

    decoded = TaskPreemption.lcm_decode(message.lcm_encode())

    assert decoded.timestamp == 123.5
    assert decoded.preempted_task == "policy_rollout"
    assert decoded.preempting_task == "teleop_openyam"
    assert decoded.joints == ["arm/joint1", "arm/joint2"]
