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

from unittest.mock import Mock

from dimos.control.task import JointCommandOutput
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.galaxea.r1pro.policyik_sim2 import PolicyIKSimModule


def test_pause_retains_commanded_gravity_offset_and_refreshes_expiry(tmp_path, request):
    module = PolicyIKSimModule(bundle=tmp_path)
    request.addfinalizer(module.stop)
    module._clock_id = "test"
    names = [f"joint_{i}" for i in range(18)]
    module._manifest = {"joints": names, "frame": "base", "kp": [80.0] * 18, "kd": [4.0] * 18}
    task = Mock()
    task.compute.return_value = JointCommandOutput(joint_names=names, positions=[0.2] * 18)
    module._task = task

    def measurement(t):
        # The measured positions differ from the loaded servo references.
        return JointState(ts=t, name=names, position=[0.25] * 18, velocity=[0.0] * 18)

    first = module.compute(measurement(1.0))
    assert first is not None
    module.set_paused(True)
    paused = module.compute(measurement(1.1))
    assert paused is not None
    assert task.compute.call_count == 1
    assert paused.joint_commands[0].values == [0.2] * 18
    assert paused.valid_until_ns > first.valid_until_ns
    assert paused.joint_commands[2].values == [80.0] * 18
    module.set_paused(False)
    assert module.compute(measurement(1.2)) is not None
    assert task.compute.call_count == 2
    module.set_estop(True)
    task.compute.return_value = None
    module.set_paused(True)
    assert module.compute(measurement(1.3)) is None
