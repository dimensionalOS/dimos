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


import pytest

from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.coordinator import TaskConfig
from dimos.control.hardware_interface import ConnectedWholeBody
from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.hand_guiding_task.hand_guiding_task import HandGuidingTask, create_task
from dimos.hardware.whole_body.spec import WholeBodyAdapter, WholeBodyConfig


@pytest.mark.parametrize("position", [None, float("nan"), float("inf")])
def test_hand_guiding_waits_for_complete_finite_feedback(position):
    task = HandGuidingTask("teach", ["joint"], 10)
    task.start()
    positions = {} if position is None else {"joint": position}
    assert (
        task.compute(CoordinatorState(joints=JointStateSnapshot(joint_positions=positions))) is None
    )


@pytest.mark.parametrize("event", ["stop", "estop", "preempt"])
def test_hand_guiding_stays_stopped_until_explicit_restart(event):
    task = HandGuidingTask("teach", ["joint"], 10)
    state = CoordinatorState(joints=JointStateSnapshot(joint_positions={"joint": 0.2}))
    assert task.compute(state) is None
    assert task.start()
    assert task.compute(state).positions == [0.2]

    if event == "stop":
        task.stop()
    elif event == "estop":
        task.set_estop(True)
        assert not task.start()
        task.set_estop(False)
    else:
        task.on_preempted("other", frozenset({"joint"}))

    assert not task.is_active()
    assert task.compute(state) is None
    assert task.start()
    assert task.compute(state).positions == [0.2]


@pytest.mark.parametrize("kp", [None, (10.0,)])
def test_hand_guiding_rejects_hardware_without_explicit_zero_stiffness(mocker, kp):
    component = HardwareComponent(
        hardware_id="arm",
        hardware_type=HardwareType.WHOLE_BODY,
        joints=["joint"],
        wb_config=WholeBodyConfig(kp=kp),
    )
    hardware = ConnectedWholeBody(mocker.Mock(spec=WholeBodyAdapter), component)
    config = TaskConfig(name="teach", type="hand_guiding", joint_names=["joint"])
    with pytest.raises(ValueError, match="explicit zero kp"):
        create_task(config, {"arm": hardware})


def test_hand_guiding_rejects_unconnected_joints():
    config = TaskConfig(name="teach", type="hand_guiding", joint_names=["missing"])
    with pytest.raises(ValueError, match="no connected hardware"):
        create_task(config, {})
