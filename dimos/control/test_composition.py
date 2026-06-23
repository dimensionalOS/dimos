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

from dimos.control.composition import ComposedControlTask
from dimos.control.task import (
    BaseControlTask,
    ControlMode,
    CoordinatorState,
    JointCommandOutput,
    JointStateSnapshot,
    ResourceClaim,
)

JOINTS = ["arm/j1", "arm/j2"]


class FakeSourceTask(BaseControlTask):
    def __init__(self) -> None:
        self.preemptions: list[tuple[str, frozenset[str]]] = []
        self.active = True
        self.output: JointCommandOutput | None = JointCommandOutput(
            joint_names=JOINTS,
            positions=[1.0, 2.0],
            mode=ControlMode.SERVO_POSITION,
        )

    @property
    def name(self) -> str:
        return "source"

    def claim(self) -> ResourceClaim:
        return ResourceClaim(frozenset(JOINTS), priority=7, mode=ControlMode.SERVO_POSITION)

    def is_active(self) -> bool:
        return self.active

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        return self.output

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        self.preemptions.append((by_task, joints))


class AddTransform(BaseControlTask):
    def __init__(self, name: str, delta: float, joints: list[str] | None = None) -> None:
        self._name = name
        self._delta = delta
        self._joints = joints or JOINTS
        self.preemptions: list[tuple[str, frozenset[str]]] = []
        self.reset_count = 0

    @property
    def name(self) -> str:
        return self._name

    def claim(self) -> ResourceClaim:
        return ResourceClaim(frozenset(self._joints), priority=7, mode=ControlMode.SERVO_POSITION)

    def is_active(self) -> bool:
        return True

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        return None

    def compute_from_reference(
        self,
        state: CoordinatorState,
        reference: JointCommandOutput,
    ) -> JointCommandOutput:
        assert reference.positions is not None
        return JointCommandOutput(
            joint_names=reference.joint_names,
            positions=[value + self._delta for value in reference.positions],
            mode=reference.mode,
        )

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        self.preemptions.append((by_task, joints))

    def reset(self) -> None:
        self.reset_count += 1


def make_state() -> CoordinatorState:
    return CoordinatorState(joints=JointStateSnapshot(), t_now=1.0, dt=0.01)


def test_composed_task_forwards_output_through_transforms_in_order() -> None:
    task = ComposedControlTask(
        "composed", FakeSourceTask(), [AddTransform("a", 1.0), AddTransform("b", 10.0)]
    )

    output = task.compute(make_state())

    assert output is not None
    assert output.positions == [12.0, 13.0]


def test_composed_task_active_state_follows_source_and_suppresses_none() -> None:
    source = FakeSourceTask()
    task = ComposedControlTask("composed", source, [AddTransform("a", 1.0)])

    source.active = False
    assert task.is_active() is False
    source.output = None
    assert task.compute(make_state()) is None


def test_composed_task_validation_rejects_empty_and_mismatched_pipeline() -> None:
    source = FakeSourceTask()

    try:
        ComposedControlTask("composed", source, [])
    except ValueError as exc:
        assert "requires at least one transform" in str(exc)
    else:
        raise AssertionError("empty transforms should fail")

    try:
        ComposedControlTask("composed", source, [AddTransform("bad", 0.0, joints=["arm/j1"])])
    except ValueError as exc:
        assert "do not match source joints" in str(exc)
    else:
        raise AssertionError("mismatched transform joints should fail")


def test_composed_task_forwards_preemption_and_resets_transforms() -> None:
    source = FakeSourceTask()
    transform = AddTransform("a", 1.0)
    task = ComposedControlTask("composed", source, [transform])

    task.on_preempted("higher", frozenset({"arm/j1"}))

    assert source.preemptions == [("higher", frozenset({"arm/j1"}))]
    assert transform.preemptions == [("higher", frozenset({"arm/j1"}))]
    assert transform.reset_count == 1
