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

"""Focused contracts for immutable control observations and task context."""

from collections.abc import Callable, Iterator
import gc
import threading
from typing import Any
from weakref import ref

from attrs.exceptions import FrozenInstanceError
import pytest

from dimos.control._control_test_helpers import RecordingTask
from dimos.control.coordinator import ControlCoordinator
from dimos.control.task import (
    ControlTask,
    ControlTaskContext,
    CoordinatorState,
    JointCommandOutput,
    JointStateSnapshot,
    ResourceClaim,
)
from dimos.control.tick_loop import TickLoop


def _state(position: float = 1.0) -> CoordinatorState:
    return CoordinatorState(
        joints=JointStateSnapshot(
            joint_positions={"arm/joint1": position},
            joint_velocities={"arm/joint1": 2.0},
            joint_efforts={"arm/joint1": 3.0},
            timestamp=4.0,
        ),
        imu={},
        t_now=5.0,
        dt=0.01,
    )


@pytest.fixture
def make_coordinator() -> Iterator[Callable[[], ControlCoordinator]]:
    coordinators: list[ControlCoordinator] = []

    def make() -> ControlCoordinator:
        coordinator = ControlCoordinator(publish_joint_state=False)
        coordinators.append(coordinator)
        return coordinator

    try:
        yield make
    finally:
        for coordinator in coordinators:
            coordinator.stop()


def test_observation_fields_and_mappings_are_immutable() -> None:
    source_positions = {"arm/joint1": 1.0}
    source_imu = {}
    joints = JointStateSnapshot(joint_positions=source_positions)
    state = CoordinatorState(joints=joints, imu=source_imu)

    source_positions["arm/joint1"] = 9.0
    source_imu["body"] = None

    assert joints.joint_positions["arm/joint1"] == 1.0
    assert state.imu == {}
    with pytest.raises(TypeError):
        joints.joint_positions["arm/joint1"] = 2.0  # type: ignore[index]
    with pytest.raises(TypeError):
        state.imu["body"] = None  # type: ignore[index]
    with pytest.raises(FrozenInstanceError):
        state.dt = 1.0
    with pytest.raises(FrozenInstanceError):
        joints.timestamp = 1.0


def test_context_is_frozen_and_returns_callback_value_by_identity() -> None:
    state = _state()
    context = ControlTaskContext(get_state=lambda: state)

    assert context.get_state() is state
    with pytest.raises(FrozenInstanceError):
        context.get_state = lambda: None  # type: ignore[method-assign]


class ObservationTask(RecordingTask):
    def __init__(
        self,
        name: str,
        *,
        active: bool,
        published: list[CoordinatorState],
        computed: list[CoordinatorState],
    ) -> None:
        super().__init__(name)
        self._active = active
        self._published = published
        self._computed = computed

    def is_active(self) -> bool:
        assert len(self._published) == 1
        return self._active

    def compute(self, state: CoordinatorState) -> None:
        self._computed.append(state)
        return None


def test_tick_publishes_complete_observation_before_filtering_and_shares_identity() -> None:
    published: list[CoordinatorState] = []
    seen_by_compute: list[CoordinatorState] = []
    inactive = ObservationTask(
        "inactive",
        active=False,
        published=published,
        computed=seen_by_compute,
    )
    active = ObservationTask(
        "active",
        active=True,
        published=published,
        computed=seen_by_compute,
    )
    loop = TickLoop(
        tick_rate=100.0,
        hardware={},
        hardware_lock=threading.Lock(),
        tasks={"inactive": inactive, "active": active},
        task_lock=threading.Lock(),
        joint_to_hardware={},
        observation_callback=published.append,
    )

    loop._tick()

    assert seen_by_compute == [published[0]]
    assert published[0].joints.timestamp > 0.0


def test_coordinator_state_is_none_then_atomically_replaced(make_coordinator) -> None:
    coordinator = make_coordinator()
    task = RecordingTask("task")
    coordinator.add_task(task)
    first = _state(1.0)
    second = _state(2.0)

    assert task.context.get_state() is None
    coordinator._set_latest_state(first)
    assert task.context.get_state() is first
    coordinator._set_latest_state(second)
    assert task.context.get_state() is second


def test_state_publication_completes_while_task_lock_is_held(make_coordinator) -> None:
    coordinator = make_coordinator()
    task = RecordingTask("task")
    coordinator.add_task(task)
    entered = threading.Event()
    release = threading.Event()
    published = threading.Event()

    def hold_task_lock() -> None:
        with coordinator._task_lock:
            entered.set()
            release.wait(timeout=1.0)

    def publish_state() -> None:
        coordinator._set_latest_state(_state())
        published.set()

    holder = threading.Thread(target=hold_task_lock)
    holder.start()
    update = threading.Thread(target=publish_state)
    try:
        assert entered.wait(timeout=1.0)
        update.start()
        assert published.wait(timeout=1.0)
    finally:
        release.set()
        holder.join(timeout=1.0)
        if update.ident is not None:
            update.join(timeout=1.0)

    assert not holder.is_alive()
    assert not update.is_alive()
    assert task.context.get_state() is not None


class StructuralTask:
    """Protocol-compatible task intentionally not derived from BaseControlTask."""

    name = "structural"

    def claim(self) -> ResourceClaim:
        return ResourceClaim(joints=frozenset())

    def is_active(self) -> bool:
        return False

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        return None

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        pass

    def on_buttons(self, msg: Any) -> bool:
        return False

    def on_cartesian_command(self, pose: Any, t_now: float) -> bool:
        return False

    def on_ee_twist_command(self, twist: Any, t_now: float) -> bool:
        return False

    def set_target_by_name(self, positions: dict[str, float], t_now: float) -> bool:
        return False

    def set_velocities_by_name(self, velocities: dict[str, float], t_now: float) -> bool:
        return False

    def reset_runtime_state(self, reactivate: bool | None = None) -> bool:
        return False


def test_registration_grants_base_task_context(make_coordinator) -> None:
    coordinator = make_coordinator()
    base = RecordingTask("base")

    assert coordinator.add_task(base)
    assert base.context is coordinator._task_context


def test_registration_accepts_structural_tasks(make_coordinator) -> None:
    coordinator = make_coordinator()
    structural = StructuralTask()

    assert coordinator.add_task(structural)
    assert isinstance(structural, ControlTask)
    assert coordinator.list_tasks() == ["structural"]


def test_registration_rejects_task_bound_to_another_coordinator(make_coordinator) -> None:
    first = make_coordinator()
    second = make_coordinator()
    shared = RecordingTask("shared")
    assert first.add_task(shared)

    with pytest.raises(RuntimeError, match="another coordinator"):
        second.add_task(shared)
    assert second.list_tasks() == []


def test_failed_routing_registration_releases_context_access(make_coordinator) -> None:
    coordinator = make_coordinator()
    failing = RecordingTask("failing")

    with pytest.raises(ValueError, match="no input port"):
        coordinator.add_task(
            failing,
            task_type="servo",
            stream_bind={"joint_command": "missing_port"},
        )
    assert coordinator.list_tasks() == []
    with pytest.raises(RuntimeError, match="not registered"):
        failing.context  # noqa: B018


def test_removal_revokes_context_and_allows_registration_with_another_coordinator(
    make_coordinator,
) -> None:
    first = make_coordinator()
    second = make_coordinator()
    task = RecordingTask("task")
    first.add_task(task)

    assert first.remove_task(task.name)
    with pytest.raises(RuntimeError, match="not registered"):
        task.context  # noqa: B018
    assert second.add_task(task)
    assert task.context is second._task_context


def test_coordinator_deletion_revokes_context_access() -> None:
    coordinator = ControlCoordinator(publish_joint_state=False)
    task = RecordingTask("task")
    coordinator.add_task(task)
    coordinator_ref = ref(coordinator)

    coordinator.stop()
    assert task.context is coordinator._task_context
    del coordinator
    gc.collect()

    assert coordinator_ref() is None
    with pytest.raises(RuntimeError, match="not registered"):
        task.context  # noqa: B018


def test_stop_clears_state_without_detaching_tasks(make_coordinator) -> None:
    coordinator = make_coordinator()
    task = RecordingTask("task")
    coordinator.add_task(task)
    original_context = task.context

    coordinator._set_latest_state(_state())
    coordinator.stop()
    assert task.context is original_context
    assert task.context.get_state() is None


def test_runtime_reset_clears_state_without_detaching_tasks(make_coordinator) -> None:
    coordinator = make_coordinator()
    task = RecordingTask("task")
    coordinator.add_task(task)
    original_context = task.context

    coordinator._set_latest_state(_state())
    assert coordinator.reset_runtime_state() == {}
    assert task.context is original_context
    assert task.context.get_state() is None
