# Copyright 2025-2026 Dimensional Inc.
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

"""Registration-scoped task observations and detached complete tick snapshots."""

from dataclasses import FrozenInstanceError
import threading

import pytest

from dimos.control._control_test_helpers import RecordingTask
from dimos.control.coordinator import ControlCoordinator
from dimos.control.task import (
    ControlTaskContext,
    CoordinatorState,
    JointStateSnapshot,
    SensorStateSnapshot,
)
from dimos.control.tick_loop import TickLoop
from dimos.hardware.whole_body.spec import IMUState


@pytest.fixture
async def coordinators(mocker):
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.start")
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.serve_module_rpc")
    instances = []

    def create():
        instance = ControlCoordinator(publish_joint_state=False)
        instances.append(instance)
        return instance

    try:
        yield create
    finally:
        for instance in instances:
            instance.stop()


def observation(position=0.2):
    return CoordinatorState(
        joints=JointStateSnapshot(
            joint_positions={"joint": position},
            joint_velocities={"joint": 0.3},
            joint_efforts={"joint": 0.4},
            timestamp=12.0,
        ),
        imu={"body": IMUState(gyroscope=(1, 2, 3))},
        sensors=SensorStateSnapshot(readings={"body": {"voltage": 24.0}}),
        t_now=10.0,
        dt=0.01,
    )


async def test_context_requires_registration_and_pre_tick_state_is_unavailable(coordinators):
    task = RecordingTask("reader")
    with pytest.raises(RuntimeError, match="not registered"):
        _ = task.context
    coordinator = coordinators()
    assert coordinator.add_task(task)
    assert task.context.get_state() is None


async def test_complete_observation_is_detached_from_publisher_and_each_reader(coordinators):
    coordinator = coordinators()
    task = RecordingTask("reader")
    coordinator.add_task(task)
    source = observation()
    coordinator._set_latest_state(source)
    source.joints.joint_positions["joint"] = -1
    source.sensors.readings["body"]["voltage"] = -1
    first = task.context.get_state()
    assert first == observation()
    first.joints.joint_velocities["joint"] = -1
    first.imu.clear()
    first.t_now = -1
    first.sensors.readings["body"]["voltage"] = -1
    second = task.context.get_state()
    assert second == observation()
    assert first is not second
    coordinator._set_latest_state(observation(0.5))
    assert task.context.get_state() == observation(0.5)


async def test_context_callback_is_fixed_but_returns_detached_observations():
    context = ControlTaskContext(get_state=lambda: None)
    assert context.get_state() is None
    with pytest.raises(FrozenInstanceError):
        context.get_state = lambda: observation()


async def test_context_is_exclusive_and_removal_revokes_then_allows_re_registration(coordinators):
    first, second = coordinators(), coordinators()
    task = RecordingTask("reader")
    first.add_task(task)
    with pytest.raises(RuntimeError, match="already registered"):
        second.add_task(task)
    assert second.list_tasks() == []
    assert first.remove_task("reader")
    with pytest.raises(RuntimeError, match="not registered"):
        _ = task.context
    assert second.add_task(task)
    second._set_latest_state(observation())
    assert task.context.get_state() == observation()


async def test_failed_route_registration_releases_task_context(coordinators):
    coordinator = coordinators()
    task = RecordingTask("reader")
    with pytest.raises(ValueError):
        coordinator.add_task(
            task, task_type="trajectory", stream_bind={"joint_command": "missing_port"}
        )
    assert coordinator.list_tasks() == []
    with pytest.raises(RuntimeError, match="not registered"):
        _ = task.context
    assert coordinator.add_task(task)


async def test_stop_and_runtime_reset_clear_observation_without_revoking_registration(coordinators):
    coordinator = coordinators()
    task = RecordingTask("reader")
    coordinator.add_task(task)
    context = task.context
    coordinator._set_latest_state(observation())
    assert coordinator.reset_runtime_state() == {}
    assert task.context is context and context.get_state() is None
    coordinator._set_latest_state(observation())
    coordinator.stop()
    assert task.context is context and context.get_state() is None


class TickReader(RecordingTask):
    def __init__(self):
        super().__init__("reader")
        self.observed = None

    def is_active(self):
        return True

    def compute(self, state):
        self.observed = self.context.get_state()
        return None


async def test_tick_publishes_complete_observation_before_task_compute_under_same_lock(
    coordinators,
):
    coordinator = coordinators()
    task = TickReader()
    coordinator.add_task(task)
    loop = TickLoop(
        tick_rate=100,
        hardware={},
        hardware_lock=threading.Lock(),
        tasks=coordinator._tasks,
        task_lock=coordinator._task_lock,
        joint_to_hardware={},
        observation_callback=coordinator._set_latest_state,
    )
    source = observation()
    loop._compute_all_tasks(source)
    assert task.observed == source
    assert task.observed is not source
    assert coordinator.get_joint_positions() == {"joint": 0.2}
