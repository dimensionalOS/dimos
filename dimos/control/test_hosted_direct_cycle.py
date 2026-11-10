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

"""The hosted fast cycle: read, compute, arbitrate and write in one tick.

A hosted connection is a ``ConnectionModule`` with ``hosted=True`` deployed
into the coordinator's own worker process. The tick loop then reads its
hardware directly for the feedback tasks compute on, and writes the arbitrated
result back, with no command or reading message in between. These tests prove
the order, the freshness, the e-stop generation gate and the bounded telemetry,
in process and once with real worker processes.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
import os
import threading
import time
from typing import Any

import pytest

from dimos.control._control_test_helpers import FAKE_ARM_JOINTS, FakeArm, fake_arm_model
from dimos.control.contract.convert import motor_joints
from dimos.control.coordinator import ControlCoordinator
from dimos.control.task import (
    BaseControlTask,
    ControlMode,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.control.tasks.trajectory_task.trajectory_task import (
    JointTrajectoryTask,
    JointTrajectoryTaskConfig,
    TrajectoryExecutionStatus,
    joint_trajectory_task,
)
from dimos.control.tick_loop import TickLoop
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.utils.testing.waiting import wait_until

MOCK_JOINTS = motor_joints(fake_arm_model(), "mock")
MOCK_MODEL = fake_arm_model().with_renamed_joints(
    dict(zip(FAKE_ARM_JOINTS, MOCK_JOINTS, strict=True))
)
AT_REST = [0.0] * len(MOCK_JOINTS)


class RecordingTransport:
    """An in-process transport that notes every published message."""

    def __init__(self, *, release: threading.Event | None = None) -> None:
        self.sent: list[Any] = []
        self.subscribers: list[Callable[[Any], Any]] = []
        self._release = release

    def broadcast(self, selfstream: Any, value: Any) -> None:
        if self._release is not None:
            self._release.wait()
        self.sent.append(value)
        for callback in list(self.subscribers):
            callback(value)

    def publish(self, value: Any) -> None:
        self.broadcast(None, value)

    def subscribe(
        self, callback: Callable[[Any], Any], selfstream: Any = None
    ) -> Callable[[], None]:
        self.subscribers.append(callback)
        return lambda: self.subscribers.remove(callback)

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass


class HostedArmCoordinator(ControlCoordinator):
    """Hosted coordinator for one FakeArm called "mock"."""

    hosted_connections = True
    mock_position_command: Out[JointState]
    mock_joint_state: In[JointState]


class SendsPositions(BaseControlTask):
    """Always active; asks for fixed positions."""

    def __init__(self, positions: dict[str, float]) -> None:
        self._name = "sends_positions"
        self._positions = positions

    def claim(self) -> ResourceClaim:
        return ResourceClaim(joints=frozenset(self._positions), priority=10)

    def is_active(self) -> bool:
        return True

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        return JointCommandOutput(
            joint_names=list(self._positions),
            positions=list(self._positions.values()),
            mode=ControlMode.POSITION,
        )

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        pass


class FollowsFeedback(SendsPositions):
    """Asks for one joint's measured position plus a step, every tick."""

    def __init__(self, joint: str, step: float) -> None:
        super().__init__({joint: step})
        self._name = "follows_feedback"
        self._joint = joint
        self._step = step

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        measured = state.joints.joint_positions[self._joint]
        return JointCommandOutput(
            joint_names=[self._joint],
            positions=[measured + self._step],
            mode=ControlMode.POSITION,
        )


class BlocksInCompute(SendsPositions):
    """Waits inside compute while ``armed`` is set, until the test releases it."""

    def __init__(self, position: float) -> None:
        super().__init__({"mock/joint1": position})
        self._name = "blocks_in_compute"
        self.armed = threading.Event()
        self.entered = threading.Event()
        self.release = threading.Event()

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        if self.armed.is_set():
            self.entered.set()
            assert self.release.wait(timeout=10.0)
        return super().compute(state)


class Rig:
    """A started hosted coordinator whose tick loop only moves when ``tick`` is called."""

    def __init__(self, mocker: Any, *, deadman_timeout_s: float = 0.1) -> None:
        mocker.patch.object(TickLoop, "start")
        self.arm = FakeArm(
            source="mock",
            model=fake_arm_model(),
            hosted=True,
            deadman_timeout_s=deadman_timeout_s,
        )
        self.telemetry = RecordingTransport()
        self.arm.set_transport("joint_state", self.telemetry)  # type: ignore[arg-type]
        self.arm.start()
        self.coordinator = HostedArmCoordinator(model=MOCK_MODEL, reading_timeout_s=5.0)
        self.commands = RecordingTransport()
        self.coordinator.set_transport("mock_position_command", self.commands)  # type: ignore[arg-type]
        self.coordinator.start()
        assert self.coordinator._tick_loop is not None
        self.tick = self.coordinator._tick_loop._tick

    def stop(self) -> None:
        self.coordinator.stop()
        self.arm.stop()


@pytest.fixture
def rig(mocker: Any) -> Iterator[Callable[..., Rig]]:
    made: list[Rig] = []

    def build(**kwargs: Any) -> Rig:
        made.append(Rig(mocker, **kwargs))
        return made[-1]

    yield build
    for r in made:
        r.stop()


def test_the_cycle_reads_this_tick_and_writes_without_an_action_message(
    rig: Callable[..., Rig],
) -> None:
    r = rig()
    r.coordinator.add_task(FollowsFeedback("mock/joint1", 0.1))

    r.tick()
    assert r.arm.writes[-1]["mock/joint1/position"] == pytest.approx(0.1)
    r.tick()
    # The second tick computed on the first tick's write, not on an older
    # published reading: nothing was ever fed into arm_joint_state.
    assert r.arm.writes[-1]["mock/joint1/position"] == pytest.approx(0.2)
    # Nothing went over the command port: the write happened in the cycle.
    assert r.commands.sent == []
    # The measured reading is queued for external consumers, off the cycle.
    wait_until(
        lambda: any(msg.position[0] == pytest.approx(0.1) for msg in r.telemetry.sent),
        timeout=2.0,
    )


def test_telemetry_publication_never_paces_the_cycle(rig: Callable[..., Rig]) -> None:
    r = rig()
    release = threading.Event()
    r.arm.set_transport("joint_state", RecordingTransport(release=release))  # type: ignore[arg-type]
    r.coordinator.add_task(SendsPositions({"mock/joint1": 0.3}))

    started = time.monotonic()
    for _ in range(5):
        r.tick()
    assert time.monotonic() - started < 2.0
    assert r.arm.writes[-1]["mock/joint1/position"] == pytest.approx(0.3)
    release.set()


def test_an_estop_latched_during_compute_rejects_the_write(rig: Callable[..., Rig]) -> None:
    r = rig(deadman_timeout_s=0.05)
    task = BlocksInCompute(0.4)
    r.coordinator.add_task(task)
    r.tick()
    writes = len(r.arm.writes)
    task.armed.set()

    thread = threading.Thread(target=r.tick)
    thread.start()
    assert task.entered.wait(timeout=5.0)
    # E-stop must not wait for heavy compute: only its own bounded state
    # change and the hardware lock, which the blocked tick does not hold.
    stopped = threading.Event()

    def stop() -> None:
        r.coordinator.set_estop(True)
        stopped.set()

    estop = threading.Thread(target=stop)
    estop.start()
    try:
        assert stopped.wait(timeout=0.5), "set_estop waits for the task lock compute held"
        assert r.coordinator._command_generation == 1
    finally:
        task.release.set()
        thread.join(timeout=5.0)
        estop.join(timeout=5.0)

    assert not thread.is_alive()
    assert len(r.arm.writes) == writes
    wait_until(lambda: r.arm.halts > 0, timeout=2.0)


def test_a_cancel_with_a_trajectory_blocked_in_compute_returns_and_drops_its_write(
    rig: Callable[..., Rig],
) -> None:
    r = rig()
    entered, release = threading.Event(), threading.Event()
    task = JointTrajectoryTask(JointTrajectoryTaskConfig(joint_names=tuple(MOCK_JOINTS)))
    original = task.compute

    def blocking_compute(state: CoordinatorState) -> JointCommandOutput | None:
        entered.set()
        assert release.wait(timeout=5.0)
        return original(state)

    task.compute = blocking_compute  # type: ignore[method-assign]
    r.coordinator.add_task(task)
    trajectory = JointTrajectory(
        joint_names=MOCK_JOINTS,
        points=[
            TrajectoryPoint(positions=AT_REST, velocities=[0.0] * len(MOCK_JOINTS)),
            TrajectoryPoint(
                positions=[value + 0.1 for value in AT_REST],
                velocities=[0.0] * len(MOCK_JOINTS),
                time_from_start=0.3,
            ),
        ],
    )
    assert task.execute(trajectory, {joint: 0.0 for joint in MOCK_JOINTS}).status is (
        TrajectoryExecutionStatus.ACCEPTED
    )
    writes = len(r.arm.writes)
    compute = threading.Thread(target=r.tick)
    compute.start()
    assert entered.wait(timeout=5.0)
    cancelled = threading.Event()

    def cancel() -> None:
        r.coordinator.cancel_trajectory()
        cancelled.set()

    cancellation = threading.Thread(target=cancel)
    cancellation.start()
    try:
        assert cancelled.wait(timeout=0.5), "cancel_trajectory waits for heavy compute"
        assert r.coordinator._command_generation == 1
    finally:
        release.set()
        compute.join(timeout=5.0)
        cancellation.join(timeout=5.0)

    assert not compute.is_alive()
    assert not cancellation.is_alive()
    # The tick computed before the cancellation may not move the robot.
    assert len(r.arm.writes) == writes


def test_a_command_computed_before_a_clear_is_not_replayed(rig: Callable[..., Rig]) -> None:
    r = rig()
    task = BlocksInCompute(0.4)
    r.coordinator.add_task(task)
    r.tick()
    writes = len(r.arm.writes)
    task.armed.set()

    thread = threading.Thread(target=r.tick)
    thread.start()
    assert task.entered.wait(timeout=5.0)
    estop = threading.Thread(target=r.coordinator.set_estop, args=(True,))
    estop.start()
    wait_until(lambda: r.coordinator._command_generation == 1, timeout=5.0)
    clear = threading.Thread(target=r.coordinator.set_estop, args=(False,))
    clear.start()
    task.release.set()
    thread.join(timeout=5.0)
    estop.join(timeout=5.0)
    clear.join(timeout=5.0)

    # The tick computed before the stop/clear pair may not move the robot.
    assert len(r.arm.writes) == writes
    # A tick after the clear, computed on a fresh reading, may.
    wait_until(lambda: r.coordinator._command_generation == 2, timeout=5.0)
    r.tick()
    assert r.arm.writes[-1]["mock/joint1/position"] == pytest.approx(0.4)


def test_a_hosted_coordinator_without_the_connection_in_process_refuses_to_start(
    mocker: Any,
) -> None:
    mocker.patch.object(TickLoop, "start")
    coordinator = HostedArmCoordinator(model=MOCK_MODEL)
    try:
        with pytest.raises(ValueError, match="needs 'mock'\\'s connection module in its worker"):
            coordinator.start()
    finally:
        coordinator.stop()


def test_local_connection_binds_the_instance_in_the_coordinator_namespace(
    mocker: Any,
) -> None:
    from dimos.control.connection.mock_joint_connection import MockJointConnection

    mocker.patch.object(TickLoop, "start")
    first = MockJointConnection(
        instance_name="one/arm", source="mock", model=fake_arm_model(), hosted=True
    )
    second = MockJointConnection(
        instance_name="two/arm", source="mock", model=fake_arm_model(), hosted=True
    )
    coordinator = HostedArmCoordinator(instance_name="one/control", model=MOCK_MODEL)
    other = HostedArmCoordinator(instance_name="three/control", model=MOCK_MODEL)
    try:
        # Two connections drive the same robot name in this process; the
        # coordinator must pick its own instance, not whichever registered first.
        assert coordinator._local_connection("mock") is first
        assert first is not second
        # A coordinator in no matching namespace must refuse, not guess.
        with pytest.raises(ValueError, match="cannot tell which one belongs"):
            other._local_connection("mock")
    finally:
        coordinator.stop()
        other.stop()
        first.stop()
        second.stop()


# End to end, with the connection and the coordinator in real worker processes.

TARGET = [0.2, -0.1, 0.3, 0.0, 0.1, -0.2, 0.25]


class HostedRobots(ControlCoordinator):
    """Ports for FakeArm driven directly; no command or reading port is used."""

    hosted_connections = True
    mock_position_command: Out[JointState]
    mock_joint_state: In[JointState]


@pytest.fixture
def hosted_system() -> Iterator[ModuleCoordinator]:
    blueprint = autoconnect(
        HostedRobots.blueprint(
            instance_name="HostedDirectCheck",
            model=MOCK_MODEL,
            reading_timeout_s=10.0,
            tasks=[joint_trajectory_task(MOCK_JOINTS)],
        ),
        FakeArm.blueprint(source="mock", model=fake_arm_model(), hosted=True),
    ).remappings(
        [
            (FakeArm, "position_command", "mock_position_command"),
            (HostedRobots, "mock_joint_state", "joint_state"),
        ]
    )
    parsed = BlueprintConfigParser(blueprint).parse(environ={}, overrides={"g": {"viewer": "none"}})
    built = ModuleCoordinator.build(blueprint, parsed)
    manager: Any = built._managers["python"]
    pids = [worker.pid for worker in manager.workers if worker.pid is not None]
    assert pids, "the hosted pair must run in real worker processes"
    yield built
    built.stop()
    for pid in pids:
        wait_until(lambda pid=pid: not _process_alive(pid), timeout=5.0)


def _process_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    return True


def test_a_real_worker_pair_runs_the_direct_cycle(hosted_system: ModuleCoordinator) -> None:
    coordinator = hosted_system.get_instance(HostedRobots)
    arm = hosted_system.get_instance(FakeArm)

    # Both modules run in the same worker: no RPC hop between the cycle and
    # the hardware it reads and writes.
    manager: Any = hosted_system._managers["python"]
    assert [worker.module_count for worker in manager.workers if worker.module_count] == [2]

    wait_until(lambda: set(MOCK_JOINTS) <= set(coordinator.get_joint_positions()), timeout=10.0)
    now = coordinator.get_joint_positions()
    trajectory = JointTrajectory(
        joint_names=MOCK_JOINTS,
        points=[
            TrajectoryPoint(
                positions=[now[joint] for joint in MOCK_JOINTS],
                velocities=[0.0] * len(MOCK_JOINTS),
            ),
            TrajectoryPoint(
                positions=TARGET,
                velocities=[0.0] * len(MOCK_JOINTS),
                time_from_start=0.3,
            ),
        ],
    )
    result = coordinator.execute_trajectory(trajectory)
    assert result.status is TrajectoryExecutionStatus.ACCEPTED, result

    def reached() -> bool:
        seen = coordinator.get_joint_positions()
        return all(
            joint in seen and seen[joint] == pytest.approx(target, abs=1e-6)
            for joint, target in zip(MOCK_JOINTS, TARGET, strict=True)
        )

    wait_until(reached, timeout=10.0)
    # The hosted connection was written directly: its deadman was fed.
    status = arm.status()
    assert status.last_command_time is not None
    assert status.last_rejection is None
    assert status.last_error is None
