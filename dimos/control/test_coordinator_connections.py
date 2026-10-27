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

"""Tests for the coordinator driving robots through connection modules, and
emergency-stopping them.

The first group runs in-process: readings are fed straight into the
coordinator's per-robot ports, or come from a ``FakeArm`` in the same
process, and the tick loop is stepped by hand, one tick at a time.

The last three run end to end, with ``FakeArm`` and ``FakeBase`` in real
worker processes.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
import threading
import time
from typing import Any

from pydantic import Field
import pytest
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control._control_test_helpers import FAKE_ARM_JOINTS, FakeArm, fake_arm_model
from dimos.control.components import HardwareComponent, HardwareType, make_joints
from dimos.control.connection.connection_module import ConnectionConfig, ConnectionModule
from dimos.control.contract.convert import motor_joints, pose_from_values, twist_to_values
from dimos.control.contract.limits import BASE_NAME
from dimos.control.coordinator import ControlCoordinator
from dimos.control.task import (
    BaseControlTask,
    ControlMode,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.control.tasks.trajectory_task.trajectory_task import (
    TrajectoryExecutionStatus,
    joint_trajectory_task,
)
import dimos.control.tick_loop as tick_mod
from dimos.control.tick_loop import TickLoop
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.utils.testing.waiting import wait_until

#: The fake arm's joints as the coordinator names them: "mock/joint1" and so on.
MOCK_JOINTS = motor_joints(fake_arm_model(), "mock")
#: The fake arm's model as the coordinator reads it: joints named after the robot.
MOCK_MODEL = fake_arm_model().with_renamed_joints(
    dict(zip(FAKE_ARM_JOINTS, MOCK_JOINTS, strict=True))
)
AT_REST = [0.0] * len(MOCK_JOINTS)


def fake_base_range() -> Twist:
    """The fastest the fake base goes: 1 m/s forward, 0.5 m/s sideways, 2 rad/s turning."""
    return Twist(Vector3(1.0, 0.5, 0.0), Vector3(0.0, 0.0, 2.0))


class FakeBaseConfig(ConnectionConfig):
    """Settings for ``FakeBase``: named "go2", told speeds.

    Attributes:
        state_rate_hz: How many times a second it reports where it is.
    """

    source: str = "go2"
    base_range: Twist | None = Field(default_factory=fake_base_range)
    state_rate_hz: float = 100.0


class FakeBase(ConnectionModule):
    """A pretend base, told speeds on ``cmd_vel``, with no hardware behind it.

    It never moves, so it reports itself at the origin on ``odom``,
    ``state_rate_hz`` times a second from its own thread.
    """

    config: FakeBaseConfig

    cmd_vel: In[Twist]
    odom: Out[PoseStamped]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._base = f"{self.config.source}/{BASE_NAME}"
        self._stop_reading = threading.Event()
        self._reader: threading.Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(
            Disposable(
                self.cmd_vel.subscribe(lambda msg: self.command(twist_to_values(msg, self._base)))
            )
        )
        self._reader = threading.Thread(target=self._read_loop, daemon=True, name="FakeBase-read")
        self._reader.start()

    @rpc
    def stop(self) -> None:
        self._stop_reading.set()
        if self._reader is not None:
            self._reader.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        super().stop()

    def connect(self) -> None:
        pass

    def write(self, values: dict[str, float]) -> None:
        pass

    def halt(self) -> None:
        pass

    def _read_loop(self) -> None:
        period = 1.0 / self.config.state_rate_hz
        origin = {f"{self._base}/x": 0.0, f"{self._base}/y": 0.0, f"{self._base}/yaw": 0.0}
        while not self._stop_reading.wait(period):
            self.odom.publish(
                pose_from_values(origin, self._base, ts=time.time(), frame_id=self.frame("odom"))
            )


class ArmCoordinator(ControlCoordinator):
    """Ports for FakeArm, called "mock"."""

    mock_position_command: Out[JointState]
    mock_joint_state: In[JointState]


class Robots(ArmCoordinator):
    """Ports for FakeArm and FakeBase, called "go2"."""

    go2_cmd_vel: Out[Twist]
    go2_odom: In[PoseStamped]


class TiltingRobots(ArmCoordinator):
    """Ports for a FakeArm with an IMU."""

    mock_imu: In[Imu]


# In-process, with the tick loop stepped by hand.


class LocalTransport:
    """An in-process transport: publishing calls every subscriber at once."""

    def __init__(self) -> None:
        self.subscribers: list[Callable[[Any], Any]] = []

    def broadcast(self, selfstream: Any, value: Any) -> None:
        for cb in list(self.subscribers):
            cb(value)

    def publish(self, value: Any) -> None:
        self.broadcast(None, value)

    def subscribe(self, cb: Callable[[Any], Any], selfstream: Any = None) -> Callable[[], None]:
        self.subscribers.append(cb)
        return lambda: self.subscribers.remove(cb)

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass


class Recorder(LocalTransport):
    """A transport on one command port that notes each message sent as (port, message)."""

    def __init__(self, port: str, sent: list[tuple[str, Any]]) -> None:
        super().__init__()
        self._port = port
        self._sent = sent

    def broadcast(self, selfstream: Any, value: Any) -> None:
        self._sent.append((self._port, value))
        super().broadcast(selfstream, value)


class SendsPositions(BaseControlTask):
    """Always active; drives fixed positions, and keeps the last state it saw."""

    def __init__(self, positions: dict[str, float]) -> None:
        self._name = "sends_positions"
        self._positions = positions
        self.last_state: CoordinatorState | None = None

    def claim(self) -> ResourceClaim:
        return ResourceClaim(joints=frozenset(self._positions), priority=10)

    def is_active(self) -> bool:
        return True

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        self.last_state = state
        return JointCommandOutput(
            joint_names=list(self._positions),
            positions=list(self._positions.values()),
            mode=ControlMode.POSITION,
        )

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        pass


class StopsOnEstop(SendsPositions):
    """``SendsPositions`` that is inert while emergency-stopped. ``estops``
    lists what each ``set_estop`` call was told, in order."""

    def __init__(self, positions: dict[str, float]) -> None:
        super().__init__(positions)
        self._name = "stops_on_estop"
        self.estops: list[bool] = []

    def is_active(self) -> bool:
        return not self.estops or not self.estops[-1]

    def set_estop(self, estopped: bool) -> None:
        self.estops.append(estopped)


class Rig:
    """A started coordinator whose tick loop only moves when ``tick`` is called."""

    def __init__(self, coordinator: ControlCoordinator) -> None:
        self.coordinator = coordinator
        self.readings: dict[str, LocalTransport] = {}
        for name in coordinator.inputs:
            if name.startswith(("mock_", "go2_")):
                self.readings[name] = LocalTransport()
                coordinator.set_transport(name, self.readings[name])  # type: ignore[arg-type]
        self.sent: list[tuple[str, Any]] = []
        self.commands: dict[str, Recorder] = {}
        for name in coordinator.outputs:
            if name.startswith(("mock_", "go2_")):
                self.commands[name] = Recorder(name, self.sent)
                coordinator.set_transport(name, self.commands[name])  # type: ignore[arg-type]
        self.arms: list[FakeArm] = []
        coordinator.start()
        assert coordinator._tick_loop is not None
        self.tick = coordinator._tick_loop._tick

    def attach_arm(self) -> FakeArm:
        """Start a ``FakeArm`` in this process, on the coordinator's "mock" ports."""
        arm = FakeArm()
        arm.set_transport("position_command", self.commands["mock_position_command"])  # type: ignore[arg-type]
        arm.set_transport("joint_state", self.readings["mock_joint_state"])  # type: ignore[arg-type]
        arm.start()
        self.arms.append(arm)
        return arm

    def tick_until(self, condition: Callable[[], bool], timeout_s: float = 2.0) -> bool:
        """Tick every 10 ms, as the tick loop does, until ``condition`` holds."""
        deadline = time.monotonic() + timeout_s
        while not condition():
            if time.monotonic() > deadline:
                return False
            self.tick()
            time.sleep(0.01)
        return True

    def report(self, port: str, msg: Any) -> None:
        self.readings[port].publish(msg)

    def report_base(self) -> None:
        """The fake base, standing at the origin."""
        origin = {"go2/base/x": 0.0, "go2/base/y": 0.0, "go2/base/yaw": 0.0}
        self.report("go2_odom", pose_from_values(origin, "go2/base", frame_id="odom"))

    def report_arm(self, *positions: float) -> None:
        """The fake arm's joint positions, joint1 first, standing still."""
        still = [0.0] * len(positions)
        msg = JointState(
            name=MOCK_JOINTS[: len(positions)],
            position=list(positions),
            velocity=still,
            effort=still,
        )
        self.report("mock_joint_state", msg)

    def positions_sent(self) -> list[list[float]]:
        return [list(msg.position) for name, msg in self.sent if name == "mock_position_command"]


@pytest.fixture
def rig(mocker: Any) -> Iterator[Callable[..., Rig]]:
    mocker.patch.object(TickLoop, "start")
    made: list[ControlCoordinator] = []
    rigs: list[Rig] = []

    def build(cls: type[ControlCoordinator] = Robots, **config: Any) -> Rig:
        made.append(cls(**{"model": MOCK_MODEL, **config}))
        rigs.append(Rig(made[-1]))
        return rigs[-1]

    yield build
    for r in rigs:
        for arm in r.arms:
            arm.stop()
    for each in made:
        each.stop()


def test_every_robot_that_has_reported_gets_a_complete_command_each_tick(
    rig: Callable[..., Rig],
) -> None:
    r = rig()
    assert r.coordinator.list_hardware() == ["mock", "go2"]
    assert r.coordinator.list_joints() == MOCK_JOINTS

    r.tick()
    assert r.sent == []

    r.report_arm(*AT_REST)
    r.tick()
    r.report(
        "go2_odom",
        pose_from_values(
            {"go2/base/x": 0, "go2/base/y": 0, "go2/base/yaw": 0}, "go2/base", frame_id="odom"
        ),
    )
    r.tick()

    assert [name for name, _ in r.sent] == [
        "mock_position_command",
        "mock_position_command",
        "go2_cmd_vel",
    ]
    assert r.positions_sent() == [AT_REST, AT_REST]
    twist = r.sent[-1][1]
    assert (twist.linear.x, twist.linear.y, twist.angular.z) == (0.0, 0.0, 0.0)
    assert r.coordinator.get_joint_positions() == dict.fromkeys(MOCK_JOINTS, 0.0)


def test_tasks_drive_connection_joints_and_see_their_sensors(rig: Callable[..., Rig]) -> None:
    r = rig(TiltingRobots)
    task = SendsPositions({"mock/joint1": 0.5})
    r.coordinator.add_task(task)

    r.report_arm(*AT_REST)
    r.report("mock_imu", Imu(orientation=Quaternion(0.0, 0.0, 0.0, 1.0)))
    r.tick()

    assert r.positions_sent() == [[0.5, *AT_REST[1:]]]
    assert task.last_state is not None
    assert task.last_state.sensors.readings["mock/imu"]["qw"] == 1.0
    assert task.last_state.joints.get_position("mock/joint2") == 0.0


def test_a_robot_whose_readings_stop_is_held_where_it_is_when_they_come_back(
    rig: Callable[..., Rig],
) -> None:
    r = rig(ArmCoordinator, reading_timeout_s=0.2)
    r.report_arm(0.3, *AT_REST[1:])
    r.tick()
    assert len(r.sent) == 1

    # Its connection module went down, e.g. to restart, and the arm was moved
    # meanwhile.
    wait_until(lambda: not r.coordinator._connections["mock"].ready_for_control(), timeout=2.0)
    r.tick()
    assert len(r.sent) == 1
    assert r.coordinator.get_joint_positions()["mock/joint1"] == 0.3

    r.report_arm(0.6, *AT_REST[1:])
    r.tick()
    assert r.positions_sent()[-1] == [0.6, *AT_REST[1:]]


def test_a_coordinator_with_robot_ports_takes_no_adapters(rig: Callable[..., Rig]) -> None:
    adapter_arm = HardwareComponent(
        hardware_id="left_arm",
        hardware_type=HardwareType.MANIPULATOR,
        joints=make_joints("left_arm", 2),
        adapter_type="mock",
    )

    with pytest.raises(ValueError, match="one or the other"):
        rig(hardware=[adapter_arm])
    with pytest.raises(ValueError, match="connection modules"):
        rig().coordinator.add_hardware(None, adapter_arm)  # type: ignore[arg-type]


def test_connection_robots_stay_out_of_per_robot_joint_states(
    rig: Callable[..., Rig], mocker: Any
) -> None:
    error = mocker.patch.object(tick_mod.logger, "error")
    r = rig(ArmCoordinator, publish_robot_joint_states=True)

    r.report_arm(*AT_REST)
    r.tick()

    assert len(r.sent) == 1
    error.assert_not_called()


def test_a_port_carrying_the_wrong_message_is_refused_at_start(rig: Callable[..., Rig]) -> None:
    class WrongBase(ArmCoordinator):
        go2_cmd_vel: Out[JointState]
        go2_odom: In[PoseStamped]

    with pytest.raises(ValueError, match="'go2_cmd_vel' is robot 'go2'.*must carry Twist"):
        rig(WrongBase)


def test_a_joint_port_without_a_model_is_refused_at_start(rig: Callable[..., Rig]) -> None:
    class BaseOnly(ControlCoordinator):
        go2_cmd_vel: Out[Twist]
        go2_odom: In[PoseStamped]

    with pytest.raises(ValueError, match="'mock' has joint ports but no model"):
        rig(ArmCoordinator, model=None)
    assert rig(BaseOnly, model=None).coordinator.list_hardware() == ["go2"]


def test_a_robot_that_never_reports_is_logged_once_and_never_commanded(
    rig: Callable[..., Rig], mocker: Any
) -> None:
    error = mocker.patch.object(tick_mod.logger, "error")
    r = rig(ArmCoordinator, reading_timeout_s=0.01)

    assert r.tick_until(lambda: error.called)
    r.tick()

    error.assert_called_once_with(
        "robot 'mock' has not reported on mock_joint_state; is the connection module's "
        "joint_state remapped onto it?"
    )
    assert r.sent == []


def test_an_estop_sends_nothing_more_and_the_deadman_halts_the_robot(
    rig: Callable[..., Rig],
) -> None:
    r = rig()
    arm = r.attach_arm()
    assert r.tick_until(lambda: len(arm.writes) > 0)
    r.report_base()
    r.tick()
    assert r.sent[-1][0] == "go2_cmd_vel"
    halts, sent = arm.halts, len(r.sent)

    assert r.coordinator.set_estop(True)

    # Ticking on sends neither robot anything, so the arm's deadman halts it.
    r.report_base()
    assert r.tick_until(lambda: arm.halts > halts)
    assert len(r.sent) == sent


def test_a_task_added_during_an_estop_starts_stopped(rig: Callable[..., Rig]) -> None:
    r = rig(ArmCoordinator)
    r.coordinator.set_estop(True)

    task = StopsOnEstop({"mock/joint1": 0.5})
    r.coordinator.add_task(task)
    r.report_arm(*AT_REST)
    r.tick()

    assert task.estops == [True]
    assert r.sent == []
    r.coordinator.set_estop(False)
    r.report_arm(*AT_REST)
    r.tick()
    assert task.estops == [True, False]
    assert r.positions_sent() == [[0.5, *AT_REST[1:]]]


def test_clearing_an_estop_holds_each_joint_where_the_next_reading_has_it(
    rig: Callable[..., Rig],
) -> None:
    r = rig(ArmCoordinator)
    task = SendsPositions({"mock/joint1": 0.5})
    r.coordinator.add_task(task)
    r.report_arm(0.3, *AT_REST[1:])
    r.tick()
    assert r.positions_sent() == [[0.5, *AT_REST[1:]]]

    r.coordinator.set_estop(True)
    # SendsPositions ignores the estop; take it away so nothing drives the arm.
    r.coordinator.remove_task(task.name)
    # Halted short of where it was told to go.
    r.report_arm(0.4, *AT_REST[1:])
    r.tick()
    assert len(r.sent) == 1

    assert r.coordinator.set_estop(False)
    # The last reading may predate the halt: nothing is sent until the next.
    r.tick()
    assert len(r.sent) == 1
    r.report_arm(0.41, *AT_REST[1:])
    r.tick()

    assert r.positions_sent()[-1] == [0.41, *AT_REST[1:]]


def test_an_estop_waits_for_a_tick_already_sending_and_a_clear_waits_for_the_stop(
    rig: Callable[..., Rig],
) -> None:
    r = rig(ArmCoordinator)
    task = StopsOnEstop({"mock/joint1": 0.5})
    r.coordinator.add_task(task)
    r.report_arm(*AT_REST)
    sending, release = threading.Event(), threading.Event()

    def held_up(msg: Any) -> None:
        sending.set()
        release.wait(5.0)

    r.commands["mock_position_command"].subscribe(held_up)
    results: dict[str, Any] = {}

    def in_thread(name: str, call: Callable[[], Any]) -> threading.Thread:
        thread = threading.Thread(target=lambda: results.update({name: call()}))
        thread.start()
        return thread

    ticking = in_thread("tick", r.tick)
    assert sending.wait(2.0)
    stopping = in_thread("stop", lambda: r.coordinator.set_estop(True))
    wait_until(r.coordinator._estop_lock.locked, timeout=2.0)
    clearing = in_thread("clear", lambda: r.coordinator.set_estop(False))

    # The stop waits for the tick to finish sending, and the clear for the stop.
    assert results == {}
    assert task.estops == []

    release.set()
    for thread in (ticking, stopping, clearing):
        thread.join(2.0)
    assert results == {"tick": None, "stop": True, "clear": True}
    assert task.estops == [True, False]
    r.report_arm(*AT_REST)
    r.tick()
    assert len(r.positions_sent()) == 2


# End to end, in real worker processes.

TARGET = [0.2, -0.1, 0.3, 0.0, 0.1, -0.2, 0.25]


@pytest.fixture
def system() -> Iterator[ModuleCoordinator]:
    """FakeArm and FakeBase behind a coordinator with a trajectory task, built with no viewer."""
    blueprint = autoconnect(
        Robots.blueprint(model=MOCK_MODEL, tasks=[joint_trajectory_task(MOCK_JOINTS)]),
        FakeArm.blueprint(),
        FakeBase.blueprint(),
    ).remappings(
        [
            (FakeArm, "position_command", "mock_position_command"),
            (Robots, "mock_joint_state", "joint_state"),
            (FakeBase, "cmd_vel", "go2_cmd_vel"),
            (Robots, "go2_odom", "odom"),
        ]
    )
    parsed = BlueprintConfigParser(blueprint).parse(environ={}, overrides={"g": {"viewer": "none"}})
    built = ModuleCoordinator.build(blueprint, parsed)
    yield built
    built.stop()


def positions(system: ModuleCoordinator) -> list[float]:
    """The fake arm's joint positions, as the coordinator sees them."""
    seen = system.get_instance(Robots).get_joint_positions()
    return [seen[joint] for joint in MOCK_JOINTS]


def drive_to_target(system: ModuleCoordinator) -> None:
    """Send the fake arm a trajectory to TARGET, and wait until it gets there."""
    coordinator = system.get_instance(Robots)
    wait_until(lambda: set(MOCK_JOINTS) <= set(coordinator.get_joint_positions()), timeout=10.0)
    trajectory = JointTrajectory(
        joint_names=MOCK_JOINTS,
        points=[
            TrajectoryPoint(positions=positions(system)),
            TrajectoryPoint(positions=TARGET, time_from_start=0.3),
        ],
    )

    result = coordinator.execute_trajectory(trajectory)

    assert result.status is TrajectoryExecutionStatus.ACCEPTED, result
    wait_until(lambda: positions(system) == pytest.approx(TARGET, abs=1e-6), timeout=10.0)


def test_a_trajectory_reaches_the_fake_arm_and_the_base_is_told_to_stand_still(
    system: ModuleCoordinator,
) -> None:
    drive_to_target(system)

    arm = system.get_instance(FakeArm).status()
    assert arm.last_rejection is None
    assert arm.last_error is None
    base = system.get_instance(FakeBase).status()
    assert base.last_command_time is not None
    assert base.last_rejection is None
    assert base.last_error is None


def test_a_restarted_connection_module_is_held_where_it_now_is(system: ModuleCoordinator) -> None:
    drive_to_target(system)

    # A restarted fake arm comes back with every joint at 0, as if it were
    # moved while its connection module was down.
    system.restart_module(FakeArm, reload_source=False)
    arm = system.get_instance(FakeArm)

    wait_until(lambda: arm.status().last_command_time is not None, timeout=10.0)
    first = arm.status().last_command_time
    assert first is not None
    # A tenth of a second of commands: an old target would have moved the arm.
    wait_until(lambda: (arm.status().last_command_time or 0.0) > first + 0.1, timeout=10.0)
    assert positions(system) == AT_REST


def test_an_estop_leaves_the_fake_robots_to_their_deadmen_and_a_clear_drives_them_again(
    system: ModuleCoordinator,
) -> None:
    drive_to_target(system)
    coordinator = system.get_instance(Robots)
    arm = system.get_instance(FakeArm)
    base = system.get_instance(FakeBase)

    assert coordinator.set_estop(True)

    wait_until(lambda: arm.status().deadman_fired and base.status().deadman_fired, timeout=10.0)

    assert coordinator.set_estop(False)

    wait_until(
        lambda: not arm.status().deadman_fired and not base.status().deadman_fired, timeout=10.0
    )
    assert positions(system) == pytest.approx(TARGET, abs=1e-6)
