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

"""Tests for the coordinator finding connection modules, driving their robots,
and stopping them.

Most run in-process: connection modules are stand-ins that answer
``describe_control``, ``halt_robot`` and ``status``, readings are fed straight
into the coordinator's per-robot ports, and the tick loop is stepped by hand,
one tick at a time.

The last four run end to end, with ``MockConnectionModule`` in real worker
processes, wired as the shipped ``coordinator-mock-connection`` blueprint. Two
drive the mock; two stop it, once with an emergency stop and once by killing
the coordinator's process.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
import os
import signal
import threading
import time
from typing import Any

import pytest

from dimos.control._control_test_helpers import turning_joints
from dimos.control.components import HardwareComponent, HardwareType, make_joints
from dimos.control.connection.connection_module import ConnectionDescription, ConnectionStatus
from dimos.control.connection.mock_connection import MockConnectionModule
from dimos.control.contract.convert import pose_from_values
from dimos.control.contract.description import ControlDescription, Limits
from dimos.control.contract.keys import Interface, Key
from dimos.control.contract.presets import imu_resource, twist_base_description
import dimos.control.coordinator as coord_mod
from dimos.control.coordinator import ControlCoordinator
from dimos.control.task import (
    BaseControlTask,
    ControlMode,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.control.tasks.trajectory_task.trajectory_task import TrajectoryExecutionStatus
import dimos.control.tick_loop as tick_mod
from dimos.control.tick_loop import TickLoop
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import Blueprint
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.robot.manipulators.common.mock import coordinator_mock_connection

ZERO_TWIST = {"go2/base/vx": 0.0, "go2/base/vy": 0.0, "go2/base/wz": 0.0}


def arm(joints: int = 2, **kwargs: Any) -> ControlDescription:
    """An arm called "arm" that reports and takes joint positions."""
    return turning_joints(
        "arm",
        [f"joint{i}" for i in range(1, joints + 1)],
        state=(Interface.POSITION,),
        command=(Interface.POSITION,),
        **kwargs,
    )


def base() -> ControlDescription:
    """A base called "go2" that reports where it is and how fast it goes."""
    limits: dict[str, Limits] = {
        Key.of("go2", "base", axis): Limits(-1.0, 1.0)
        for axis in (Interface.VX, Interface.VY, Interface.WZ)
    }
    return twist_base_description("go2", limits=limits)


def arm_reading(*positions: float) -> JointState:
    """The arm's joint positions, joint1 first."""
    return JointState(
        name=[f"arm/joint{i}" for i in range(1, len(positions) + 1)], position=list(positions)
    )


STATUS = ConnectionStatus(
    connected=True,
    last_state_time=1.0,
    last_command_time=2.0,
    deadman_fired=False,
    last_rejection=None,
    last_error=None,
)


class StandIn:
    """Plays a module proxy: ``rpcs`` names what it serves.

    A connection stand-in also answers ``halt_robot`` and ``status`` when the
    coordinator calls them by name, unless ``answers`` is False. ``halt_robot``
    first waits for ``halt_gate`` when one is given, then raises ``error`` if
    set, or answers ``halted``.
    """

    def __init__(
        self,
        name: str,
        descriptions: list[ControlDescription] | None = None,
        session: str = "s1",
    ) -> None:
        self.remote_name = name
        self.descriptions = descriptions or []
        self.session = session
        connection = descriptions is not None
        self.rpcs = {"start", "stop", *(["describe_control"] if connection else [])}
        self.asked = 0
        self.closed = False
        self.answers = True
        self.halted = True
        self.halts = 0
        self.halting = threading.Event()
        self.halt_gate: threading.Event | None = None
        self.error: Exception | None = None

    def describe_control(self) -> ConnectionDescription:
        self.asked += 1
        return ConnectionDescription(self.session, tuple(self.descriptions))

    def halt_robot(self) -> bool:
        self.halts += 1
        self.halting.set()
        if self.halt_gate is not None:
            self.halt_gate.wait(5.0)
        if self.error is not None:
            raise self.error
        return self.halted

    def status(self) -> ConnectionStatus:
        return STATUS

    def stop_rpc_client(self) -> None:
        self.closed = True


class LocalTransport:
    """An in-process transport: publishing calls every subscriber at once."""

    def __init__(self) -> None:
        self.subscribers: list[Callable[[Any], Any]] = []

    def publish(self, value: Any) -> None:
        for cb in list(self.subscribers):
            cb(value)

    def subscribe(self, cb: Callable[[Any], Any], selfstream: Any = None) -> Callable[[], None]:
        self.subscribers.append(cb)
        return lambda: self.subscribers.remove(cb)

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass


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


class Robots(ControlCoordinator):
    """Ports for an arm called "arm" and a base called "go2"."""

    arm_position_command: Out[JointState]
    arm_joint_state: In[JointState]
    arm_imu: In[Imu]
    go2_base_command: Out[Twist]
    go2_odom: In[PoseStamped]
    go2_base_velocity: In[Twist]


class Rig:
    """A started coordinator whose tick loop only moves when ``tick`` is called."""

    def __init__(self, coordinator: ControlCoordinator) -> None:
        self.coordinator = coordinator
        self.readings: dict[str, LocalTransport] = {}
        for name, port in coordinator.inputs.items():
            if name.startswith(("arm_", "go2_")):
                self.readings[name] = LocalTransport()
                port.transport = self.readings[name]  # type: ignore[assignment]
        self.sent: list[tuple[str, Any]] = []
        for name, out in coordinator.outputs.items():
            if name.endswith("_command"):
                out.subscribe(self._recorder(name))
        coordinator.start()
        assert coordinator._tick_loop is not None
        self.tick = coordinator._tick_loop._tick
        # Calls the coordinator makes by module name go to the stand-ins.
        self.modules: dict[str, StandIn] = {}
        coordinator.rpc.call = self._call  # type: ignore[method-assign, assignment]

    def _recorder(self, port: str) -> Callable[[Any], None]:
        return lambda msg: self.sent.append((port, msg))

    def find(self, *modules: StandIn) -> None:
        self.modules = {module.remote_name: module for module in modules}
        self.coordinator.on_system_modules(list(modules))  # type: ignore[arg-type]

    def _call(self, name: str, arguments: Any, answer: Callable[[Any], None]) -> Callable[[], None]:
        module_name, method = name.split("/")
        module = self.modules[module_name]
        if module.answers:
            try:
                answer(getattr(module, method)(*arguments[0], **arguments[1]))
            except Exception as error:
                answer(error)
        return lambda: None

    def report(self, port: str, msg: Any) -> None:
        self.readings[port].publish(msg)

    def report_base(self) -> None:
        self.report(
            "go2_odom",
            pose_from_values({"go2/base/x": 0, "go2/base/y": 0, "go2/base/yaw": 0}, "go2/base"),
        )
        self.report("go2_base_velocity", Twist())

    def positions_sent(self) -> list[list[float]]:
        return [list(msg.position) for name, msg in self.sent if name == "arm_position_command"]


@pytest.fixture
def rig(mocker: Any) -> Iterator[Callable[..., Rig]]:
    mocker.patch.object(TickLoop, "start")
    made: list[Rig] = []

    def build(cls: type[ControlCoordinator] = Robots, **config: Any) -> Rig:
        made.append(Rig(cls(**config)))
        return made[-1]

    yield build
    for each in made:
        each.coordinator.stop()


def test_only_modules_serving_describe_control_are_asked(rig: Callable[..., Rig]) -> None:
    r = rig()
    other = StandIn("Other")
    connection = StandIn("Mock", [arm()])

    r.find(other, connection)

    assert other.asked == 0
    assert connection.asked == 1
    assert r.coordinator.list_hardware() == ["arm"]
    assert r.coordinator.list_joints() == ["arm/joint1", "arm/joint2"]


def test_every_call_asks_again_and_replaces_what_was_found(rig: Callable[..., Rig]) -> None:
    r = rig()
    connection = StandIn("Mock", [arm(joints=2)])
    r.find(connection)

    connection.descriptions = [arm(joints=3)]
    r.find(connection)
    assert connection.asked == 2
    assert r.coordinator.list_joints() == ["arm/joint1", "arm/joint2", "arm/joint3"]

    r.find()
    assert r.coordinator.list_hardware() == []
    assert r.coordinator.list_joints() == []


def test_two_robots_with_one_name_are_refused(rig: Callable[..., Rig]) -> None:
    r = rig()

    with pytest.raises(ValueError, match="'arm'"):
        r.find(StandIn("First", [arm(joints=2)]), StandIn("Duplicate", [arm(joints=3)]))


def test_every_proxy_it_was_given_is_closed(rig: Callable[..., Rig]) -> None:
    r = rig()
    modules = [StandIn("Other"), StandIn("Mock", [arm()])]

    r.find(*modules)

    assert all(module.closed for module in modules)


def test_a_coordinator_with_adapters_refuses_connections(rig: Callable[..., Rig]) -> None:
    adapter_arm = HardwareComponent(
        hardware_id="left_arm",
        hardware_type=HardwareType.MANIPULATOR,
        joints=make_joints("left_arm", 2),
        adapter_type="mock",
    )
    r = rig(hardware=[adapter_arm])

    with pytest.raises(ValueError, match="adapters"):
        r.find(StandIn("Mock", [arm()]))


def test_a_robot_it_cannot_hold_is_refused(rig: Callable[..., Rig]) -> None:
    r = rig()
    limp = turning_joints(
        "limp", ["joint1"], state=(Interface.POSITION,), command=(Interface.EFFORT,)
    )

    with pytest.raises(ValueError, match="'limp'"):
        r.find(StandIn("Limp", [limp]))
    with pytest.raises(ValueError, match="'g1'.*stiffness"):
        r.find(
            StandIn(
                "Body",
                [
                    turning_joints(
                        "g1",
                        ["hip"],
                        state=(Interface.POSITION, Interface.VELOCITY, Interface.EFFORT),
                        command=(
                            Interface.POSITION,
                            Interface.VELOCITY,
                            Interface.EFFORT,
                            Interface.KP,
                            Interface.KD,
                        ),
                    )
                ],
            )
        )


def test_a_robot_without_its_ports_is_refused(rig: Callable[..., Rig]) -> None:
    r = rig()
    other = turning_joints(
        "other", ["joint1"], state=(Interface.POSITION,), command=(Interface.POSITION,)
    )

    with pytest.raises(ValueError) as raised:
        r.find(StandIn("Other", [other]), StandIn("Mock", [arm()]))

    assert "other_position_command: Out[JointState]" in str(raised.value)
    assert "other_joint_state: In[JointState]" in str(raised.value)
    assert r.coordinator.list_hardware() == []


def test_every_robot_that_has_reported_gets_a_complete_command_each_tick(
    rig: Callable[..., Rig],
) -> None:
    r = rig()
    r.find(StandIn("Mock", [arm()]), StandIn("Go2", [base()]))

    r.tick()
    assert r.sent == []

    r.report("arm_joint_state", arm_reading(0.3, -0.2))
    r.tick()
    r.report_base()
    r.tick()

    assert [name for name, _ in r.sent] == [
        "arm_position_command",
        "arm_position_command",
        "go2_base_command",
    ]
    assert r.positions_sent() == [[0.3, -0.2], [0.3, -0.2]]
    twist = r.sent[-1][1]
    assert (twist.linear.x, twist.linear.y, twist.angular.z) == (0.0, 0.0, 0.0)
    assert r.coordinator.get_joint_positions() == {"arm/joint1": 0.3, "arm/joint2": -0.2}


def test_tasks_drive_connection_joints_and_see_their_sensors(rig: Callable[..., Rig]) -> None:
    r = rig()
    r.find(StandIn("Mock", [arm(others=[imu_resource()])]))
    task = SendsPositions({"arm/joint1": 0.5})
    r.coordinator.add_task(task)

    r.report("arm_joint_state", arm_reading(0.0, -0.2))
    r.report("arm_imu", Imu(orientation=Quaternion(0.0, 0.0, 0.0, 1.0)))
    r.tick()

    assert r.positions_sent() == [[0.5, -0.2]]
    assert task.last_state is not None
    assert task.last_state.sensors.readings["arm/imu"]["qw"] == 1.0
    # The arm reports no velocity, so none is made up.
    assert task.last_state.joints.get_velocity("arm/joint1") is None


def test_found_again_a_robot_keeps_its_holds_and_waits_to_report(
    rig: Callable[..., Rig],
) -> None:
    r = rig()
    connection = StandIn("Mock", [arm()])
    r.find(connection)
    r.report("arm_joint_state", arm_reading(0.3, 0.0))
    r.tick()
    # A held arm sags a little below where it is held.
    r.report("arm_joint_state", arm_reading(0.29, 0.0))
    r.tick()

    # Something unrelated was loaded, so every connection module is found again.
    r.find(StandIn("Camera"), connection)
    r.tick()
    assert len(r.sent) == 2
    assert r.coordinator.get_joint_positions()["arm/joint1"] == 0.29

    # Still held where it was, not where it sagged to.
    r.report("arm_joint_state", arm_reading(0.28, 0.0))
    r.tick()
    assert r.positions_sent()[-1] == [0.3, 0.0]


def test_a_restarted_robot_is_held_where_it_now_is(rig: Callable[..., Rig]) -> None:
    r = rig()
    r.find(StandIn("Mock", [arm()], session="first"))
    r.report("arm_joint_state", arm_reading(0.3, 0.0))
    r.tick()

    # Its module restarted, and the arm was moved while it was down.
    r.find(StandIn("Mock", [arm()], session="second"))
    r.tick()
    assert len(r.sent) == 1

    r.report("arm_joint_state", arm_reading(0.7, 0.1))
    r.tick()
    assert r.positions_sent()[-1] == [0.7, 0.1]


def test_a_robot_described_differently_waits_for_a_reading_in_its_new_shape(
    rig: Callable[..., Rig],
) -> None:
    r = rig()
    connection = StandIn("Mock", [arm(joints=2)])
    r.find(connection)
    r.report("arm_joint_state", arm_reading(0.0, 0.0))
    r.tick()

    connection.descriptions = [arm(joints=3)]
    r.find(connection)
    r.tick()
    assert len(r.sent) == 1
    assert r.coordinator.get_joint_positions() == {}

    r.report("arm_joint_state", arm_reading(0.0, 0.0, 0.0))
    r.tick()
    assert len(r.positions_sent()[-1]) == 3


def test_a_robot_whose_readings_stop_is_not_commanded(rig: Callable[..., Rig]) -> None:
    r = rig()
    r.find(StandIn("Mock", [arm(deadman_timeout_s=0.2)]))
    r.report("arm_joint_state", arm_reading(0.3, 0.0))
    r.tick()
    assert len(r.sent) == 1

    time.sleep(0.3)
    r.tick()
    assert len(r.sent) == 1
    assert r.coordinator.get_joint_positions()["arm/joint1"] == 0.3

    # It comes back somewhere else, and is held there.
    r.report("arm_joint_state", arm_reading(0.6, 0.0))
    r.tick()
    assert r.positions_sent()[-1] == [0.6, 0.0]


def test_connection_robots_stay_out_of_per_robot_joint_states(
    rig: Callable[..., Rig], mocker: Any
) -> None:
    error = mocker.patch.object(tick_mod.logger, "error")
    r = rig(publish_robot_joint_states=True)
    r.find(StandIn("Mock", [arm()]))

    r.report("arm_joint_state", arm_reading(0.0, 0.0))
    r.tick()

    assert len(r.sent) == 1
    error.assert_not_called()


def test_estop_sends_nothing_more_and_halts_each_connection_module_once(
    rig: Callable[..., Rig],
) -> None:
    r = rig()
    # One module running two robots, and a module that is not a connection.
    both = StandIn("Both", [arm(), base()])
    r.find(both, StandIn("Camera"))
    r.report("arm_joint_state", arm_reading(0.0, 0.0))
    r.report_base()
    r.tick()
    assert len(r.sent) == 2

    assert r.coordinator.set_estop(True)

    assert both.halts == 1
    r.report("arm_joint_state", arm_reading(0.0, 0.0))
    r.report_base()
    r.tick()
    assert len(r.sent) == 2


def test_a_module_that_does_not_answer_is_named_and_does_not_hold_up_the_rest(
    rig: Callable[..., Rig], mocker: Any
) -> None:
    mocker.patch.object(coord_mod, "_CONNECTION_RPC_TIMEOUT_S", 0.05)
    error = mocker.patch.object(coord_mod.logger, "error")
    r = rig()
    silent = StandIn("Silent", [arm()])
    silent.answers = False
    working = StandIn("Working", [base()])
    r.find(silent, working)

    assert r.coordinator.set_estop(True) is False

    assert working.halts == 1
    logged = str(error.call_args_list)
    assert "'Silent' did not answer halt_robot" in logged
    assert "'Working'" not in logged


def test_a_module_whose_halt_fails_is_named(rig: Callable[..., Rig], mocker: Any) -> None:
    error = mocker.patch.object(coord_mod.logger, "error")
    r = rig()
    broken = StandIn("Broken", [arm()])
    broken.error = RuntimeError("no")
    refuses = StandIn("Refuses", [base()])
    refuses.halted = False
    r.find(broken, refuses)

    assert r.coordinator.set_estop(True) is False

    logged = str(error.call_args_list)
    assert "'Broken' failed halt_robot" in logged
    assert "'Refuses' could not halt its robot" in logged


def test_a_clear_sent_during_a_stop_waits_for_its_halts(rig: Callable[..., Rig]) -> None:
    r = rig()
    slow = StandIn("Slow", [arm()])
    slow.halt_gate = threading.Event()
    r.find(slow)
    r.report("arm_joint_state", arm_reading(0.0, 0.0))
    r.tick()
    assert len(r.sent) == 1
    results: dict[str, bool] = {}

    def call(name: str, estopped: bool) -> threading.Thread:
        thread = threading.Thread(
            target=lambda: results.update({name: r.coordinator.set_estop(estopped)})
        )
        thread.start()
        return thread

    stopping = call("stop", True)
    assert slow.halting.wait(2.0)
    clearing = call("clear", False)
    time.sleep(0.05)

    # The clear waits behind the halt, so the arm is still sent nothing.
    assert results == {}
    r.report("arm_joint_state", arm_reading(0.0, 0.0))
    r.tick()
    assert len(r.sent) == 1

    slow.halt_gate.set()
    stopping.join(2.0)
    clearing.join(2.0)
    assert results == {"stop": True, "clear": True}
    r.report("arm_joint_state", arm_reading(0.0, 0.0))
    r.tick()
    assert len(r.sent) == 2


def test_clearing_the_estop_holds_each_joint_where_the_robot_now_is(
    rig: Callable[..., Rig],
) -> None:
    r = rig()
    r.find(StandIn("Mock", [arm()]))
    task = SendsPositions({"arm/joint1": 0.5})
    r.coordinator.add_task(task)
    r.report("arm_joint_state", arm_reading(0.3, 0.0))
    r.tick()
    assert r.positions_sent() == [[0.5, 0.0]]

    r.coordinator.set_estop(True)
    # SendsPositions has no set_estop; take it away so nothing drives the arm.
    r.coordinator.remove_task(task.name)
    # Halted short of where it was told to go.
    r.report("arm_joint_state", arm_reading(0.4, 0.0))
    r.tick()
    assert len(r.sent) == 1

    assert r.coordinator.set_estop(False)
    # The last reading may predate the halt: nothing is sent until the next.
    r.tick()
    assert len(r.sent) == 1
    r.report("arm_joint_state", arm_reading(0.41, 0.0))
    r.tick()

    assert r.positions_sent() == [[0.5, 0.0], [0.41, 0.0]]


def test_connection_status_is_by_robot_and_none_for_a_silent_module(
    rig: Callable[..., Rig], mocker: Any
) -> None:
    mocker.patch.object(coord_mod, "_CONNECTION_RPC_TIMEOUT_S", 0.05)
    r = rig()
    silent = StandIn("Silent", [base()])
    silent.answers = False
    r.find(StandIn("Mock", [arm()]), silent, StandIn("Camera"))

    assert r.coordinator.get_connection_status() == {"arm": STATUS, "go2": None}


# End to end, in real worker processes.

JOINTS = make_joints("mock", 7)
TARGET = [0.2, -0.1, 0.3, 0.0, 0.1, -0.2, 0.25]


def wait_until(condition: Callable[[], bool], timeout_s: float = 10.0) -> bool:
    deadline = time.monotonic() + timeout_s
    while not condition():
        if time.monotonic() > deadline:
            return False
        time.sleep(0.02)
    return True


@pytest.fixture
def system() -> Iterator[ModuleCoordinator]:
    """The shipped coordinator-mock-connection blueprint, built with no viewer."""
    blueprint: Blueprint = coordinator_mock_connection
    parsed = BlueprintConfigParser(blueprint).parse(environ={}, overrides={"g": {"viewer": "none"}})
    built = ModuleCoordinator.build(blueprint, parsed)
    yield built
    built.stop()


@pytest.fixture
def slow_deadman_system() -> Iterator[ModuleCoordinator]:
    """The same blueprint, with the mock's deadman long enough that a busy CI
    machine never trips it while the coordinator runs."""
    blueprint: Blueprint = coordinator_mock_connection
    parsed = BlueprintConfigParser(blueprint).parse(
        environ={},
        overrides={"g": {"viewer": "none"}, MockConnectionModule.name: {"deadman_timeout_s": 0.5}},
    )
    built = ModuleCoordinator.build(blueprint, parsed)
    yield built
    built.stop()


def positions(system: ModuleCoordinator) -> list[float]:
    """The mock's joint positions, as the coordinator sees them."""
    seen = system.get_instance(ControlCoordinator).get_joint_positions()
    return [seen[joint] for joint in JOINTS]


def drive_to_target(system: ModuleCoordinator) -> None:
    """Send the mock a trajectory to TARGET, and wait until it gets there."""
    coordinator = system.get_instance(ControlCoordinator)
    assert wait_until(lambda: set(JOINTS) <= set(coordinator.get_joint_positions()))
    trajectory = JointTrajectory(
        joint_names=JOINTS,
        points=[
            TrajectoryPoint(positions=positions(system)),
            TrajectoryPoint(positions=TARGET, time_from_start=0.3),
        ],
    )

    result = coordinator.execute_trajectory(trajectory)

    assert result.status is TrajectoryExecutionStatus.ACCEPTED, result
    assert wait_until(lambda: positions(system) == pytest.approx(TARGET, abs=1e-6))


def test_a_trajectory_reaches_the_mock(system: ModuleCoordinator) -> None:
    drive_to_target(system)

    status = system.get_instance(MockConnectionModule).status()
    assert status.last_rejection is None
    assert status.last_error is None


def test_a_restarted_mock_is_held_where_it_now_is(system: ModuleCoordinator) -> None:
    drive_to_target(system)

    # A restarted mock comes back with every joint at 0, as if it were moved
    # while its driver was down.
    system.restart_module(MockConnectionModule, reload_source=False)
    mock = system.get_instance(MockConnectionModule)

    assert wait_until(lambda: mock.status().last_command_time is not None)
    time.sleep(0.3)
    assert positions(system) == [0.0] * len(JOINTS)


def test_estop_halts_the_mock_and_clearing_drives_it_again(
    slow_deadman_system: ModuleCoordinator,
) -> None:
    system = slow_deadman_system
    coordinator = system.get_instance(ControlCoordinator)
    mock = system.get_instance(MockConnectionModule)
    assert wait_until(lambda: mock.status().last_command_time is not None)

    assert coordinator.set_estop(True)
    stopped_at = time.time()
    status = coordinator.get_connection_status()["mock"]
    assert status is not None and status.connected
    # Sent nothing more, so its deadman fires too.
    assert wait_until(lambda: mock.status().deadman_fired, timeout_s=2.5)

    assert coordinator.set_estop(False)
    assert wait_until(lambda: (mock.status().last_command_time or 0.0) > stopped_at + 0.5)
    assert not mock.status().deadman_fired


def test_killing_the_coordinator_lets_the_mock_deadman_halt_it(
    slow_deadman_system: ModuleCoordinator,
) -> None:
    system = slow_deadman_system
    mock = system.get_instance(MockConnectionModule)
    assert wait_until(lambda: mock.status().last_command_time is not None)
    assert not mock.status().deadman_fired

    workers = system._managers["python"].workers  # type: ignore[attr-defined]
    [worker] = [w for w in workers if any("Coordinator" in n for n in w.module_names)]
    assert "MockConnectionModule" not in worker.module_names
    assert worker.pid is not None
    os.kill(worker.pid, signal.SIGKILL)

    # The deadman fires 0.5 s after the last command; the rest is slack for a
    # busy CI machine.
    assert wait_until(lambda: mock.status().deadman_fired, timeout_s=2.5)
