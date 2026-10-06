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

"""Tests for the coordinator finding connection modules and driving their robots.

Most run in-process: connection modules are stand-ins that only answer
``describe_control``, readings are fed straight into the coordinator's
per-robot ports, and the tick loop is stepped by hand, one tick at a time.

The last two run end to end, with ``MockConnectionModule`` in real worker
processes, wired as the shipped ``coordinator-mock-connection`` blueprint.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
import time
from typing import Any

import pytest

from dimos.control.components import HardwareComponent, HardwareType, make_joints
from dimos.control.connection.connection_module import ConnectionDescription
from dimos.control.connection.mock_connection import MockConnectionModule
from dimos.control.contract.convert import pose_from_values
from dimos.control.contract.description import ControlDescription, Limits
from dimos.control.contract.keys import EFFORT, POSITION, VX, VY, WZ, Key
from dimos.control.contract.presets import (
    imu_resource,
    manipulator_description,
    pd_joint_description,
    twist_base_description,
)
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
    return manipulator_description(
        "arm",
        [f"joint{i}" for i in range(1, joints + 1)],
        state=(POSITION,),
        command=(POSITION,),
        **kwargs,
    )


def base() -> ControlDescription:
    """A base called "go2" that reports where it is and how fast it goes."""
    limits: dict[str, Limits] = {
        Key.of("go2", "base", axis): Limits(-1.0, 1.0) for axis in (VX, VY, WZ)
    }
    return twist_base_description("go2", limits=limits)


def arm_reading(*positions: float) -> JointState:
    """The arm's joint positions, joint1 first."""
    return JointState(
        name=[f"arm/joint{i}" for i in range(1, len(positions) + 1)], position=list(positions)
    )


class StandIn:
    """Plays a module proxy: ``rpcs`` names what it serves."""

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

    def describe_control(self) -> ConnectionDescription:
        self.asked += 1
        return ConnectionDescription(self.session, tuple(self.descriptions))

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

    def _recorder(self, port: str) -> Callable[[Any], None]:
        return lambda msg: self.sent.append((port, msg))

    def find(self, *modules: StandIn) -> None:
        self.coordinator.on_system_modules(list(modules))  # type: ignore[arg-type]

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
    limp = manipulator_description("limp", ["joint1"], state=(POSITION,), command=(EFFORT,))

    with pytest.raises(ValueError, match="'limp'"):
        r.find(StandIn("Limp", [limp]))
    with pytest.raises(ValueError, match="'g1'.*stiffness"):
        r.find(StandIn("Body", [pd_joint_description("g1", ["hip"])]))


def test_a_robot_without_its_ports_is_refused(rig: Callable[..., Rig]) -> None:
    r = rig()
    other = manipulator_description("other", ["joint1"], state=(POSITION,), command=(POSITION,))

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
    r.find(StandIn("Mock", [arm(sensors=[imu_resource()])]))
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
