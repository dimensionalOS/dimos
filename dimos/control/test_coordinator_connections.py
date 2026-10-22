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

"""Tests for the coordinator driving robots through connection modules.

The first group runs in-process: readings are fed straight into the
coordinator's per-robot ports, and the tick loop is stepped by hand, one
tick at a time.

The last two run end to end, with ``FakeArm`` and ``FakeBase`` in real
worker processes.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
import logging
import time
from typing import Any

import pytest

from dimos.control._control_test_helpers import FAKE_ARM_JOINTS, FakeArm, fake_arm_model
from dimos.control.components import HardwareComponent, HardwareType, make_joints
from dimos.control.connection.connection_module import ConnectionConfig, ConnectionModule
from dimos.control.contract.convert import pose_from_values
from dimos.control.contract.ports import TwistPort
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
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint

MOCK_JOINTS = [f"mock/{joint}" for joint in FAKE_ARM_JOINTS]
#: The fake arm's model as the coordinator reads it: joints named after the robot.
MOCK_MODEL = fake_arm_model().with_renamed_joints({j: f"mock/{j}" for j in FAKE_ARM_JOINTS})
AT_REST = [0.0] * len(MOCK_JOINTS)


class FakeBaseConfig(ConnectionConfig):
    source: str = "go2"
    cmd_vel: TwistPort = TwistPort(range=Twist(Vector3(1.0, 0.5, 0.0), Vector3(0.0, 0.0, 2.0)))


class FakeBase(ConnectionModule):
    """A pretend base called "go2": speeds in on cmd_vel, where it is out on odom.

    It never moves, so it always reports itself at the origin.
    """

    config: FakeBaseConfig
    cmd_vel: In[Twist]
    odom: Out[PoseStamped]

    def connect(self) -> None:
        pass

    def read_state(self) -> dict[str, float]:
        return dict.fromkeys(self.description.state_keys(), 0.0)

    def write(self, values: dict[str, float]) -> None:
        pass

    def halt(self) -> None:
        pass


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


class Rig:
    """A started coordinator whose tick loop only moves when ``tick`` is called."""

    def __init__(self, coordinator: ControlCoordinator) -> None:
        self.coordinator = coordinator
        self.readings: dict[str, LocalTransport] = {}
        for name, port in coordinator.inputs.items():
            if name.startswith(("mock_", "go2_")):
                self.readings[name] = LocalTransport()
                port.transport = self.readings[name]  # type: ignore[assignment]
        self.sent: list[tuple[str, Any]] = []
        for name, out in coordinator.outputs.items():
            if name.startswith(("mock_", "go2_")):
                out.subscribe(self._recorder(name))
        coordinator.start()
        assert coordinator._tick_loop is not None
        self.tick = coordinator._tick_loop._tick

    def _recorder(self, port: str) -> Callable[[Any], None]:
        return lambda msg: self.sent.append((port, msg))

    def report(self, port: str, msg: Any) -> None:
        self.readings[port].publish(msg)

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

    def build(cls: type[ControlCoordinator] = Robots, **config: Any) -> Rig:
        made.append(cls(**{"model": MOCK_MODEL, **config}))
        return Rig(made[-1])

    yield build
    for each in made:
        each.stop()


@pytest.fixture
def tick_log(caplog: pytest.LogCaptureFixture) -> Iterator[pytest.LogCaptureFixture]:
    """``caplog`` hearing the tick loop's logger, which does not propagate to the root."""
    tick_logger = logging.getLogger(tick_mod.logger.name)
    tick_logger.addHandler(caplog.handler)
    try:
        yield caplog
    finally:
        tick_logger.removeHandler(caplog.handler)


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
    time.sleep(0.3)
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
    rig: Callable[..., Rig], tick_log: pytest.LogCaptureFixture
) -> None:
    r = rig(ArmCoordinator, reading_timeout_s=0.01)

    r.tick()
    time.sleep(0.12)
    r.tick()
    r.tick()

    errors = [record for record in tick_log.records if record.levelno == logging.ERROR]
    assert len(errors) == 1
    assert (
        "robot 'mock' has not reported on mock_joint_state; is the connection module's "
        "joint_state remapped onto it?"
    ) in tick_log.text
    assert r.sent == []


# End to end, in real worker processes.

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
    """FakeArm and FakeBase behind a coordinator with a trajectory task, built with no viewer."""
    blueprint = autoconnect(
        Robots.blueprint(
            instance_name="ControlCoordinator",
            model=MOCK_MODEL,
            tasks=[joint_trajectory_task(MOCK_JOINTS)],
        ),
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
    assert wait_until(lambda: set(MOCK_JOINTS) <= set(coordinator.get_joint_positions()))
    trajectory = JointTrajectory(
        joint_names=MOCK_JOINTS,
        points=[
            TrajectoryPoint(positions=positions(system)),
            TrajectoryPoint(positions=TARGET, time_from_start=0.3),
        ],
    )

    result = coordinator.execute_trajectory(trajectory)

    assert result.status is TrajectoryExecutionStatus.ACCEPTED, result
    assert wait_until(lambda: positions(system) == pytest.approx(TARGET, abs=1e-6))


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

    assert wait_until(lambda: arm.status().last_command_time is not None)
    time.sleep(0.3)
    assert positions(system) == AT_REST
