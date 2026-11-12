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
import math
import os
from pathlib import Path
import threading
import time
from typing import Any

import pytest
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control._control_test_helpers import (
    FakeArm,
    FakeArmConfig,
    fake_arm_model,
)
from dimos.control.components import HardwareComponent, HardwareType, make_joints
from dimos.control.connection.connection_module import ConnectionConfig, ConnectionModule
from dimos.control.contract.convert import (
    joint_state_to_values,
    motor_joints,
)
from dimos.control.contract.keys import Interface
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.task import (
    BaseControlTask,
    ControlMode,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.control.tasks.operator_hold_task.operator_hold_task import (
    OPERATOR_HOLD_TASK_NAME,
    OperatorHoldTask,
    operator_hold_task,
)
from dimos.control.tasks.trajectory_task.trajectory_task import (
    TrajectoryExecutionStatus,
    joint_trajectory_task,
)
from dimos.control.tasks.velocity_task.velocity_task import (
    JointVelocityTask,
    JointVelocityTaskConfig,
)
import dimos.control.tick_loop as tick_mod
from dimos.control.tick_loop import TickLoop
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.robot.assets.model import JointDescription, RobotModel
from dimos.utils.testing.waiting import wait_until

#: The fake arm's joints as the coordinator names them: "mock/joint1" and so on.
MOCK_JOINTS = motor_joints(fake_arm_model(), "mock")
AT_REST = [0.0] * len(MOCK_JOINTS)


BASE_JOINTS = ("base_forward", "base_lateral", "base_turn")
# Control-only limit declarations: not an odometry/kinematics model.
BASE_MODEL = fake_arm_model().with_added_joints(
    *(
        JointDescription(
            name=name,
            type=kind,
            parent_link="link0",
            child_link=f"{name}_resource",
            velocity=limit,
            acceleration=acceleration,
        )
        for name, kind, limit, acceleration in zip(
            BASE_JOINTS,
            ("prismatic", "prismatic", "continuous"),
            (1.0, 0.5, 2.0),
            (1.0, 1.0, 2.0),
            strict=True,
        )
    )
)


class FakeBaseConfig(ConnectionConfig):
    """Fake robot's velocity-only body action resources."""

    source: str = "go2"
    model: RobotModel | None = BASE_MODEL
    joint_commands: dict[str, tuple[Interface, ...]] = {
        name: (Interface.VELOCITY,) for name in BASE_JOINTS
    }
    state_rate_hz: float = 100.0
    initial_yaw: float = 0.0


class FakeBase(ConnectionModule):
    """Native body-rate feedback with independent simulated pose telemetry."""

    config: FakeBaseConfig
    velocity_command: In[JointState]
    joint_state: Out[JointState]
    odometry: Out[Odometry]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        source = self.config.source
        self._base = (f"{source}/base_forward", f"{source}/base_lateral", f"{source}/base_turn")
        self._stop_reading = threading.Event()
        self._reader: threading.Thread | None = None
        self.hardware_messages: list[Twist] = []

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(
            Disposable(
                self.velocity_command.subscribe(
                    lambda msg: self.command(joint_state_to_values(msg, (Interface.VELOCITY,)))
                )
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
        self.hardware_messages.append(
            Twist(
                linear=[
                    values[f"{self._base[0]}/velocity"],
                    values[f"{self._base[1]}/velocity"],
                    0.0,
                ],
                angular=[0.0, 0.0, values[f"{self._base[2]}/velocity"]],
            )
        )

    def halt(self) -> None:
        self.hardware_messages.append(Twist())

    def on_vendor_feedback(self, body_velocity: Twist, pose: PoseStamped | None = None) -> None:
        self.joint_state.publish(
            JointState(
                ts=time.time(),
                frame_id=self.frame("base_link"),
                name=list(self._base),
                velocity=[body_velocity.linear.x, body_velocity.linear.y, body_velocity.angular.z],
            )
        )
        if pose is not None:
            self.odometry.publish(
                Odometry(
                    ts=pose.ts,
                    frame_id=pose.frame_id,
                    child_frame_id=self.frame("base_link"),
                    pose=Pose(position=pose.position, orientation=pose.orientation),
                    twist=body_velocity,
                )
            )

    def _read_loop(self) -> None:
        period = 1.0 / self.config.state_rate_hz
        while not self._stop_reading.wait(period):
            self.on_vendor_feedback(
                Twist(),
                PoseStamped(
                    ts=time.time(),
                    frame_id=self.frame("odom"),
                    orientation=Quaternion.from_euler(Vector3(0.0, 0.0, self.config.initial_yaw)),
                ),
            )


def test_native_body_feedback_and_commands_are_independent_of_odometry(
    mocker: Any,
    request: pytest.FixtureRequest,
) -> None:
    base = FakeBase()
    request.addfinalizer(base.stop)
    published = mocker.patch.object(base.joint_state, "publish")
    odometry = mocker.patch.object(base.odometry, "publish")
    rates = (0.4, -0.2, 0.3)
    values = dict(zip((f"{name}/velocity" for name in GO2_BASE_AXES), rates, strict=True))
    base.write(values)  # No yaw or odometry receipt required.
    body = base.hardware_messages[-1]
    assert (body.linear.x, body.linear.y, body.angular.z) == rates
    base.on_vendor_feedback(body)
    feedback = published.call_args.args[0]
    assert feedback.name == GO2_BASE_AXES
    assert feedback.position == []
    assert feedback.velocity == list(rates)
    odometry.assert_not_called()
    pose = PoseStamped(
        position=Vector3(1.0, -2.0, 0.0),
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, math.pi / 2)),
    )
    base.on_vendor_feedback(body, pose)
    assert published.call_args.args[0].velocity == list(rates)
    telemetry = odometry.call_args.args[0]
    assert telemetry.position == pose.position
    assert telemetry.orientation == pose.orientation
    assert telemetry.linear_velocity == body.linear


class ArmCoordinator(ControlCoordinator):
    """Ports for FakeArm, called "mock"."""

    mock_position_command: Out[JointState]
    mock_joint_state: In[JointState]


class Robots(ArmCoordinator):
    """Ports for FakeArm and FakeBase, called "go2"."""

    go2_velocity_command: Out[JointState]
    go2_joint_state: In[JointState]


class TiltingRobots(ArmCoordinator):
    """Ports for a FakeArm with an IMU."""

    mock_imu: In[Imu]


class WholeRobot(ControlCoordinator):
    """Ports for one robot, "mock", with joints and a base."""

    mock_position_command: Out[JointState]
    mock_joint_state: In[JointState]
    mock_velocity_command: Out[JointState]


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


#: The fake base's axes, as a task claims them.
GO2_BASE_AXES = ["go2/base_forward", "go2/base_lateral", "go2/base_turn"]


def velocity_task(joints: list[str], timeout: float = 0.2) -> JointVelocityTask:
    task = JointVelocityTask(
        "velocity", JointVelocityTaskConfig(joints, priority=10000, timeout=timeout)
    )
    task.start()
    return task


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

    def report_base(self, robot: str = "go2") -> None:
        """The fake base reporting native body rates while stationary."""
        names = (f"{robot}/base_forward", f"{robot}/base_lateral", f"{robot}/base_turn")
        self.report(f"{robot}_joint_state", JointState(name=list(names), velocity=[0.0, 0.0, 0.0]))

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

    def velocities_sent(self) -> list[JointState]:
        return [msg for name, msg in self.sent if name == "go2_velocity_command"]


@pytest.fixture
def rig(mocker: Any) -> Iterator[Callable[..., Rig]]:
    mocker.patch.object(TickLoop, "start")
    made: list[ControlCoordinator] = []
    rigs: list[Rig] = []

    def build(cls: type[ControlCoordinator] = Robots, **config: Any) -> Rig:
        declarations: list[ConnectionConfig]
        if issubclass(cls, WholeRobot):
            arm_config = ConnectionConfig(
                source="mock",
                model=BASE_MODEL,
                joint_commands={
                    **FakeArmConfig().joint_commands,
                    **FakeBaseConfig().joint_commands,
                    "joint2": (Interface.POSITION, Interface.VELOCITY),
                },
            )
            declarations = [arm_config]
        else:
            declarations = [FakeArmConfig()]
            if not issubclass(cls, (ArmCoordinator, TiltingRobots)) or issubclass(cls, Robots):
                declarations.append(FakeBaseConfig())
        made.append(cls(**{"connections": declarations, **config}))
        rigs.append(Rig(made[-1]))
        return rigs[-1]

    yield build
    for r in rigs:
        for arm in r.arms:
            arm.stop()
    for each in made:
        each.stop()


def test_aggregate_state_waits_for_complete_initial_readings(rig: Callable[..., Rig]) -> None:
    r = rig(publish_joint_state=True)
    published: list[tuple[str, Any]] = []
    r.coordinator.set_transport("coordinator_joint_state", Recorder("state", published))

    r.tick()
    assert published == []
    r.report_arm(0.1, 0.2)
    r.tick()
    assert published == []
    r.report(
        "mock_joint_state",
        JointState(name=MOCK_JOINTS[2:], position=[0.3] * (len(MOCK_JOINTS) - 2)),
    )
    r.tick()
    assert published == []
    r.report_base()
    r.tick()

    assert len(published) == 1
    # The existing aggregate is position-oriented: body axes have no q to publish.
    assert published[0][1].name == MOCK_JOINTS
    assert published[0][1].position == [0.1, 0.2, *([0.3] * (len(MOCK_JOINTS) - 2))]
    assert r.positions_sent()[-1] == published[0][1].position[: len(MOCK_JOINTS)]


def test_a_reading_arriving_after_the_snapshot_waits_for_the_next_tick(
    rig: Callable[..., Rig], mocker: Any
) -> None:
    r = rig(ArmCoordinator, publish_joint_state=True)
    published: list[tuple[str, Any]] = []
    r.coordinator.set_transport("coordinator_joint_state", Recorder("state", published))
    loop = r.coordinator._tick_loop
    assert loop is not None
    read_all = loop._read_all_hardware

    def report_after_snapshot():
        snapshot = read_all()
        r.report_arm(*([0.2] * len(MOCK_JOINTS)))
        return snapshot

    mocker.patch.object(loop, "_read_all_hardware", side_effect=report_after_snapshot)
    r.tick()
    assert published == []
    r.tick()
    assert len(published) == 1
    assert published[0][1].name == MOCK_JOINTS
    assert published[0][1].position == [0.2] * len(MOCK_JOINTS)


def test_every_robot_that_has_reported_gets_a_complete_command_each_tick(
    rig: Callable[..., Rig],
) -> None:
    r = rig()
    assert r.coordinator.list_hardware() == ["mock", "go2"]
    assert r.coordinator.list_joints() == [*MOCK_JOINTS, *GO2_BASE_AXES]

    r.tick()
    assert r.sent == []

    r.report_arm(*AT_REST)
    r.tick()
    r.report_base()
    r.tick()

    assert [name for name, _ in r.sent] == [
        "mock_position_command",
        "mock_position_command",
        "go2_velocity_command",
    ]
    assert r.positions_sent() == [AT_REST, AT_REST]
    twist = r.sent[-1][1]
    assert tuple(twist.velocity) == (0.0, 0.0, 0.0)
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


def test_the_gripper_task_opens_and_closes_a_connection_robots_gripper_within_its_model(
    rig: Callable[..., Rig], tmp_path: Path
) -> None:
    # One arm joint, and a gripper open at 0 rad and closed at 0.85 rad, as the
    # xArm Gripper's drive_joint is.
    urdf = tmp_path / "arm_with_gripper.urdf"
    urdf.write_text(
        '<robot name="arm"><link name="base"/><link name="link1"/><link name="knuckle"/>'
        '<joint name="mock/joint1" type="revolute"><parent link="base"/><child link="link1"/>'
        '<limit lower="-3.1" upper="3.1" velocity="3" effort="50"/></joint>'
        '<joint name="mock/drive_joint" type="revolute"><parent link="link1"/>'
        '<child link="knuckle"/><limit lower="0" upper="0.85" velocity="2" effort="50"/></joint>'
        "</robot>"
    )
    gripper = TaskConfig(
        name="gripper",
        type="gripper",
        joint_names=["mock/drive_joint"],
        params={"open_at": "lower"},
    )
    declaration = ConnectionConfig(
        source="mock",
        model=RobotModel.from_file(urdf),
        joint_commands={
            "mock/joint1": (Interface.POSITION,),
            "mock/drive_joint": (Interface.POSITION,),
        },
    )
    r = rig(ArmCoordinator, connections=[declaration], tasks=[gripper])

    r.report(
        "mock_joint_state",
        JointState(name=["mock/joint1", "mock/drive_joint"], position=[0.0, 0.2]),
    )
    r.tick()
    assert r.coordinator.task_invoke("gripper", "get_normalized") == pytest.approx([1 - 0.2 / 0.85])

    r.coordinator.task_invoke("gripper", "set_normalized", {"values": [1.0]})
    r.tick()
    r.coordinator.task_invoke("gripper", "set_normalized", {"values": [0.0]})
    r.tick()
    # Held where it reads until told, then open, then closed.
    assert r.positions_sent() == [[0.0, 0.2], [0.0, 0.0], [0.0, 0.85]]


@pytest.mark.parametrize("target", ["base", "joint", "unsupported"])
def test_connection_routes_each_winning_interface(rig: Callable[..., Rig], target: str) -> None:
    r = rig(ArmCoordinator if target == "unsupported" else WholeRobot)
    r.coordinator.add_task(SendsPositions({"mock/joint1": 0.5}))
    names = (
        ["mock/base_forward", "mock/base_lateral", "mock/base_turn"]
        if target == "base"
        else ["mock/joint2"]
    )
    task = velocity_task(names)
    r.coordinator.add_task(task, task_type="velocity")
    r.report_arm(*AT_REST)
    if target != "unsupported":
        r.report_base("mock")
    task.set_velocities([0.4, -0.2, 0.3] if target == "base" else [0.4], t_now=time.perf_counter())
    r.tick()
    if target == "unsupported":
        assert r.sent == []
        task.stop()
        r.coordinator.remove_task("sends_positions")
        r.tick()
        assert r.positions_sent()[-1] == AT_REST  # No partial target update.
        return
    commands = dict(r.sent)
    assert r.positions_sent()[-1] == (
        [0.5, *AT_REST[1:]] if target == "base" else [0.5, *AT_REST[2:]]
    )
    if target == "base":
        twist = commands["mock_velocity_command"]
        assert tuple(twist.velocity) == (0.4, -0.2, 0.3)
        assert list(twist.name) == names
        task.stop()
        r.tick()
        twist = dict(r.sent)["mock_velocity_command"]
        assert tuple(twist.velocity) == (0.0, 0.0, 0.0)
    else:
        assert list(commands["mock_velocity_command"].name) == [
            *names,
            "mock/base_forward",
            "mock/base_lateral",
            "mock/base_turn",
        ]
        assert list(commands["mock_velocity_command"].velocity) == [0.4, 0.0, 0.0, 0.0]


@pytest.mark.parametrize("scoped", [False, True])
def test_mixed_hold_scope_preserves_velocity_joint_feedback(
    rig: Callable[..., Rig],
    scoped: bool,
) -> None:
    r = rig(WholeRobot, tasks=[operator_hold_task(joint_names=MOCK_JOINTS if scoped else None)])
    hold = r.coordinator.get_task(OPERATOR_HOLD_TASK_NAME)
    assert isinstance(hold, OperatorHoldTask)
    axes = ["mock/base_forward", "mock/base_lateral", "mock/base_turn"]
    velocity = JointVelocityTask("velocity", JointVelocityTaskConfig(axes, priority=50))
    r.coordinator.add_task(velocity)
    r.report_arm(*AT_REST)
    r.report_base("mock")
    moving = [0.4, -0.2, 0.3]
    assert velocity.set_velocities(moving, time.perf_counter())
    r.tick()
    assert list(dict(r.sent)["mock_velocity_command"].velocity) == moving
    hold.request("manual")
    r.tick()
    expected = MOCK_JOINTS if scoped else [*MOCK_JOINTS, *axes]
    assert hold.claim().joints == frozenset(expected)
    assert "mock/base_forward" not in r.coordinator.get_joint_positions()
    assert r.positions_sent()[-1] == AT_REST
    assert list(dict(r.sent)["mock_velocity_command"].velocity) == (
        moving if scoped else [0.0] * len(axes)
    )


def test_velocity_estop_invalidates_commands_until_fresh_post_clear_input(
    rig: Callable[..., Rig],
) -> None:
    r = rig()
    task = velocity_task(GO2_BASE_AXES)
    r.coordinator.add_task(task)
    r.report_base()
    assert task.set_velocities([0.4, 0.0, 0.0], time.perf_counter())
    r.tick()
    assert r.velocities_sent()[-1].velocity[0] == 0.4
    r.coordinator.set_estop(True)
    assert not task.set_velocities([0.8, 0.0, 0.0], time.perf_counter())
    before = len(r.sent)
    r.tick()
    assert len(r.sent) == before
    r.coordinator.set_estop(False)
    r.tick()
    assert len(r.sent) == before
    r.report_base()
    r.tick()
    assert r.velocities_sent()[-1].velocity[0] == 0.0
    assert task.set_velocities([0.2, 0.0, 0.0], time.perf_counter())
    r.tick()
    assert r.velocities_sent()[-1].velocity[0] == 0.2


def test_expired_velocity_sends_zero_with_fresh_feedback(
    rig: Callable[..., Rig], mocker: Any
) -> None:
    # Separate command and feedback clocks: a scheduling pause must not hide the zero command.
    clock = [1000.0]
    mocker.patch.object(time, "perf_counter", side_effect=lambda: clock[0])
    mocker.patch.object(time, "monotonic", return_value=2000.0)
    r = rig()
    task = velocity_task(GO2_BASE_AXES, timeout=0.05)
    r.coordinator.add_task(task)
    r.report_base()
    task.set_velocities([0.4, -0.2, 0.3], t_now=clock[0])
    r.tick()
    assert r.velocities_sent()[-1].velocity[0] == 0.4
    clock[0] += 0.1
    before = len(r.velocities_sent())
    time.sleep(0.25)
    r.tick()
    assert len(r.velocities_sent()) == before + 1
    twist = r.velocities_sent()[-1]
    assert tuple(twist.velocity) == (0.0, 0.0, 0.0)


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
        go2_velocity_command: Out[Twist]
        go2_joint_state: In[JointState]

    with pytest.raises(
        ValueError, match="'go2_velocity_command' is robot 'go2'.*must carry JointState"
    ):
        rig(WrongBase)


def test_a_joint_port_without_a_declaration_is_refused_at_start(rig: Callable[..., Rig]) -> None:
    with pytest.raises(ValueError, match="connection declaration"):
        rig(ArmCoordinator, connections=[])


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
    assert r.sent[-1][0] == "go2_velocity_command"
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
def system(request: pytest.FixtureRequest) -> Iterator[ModuleCoordinator]:
    """FakeArm and FakeBase behind a coordinator with a trajectory task, built with no viewer.

    A coordinator that goes ``reading_timeout_s`` without a reading stops
    commanding the robot, so a test machine that freezes that long can leave
    the arm short of TARGET. The default here outlasts such a freeze; a test
    that needs a silence noticed passes a shorter one, in seconds, with
    ``indirect=True``.
    """
    arm_declaration = ConnectionConfig(
        **FakeArmConfig().model_dump(include=set(ConnectionConfig.model_fields))
    )
    base_declaration = ConnectionConfig(
        **FakeBaseConfig().model_dump(include=set(ConnectionConfig.model_fields))
    )
    blueprint = autoconnect(
        Robots.blueprint(
            connections=[arm_declaration, base_declaration],
            tasks=[joint_trajectory_task(MOCK_JOINTS)],
            reading_timeout_s=getattr(request, "param", 10.0),
        ),
        FakeArm.blueprint(**arm_declaration.model_dump()),
        FakeBase.blueprint(**base_declaration.model_dump(), initial_yaw=math.pi / 2),
    ).remappings(
        [
            (FakeArm, "position_command", "mock_position_command"),
            (Robots, "mock_joint_state", "joint_state"),
            (FakeBase, "velocity_command", "go2_velocity_command"),
            (FakeBase, "joint_state", "go2_joint_state"),
        ]
    )
    parsed = BlueprintConfigParser(blueprint).parse(
        environ={},
        overrides={"g": {"viewer": "none"}},
        global_overrides={"zenoh_scout_addr": os.environ["ZENOH_SCOUT_ADDR"]},
    )
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
    wait_until(
        lambda: (
            "joint_trajectory" in coordinator.list_tasks()
            and set(MOCK_JOINTS) <= set(coordinator.get_joint_positions())
        ),
        timeout=10.0,
    )
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
    seen = system.get_instance(Robots).get_joint_positions()
    assert not set(GO2_BASE_AXES) & seen.keys()
    base = system.get_instance(FakeBase).status()
    assert base.last_command_time is not None
    assert base.last_rejection is None
    assert base.last_error is None


# Only a silence longer than the reading timeout makes the coordinator forget
# where it held the arm, so the restart has to outlast it.
@pytest.mark.parametrize("system", [0.2], indirect=True)
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
