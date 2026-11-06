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

"""The R1 Pro's control wiring end to end, with no hardware.

A real ``R1ProCoordinator`` runs its real tasks — the whole-body joint
trajectory task and the chassis velocity task — and drives a real
``R1ProConnection`` through the contract ports (``r1pro_position_command``,
``r1pro_cmd_vel``, ``r1pro_joint_state``, ``r1pro_imu``) on in-process
transports. The vendor ROS stack is faked (``_connection_test_fakes``), so
what the vendor topics would carry is checked directly, and feedback is
injected there to verify the readback path into the coordinator's state.

The coordinator's tick loop is stepped by hand, one tick at a time; the
connection module's own threads (publish loop, deadman) run for real.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
import importlib
import threading
import time
from typing import Any

import pytest

from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import (
    TrajectoryExecutionStatus,
    joint_trajectory_task,
)
import dimos.control.tick_loop as tick_mod
from dimos.control.tick_loop import TickLoop
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.robot.galaxea.r1pro._connection_test_fakes import (
    MEASURED_Q,
    FakeRos,
    LocalTransport,
    RosHeader,
    RosImu,
    feed_all_segments,
    install_fake_ros,
)
from dimos.robot.galaxea.r1pro.blueprints.basic.r1pro_coordinator import (
    R1ProCoordinator,
    r1pro_control,
)
from dimos.robot.galaxea.r1pro.config import R1PRO_MODEL
from dimos.robot.galaxea.r1pro.connection import (
    R1PRO_BASE_JOINTS,
    R1PRO_UPPER_BODY_JOINTS,
    R1ProConnection,
)
from dimos.utils.testing.waiting import wait_until

#: The tasks the shipped blueprint reuses, unmodified.
DEFAULT_TASKS = [
    joint_trajectory_task(R1PRO_UPPER_BODY_JOINTS),
    TaskConfig(name="vel_chassis", type="velocity", joint_names=R1PRO_BASE_JOINTS, priority=10),
]

#: The gripper tasks the manipulation blueprint wires: one per jaw,
#: feedback-gated, refusing the unscoped broadcast.
GRIPPER_TASKS = [
    TaskConfig(
        name="r1pro_left_gripper",
        type="gripper",
        joint_names=["r1pro/left_gripper"],
        priority=10,
        params={"require_feedback": True, "accept_broadcast": False},
    ),
    TaskConfig(
        name="r1pro_right_gripper",
        type="gripper",
        joint_names=["r1pro/right_gripper"],
        priority=10,
        params={"require_feedback": True, "accept_broadcast": False},
    ),
]


class Rig:
    """A hand-stepped ``R1ProCoordinator`` wired to ``R1ProConnection`` ports."""

    def __init__(self, coordinator: R1ProCoordinator, fake: FakeRos) -> None:
        self.coordinator = coordinator
        self.fake = fake
        self.position_commands = LocalTransport()
        self.base_commands = LocalTransport()
        self.joint_states = LocalTransport()
        self.imus = LocalTransport()
        # The public twist bus, as teleop/nav publishers see it.
        self.twist_bus = LocalTransport()
        # The unscoped gripper broadcast bus.
        self.gripper_bus = LocalTransport()
        coordinator.set_transport("twist_command", self.twist_bus)  # type: ignore[arg-type]
        coordinator.set_transport("gripper_command", self.gripper_bus)  # type: ignore[arg-type]
        coordinator.set_transport("r1pro_position_command", self.position_commands)  # type: ignore[arg-type]
        coordinator.set_transport("r1pro_cmd_vel", self.base_commands)  # type: ignore[arg-type]
        coordinator.set_transport("r1pro_joint_state", self.joint_states)  # type: ignore[arg-type]
        coordinator.set_transport("r1pro_imu", self.imus)  # type: ignore[arg-type]
        coordinator.start()
        assert coordinator._tick_loop is not None
        self.tick = coordinator._tick_loop._tick
        self.connections: list[R1ProConnection] = []
        self._feed_stops: list[threading.Event] = []
        self._streamed: list[list[list[float] | None]] = []

    def attach(self, feedback: list[float] | None = None, **config: Any) -> R1ProConnection:
        """Start an ``R1ProConnection`` on the shared ports, on the fake vendor stack.

        The fake robot reports ``feedback`` (flat 18-joint layout) once the
        module has subscribed, so ``start``'s feedback discovery succeeds.
        """
        config.setdefault("boot_vendor_stack", False)
        config.setdefault("deadman_timeout_s", 5.0)
        config.setdefault("sensor_stats_interval_s", 0.0)
        config.setdefault("enable_wrist_color", False)
        conn = R1ProConnection(**config)
        conn.set_transport("position_command", self.position_commands)  # type: ignore[arg-type]
        conn.set_transport("cmd_vel", self.base_commands)  # type: ignore[arg-type]
        conn.set_transport("joint_state", self.joint_states)  # type: ignore[arg-type]
        conn.set_transport("imu_chassis", self.imus)  # type: ignore[arg-type]
        for name in set(conn.outputs) - {"joint_state", "imu_chassis"}:
            conn.set_transport(name, LocalTransport())  # type: ignore[arg-type]
        feed_stop = threading.Event()
        self._feed_stops.append(feed_stop)
        streamed = [feedback]
        self._streamed.append(streamed)
        threading.Thread(
            target=self._feed_while_alive, args=(feed_stop, streamed), daemon=True
        ).start()
        conn.start()
        self.connections.append(conn)
        return conn

    def detach(self, conn: R1ProConnection) -> None:
        """Take a connection down: halt it and stop its fake vendor topics."""
        conn.stop()
        self._feed_stops[self.connections.index(conn)].set()

    def _feed_while_alive(self, stop: threading.Event, streamed: list[list[float] | None]) -> None:
        """Stream feedback like the real ~100 Hz vendor topics while the
        connection is up; the topics die with it, as on the real robot.

        A single frame can go stale (reading timeout) before a loaded CI
        runner steps the next tick, so the fake keeps publishing. The latest
        value passed to ``feed`` is what keeps streaming.
        """

        def subscribed() -> bool:
            return bool(self.fake.nodes) and self.fake.control.listener_count() >= 4

        while not stop.is_set() and not subscribed():
            stop.wait(0.01)
        while not stop.is_set():
            feed_all_segments(self.fake, q=streamed[0])
            # The coordinator's source is ready only once the IMU reports too.
            self.fake.sensors.feed(
                "/hdas/imu_chassis", RosImu(header=RosHeader(frame_id="imu_chassis"))
            )
            self.fake.sensors.feed(
                "/hdas/imu_torso", RosImu(header=RosHeader(frame_id="imu_torso"))
            )
            stop.wait(0.01)

    def feed(self, ts: float | None = None, q: list[float] | None = None) -> None:
        """One feedback frame per segment on the newest connection's control
        node; the streaming thread keeps publishing this value afterwards."""
        for streamed in self._streamed:
            streamed[0] = q
        feed_all_segments(self.fake, ts, q)

    def tick_until(self, condition: Callable[[], bool], timeout_s: float = 15.0) -> None:
        """Tick every 10 ms, as the tick loop does, until ``condition`` holds."""
        deadline = time.monotonic() + timeout_s
        while not condition():
            if time.monotonic() > deadline:
                raise TimeoutError("condition never held")
            self.tick()
            time.sleep(0.01)

    def positions_sent(self) -> list[list[float]]:
        return [list(msg.position) for msg in self.position_commands.sent]

    def twists_sent(self) -> list[Twist]:
        return list(self.base_commands.sent)

    def joint_positions(self) -> list[float]:
        seen = self.coordinator.get_joint_positions()
        return [seen[joint] for joint in R1PRO_UPPER_BODY_JOINTS]


@pytest.fixture
def fake_ros(monkeypatch: pytest.MonkeyPatch) -> FakeRos:
    return install_fake_ros(monkeypatch)


@pytest.fixture
def rig(mocker: Any, fake_ros: FakeRos) -> Iterator[Callable[..., Rig]]:
    # The tick loop is stepped by hand, not threaded; clearing its stop event
    # keeps it "running", so the coordinator still syncs stream subscriptions.
    mocker.patch.object(TickLoop, "start", lambda self: self._stop_event.clear())
    made: list[Rig] = []

    def build(**config: Any) -> Rig:
        coordinator = R1ProCoordinator(
            instance_name="ControlCoordinator",
            tick_rate=100,
            **{"model": R1PRO_MODEL, "tasks": list(DEFAULT_TASKS), **config},
        )
        made.append(Rig(coordinator, fake_ros))
        return made[-1]

    yield build
    for r in made:
        for stop in r._feed_stops:
            stop.set()
        for conn in r.connections:
            conn.stop()
        r.coordinator.stop()


def test_the_r1pro_is_one_connection_robot_with_18_joints_3_base_axes_and_no_adapters(
    rig: Callable[..., Rig],
) -> None:
    r = rig()
    assert r.coordinator.list_hardware() == ["r1pro"]
    # The 18 upper-body joints in the flat vendor order, then the base axes.
    assert r.coordinator.list_joints() == [*R1PRO_UPPER_BODY_JOINTS, *R1PRO_BASE_JOINTS]
    # The shipped tasks are reused as-is: one trajectory task, one velocity task.
    assert set(r.coordinator._tasks) == {"joint_trajectory", "vel_chassis"}


def test_feedback_flows_from_the_vendor_topics_into_the_coordinator(
    rig: Callable[..., Rig],
) -> None:
    r = rig()
    r.attach()
    r.tick_until(lambda: bool(r.positions_sent()))

    # The coordinator's state is what the fake robot reported, by full name.
    assert r.joint_positions() == pytest.approx(MEASURED_Q)
    joints = r.coordinator._connections["r1pro"].read_joints()
    assert set(joints.joint_positions) == set(R1PRO_UPPER_BODY_JOINTS)


def test_an_idle_coordinator_holds_the_joints_where_they_are_and_zeroes_the_base(
    rig: Callable[..., Rig],
) -> None:
    r = rig()
    r.attach()
    r.tick_until(lambda: bool(r.position_commands.sent) and bool(r.base_commands.sent))

    # No task drives anything: the 18 joints are held at their measured
    # positions and the chassis is told 0 on every axis.
    assert r.positions_sent()[-1] == pytest.approx(MEASURED_Q)
    twist = r.twists_sent()[-1]
    assert (twist.linear.x, twist.linear.y, twist.angular.z) == (0.0, 0.0, 0.0)
    assert r.connections[0].status().last_rejection is None


def test_a_trajectory_drives_the_upper_body_through_the_whole_chain(
    rig: Callable[..., Rig],
) -> None:
    r = rig()
    conn = r.attach()
    r.tick_until(lambda: bool(r.positions_sent()))

    start = r.joint_positions()
    goal = [q + 0.02 for q in start]
    trajectory = JointTrajectory(
        joint_names=list(R1PRO_UPPER_BODY_JOINTS),
        points=[
            TrajectoryPoint(positions=start),
            TrajectoryPoint(positions=goal, time_from_start=0.3),
        ],
    )
    result = r.coordinator.execute_trajectory(trajectory)
    assert result.status is TrajectoryExecutionStatus.ACCEPTED, result

    # Task -> ConnectionSource -> r1pro_position_command -> connection -> vendor topics.
    def at_goal() -> bool:
        sent = r.fake.control.sent_to("/motion_target/target_joint_state_arm_left")
        return bool(sent) and list(sent[-1].position) == pytest.approx(goal[4:11], abs=1e-3)

    r.tick_until(at_goal)
    assert conn.status().last_rejection is None
    assert conn.status().last_error is None

    # The robot reporting the goal back closes the loop: the coordinator sees it.
    r.feed(q=goal)
    r.tick_until(lambda: r.joint_positions() == pytest.approx(goal, abs=1e-6))


def test_a_twist_drives_the_chassis_and_the_connection_clamps_it(
    rig: Callable[..., Rig],
) -> None:
    r = rig()
    r.attach()
    r.tick_until(lambda: bool(r.twists_sent()))

    # The public twist bus is subscribed because the connection has a base.
    assert "twist_command" in r.coordinator._stream_unsubs

    r.twist_bus.publish(Twist(linear=Vector3(0.5, -0.2, 0.0), angular=Vector3(0.0, 0.0, 0.7)))
    r.tick()
    twist = r.twists_sent()[-1]
    assert (twist.linear.x, twist.linear.y, twist.angular.z) == (0.5, -0.2, 0.7)
    sent = r.fake.control.sent_to("/motion_target/target_speed_chassis")[-1]
    assert (sent.twist.linear.x, sent.twist.linear.y, sent.twist.angular.z) == (0.5, -0.2, 0.7)

    # Past the base range, the connection clamps before the vendor topic.
    r.twist_bus.publish(Twist(linear=Vector3(5.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, -9.0)))
    r.tick()
    sent = r.fake.control.sent_to("/motion_target/target_speed_chassis")[-1]
    assert (sent.twist.linear.x, sent.twist.angular.z) == (1.0, -2.0)


def test_a_command_past_a_urdf_limit_goes_nowhere(rig: Callable[..., Rig], mocker: Any) -> None:
    from dimos.control.task import (
        BaseControlTask,
        ControlMode,
        CoordinatorState,
        JointCommandOutput,
        ResourceClaim,
    )

    class PastTheLimit(BaseControlTask):
        """Always active; tells torso_joint1 99 rad, ~50x past its URDF limit."""

        def __init__(self) -> None:
            self._name = "past_the_limit"

        def claim(self) -> ResourceClaim:
            return ResourceClaim(joints=frozenset({"r1pro/torso_joint1"}), priority=10)

        def is_active(self) -> bool:
            return True

        def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
            return JointCommandOutput(
                joint_names=["r1pro/torso_joint1"],
                positions=[99.0],
                mode=ControlMode.POSITION,
            )

        def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
            pass

    error = mocker.patch.object(tick_mod.logger, "error")
    r = rig()
    conn = r.attach()
    r.tick_until(lambda: bool(r.positions_sent()))
    before = list(r.fake.control.published)

    r.coordinator.add_task(PastTheLimit())
    r.tick()
    r.tick()

    # The source refuses the whole tick's command before anything is sent,
    # so the vendor topics stay untouched and the module sees no rejection.
    error.assert_called()
    assert r.fake.control.published == before
    assert conn.status().last_rejection is None


def test_an_estop_stops_the_commands_and_the_deadman_halts_the_robot(
    rig: Callable[..., Rig],
) -> None:
    r = rig(reading_timeout_s=2.0)
    conn = r.attach(deadman_timeout_s=0.05)
    r.tick_until(lambda: bool(r.positions_sent()))
    r.twist_bus.publish(Twist(linear=Vector3(0.4, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0)))
    r.tick()

    assert r.coordinator.set_estop(True)
    sent, speeds = len(r.position_commands.sent), len(r.twists_sent())
    r.tick()
    r.tick()
    assert len(r.position_commands.sent) == sent
    assert len(r.twists_sent()) == speeds

    # The coordinator going quiet leaves the robot to its own deadman: the
    # chassis is zeroed and the arms are held where they were last measured.
    wait_until(lambda: conn.status().deadman_fired, timeout=5.0)
    chassis = r.fake.control.sent_to("/motion_target/target_speed_chassis")[-1]
    assert (chassis.twist.linear.x, chassis.twist.angular.z) == (0.0, 0.0)
    held = r.fake.control.sent_to("/motion_target/target_joint_state_torso")[-1]
    assert list(held.position) == pytest.approx(MEASURED_Q[0:4])

    # Clearing the estop re-holds at the next reading and drives again.
    assert r.coordinator.set_estop(False)
    r.tick_until(lambda: len(r.position_commands.sent) > sent)


def test_a_connection_that_goes_quiet_stops_being_commanded_until_it_returns(
    rig: Callable[..., Rig],
) -> None:
    r = rig(reading_timeout_s=0.2)
    first = r.attach()
    r.tick_until(lambda: bool(r.positions_sent()))

    # The connection module goes down: it halts the robot and closes its nodes.
    r.detach(first)
    assert first.status().connected is False
    assert all(node.stopped for node in r.fake.nodes)

    # Commands keep flowing until the readings age past the timeout, then stop.
    deadline = time.monotonic() + 0.4
    while time.monotonic() < deadline:
        r.tick()
        time.sleep(0.02)
    sent = len(r.position_commands.sent)
    deadline = time.monotonic() + 0.2
    while time.monotonic() < deadline:
        r.tick()
        time.sleep(0.02)
    assert len(r.position_commands.sent) == sent
    assert not r.coordinator._connections["r1pro"].ready_for_control()

    # It comes back (a fresh module, as after a restart) reporting a moved arm.
    moved = [q + 0.01 for q in MEASURED_Q]
    r.attach(feedback=moved)
    r.tick_until(lambda: len(r.position_commands.sent) > sent)

    # The old target is not replayed: the arm is held where it now is.
    assert r.positions_sent()[-1] == pytest.approx(moved)


# Blueprint, registry, and consumer contracts — no hardware, nothing started.


def _parse(bp: Any) -> None:
    BlueprintConfigParser(bp).parse(environ={}, overrides={"g": {"viewer": "none"}})


def test_the_control_blueprint_pairs_the_connection_ports_with_the_coordinators() -> None:
    bp = r1pro_control(boot_vendor_stack=False)
    _parse(bp)

    pairs = {(old, new) for (_, old), new in bp.remapping_map.items()}
    assert ("position_command", "r1pro_position_command") in pairs
    assert ("cmd_vel", "r1pro_cmd_vel") in pairs
    assert ("r1pro_joint_state", "joint_state") in pairs
    assert ("r1pro_imu", "imu_chassis") in pairs
    # The wheel-odometry pose keeps the name navigation consumers read.
    assert ("odom", "chassis_odom") in pairs

    # The remap targets are the coordinator's real ports, and vice versa.
    # (Ports read off the classes: instantiating the modules would spawn threads.)
    from dimos.core.coordination.blueprints import BlueprintAtom

    coord_atom = BlueprintAtom.create(R1ProCoordinator, {})
    coord_ins = {s.name for s in coord_atom.streams if s.direction == "in"}
    coord_outs = {s.name for s in coord_atom.streams if s.direction == "out"}
    assert {"r1pro_position_command", "r1pro_cmd_vel"} <= coord_outs
    assert {"r1pro_joint_state", "r1pro_imu"} <= coord_ins
    conn_atom = BlueprintAtom.create(R1ProConnection, {})
    conn_ins = {s.name for s in conn_atom.streams if s.direction == "in"}
    conn_outs = {s.name for s in conn_atom.streams if s.direction == "out"}
    assert {"position_command", "cmd_vel"} <= conn_ins
    assert {"joint_state", "imu_chassis", "imu_torso", "odom", "odometry", "tf"} <= conn_outs


def test_the_old_adapter_wiring_is_gone_from_the_control_blueprint() -> None:
    bp = r1pro_control(boot_vendor_stack=False)
    streams = {name for name, _type in bp.transport_map}
    # The transport adapters' topics are gone with them.
    assert "motor_states" not in streams
    assert "motor_command" not in streams
    assert "chassis_cmd_vel" not in streams
    assert "joint_command" not in streams
    # The consumer contracts survive: the public twist bus, both IMUs, wheel
    # odometry, the chassis pose, and the coordinator's joint state.
    assert {"cmd_vel", "twist_command"} <= streams
    assert {"imu_chassis", "imu_torso", "odometry", "chassis_odom"} <= streams
    assert "coordinator_joint_state" in streams
    # No module in the blueprint is a hardware adapter.
    assert all("Adapter" not in atom.module.__name__ for atom in bp.blueprints)


def test_every_public_r1pro_blueprint_composes() -> None:
    from dimos.robot.galaxea.r1pro.blueprints.basic.r1pro_coordinator import r1pro_coordinator
    from dimos.robot.galaxea.r1pro.blueprints.basic.r1pro_teleop import r1pro_teleop
    from dimos.robot.galaxea.r1pro.blueprints.manipulation.r1pro_manipulation import (
        r1pro_manipulation,
    )
    from dimos.robot.galaxea.r1pro.blueprints.navigation.r1pro_nav import r1pro_nav

    for bp in (r1pro_coordinator, r1pro_teleop, r1pro_nav, r1pro_manipulation):
        _parse(bp)

    # Teleop keeps driving the coordinator's twist_command, as before.
    pairs = {(old, new) for (_, old), new in r1pro_teleop.remapping_map.items()}
    assert ("tele_cmd_vel", "twist_command") in pairs
    # Navigation keeps the /cmd_vel bus and the chassis pose it plans from.
    nav_streams = {name for name, _type in r1pro_nav.transport_map}
    assert {"cmd_vel", "twist_command", "chassis_odom", "odometry"} <= nav_streams
    # Manipulation keeps the coordinator's joint state it plans against.
    man_streams = {name for name, _type in r1pro_manipulation.transport_map}
    assert "coordinator_joint_state" in man_streams


def test_every_r1pro_blueprint_is_registered_and_importable() -> None:
    from dimos.robot.all_blueprints import all_blueprints

    names = sorted(name for name in all_blueprints if name.startswith("r1pro"))
    assert names == [
        "r1pro-coordinator",
        "r1pro-manipulation",
        "r1pro-nav",
        "r1pro-planar-preview",
        "r1pro-teleop",
        "r1pro-vr-teleop",
    ]
    for name in names:
        module_path, _, attr = all_blueprints[name].partition(":")
        assert hasattr(importlib.import_module(module_path), attr)


def test_grippers_are_command_only_contract_joints_and_stay_out_of_planning() -> None:
    from dimos.control.contract.limits import limits, movable_joints
    from dimos.robot.galaxea.r1pro.config import R1PRO_COORDINATOR_MODEL
    from dimos.robot.galaxea.r1pro.connection import R1ProConnectionConfig

    # Planning stays 18 DOF: no gripper is a movable joint of the planning
    # model, so no trajectory ever carries a jaw.
    assert not any("gripper" in joint.name for joint in movable_joints(R1PRO_MODEL))
    assert not any("gripper" in joint for joint in R1PRO_UPPER_BODY_JOINTS)
    # The contract carries the jaws with the vendor's real opening range.
    table = limits(R1ProConnectionConfig())
    assert (table["r1pro/left_gripper/position"].lo, table["r1pro/left_gripper/position"].hi) == (
        pytest.approx(0.0),
        pytest.approx(0.1),
    )
    assert "r1pro/right_gripper/position" in table
    # The coordinator model exposes them so tasks can drive them...
    assert {joint.name for joint in movable_joints(R1PRO_COORDINATOR_MODEL)} >= {
        "r1pro/left_gripper",
        "r1pro/right_gripper",
    }
    # ...but the shipped control blueprint's tasks never do: an idle stack
    # sends the jaws nothing.
    for task in DEFAULT_TASKS:
        assert not any("gripper" in joint for joint in task.joint_names)


def _gripper_rig(rig: Callable[..., Rig], **config: Any) -> Rig:
    """A coordinator with the manipulation blueprint's gripper tasks wired."""
    from dimos.robot.galaxea.r1pro.config import R1PRO_COORDINATOR_MODEL
    from dimos.robot.galaxea.r1pro.connection import R1PRO_GRIPPER_JOINTS

    config.setdefault("model", R1PRO_COORDINATOR_MODEL)
    config.setdefault("command_only_joints", list(R1PRO_GRIPPER_JOINTS))
    config.setdefault("tasks", [*DEFAULT_TASKS, *GRIPPER_TASKS])
    return rig(**config)


def test_a_gripper_task_drives_only_its_own_jaw_through_the_whole_chain(
    rig: Callable[..., Rig],
) -> None:
    from dimos.robot.galaxea.r1pro._connection_test_fakes import gripper_feedback

    r = _gripper_rig(rig)
    r.attach()
    r.tick_until(lambda: bool(r.positions_sent()))

    # Idle: the jaws are command-only — no hold, no zero-fill, no vendor target.
    assert all("gripper" not in name for msg in r.position_commands.sent for name in msg.name)
    assert r.fake.control.sent_to("/motion_target/target_position_gripper_left") == []
    assert r.fake.control.sent_to("/motion_target/target_position_gripper_right") == []

    # The unscoped broadcast is refused: one message can never move both sides.
    from dimos.msgs.std_msgs.Float32 import Float32

    r.gripper_bus.publish(Float32(data=0.5))
    r.tick()
    r.tick()
    assert r.fake.control.sent_to("/motion_target/target_position_gripper_left") == []
    assert r.fake.control.sent_to("/motion_target/target_position_gripper_right") == []

    # The measured openings flow back: vendor mm on the feedback topics, 0-1
    # normalized at the task.
    r.fake.control.feed("/hdas/feedback_gripper_left", gripper_feedback(20.0))
    r.fake.control.feed("/hdas/feedback_gripper_right", gripper_feedback(40.0))
    r.tick_until(
        lambda: r.coordinator.task_invoke("r1pro_left_gripper", "get_normalized", {})
        == [pytest.approx(0.2)]
    )
    assert r.coordinator.task_invoke("r1pro_right_gripper", "get_normalized", {}) == [
        pytest.approx(0.4)
    ]

    # Command the left jaw half open: task -> source -> connection -> its own
    # vendor topic, in mm. The right jaw is untouched.
    assert r.coordinator.task_invoke("r1pro_left_gripper", "set_normalized", {"values": [0.5]})
    r.tick_until(
        lambda: bool(r.fake.control.sent_to("/motion_target/target_position_gripper_left"))
    )
    sent = r.fake.control.sent_to("/motion_target/target_position_gripper_left")
    assert list(sent[-1].position) == [50.0]
    assert r.fake.control.sent_to("/motion_target/target_position_gripper_right") == []


def test_a_jaw_whose_feedback_drops_loses_its_target_and_is_not_replayed(
    rig: Callable[..., Rig],
) -> None:
    from dimos.robot.galaxea.r1pro._connection_test_fakes import gripper_feedback

    r = _gripper_rig(rig, reading_timeout_s=0.1)
    r.attach(gripper_feedback_timeout_s=0.1)
    r.tick_until(lambda: bool(r.positions_sent()))

    r.fake.control.feed("/hdas/feedback_gripper_left", gripper_feedback(20.0))
    r.tick_until(
        lambda: r.coordinator.task_invoke("r1pro_left_gripper", "get_normalized", {})
        == [pytest.approx(0.2)]
    )
    assert r.coordinator.task_invoke("r1pro_left_gripper", "set_normalized", {"values": [0.5]})
    r.tick_until(
        lambda: bool(r.fake.control.sent_to("/motion_target/target_position_gripper_left"))
    )
    sent = r.fake.control.sent_to("/motion_target/target_position_gripper_left")
    assert len(sent) == 1

    # The jaw's feedback stops (the fake streams only the arms); the readback
    # ages out, the task drops its latched target, and the intent is withdrawn.
    deadline = time.monotonic() + 0.5
    while time.monotonic() < deadline:
        r.tick()
        time.sleep(0.01)

    # Feedback returns — a transient failure between ticks. The interrupted
    # intent is not replayed, even though the task's last command is known.
    r.fake.control.feed("/hdas/feedback_gripper_left", gripper_feedback(20.0))
    deadline = time.monotonic() + 0.3
    while time.monotonic() < deadline:
        r.tick()
        time.sleep(0.01)
    assert len(r.fake.control.sent_to("/motion_target/target_position_gripper_left")) == 1

    # An explicit new command at the same opening goes through (the feedback
    # is kept alive while it is issued — a single frame ages out fast).
    assert r.coordinator.task_invoke("r1pro_left_gripper", "set_normalized", {"values": [0.5]})
    deadline = time.monotonic() + 5.0
    while len(r.fake.control.sent_to("/motion_target/target_position_gripper_left")) < 2:
        if time.monotonic() > deadline:
            raise TimeoutError("the explicit new command was never sent")
        r.fake.control.feed("/hdas/feedback_gripper_left", gripper_feedback(20.0))
        r.tick()
        time.sleep(0.01)
    assert [
        list(msg.position)
        for msg in r.fake.control.sent_to("/motion_target/target_position_gripper_left")
    ] == [[50.0], [50.0]]
