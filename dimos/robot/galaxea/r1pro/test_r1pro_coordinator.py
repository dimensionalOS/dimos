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

"""The R1 Pro's control wiring end to end, with no hardware."""

from __future__ import annotations

from collections.abc import Callable, Iterator
import importlib
import math
import threading
import time
from typing import Any

import pytest

from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import (
    TrajectoryExecutionStatus,
    joint_trajectory_task,
)
from dimos.control.tick_loop import TickLoop
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.robot.galaxea.r1pro._connection_test_fakes import (
    CHASSIS_FEEDBACK_TOPIC,
    MEASURED_Q,
    FakeRos,
    LocalTransport,
    RosHeader,
    RosImu,
    chassis_speed,
    feed_all_segments,
    feed_chassis_interval,
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
    R1ProConnectionConfig,
)
from dimos.utils.testing.waiting import wait_until

#: The tasks the shipped blueprint reuses, unmodified.
DEFAULT_TASKS = [
    joint_trajectory_task(R1PRO_UPPER_BODY_JOINTS),
    TaskConfig(
        name="vel_chassis",
        type="twist_velocity",
        joint_names=R1PRO_BASE_JOINTS,
        priority=10,
        params={
            "axes": ["linear.x", "linear.y", "angular.z"],
            "yaw_joint": R1PRO_BASE_JOINTS[2],
            "timeout": 0.2,
        },
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
        coordinator.set_transport("twist_command", self.twist_bus)  # type: ignore[arg-type]
        coordinator.set_transport("r1pro_position_command", self.position_commands)  # type: ignore[arg-type]
        coordinator.set_transport("r1pro_velocity_command", self.base_commands)  # type: ignore[arg-type]
        coordinator.set_transport("r1pro_joint_state", self.joint_states)  # type: ignore[arg-type]
        coordinator.set_transport("r1pro_imu", self.imus)  # type: ignore[arg-type]
        coordinator.start()
        assert coordinator._tick_loop is not None
        self.tick = coordinator._tick_loop._tick
        self.connections: list[R1ProConnection] = []
        self._feed_stops: list[threading.Event] = []
        self._streamed: list[list[list[float] | None]] = []
        self.yaw = math.pi / 2

    def attach(self, feedback: list[float] | None = None, **config: Any) -> R1ProConnection:
        """Start an ``R1ProConnection`` on the shared ports, on the fake vendor stack."""
        config.setdefault("boot_vendor_stack", False)
        config.setdefault("deadman_timeout_s", 5.0)
        config.setdefault("sensor_stats_interval_s", 0.0)
        config.setdefault("enable_wrist_color", False)
        conn = R1ProConnection(**config)
        conn.set_transport("position_command", self.position_commands)  # type: ignore[arg-type]
        conn.set_transport("velocity_command", self.base_commands)  # type: ignore[arg-type]
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
        """Keep measured feedback fresh across slow CI ticks, until detached."""

        def subscribed() -> bool:
            return bool(self.fake.nodes) and self.fake.control.listener_count() >= 4

        while not stop.is_set() and not subscribed():
            stop.wait(0.01)
        feed_chassis_interval(self.fake, turn=self.yaw)
        previous_yaw = self.yaw
        previous_ts = time.time()
        self.fake.control.feed(CHASSIS_FEEDBACK_TOPIC, chassis_speed(0.0, 0.0, 0.0, previous_ts))
        while not stop.is_set():
            feed_all_segments(self.fake, q=streamed[0])
            ts, yaw = time.time(), self.yaw
            wz = (yaw - previous_yaw) / (ts - previous_ts)
            self.fake.control.feed(CHASSIS_FEEDBACK_TOPIC, chassis_speed(0.0, 0.0, wz, ts))
            previous_ts, previous_yaw = ts, yaw
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
        # Test helper only: inspect canonical world velocities as named axes.
        return [
            Twist(Vector3(*list(msg.velocity)[:2], 0.0), Vector3(0.0, 0.0, msg.velocity[2]))
            for msg in self.base_commands.sent
        ]

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
            connections=[R1ProConnectionConfig()],
            tasks=list(DEFAULT_TASKS),
            **config,
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


def test_real_coordinator_holds_then_drives_arm_and_base_on_canonical_planes(
    rig: Callable[..., Rig],
) -> None:
    r = rig()
    assert r.coordinator.list_hardware() == ["r1pro"]
    assert r.coordinator.list_joints() == [*R1PRO_UPPER_BODY_JOINTS, *R1PRO_BASE_JOINTS]
    assert set(r.coordinator._tasks) == {"joint_trajectory", "vel_chassis"}
    conn = r.attach()
    r.tick_until(lambda: bool(r.position_commands.sent) and bool(r.base_commands.sent))
    assert r.joint_positions() == pytest.approx(MEASURED_Q)
    joints = r.coordinator._connections["r1pro"].read_joints()
    assert set(joints.joint_positions) == set(R1PRO_UPPER_BODY_JOINTS + R1PRO_BASE_JOINTS)
    assert list(r.position_commands.sent[-1].name) == R1PRO_UPPER_BODY_JOINTS
    assert list(r.base_commands.sent[-1].name) == R1PRO_BASE_JOINTS
    assert r.positions_sent()[-1] == pytest.approx(MEASURED_Q)
    assert list(r.base_commands.sent[-1].velocity) == [0.0] * 3

    goal = [q + 0.02 for q in MEASURED_Q]
    result = r.coordinator.execute_trajectory(
        JointTrajectory(
            joint_names=list(R1PRO_UPPER_BODY_JOINTS),
            points=[
                TrajectoryPoint(positions=MEASURED_Q),
                TrajectoryPoint(positions=goal, time_from_start=0.1),
            ],
        )
    )
    assert result.status is TrajectoryExecutionStatus.ACCEPTED, result
    assert "twist_command" in r.coordinator._stream_unsubs
    r.twist_bus.publish(Twist(linear=Vector3(0.5, -0.2, 0.0), angular=Vector3(0.0, 0.0, 0.7)))
    r.tick()
    # Real measured pi/2 yaw: body -> canonical world velocities -> vendor body.
    assert list(r.base_commands.sent[-1].velocity) == pytest.approx([0.2, 0.5, 0.7])
    sent = r.fake.control.sent_to("/motion_target/target_speed_chassis")[-1].twist
    assert (sent.linear.x, sent.linear.y, sent.angular.z) == pytest.approx((0.5, -0.2, 0.7))

    def at_goal() -> bool:
        sent = r.fake.control.sent_to("/motion_target/target_joint_state_arm_left")
        return bool(sent) and list(sent[-1].position) == pytest.approx(goal[4:11], abs=1e-3)

    r.tick_until(at_goal)
    assert conn.status().last_rejection is conn.status().last_error is None
    for segment, values in (
        ("torso", goal[:4]),
        ("arm_left", goal[4:11]),
        ("arm_right", goal[11:]),
    ):
        msg = r.fake.control.sent_to(f"/motion_target/target_joint_state_{segment}")[-1]
        assert list(msg.position) == pytest.approx(values, abs=1e-3)
        assert list(msg.velocity) == [conn.config.tracking_speed] * len(values)
    r.feed(q=goal)
    r.tick_until(lambda: r.joint_positions() == pytest.approx(goal, abs=1e-6))

    # Bounds reject the whole tick rather than clipping or sending arm-only output.
    before = list(r.fake.control.published)
    r.twist_bus.publish(Twist(linear=Vector3(5.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, -9.0)))
    r.tick()
    assert r.fake.control.published == before


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

    r.twist_bus.publish(Twist(linear=Vector3(0.6, 0.0, 0.0)))
    # Neither pre-estop nor latched input may replay after clearing.
    assert r.coordinator.set_estop(False)
    r.tick_until(lambda: len(r.position_commands.sent) > sent)
    assert list(r.base_commands.sent[-1].velocity) == [0.0] * 3
    r.twist_bus.publish(Twist(linear=Vector3(0.3, 0.0, 0.0)))
    r.tick()
    assert list(r.base_commands.sent[-1].velocity) == pytest.approx([0.0, 0.3, 0.0])


def test_raw_twist_rotates_each_compute_and_expires_without_new_input(
    rig: Callable[..., Rig],
) -> None:
    r = rig()
    r.attach()
    r.tick_until(lambda: bool(r.positions_sent()))
    r.twist_bus.publish(Twist(linear=Vector3(0.4, 0.0, 0.0)))
    r.tick()
    assert list(r.base_commands.sent[-1].velocity) == pytest.approx([0.0, 0.4, 0.0])
    r.yaw = 0.7
    r.tick_until(
        lambda: r.coordinator.get_joint_positions().get(R1PRO_BASE_JOINTS[2]) == pytest.approx(0.7),
        timeout_s=2.0,
    )
    r.tick()
    assert list(r.base_commands.sent[-1].velocity) == pytest.approx(
        [0.4 * math.cos(0.7), 0.4 * math.sin(0.7), 0.0]
    )
    time.sleep(0.21)
    r.tick()
    assert list(r.base_commands.sent[-1].velocity) == [0.0] * 3


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
    source = r.coordinator._connections["r1pro"]
    r.tick_until(lambda: not source.ready_for_control())
    sent = len(r.position_commands.sent)
    r.tick()
    r.tick()
    assert len(r.position_commands.sent) == sent

    # It comes back (a fresh module, as after a restart) reporting a moved arm.
    moved = [q + 0.01 for q in MEASURED_Q]
    r.attach(feedback=moved)
    r.tick_until(lambda: len(r.position_commands.sent) > sent)

    # The old target is not replayed: the arm is held where it now is.
    assert r.positions_sent()[-1] == pytest.approx(moved)


# Blueprint, registry, and consumer contracts — no hardware, nothing started.


def test_the_control_blueprint_pairs_the_connection_ports_with_the_coordinators() -> None:
    bp = r1pro_control(boot_vendor_stack=False)
    BlueprintConfigParser(bp).parse(environ={}, overrides={"g": {"viewer": "none"}})

    streams = {name for name, _type in bp.transport_map}
    assert not {"motor_states", "motor_command", "chassis_cmd_vel", "joint_command"} & streams
    assert {
        "cmd_vel",
        "twist_command",
        "imu_chassis",
        "imu_torso",
        "odometry",
        "chassis_odom",
        "coordinator_joint_state",
    } <= streams
    assert all("Adapter" not in atom.module.__name__ for atom in bp.blueprints)
    pairs = {(old, new) for (_, old), new in bp.remapping_map.items()}
    assert ("position_command", "r1pro_position_command") in pairs
    assert ("velocity_command", "r1pro_velocity_command") in pairs
    assert ("r1pro_joint_state", "joint_state") in pairs
    assert ("r1pro_imu", "imu_chassis") in pairs
    # The wheel-odometry pose keeps the name navigation consumers read.
    assert ("odom", "chassis_odom") in pairs


def test_every_public_r1pro_blueprint_is_registered_importable_and_composes() -> None:
    from dimos.robot.all_blueprints import all_blueprints

    names = sorted(name for name in all_blueprints if name.startswith("r1pro"))
    assert names == [
        "r1pro-coordinator",
        "r1pro-manipulation",
        "r1pro-nav",
        "r1pro-planar-preview",
        "r1pro-teleop",
    ]
    blueprints = {}
    for name in names:
        module_path, _, attr = all_blueprints[name].partition(":")
        bp = getattr(importlib.import_module(module_path), attr)
        BlueprintConfigParser(bp).parse(environ={}, overrides={"g": {"viewer": "none"}})
        blueprints[name] = bp
    pairs = {(old, new) for (_, old), new in blueprints["r1pro-teleop"].remapping_map.items()}
    assert ("tele_cmd_vel", "twist_command") in pairs
    nav_streams = {name for name, _type in blueprints["r1pro-nav"].transport_map}
    assert {"cmd_vel", "twist_command", "chassis_odom", "odometry"} <= nav_streams
    man_streams = {name for name, _type in blueprints["r1pro-manipulation"].transport_map}
    assert "coordinator_joint_state" in man_streams


def test_no_gripper_enters_the_control_path() -> None:
    from dimos.control.contract.limits import limits, movable_joints
    from dimos.robot.galaxea.r1pro.connection import R1ProConnectionConfig

    # The gripper fingers stay welded out of the model, so no key, task
    # joint, or stream drives them.
    assert not any("gripper" in joint.name for joint in movable_joints(R1PRO_MODEL))
    assert not any("gripper" in key for key in limits(R1ProConnectionConfig()))
    assert not any("gripper" in joint for joint in R1PRO_UPPER_BODY_JOINTS)
    for task in DEFAULT_TASKS:
        assert not any("gripper" in joint for joint in task.joint_names)
