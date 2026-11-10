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
import time
from typing import Any

import pytest

from dimos.control.tasks.trajectory_task.trajectory_task import (
    TrajectoryExecutionStatus,
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
    LocalTransport,
    RosHeader,
    RosImu,
    chassis_speed,
    feed_chassis_interval,
)
from dimos.robot.galaxea.r1pro.blueprints.basic.r1pro_coordinator import (
    R1ProCoordinator,
    r1pro_control,
)
from dimos.robot.galaxea.r1pro.connection import (
    R1PRO_BASE_JOINTS,
    R1PRO_UPPER_BODY_JOINTS,
    R1ProConnectionConfig,
)
from dimos.robot.galaxea.r1pro.test_connection import (
    LEFT_TOPIC,
    RIGHT_TOPIC,
    TORSO_TOPIC,
    Rig as ConnectionRig,
    fake_ros as fake_ros,
    rig as rig,
)
from dimos.utils.testing.waiting import wait_until

# Reuse the actual shipped task declarations, rather than a second test-only copy.
DEFAULT_TASKS = next(
    atom.kwargs["tasks"]
    for atom in r1pro_control(boot_vendor_stack=False).blueprints
    if atom.module is R1ProCoordinator
)


@pytest.fixture
def system(rig: Callable[..., ConnectionRig], mocker: Any) -> Iterator[tuple[Any, ...]]:
    """One real R1Pro connection, the shipped tasks, and a hand-stepped coordinator."""
    mocker.patch.object(TickLoop, "start", lambda self: self._stop_event.clear())
    r = rig(start=False)
    coordinator = R1ProCoordinator(
        instance_name="ControlCoordinator",
        connections=[R1ProConnectionConfig()],
        tasks=list(DEFAULT_TASKS),
    )
    twist = LocalTransport()
    coordinator.set_transport("twist_command", twist)  # type: ignore[arg-type]
    for target, source in {
        "r1pro_position_command": "position_command",
        "r1pro_velocity_command": "velocity_command",
        "r1pro_joint_state": "joint_state",
        "r1pro_imu": "imu_chassis",
    }.items():
        coordinator.set_transport(target, r.ports[source])  # type: ignore[arg-type]
    coordinator.start()
    assert coordinator._tick_loop is not None
    tick_loop = coordinator._tick_loop
    r.start()
    feed_chassis_interval(r.fake, turn=math.pi / 2)

    def tick() -> None:
        r.feed()
        r.fake.control.feed(CHASSIS_FEEDBACK_TOPIC, chassis_speed(0.0, 0.0, 0.0, time.time()))
        r.fake.sensors.feed("/hdas/imu_chassis", RosImu(header=RosHeader()))
        tick_loop._tick()

    try:
        yield coordinator, r, twist, tick
    finally:
        coordinator.stop()


def test_registered_tasks_reach_r1pro_vendor_topics(system: tuple[Any, ...]) -> None:
    coordinator, r, twist, tick = system
    wait_until(lambda: (tick(), bool(r.joint_targets()))[1], timeout=5.0, interval=0.01)
    assert set(coordinator._tasks) == {"joint_trajectory", "vel_chassis"}
    assert set(coordinator.list_joints()) == set(R1PRO_UPPER_BODY_JOINTS + R1PRO_BASE_JOINTS)
    assert not any("gripper" in j for task in DEFAULT_TASKS for j in task.joint_names)
    for topic, values in zip(
        (TORSO_TOPIC, LEFT_TOPIC, RIGHT_TOPIC),
        (MEASURED_Q[:4], MEASURED_Q[4:11], MEASURED_Q[11:]),
        strict=True,
    ):
        assert list(r.joint_targets()[topic].position) == pytest.approx(values)
    assert r.speeds()[-1].twist.linear.x == 0.0
    goal = [q + 0.02 for q in MEASURED_Q]
    result = coordinator.execute_trajectory(
        JointTrajectory(
            joint_names=list(R1PRO_UPPER_BODY_JOINTS),
            points=[
                TrajectoryPoint(positions=MEASURED_Q),
                TrajectoryPoint(positions=goal, time_from_start=0.1),
            ],
        )
    )
    assert result.status is TrajectoryExecutionStatus.ACCEPTED, result
    twist.publish(Twist(linear=Vector3(0.5, -0.2, 0.0), angular=Vector3(0.0, 0.0, 0.7)))
    tick()
    assert list(r.ports["velocity_command"].sent[-1].velocity) == pytest.approx([0.2, 0.5, 0.7])
    sent = r.speeds()[-1].twist
    assert (sent.linear.x, sent.linear.y, sent.angular.z) == pytest.approx((0.5, -0.2, 0.7))
    wait_until(
        lambda: (
            tick(),
            list(r.joint_targets()[LEFT_TOPIC].position) == pytest.approx(goal[4:11], abs=1e-3),
        )[1],
        timeout=5.0,
        interval=0.01,
    )
    assert r.module.status().last_rejection is r.module.status().last_error is None
    for topic, values in zip(
        (TORSO_TOPIC, LEFT_TOPIC, RIGHT_TOPIC), (goal[:4], goal[4:11], goal[11:]), strict=True
    ):
        msg = r.joint_targets()[topic]
        assert list(msg.position) == pytest.approx(values, abs=1e-3)
        assert list(msg.velocity) == [r.module.config.tracking_speed] * len(values)
    r.module.halt()
    stopped = r.speeds()[-1].twist
    assert (stopped.linear.x, stopped.linear.y, stopped.angular.z) == (0.0, 0.0, 0.0)
    held = [q for msg in r.joint_targets().values() for q in msg.position]
    assert held == pytest.approx(MEASURED_Q)  # Hold measurements, not the commanded goal.


@pytest.mark.parametrize("name", ["coordinator", "manipulation", "nav", "planar-preview", "teleop"])
def test_public_r1pro_assemblies_parse_with_canonical_consumer_wiring(name: str) -> None:
    from dimos.robot.all_blueprints import all_blueprints

    module_path, _, attr = all_blueprints[f"r1pro-{name}"].partition(":")
    bp = getattr(importlib.import_module(module_path), attr)
    BlueprintConfigParser(bp).parse(environ={}, overrides={"g": {"viewer": "none"}})
    if name == "planar-preview":
        return
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
    assert {
        ("position_command", "r1pro_position_command"),
        ("velocity_command", "r1pro_velocity_command"),
        ("r1pro_joint_state", "joint_state"),
        ("r1pro_imu", "imu_chassis"),
        ("odom", "chassis_odom"),
    } <= pairs
    if name == "teleop":
        assert ("tele_cmd_vel", "twist_command") in pairs
