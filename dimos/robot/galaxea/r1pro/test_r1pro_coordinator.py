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
import math
import time
from typing import Any

import pytest

from dimos.control.coordinator import TaskConfig
from dimos.control.tick_loop import TickLoop
from dimos.robot.galaxea.r1pro._connection_test_fakes import (
    CHASSIS_FEEDBACK_TOPIC,
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
from dimos.robot.galaxea.r1pro.config import R1PRO_MODEL
from dimos.robot.galaxea.r1pro.connection import (
    R1PRO_UPPER_BODY_JOINTS,
    R1ProConnectionConfig,
)
from dimos.robot.galaxea.r1pro.test_connection import (
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


@pytest.fixture
def system(
    request: pytest.FixtureRequest, rig: Callable[..., ConnectionRig], mocker: Any
) -> Iterator[tuple[Any, ...]]:
    """One real R1Pro connection, the shipped tasks, and a hand-stepped coordinator."""
    mocker.patch.object(TickLoop, "start", lambda self: self._stop_event.clear())
    config = getattr(request, "param", {})
    r = rig(start=False, **config.get("connection", {}))
    coordinator = R1ProCoordinator(
        instance_name="ControlCoordinator",
        connections=[R1ProConnectionConfig()],
        tasks=[*DEFAULT_TASKS, *config.get("tasks", [])],
        reading_timeout_s=config.get("reading_timeout_s", 0.2),
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
    r.ports["gripper_command"] = LocalTransport()
    coordinator.set_transport("gripper_command", r.ports["gripper_command"])  # type: ignore[arg-type]
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


def test_grippers_are_command_only_contract_joints_and_stay_out_of_planning() -> None:
    from dimos.control.contract.limits import limits, movable_joints
    from dimos.robot.galaxea.r1pro.config import R1PRO_CONTROL_MODEL
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
    assert {joint.name for joint in movable_joints(R1PRO_CONTROL_MODEL)} >= {
        "r1pro/left_gripper",
        "r1pro/right_gripper",
    }
    # ...but the shipped control blueprint's tasks never do: an idle stack
    # sends the jaws nothing.
    for task in DEFAULT_TASKS:
        assert not any("gripper" in joint for joint in task.joint_names)


@pytest.mark.parametrize("system", [{"tasks": GRIPPER_TASKS}], indirect=True)
def test_a_gripper_task_drives_only_its_own_jaw_through_the_whole_chain(
    system: tuple[Any, ...],
) -> None:
    from dimos.robot.galaxea.r1pro._connection_test_fakes import gripper_feedback

    coordinator, r, twist, tick = system
    wait_until(
        lambda: (tick(), bool(r.ports["position_command"].sent))[1], timeout=5.0, interval=0.01
    )
    assert all(
        "gripper" not in name for msg in r.ports["position_command"].sent for name in msg.name
    )
    assert r.fake.control.sent_to("/motion_target/target_position_gripper_left") == []
    assert r.fake.control.sent_to("/motion_target/target_position_gripper_right") == []
    from dimos.msgs.std_msgs.Float32 import Float32

    r.ports["gripper_command"].publish(Float32(data=0.5))
    tick()
    tick()
    assert r.fake.control.sent_to("/motion_target/target_position_gripper_left") == []
    assert r.fake.control.sent_to("/motion_target/target_position_gripper_right") == []
    r.fake.control.feed("/hdas/feedback_gripper_left", gripper_feedback(20.0))
    r.fake.control.feed("/hdas/feedback_gripper_right", gripper_feedback(40.0))
    wait_until(
        lambda: (
            tick(),
            coordinator.task_invoke("r1pro_left_gripper", "get_normalized", {})
            == [pytest.approx(0.2)],
        )[1],
        timeout=5.0,
        interval=0.01,
    )
    assert coordinator.task_invoke("r1pro_right_gripper", "get_normalized", {}) == [
        pytest.approx(0.4)
    ]
    assert coordinator.task_invoke("r1pro_left_gripper", "set_normalized", {"values": [0.5]})
    wait_until(
        lambda: (
            tick(),
            bool(r.fake.control.sent_to("/motion_target/target_position_gripper_left")),
        )[1],
        timeout=5.0,
        interval=0.01,
    )
    sent = r.fake.control.sent_to("/motion_target/target_position_gripper_left")
    assert list(sent[-1].position) == [50.0]
    assert r.fake.control.sent_to("/motion_target/target_position_gripper_right") == []


@pytest.mark.parametrize(
    "system",
    [
        {
            "tasks": GRIPPER_TASKS,
            "reading_timeout_s": 0.1,
            "connection": {"gripper_feedback_timeout_s": 0.1},
        }
    ],
    indirect=True,
)
def test_a_jaw_whose_feedback_drops_loses_its_target_and_is_not_replayed(
    system: tuple[Any, ...],
) -> None:
    from dimos.robot.galaxea.r1pro._connection_test_fakes import gripper_feedback

    coordinator, r, twist, tick = system
    wait_until(
        lambda: (tick(), bool(r.ports["position_command"].sent))[1], timeout=5.0, interval=0.01
    )
    r.fake.control.feed("/hdas/feedback_gripper_left", gripper_feedback(20.0))
    wait_until(
        lambda: (
            tick(),
            coordinator.task_invoke("r1pro_left_gripper", "get_normalized", {})
            == [pytest.approx(0.2)],
        )[1],
        timeout=5.0,
        interval=0.01,
    )
    assert coordinator.task_invoke("r1pro_left_gripper", "set_normalized", {"values": [0.5]})
    wait_until(
        lambda: (
            tick(),
            bool(r.fake.control.sent_to("/motion_target/target_position_gripper_left")),
        )[1],
        timeout=5.0,
        interval=0.01,
    )
    sent = r.fake.control.sent_to("/motion_target/target_position_gripper_left")
    assert len(sent) == 1
    deadline = time.monotonic() + 0.5
    while time.monotonic() < deadline:
        tick()
        time.sleep(0.01)
    r.fake.control.feed("/hdas/feedback_gripper_left", gripper_feedback(20.0))
    deadline = time.monotonic() + 0.3
    while time.monotonic() < deadline:
        tick()
        time.sleep(0.01)
    assert len(r.fake.control.sent_to("/motion_target/target_position_gripper_left")) == 1
    assert coordinator.task_invoke("r1pro_left_gripper", "set_normalized", {"values": [0.5]})
    deadline = time.monotonic() + 5.0
    while len(r.fake.control.sent_to("/motion_target/target_position_gripper_left")) < 2:
        if time.monotonic() > deadline:
            raise TimeoutError("the explicit new command was never sent")
        r.fake.control.feed("/hdas/feedback_gripper_left", gripper_feedback(20.0))
        tick()
        time.sleep(0.01)
    assert [
        list(msg.position)
        for msg in r.fake.control.sent_to("/motion_target/target_position_gripper_left")
    ] == [[50.0], [50.0]]
