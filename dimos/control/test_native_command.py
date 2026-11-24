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

"""Native arbitration, ordinary worker delivery and sender-side cancellation."""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from functools import partial
import os
from threading import Event
from typing import Any

import pytest
from reactivex.disposable import Disposable

from dimos.control._control_test_helpers import FakeArm, FakeArmConfig, fake_arm_model
from dimos.control.connection.connection_module import ConnectionConfig
from dimos.control.connection_source import ConnectionSource
from dimos.control.coordinator import ControlCoordinator
from dimos.control.group_command import GroupCommandOutput
from dimos.control.task import BaseControlTask, CoordinatorState, JointCommandOutput, ResourceClaim
from dimos.control.tick_loop import TickLoop
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.core.transport import ZenohTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.testing.waiting import wait_until

J1, J2, J3 = "mock/joint1", "mock/joint2", "mock/joint3"
POSE = PoseStamped(ts=17.5, frame_id="tool_base", position=[0.25, 0.5, 0.75])


class Reference(BaseControlTask):
    def __init__(self, name: str, priority: int, output: GroupCommandOutput | JointCommandOutput):
        self._name = name
        self._priority = priority
        self._output = output
        self.active = True

    def claim(self) -> ResourceClaim:
        # Native tasks do not redeclare any of the joints behind the port.
        return ResourceClaim(frozenset(), priority=self._priority)

    def is_active(self) -> bool:
        return self.active

    def compute(self, state: CoordinatorState) -> GroupCommandOutput | JointCommandOutput:
        return self._output

    def cancel(self) -> None:
        self.active = False

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        return None


class NativeCoordinator(ControlCoordinator):
    mock_pose: Out[PoseStamped]
    mock_other_pose: Out[PoseStamped]
    mock_position_command: Out[JointState]
    mock_joint_state: In[JointState]

    @rpc
    def scenario(self, mode: str) -> int:
        with self._task_lock:
            self._tasks.clear()
            self._tasks.update(
                low=Reference("low", 0, JointCommandOutput([J2, J3], positions=[0.4, 0.6])),
                pose=Reference("pose", 20, GroupCommandOutput("mock/pose", POSE)),
            )
            if mode in {"native", "invalid_scalar"}:
                self._tasks["other_pose"] = Reference(
                    "other_pose", 10, GroupCommandOutput("mock/other_pose", POSE)
                )
                if mode == "invalid_scalar":
                    self._tasks["top"] = Reference(
                        "top", 30, JointCommandOutput([J1], positions=[float("nan")])
                    )
            else:
                joints, values = ([J1], [0.8]) if mode == "blocked" else ([J1, J2], [0.8, 0.7])
                self._tasks["top"] = Reference(
                    "top", 30, JointCommandOutput(joints, positions=values)
                )
        return os.getpid()


class RecordingConnection(FakeArm):
    pose: In[PoseStamped]
    other_pose: In[PoseStamped]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.received: dict[str, tuple[str, str, float, float, float, float]] = {}

    @rpc
    def start(self) -> None:
        super().start()
        for port in (self.pose, self.other_pose):
            self.register_disposable(Disposable(port.subscribe(partial(self.record, port.name))))

    def record(self, port: str, msg: PoseStamped) -> None:
        self.received[port] = (type(msg).__name__, msg.frame_id, msg.ts, msg.x, msg.y, msg.z)

    @rpc
    def evidence(self) -> tuple[int, bool, dict[str, Any], dict[str, float]]:
        return (
            os.getpid(),
            type(self.pose.transport) is ZenohTransport,
            self.received.copy(),
            self.writes[-1] if self.writes else {},
        )


def assembly() -> Blueprint:
    declaration = FakeArmConfig(
        model=fake_arm_model(("joint1", "joint2", "joint3")),
        joint_commands={
            k: v
            for k, v in FakeArmConfig().joint_commands.items()
            if k in {"joint1", "joint2", "joint3"}
        },
        group_commands={
            "pose": frozenset({"joint1", "joint2"}),
            "other_pose": frozenset({"joint2", "joint3"}),
        },
        deadman_timeout_s=2.0,
    )
    return (
        autoconnect(
            NativeCoordinator.blueprint(
                connections=[
                    ConnectionConfig(
                        **declaration.model_dump(include=set(ConnectionConfig.model_fields))
                    )
                ],
                reading_timeout_s=4.0,
            ),
            RecordingConnection.blueprint(**declaration.model_dump()),
        )
        .solidify()
        .remappings(
            [
                (RecordingConnection, "pose", "mock_pose"),
                (RecordingConnection, "other_pose", "mock_other_pose"),
                (RecordingConnection, "position_command", "mock_position_command"),
                (RecordingConnection, "joint_state", "mock_joint_state"),
            ]
        )
    )


def test_native_command_over_ordinary_worker_remap() -> None:
    blueprint = assembly()
    parsed = BlueprintConfigParser(blueprint).parse(
        environ={},
        overrides={"g": {"viewer": "none"}},
        global_overrides={"zenoh_scout_addr": os.environ["ZENOH_SCOUT_ADDR"]},
    )
    system = ModuleCoordinator.build(blueprint, parsed)
    try:
        cc = system.get_instance(NativeCoordinator)
        receiver = system.get_instance(RecordingConnection)
        wait_until(lambda: J3 in cc.get_joint_positions(), timeout=10)
        sender_pid = cc.scenario("blocked")
        wait_until(
            lambda: receiver.evidence()[3]
            == {f"{J1}/position": 0.8, f"{J2}/position": 0.4, f"{J3}/position": 0.6},
            timeout=10,
        )
        receiver_pid, ordinary_transport, received, _ = receiver.evidence()
        assert len({sender_pid, receiver_pid, os.getpid()}) == 3
        assert ordinary_transport
        assert not received

        cc.scenario("partial")
        wait_until(
            lambda: receiver.evidence()[3]
            == {f"{J1}/position": 0.8, f"{J2}/position": 0.7, f"{J3}/position": 0.6},
            timeout=10,
        )
        assert not receiver.evidence()[2]

        cc.scenario("invalid_scalar")
        wait_until(lambda: receiver.evidence()[2], timeout=10)
        assert receiver.evidence()[2] == {
            "pose": ("PoseStamped", "tool_base", 17.5, 0.25, 0.5, 0.75)
        }
        wait_until(lambda: receiver.evidence()[3] == {f"{J3}/position": 0.6}, timeout=10)
    finally:
        system.stop()


@pytest.mark.parametrize("stop", ["cancel", "estop"])
def test_stop_discards_a_computed_native_result(
    monkeypatch: pytest.MonkeyPatch, request: pytest.FixtureRequest, stop: str
) -> None:
    cc = NativeCoordinator()
    request.addfinalizer(cc.stop)
    source = ConnectionSource(
        ConnectionConfig(
            source="mock",
            model=fake_arm_model(("joint1", "joint2")),
            group_commands={"pose": frozenset({"joint1", "joint2"})},
        ),
        [],
        ["joint_state"],
        30.0,
        group_ports={"mock/pose": cc.mock_pose},
    )
    source.on_message("joint_state", JointState(name=[J1, J2], position=[0.0, 0.0]))
    cc._tasks["pose"] = Reference("pose", 20, GroupCommandOutput("mock/pose", POSE))
    loop = TickLoop(
        100,
        {},
        cc._hardware_lock,
        cc._tasks,
        cc._task_lock,
        {},
        connections={"mock": source},
        group_commands_allowed=lambda: not cc._estopped,
    )
    cc._tick_loop = loop
    loop._stop_event.clear()  # Drive ticks by hand without a second tick thread.
    published: list[PoseStamped] = []
    cc.mock_pose.subscribe(published.append)
    loop._tick()
    assert published == [POSE]

    computed, release = Event(), Event()
    compute = loop._compute_all_tasks

    def pause_after_compute(state: CoordinatorState):
        result = compute(state)
        computed.set()
        assert release.wait(timeout=5)
        return result

    monkeypatch.setattr(loop, "_compute_all_tasks", pause_after_compute)
    with ThreadPoolExecutor(max_workers=1) as worker:
        stale_tick = worker.submit(loop._tick)
        try:
            assert computed.wait(timeout=5)
            if stop == "cancel":
                cc.task_invoke("pose", "cancel")
            else:
                cc.set_estop(True)
        finally:
            release.set()
        stale_tick.result(timeout=5)
    loop._tick()
    assert published == [POSE]


def test_group_needs_fresh_feedback_for_each_covered_axis(
    monkeypatch: pytest.MonkeyPatch,
    request: pytest.FixtureRequest,
) -> None:
    clock = 10.0
    monkeypatch.setattr("dimos.control.connection_source.time.monotonic", lambda: clock)
    cc = NativeCoordinator()
    request.addfinalizer(cc.stop)
    source = ConnectionSource(
        ConnectionConfig(
            source="mock",
            model=fake_arm_model(("joint1", "joint2")),
            group_commands={"pose": frozenset({"joint1", "joint2"})},
        ),
        [],
        ["joint_state"],
        1.0,
        group_ports={"mock/pose": cc.mock_pose},
    )
    received: list[PoseStamped] = []
    cc.mock_pose.subscribe(received.append)
    source.on_message("joint_state", JointState(name=[J1, J2], position=[0.0, 0.0]))
    assert source.publish_group("mock/pose", POSE)
    for reading_time in (10.75, 11.5):
        clock = reading_time
        source.on_message("joint_state", JointState(name=[J1], position=[0.0]))
    assert source.ready_for_control()  # Ordinary legacy port remains fresh.
    assert not source.publish_group("mock/pose", POSE)
    assert received == [POSE]
    source.on_message("joint_state", JointState(name=[J2], position=[0.0]))
    assert source.publish_group("mock/pose", POSE)


def test_group_outputs_cannot_alias() -> None:
    class AmbiguousCoordinator(ControlCoordinator):
        left_joint_state: In[JointState]
        left_arm_joint_state: In[JointState]
        left_arm_pose: Out[PoseStamped]

    model = fake_arm_model(("joint1",))
    joint_set = frozenset({"joint1"})
    cc = AmbiguousCoordinator(
        connections=[
            ConnectionConfig(source="left", model=model, group_commands={"arm_pose": joint_set}),
            ConnectionConfig(source="left_arm", model=model, group_commands={"pose": joint_set}),
        ]
    )
    try:
        with pytest.raises(ValueError, match="already bound"):
            cc._connect_robots()
    finally:
        cc.stop()
