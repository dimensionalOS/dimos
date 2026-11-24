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

"""Native port ownership and fencing over real transport in separate workers."""

from __future__ import annotations

from functools import partial
import os
import time
from typing import Any

import pytest
from reactivex.disposable import Disposable

from dimos.control._control_test_helpers import FakeArm, FakeArmConfig, fake_arm_model
from dimos.control.connection.connection_module import ConnectionConfig
from dimos.control.connection_source import ConnectionSource
from dimos.control.coordinator import ControlCoordinator
from dimos.control.group_command import GroupCommandOutput
from dimos.control.native_command import CommandTicket, native_transport, wire_native_commands
from dimos.control.task import BaseControlTask, CoordinatorState, JointCommandOutput, ResourceClaim
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.core.transport import pZenohTransport
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
            for name, priority, output in (
                (
                    "top",
                    30,
                    JointCommandOutput(
                        [J1, J2] if mode == "legacy" else [J1],
                        positions=[0.8, 0.7] if mode == "legacy" else [0.8],
                    ),
                ),
                (
                    "pose",
                    20,
                    GroupCommandOutput("mock/pose", POSE if mode != "invalid" else object()),
                ),
                ("other_pose", 10, GroupCommandOutput("mock/other_pose", POSE)),
                ("scalar", 0, JointCommandOutput([J2, J3], positions=[0.4, 0.6])),
            ):
                if (
                    (name == "top" and mode not in {"scalar", "legacy"})
                    or (name == "other_pose" and mode != "native")
                    or (name == "pose" and mode == "legacy")
                ):
                    continue
                self._tasks[name] = Reference(name, priority, output)
        return os.getpid()

    @rpc
    def delay_pose_packets(self) -> tuple[int, int, int, PoseStamped]:
        transport = native_transport(self.mock_pose, "mock/pose")
        self._undelayed_send = transport._send
        captured: list[tuple[int, int, int, PoseStamped]] = []

        def delay(frame: Any) -> None:
            if frame[2]:
                if not captured:
                    captured.append(frame)
                return
            self._undelayed_send(frame)

        transport._send = delay  # type: ignore[method-assign]
        wait_until(lambda: bool(captured), timeout=2)
        return captured[0]

    @rpc
    def resume_pose_packets(self) -> None:
        transport = native_transport(self.mock_pose, "mock/pose")
        transport._send = self._undelayed_send  # type: ignore[method-assign]


class RecordingConnection(FakeArm):
    pose: In[PoseStamped]
    other_pose: In[PoseStamped]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.received: list[tuple[str, str, str, float, float]] = []
        self.held: tuple[In[PoseStamped], CommandTicket] | None = None
        self.defer_pose = False
        self.watch_sequence = 0
        self.replay_seen = False

    @rpc
    def start(self) -> None:
        super().start()
        for port in (self.pose, self.other_pose):
            self.register_disposable(
                Disposable(
                    self.subscribe_native(
                        port,
                        partial(self.record, port),
                    )
                )
            )
        transport = native_transport(self.pose, "mock/pose")

        def observe(wire: Any) -> None:
            if wire[1] == self.watch_sequence:
                self.replay_seen = True

        # Test-only observation of the actual replay on the receiver's transport.
        self.register_disposable(Disposable(pZenohTransport.subscribe(transport, observe)))

    def record(self, port: In[PoseStamped], msg: PoseStamped, ticket: CommandTicket) -> None:
        if port is self.pose and self.defer_pose:
            self.held = port, ticket
            self.defer_pose = False
            return
        self.write_native(
            port,
            ticket,
            lambda: self.received.append(
                (port.name, type(msg).__name__, msg.frame_id, msg.ts, msg.x)
            ),
        )

    @rpc
    def evidence(self) -> tuple[int, list[Any], dict[str, float], int, bool, int]:
        transport = native_transport(self.pose, "mock/pose")
        return (
            os.getpid(),
            self.received[-10:],
            self.writes[-1] if self.writes else {},
            transport._seen,
            self.replay_seen,
            self.halts,
        )

    @rpc
    def pose_count(self) -> int:
        return sum(item[0] == "pose" for item in self.received)

    @rpc
    def watch_replay(self, sequence: int) -> int:
        self.watch_sequence = sequence
        return native_transport(self.pose, "mock/pose")._received

    @rpc
    def defer_one_pose(self) -> bool:
        if self.held is not None:
            return True
        self.defer_pose = True
        return False

    @rpc
    def retry_seen_ticket(self) -> bool:
        assert self.held is not None
        port, ticket = self.held
        return self.write_native(port, ticket, lambda: pytest.fail("revoked write ran"))


def assembly() -> tuple[Blueprint, FakeArmConfig]:
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
    blueprint = (
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
    return blueprint, declaration


def test_bad_native_binding_and_unknown_joint() -> None:
    blueprint, declaration = assembly()
    with pytest.raises(ValueError, match="typed Out"):
        wire_native_commands(
            blueprint.remappings([(RecordingConnection, "pose", "elsewhere")]),
            NativeCoordinator.name,
            RecordingConnection.name,
            declaration,
        )
    with pytest.raises(ValueError, match="typed Out"):
        wire_native_commands(
            blueprint.remappings([(RecordingConnection, "pose", "mock_position_command")]),
            NativeCoordinator.name,
            RecordingConnection.name,
            declaration,
        )
    declaration.group_commands["pose"] = frozenset({"unknown"})
    with pytest.raises(ValueError, match="movable model joints"):
        ConnectionSource(declaration, ["position_command"], ["joint_state"], 1.0)


def test_native_arbitration_and_remote_revoke() -> None:
    blueprint, declaration = assembly()
    blueprint = wire_native_commands(
        blueprint, NativeCoordinator.name, RecordingConnection.name, declaration
    )
    parsed = BlueprintConfigParser(blueprint).parse(
        environ={},
        overrides={"g": {"viewer": "none"}},
        global_overrides={"zenoh_scout_addr": os.environ["ZENOH_SCOUT_ADDR"]},
    )
    system = ModuleCoordinator.build(blueprint, parsed)
    wire = pZenohTransport[tuple[int, int, int, PoseStamped]]("mock_pose")
    try:
        cc, receiver = (
            system.get_instance(NativeCoordinator),
            system.get_instance(RecordingConnection),
        )
        wait_until(lambda: J3 in cc.get_joint_positions(), timeout=10)
        assert cc.scenario("native") not in {os.getpid(), receiver.evidence()[0]}
        assert receiver.evidence()[0] != os.getpid()
        expected = ("pose", "PoseStamped", "tool_base", 17.5, 0.25)
        wait_until(lambda: expected in receiver.evidence()[1], timeout=10)
        _, received, scalar, _, _, _ = receiver.evidence()
        assert set(received) == {expected}
        assert scalar == {f"{J3}/position": 0.6}

        # A high-priority scalar on J1 defeats the whole J1/J2 input; the low
        # scalar then drives BOTH J2 and J3. The other pose contender is absent.
        cc.scenario("scalar")
        wait_until(
            lambda: (
                receiver.evidence()[2]
                == {
                    f"{J1}/position": 0.8,
                    f"{J2}/position": 0.4,
                    f"{J3}/position": 0.6,
                }
            ),
            timeout=10,
        )
        cc.scenario("legacy")
        wait_until(
            lambda: receiver.evidence()[2]
            == {
                f"{J1}/position": 0.8,
                f"{J2}/position": 0.7,
                f"{J3}/position": 0.6,
            },
            timeout=10,
        )
        cc.scenario("invalid")
        wait_until(lambda: receiver.evidence()[2].get(f"{J2}/position") == 0.4, timeout=10)

        cc.scenario("native")
        wait_until(lambda: receiver.evidence()[2] == {f"{J3}/position": 0.6}, timeout=10)
        wait_until(receiver.defer_one_pose, timeout=2)
        old = cc.delay_pose_packets()
        assert receiver.watch_replay(old[1]) < old[1]
        cc.task_invoke("pose", "cancel")
        wait_until(lambda: receiver.evidence()[3] > old[0], timeout=10)
        accepted = receiver.pose_count()
        assert time.monotonic_ns() < min(old[2], old[1] + int(2e9))
        wire.broadcast(None, old)
        wait_until(lambda: receiver.evidence()[4], timeout=10)
        assert receiver.pose_count() == accepted
        assert not receiver.retry_seen_ticket()
        cc.resume_pose_packets()

        cc.set_estop(True)
        wait_until(lambda: receiver.status().deadman_fired, timeout=6)
        assert receiver.evidence()[-1] >= 1
        cc.set_estop(False)
        cc.scenario("native")
        wait_until(lambda: not receiver.status().deadman_fired, timeout=6)
        cc.stop()
        wait_until(lambda: receiver.status().deadman_fired, timeout=6)
        assert receiver.status().connected
    finally:
        wire.stop()
        system.stop()
