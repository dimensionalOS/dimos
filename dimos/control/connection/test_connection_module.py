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

"""Tests for ``ConnectionModule``, driven through ``FakeArm``.

Real modules with real threads, no coordinator. Every port is given an
in-process transport, so nothing touches the network.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
import math
import pickle
import threading
import time
from typing import Any

import pytest

from dimos.control._control_test_helpers import FakeArm
from dimos.control.connection.connection_module import ConnectionStatus
from dimos.control.contract.description import Limits
from dimos.control.contract.keys import Interface
from dimos.control.contract.validate import DescriptionError
from dimos.msgs.sensor_msgs.JointState import JointState

JOINT1 = "mock/joint1/position"
DEADMAN_S = 0.05


class LocalTransport:
    """An in-process transport: publishing calls every subscriber at once."""

    def __init__(self) -> None:
        self.sent: list[Any] = []
        self.subscribers: list[Callable[[Any], Any]] = []

    def broadcast(self, selfstream: Any, value: Any) -> None:
        self.sent.append(value)
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


class Rig:
    """One module, with an in-process transport on every port."""

    def __init__(self, module: FakeArm) -> None:
        self.module = module
        self.ports = {name: LocalTransport() for name in {**module.inputs, **module.outputs}}
        for name, transport in self.ports.items():
            module.set_transport(name, transport)  # type: ignore[arg-type]

    def send(self, port: str, msg: Any) -> None:
        self.ports[port].publish(msg)

    def move(self, **positions: float) -> None:
        """Send joint positions on ``position_command``, e.g. ``move(joint1=0.5)``."""
        names = [f"{self.module.config.source}/{joint}" for joint in positions]
        self.send("position_command", JointState(name=names, position=list(positions.values())))

    def sent(self, port: str) -> list[Any]:
        return list(self.ports[port].sent)


@pytest.fixture
def rig() -> Iterator[Callable[..., Rig]]:
    """Build modules, start them, and stop every one at the end of the test."""
    made: list[Rig] = []

    def build(cls: type[FakeArm] = FakeArm, **config: Any) -> Rig:
        config.setdefault("deadman_timeout_s", DEADMAN_S)
        made.append(Rig(cls(**config)))
        made[-1].module.start()
        return made[-1]

    yield build
    for each in made:
        each.module.stop()


def wait_until(condition: Callable[[], bool], timeout_s: float = 2.0) -> bool:
    deadline = time.monotonic() + timeout_s
    while not condition():
        if time.monotonic() > deadline:
            return False
        time.sleep(0.005)
    return True


def writes(r: Rig) -> list[dict[str, float]]:
    return list(r.module.writes)


class FailingWrite(FakeArm):
    def write(self, values: dict[str, float]) -> None:
        raise RuntimeError("motor said no")


class Watched(FakeArm):
    """Remembers whether ``connect`` was called."""

    connected = False

    def connect(self) -> None:
        self.connected = True
        super().connect()


@pytest.mark.parametrize(
    ("config", "error", "match"),
    [
        ({"state_rate_hz": 0.0}, DescriptionError, "state_rate_hz"),
        ({"model": None}, ValueError, "'position_command' is a JointSpec, which needs"),
    ],
)
def test_start_fails_before_connecting_on_a_bad_config(
    config: dict[str, Any], error: type[Exception], match: str
) -> None:
    module = Watched(**config)
    try:
        with pytest.raises(error, match=match):
            module.start()
        assert not module.connected
    finally:
        module.stop()


def test_the_description_comes_from_the_config_alone() -> None:
    module = FakeArm()
    try:
        described = module.description
        assert "description" not in FakeArm.rpcs
        assert described.source == "mock"
        assert described.joint_names() == tuple(f"mock/joint{i}" for i in range(1, 8))
        joint = described.resource("joint1")
        assert joint is not None
        assert joint.command_interfaces == (Interface.POSITION,)
        assert joint.state_interfaces == (
            Interface.POSITION,
            Interface.VELOCITY,
            Interface.EFFORT,
        )
        assert described.limits[JOINT1] == Limits(-math.pi, math.pi)
        assert module.description is described
    finally:
        module.stop()


def test_a_position_command_is_written_and_the_readings_follow(rig: Callable[..., Rig]) -> None:
    r = rig()
    r.move(joint1=0.5)
    assert writes(r) == [{JOINT1: 0.5}]
    assert r.module.status().last_command_time is not None
    assert wait_until(lambda: any(m.position[0] == 0.5 for m in r.sent("joint_state")))
    msg = r.sent("joint_state")[-1]
    assert isinstance(msg, JointState)
    assert msg.name == [f"mock/joint{i}" for i in range(1, 8)]
    assert msg.velocity == msg.effort == [0.0] * 7


def test_a_command_from_another_thread_is_written(rig: Callable[..., Rig]) -> None:
    r = rig()
    sender = threading.Thread(target=r.move, kwargs={"joint1": 0.3})
    sender.start()
    sender.join(2.0)
    assert writes(r) == [{JOINT1: 0.3}]
    assert wait_until(lambda: r.module.halts == 1)


def test_a_command_past_its_limit_is_refused_and_shown(rig: Callable[..., Rig]) -> None:
    r = rig()
    r.move(joint1=4.0)
    assert "limit" in str(r.module.status().last_rejection)
    assert writes(r) == []
    assert r.module.status().last_command_time is None


def test_an_unknown_key_is_refused_and_shown(rig: Callable[..., Rig]) -> None:
    r = rig()
    r.module.command({"mock/joint1/velocity": 0.1})
    assert "unknown_key" in str(r.module.status().last_rejection)
    r.module.command({"mock/joint1/position": math.nan})
    assert "non-finite" in str(r.module.status().last_rejection)
    assert writes(r) == []


def test_keys_of_other_robots_are_skipped_and_feed_nothing(rig: Callable[..., Rig]) -> None:
    r = rig()
    r.module.command({"other/joint1/position": 9.0})
    assert writes(r) == []
    assert r.module.status().last_rejection is None
    assert r.module.status().last_command_time is None
    r.send("position_command", JointState(name=["other/joint1", "mock/joint1"], position=[9, 0.5]))
    assert writes(r) == [{JOINT1: 0.5}]


def test_the_deadman_waits_for_the_first_command(rig: Callable[..., Rig]) -> None:
    r = rig()
    time.sleep(4 * DEADMAN_S)
    assert r.module.halts == 0
    assert not r.module.status().deadman_fired


def test_the_deadman_halts_once_then_starts_again_on_the_next_command(
    rig: Callable[..., Rig],
) -> None:
    r = rig()
    r.move(joint1=0.1)
    assert wait_until(lambda: r.module.halts == 1)
    assert r.module.status().deadman_fired
    time.sleep(4 * DEADMAN_S)
    assert r.module.halts == 1

    r.move(joint1=0.2)
    assert len(writes(r)) == 2
    assert not r.module.status().deadman_fired
    assert wait_until(lambda: r.module.halts == 2)


def test_steady_commands_keep_the_deadman_quiet(rig: Callable[..., Rig]) -> None:
    r = rig()
    for step in range(10):
        r.move(joint1=step / 100)
        time.sleep(DEADMAN_S / 4)
    assert r.module.halts == 0
    assert wait_until(lambda: r.module.halts == 1)


def test_a_failing_write_halts_shows_the_error_and_carries_on(
    rig: Callable[..., Rig],
) -> None:
    r = rig(FailingWrite)
    r.move(joint1=0.1)
    assert r.module.halts == 1
    status = r.module.status()
    assert "motor said no" in str(status.last_error)
    assert status.connected
    # The failed write ended the deadman, so it does not halt a second time.
    time.sleep(4 * DEADMAN_S)
    assert r.module.halts == 1
    r.move(joint1=0.2)
    assert r.module.halts == 2


def test_stopping_halts_once_and_a_second_stop_is_harmless(rig: Callable[..., Rig]) -> None:
    r = rig()
    r.module.stop()
    r.module.stop()
    assert r.module.halts == 1
    assert not r.module.status().connected
    r.move(joint1=0.1)
    assert writes(r) == []


def test_frame_takes_the_config_prefix(rig: Callable[..., Rig]) -> None:
    assert rig().module.frame("odom") == "odom"
    assert rig(frame_id_prefix="go2b").module.frame("odom") == "go2b/odom"


def test_status_is_an_rpc_that_crosses_processes(rig: Callable[..., Rig]) -> None:
    assert "status" in FakeArm.rpcs
    assert "describe_control" not in FakeArm.rpcs
    r = rig()
    status = r.module.status()
    assert isinstance(status, ConnectionStatus)
    assert pickle.loads(pickle.dumps(status)) == status
    assert len(status.session_id) == 32
    assert status.connected
    assert status.last_command_time is None
    assert status.deadman_fired is False
    assert status.last_rejection is None
    assert status.last_error is None


def test_the_session_id_is_new_each_start_and_steady_in_between(
    rig: Callable[..., Rig],
) -> None:
    first = rig().module
    session = first.status().session_id
    assert first.status().session_id == session
    first.stop()
    assert rig().module.status().session_id != session
