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
import threading
import time
from typing import Any

from pydantic import ValidationError
import pytest

from dimos.control._control_test_helpers import FAKE_ARM_JOINTS, FakeArm
from dimos.control.connection import connection_module
from dimos.control.connection.arm_config import (
    ArmConfigState,
    CommandResult,
    ConfigApplied,
    ConfigKeys,
    ConfigPatch,
    ConfigScope,
    JointGains,
)
from dimos.control.connection.connection_module import ConnectionStatus
from dimos.control.connection.mock_joint_connection import MockJointConnection
from dimos.control.contract.limits import Limits
from dimos.core.core import rpc
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.testing.cbor import roundtrip

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
    ("config", "match"),
    [
        ({"joint_commands": {}}, "told nothing"),
        ({"model": None}, "no model"),
    ],
)
def test_start_fails_before_connecting_on_a_bad_config(config: dict[str, Any], match: str) -> None:
    module = Watched(**config)
    try:
        with pytest.raises(ValueError, match=match):
            module.start()
        assert not module.connected
    finally:
        module.stop()


def test_the_limits_come_from_the_config_alone() -> None:
    module = FakeArm()
    try:
        table = module.limits
        assert "limits" not in FakeArm.rpcs
        assert table == {f"mock/joint{i}/position": Limits(-math.pi, math.pi) for i in range(1, 8)}
        assert module.limits is table
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


def test_steady_commands_keep_the_deadman_quiet(rig: Callable[..., Rig], mocker: Any) -> None:
    r = rig()
    loop = r.module._loop
    assert loop is not None
    # The timer's clock is advanced by hand, only after each command has
    # reached the hardware, so a slow test machine cannot miss a deadline.
    clock = mocker.patch.object(loop, "time", return_value=loop.time())
    for step in range(1, 7):
        armed = r.module._deadman
        r.move(joint1=step / 100)
        # Advance only once this command's timer is armed on the loop.
        assert wait_until(
            lambda a=armed: r.module._deadman is not None and r.module._deadman is not a
        )
        clock.return_value += DEADMAN_S * 0.75
        loop.call_soon_threadsafe(lambda: None)
        assert r.module.halts == 0
    clock.return_value += DEADMAN_S * 1.01
    loop.call_soon_threadsafe(lambda: None)
    assert wait_until(lambda: r.module.halts == 1)


def test_an_old_timer_firing_after_a_fresh_command_does_not_halt(rig: Callable[..., Rig]) -> None:
    """A command may be accepted on another thread just as the previous
    deadman timer is due; that timer must then stay quiet."""
    r = rig()
    r.move(joint1=0.1)
    assert wait_until(lambda: len(r.module.writes) == 1)
    r.module._deadman_expired(r.module._command_seq - 1)
    assert r.module.halts == 0
    assert not r.module.status().deadman_fired
    r.module._deadman_expired(r.module._command_seq)
    assert r.module.halts == 1
    assert r.module.status().deadman_fired


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
    r = rig()
    status = r.module.status()
    assert isinstance(status, ConnectionStatus)
    assert roundtrip(status) == status
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


# --- The arm config contract -------------------------------------------------

ARM = "fake/arm"
CONFIG_RPCS = {"config_keys", "config_get", "config_set", "config_reset", "config_command"}
DEFAULT_GAINS = JointGains(kp=50.0, kd=1.0)


class TunableArm(FakeArm):
    """A fake arm whose joint gains a config session can change, up to kp 100.

    Its stop waits for a calibration tool to park it, as a real driver's does.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.overrides: dict[str, JointGains] = {}
        self.fail_writes = False

    @rpc
    def stop(self) -> None:
        self._await_config_park()
        super().stop()

    def write(self, values: dict[str, float]) -> None:
        if self.fail_writes:
            raise RuntimeError("bus off")
        super().write(values)

    def _config_keys(self) -> ConfigKeys:
        return ConfigKeys(
            source="ignored",
            sessions=False,
            arms={ARM: list(FAKE_ARM_JOINTS)},
            readable=["gains"],
            session=["gains"],
        )

    def _config_state(self) -> dict[str, ArmConfigState]:
        gains = {joint: self.overrides.get(joint, DEFAULT_GAINS) for joint in FAKE_ARM_JOINTS}
        return {ARM: ArmConfigState(arm_id=ARM, gains=gains)}

    def _config_apply(
        self, patch: ConfigPatch, scope: ConfigScope, force: bool, apply_now: bool
    ) -> ConfigApplied:
        if patch.arm_id != ARM:
            raise ValueError(f"no arm {patch.arm_id!r}")
        if scope == "persist":
            return super()._config_apply(patch, scope, force, apply_now)
        unknown = sorted(set(patch.gains or {}) - set(FAKE_ARM_JOINTS))
        if unknown:
            raise ValueError(f"not joints of {ARM}: {unknown}")
        applied, clamped = {}, []
        for joint, gains in (patch.gains or {}).items():
            applied[joint] = JointGains(kp=min(gains.kp, 100.0), kd=gains.kd)
            if applied[joint].kp != gains.kp:
                clamped.append(f"gains.{joint}.kp")
        self.overrides.update(applied)
        rest = [key for key in patch.keys() if not key.startswith("gains.")]
        return ConfigApplied(
            arm_id=ARM,
            scope=scope,
            applied=ConfigPatch(arm_id=ARM, gains=applied or None),
            clamped=clamped,
            unsupported=rest,
            why=dict.fromkeys(rest, "gains only"),
        )

    def _config_undo(self) -> str:
        undone, self.overrides = sorted(self.overrides), {}
        return f"gains of {undone} back" if undone else "nothing to undo"


def _messages(spy: Any) -> list[str]:
    return [str(call.args[0]) for call in spy.call_args_list]


def _begin(module: FakeArm, **kwargs: Any) -> str:
    applied = module.config_set(ConfigPatch(arm_id=ARM), **kwargs)
    assert applied.session_id is not None
    return applied.session_id


def _live(module: FakeArm) -> bool:
    return module.status().config_session_id is not None


@pytest.fixture
def quick_park(mocker: Any) -> None:
    """No calibration tool parks in most tests; a stop need not wait a second for one."""
    mocker.patch.object(connection_module, "CONFIG_PARK_GRACE_S", 0.02)


def test_every_connection_serves_the_config_rpcs() -> None:
    assert CONFIG_RPCS <= set(FakeArm.rpcs)
    assert CONFIG_RPCS <= set(MockJointConnection.rpcs)


def test_without_config_sessions_only_the_reads_answer(rig: Callable[..., Rig]) -> None:
    r = rig()
    assert not r.module.config.config_sessions
    assert r.module.config_keys() == ConfigKeys(source="mock", sessions=False)
    assert r.module.config_get() == {}
    for call in (
        lambda: r.module.config_set(ConfigPatch(arm_id=ARM)),
        lambda: r.module.config_reset(),
        lambda: r.module.config_command({"joint1": 0.1}, "x"),
    ):
        with pytest.raises(RuntimeError, match="config sessions are off"):
            call()
    assert writes(r) == [] and not _live(r.module)


def test_a_driver_without_hooks_can_change_nothing(rig: Callable[..., Rig]) -> None:
    r = rig(config_sessions=True)
    patch = ConfigPatch(arm_id=ARM, gains={"joint1": JointGains(kp=60.0, kd=2.0)})
    saved = r.module.config_set(patch, scope="persist")
    assert saved.unsupported == patch.keys() and not saved.applied_now
    assert saved.applied == ConfigPatch(arm_id=ARM) and saved.session_id is None
    assert not _live(r.module)
    live = r.module.config_set(patch)
    assert live.unsupported == patch.keys() and live.session_id is not None
    assert r.module.config_keys().arms == {} and r.module.config_get() == {}


def test_a_config_session_round_trip(rig: Callable[..., Rig], quick_park: None) -> None:
    r = rig(TunableArm, config_sessions=True, deadman_timeout_s=10)
    keys = r.module.config_keys()
    assert keys.source == "mock" and keys.sessions and keys.max_lease_s == 30.0
    assert keys.arms == {ARM: list(FAKE_ARM_JOINTS)}

    started = r.module.config_set(ConfigPatch(arm_id=ARM))
    session = started.session_id
    assert session is not None and len(session) == 32 and started.lease_s == 2.0
    assert r.module.status().config_session_id == session

    patch = ConfigPatch(
        arm_id=ARM, gains={"joint2": JointGains(kp=900.0, kd=2.0)}, notes="not live"
    )
    applied = r.module.config_set(patch, session_id=session)
    assert applied.applied.gains == {"joint2": JointGains(kp=100.0, kd=2.0)}
    assert applied.clamped == ["gains.joint2.kp"] and applied.unsupported == ["notes"]
    assert applied.session_id == session
    state = r.module.config_get()[ARM]
    assert state.gains["joint2"].kp == 100.0 and state.session_id == session

    assert r.module.config_command({"joint1": 0.3}, session) == CommandResult(ok=True)
    assert writes(r) == [{JOINT1: 0.3}]
    assert r.module.status().last_command_time is not None

    r.module.config_reset()
    assert not _live(r.module) and r.module.overrides == {}
    assert r.module.config_get()[ARM].session_id is None
    # Without a session, a reset does nothing.
    r.module.config_reset()


def test_a_reset_that_keeps_the_session_undoes_its_changes(rig: Callable[..., Rig]) -> None:
    r = rig(TunableArm, config_sessions=True, deadman_timeout_s=10)
    session = _begin(r.module)
    gains = {"joint1": JointGains(kp=60.0, kd=2.0)}
    r.module.config_set(ConfigPatch(arm_id=ARM, gains=gains), session_id=session)
    r.module.config_reset(keep_session=True)
    assert r.module.overrides == {} and r.module.status().config_session_id == session
    r.module.config_reset()


def test_a_live_session_needs_its_id_and_a_wrong_one_ends_it(rig: Callable[..., Rig]) -> None:
    r = rig(TunableArm, config_sessions=True, deadman_timeout_s=10)
    gains = ConfigPatch(arm_id=ARM, gains={"joint1": JointGains(kp=60.0, kd=2.0)})
    session = _begin(r.module)
    r.module.config_set(gains, session_id=session)
    with pytest.raises(RuntimeError, match="a config session is live; pass the session_id"):
        r.module.config_set(gains)
    # Like any config call that raises, that ended the session and undid it.
    assert not _live(r.module) and r.module.overrides == {}
    with pytest.raises(RuntimeError, match="the config session ended \\(config_set raised"):
        r.module.config_command({"joint1": 0.1}, session)

    session = _begin(r.module)
    with pytest.raises(RuntimeError, match="is not the live config session"):
        r.module.config_command({"joint1": 0.1}, "0" * 32)
    assert not _live(r.module)
    with pytest.raises(RuntimeError, match="needs the session_id"):
        r.module.config_command({"joint1": 0.1}, None)  # type: ignore[arg-type]
    assert writes(r) == []


def test_persisting_needs_no_session(rig: Callable[..., Rig]) -> None:
    r = rig(TunableArm, config_sessions=True)
    saved = r.module.config_set(ConfigPatch(arm_id=ARM, notes="x"), scope="persist")
    assert saved.session_id is None and not _live(r.module)


@pytest.mark.parametrize(
    "lease,error",
    [
        (0.0, ValueError),
        (-1.0, ValueError),
        (float("nan"), ValueError),
        (30.5, ValueError),
        ("2", TypeError),
        (True, TypeError),
    ],
)
def test_a_lease_is_a_positive_number_of_seconds_up_to_thirty(
    rig: Callable[..., Rig], lease: Any, error: type[Exception]
) -> None:
    r = rig(TunableArm, config_sessions=True)
    with pytest.raises(error, match="lease_s"):
        r.module.config_set(ConfigPatch(arm_id=ARM), lease_s=lease)
    assert not _live(r.module)


def test_a_lapsed_lease_undoes_every_live_change_and_logs_one_line(
    rig: Callable[..., Rig], mocker: Any
) -> None:
    r = rig(TunableArm, config_sessions=True, deadman_timeout_s=10)
    warning = mocker.spy(connection_module.logger, "warning")
    session = _begin(r.module, lease_s=0.2)
    gains = {"joint2": JointGains(kp=40.0, kd=2.0)}
    r.module.config_set(ConfigPatch(arm_id=ARM, gains=gains), session_id=session)
    assert wait_until(lambda: not _live(r.module))
    assert r.module.overrides == {}
    lines = [line for line in _messages(warning) if "ended" in line]
    assert len(lines) == 1
    assert "no config call within its lease of 0.2 s" in lines[0]
    assert "gains of ['joint2'] back" in lines[0]
    with pytest.raises(RuntimeError, match="ended \\(no config call within its lease"):
        r.module.config_set(ConfigPatch(arm_id=ARM), session_id=session)


def test_every_config_call_renews_the_lease(rig: Callable[..., Rig]) -> None:
    r = rig(TunableArm, config_sessions=True, deadman_timeout_s=10)
    session = _begin(r.module, lease_s=0.3)
    calls = [r.module.config_get, r.module.config_keys]
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        calls[0]()
        calls.reverse()
        time.sleep(0.05)
    assert _live(r.module)
    r.module.config_command({"joint1": 0.1}, session)
    r.module.config_reset()


def test_a_config_session_drops_position_commands_and_says_so_once(
    rig: Callable[..., Rig], mocker: Any
) -> None:
    r = rig(TunableArm, config_sessions=True, deadman_timeout_s=10)
    warning = mocker.spy(connection_module.logger, "warning")
    info = mocker.spy(connection_module.logger, "info")
    r.move(joint1=0.5)
    _begin(r.module)
    for target in (0.6, 0.7, 0.8):
        r.move(joint1=target)
    assert writes(r) == [{JOINT1: 0.5}]
    assert len([line for line in _messages(warning) if "dropping position commands" in line]) == 1
    r.module.config_reset()
    ended = [line for line in _messages(info) if "ended (config_reset)" in line]
    assert len(ended) == 1 and "3 position commands dropped" in ended[0]
    r.move(joint1=0.9)
    assert writes(r)[-1] == {JOINT1: 0.9}


def test_a_command_checked_just_before_a_session_began_is_still_dropped(
    rig: Callable[..., Rig], mocker: Any
) -> None:
    r = rig(TunableArm, config_sessions=True, deadman_timeout_s=10)
    check = connection_module.validate_command

    def session_begins_meanwhile(*args: Any) -> Any:
        result = check(*args)
        if not _live(r.module):
            _begin(r.module)
        return result

    mocker.patch.object(connection_module, "validate_command", session_begins_meanwhile)
    r.move(joint1=0.5)
    assert writes(r) == [] and _live(r.module)
    r.module.config_reset()


def test_the_deadman_halting_the_arm_ends_the_session(rig: Callable[..., Rig]) -> None:
    r = rig(TunableArm, config_sessions=True)
    session = _begin(r.module)
    assert r.module.config_command({"joint1": 0.3}, session).ok
    # No more targets: the deadman halts the arm, and a halt ends the session.
    assert wait_until(lambda: r.module.status().deadman_fired)
    assert not _live(r.module) and r.module.halts == 1
    with pytest.raises(RuntimeError, match="the config session ended \\(the connection halted\\)"):
        r.module.config_command({"joint1": 0.3}, session)


def test_a_command_the_hardware_did_not_take_is_not_ok_and_ends_the_session(
    rig: Callable[..., Rig],
) -> None:
    r = rig(TunableArm, config_sessions=True, deadman_timeout_s=10)
    session = _begin(r.module)
    r.module.fail_writes = True
    result = r.module.config_command({"joint1": 0.3}, session)
    assert not result.ok and result.error is not None and "bus off" in result.error
    assert "the config session ended" in result.error
    assert not _live(r.module) and r.module.halts == 1


def test_bad_arguments_are_refused_and_end_the_session(rig: Callable[..., Rig]) -> None:
    r = rig(TunableArm, config_sessions=True, deadman_timeout_s=10)
    gains = {"joint3": JointGains(kp=60.0, kd=2.0)}
    for call, error, match in (
        (lambda s: r.module.config_command({"joint1": 4.0}, s), ValueError, "limit"),
        (lambda s: r.module.config_command({"nope": 0.0}, s), ValueError, "not position-"),
        (lambda s: r.module.config_command({"joint1": "0.1"}, s), TypeError, "must be a number"),
        (lambda s: r.module.config_command([0.1], s), TypeError, "targets must be a dict"),
        (
            lambda s: r.module.config_set(
                {"arm_id": ARM, "gains": {"joint1": {"kp": "80", "kd": 1.0}}}, session_id=s
            ),
            ValidationError,
            "valid number",
        ),
        (
            lambda s: r.module.config_set(
                ConfigPatch(arm_id=ARM, gains={"nope": JointGains(kp=1.0, kd=1.0)}), session_id=s
            ),
            ValueError,
            "not joints",
        ),
        (lambda s: r.module.config_set(ConfigPatch(arm_id="x"), session_id=s), ValueError, "arm"),
        (lambda s: r.module.config_set(ARM, session_id=s), TypeError, "ConfigPatch"),
        (
            lambda s: r.module.config_set(ConfigPatch(arm_id=ARM), "forever", session_id=s),
            ValueError,
            "scope",
        ),
        (
            lambda s: r.module.config_set(ConfigPatch(arm_id=ARM), force="yes", session_id=s),
            TypeError,
            "True or False",
        ),
        (lambda s: r.module.config_reset(keep_session=0), TypeError, "True or False"),
    ):
        session = _begin(r.module)
        r.module.config_set(ConfigPatch(arm_id=ARM, gains=gains), session_id=session)
        with pytest.raises(error, match=match):
            call(session)
        assert not _live(r.module) and r.module.overrides == {}
    assert writes(r) == []


def test_stopping_ends_the_session_and_undoes_it(
    rig: Callable[..., Rig], mocker: Any, quick_park: None
) -> None:
    mocker.patch.object(connection_module, "CONFIG_PARK_WAIT_S", 0.1)
    r = rig(TunableArm, config_sessions=True, deadman_timeout_s=10)
    session = _begin(r.module)
    gains = {"joint1": JointGains(kp=60.0, kd=2.0)}
    r.module.config_set(ConfigPatch(arm_id=ARM, gains=gains), session_id=session)
    r.module.stop()
    assert not _live(r.module) and r.module.overrides == {}
    assert r.module._config_ended == "the connection halted"
    with pytest.raises(RuntimeError, match="not running"):
        r.module.config_set(ConfigPatch(arm_id=ARM))


def _park_while_stopping(module: TunableArm, order: list[str], stream_s: float) -> threading.Thread:
    """Stop ``module`` on a thread while this one parks, as a calibration tool does."""

    def stop() -> None:
        module.stop()
        order.append("stopped")

    stopper = threading.Thread(target=stop)
    stopper.start()
    time.sleep(0.05)
    session = _begin(module)
    order.append("park began")
    deadline = time.monotonic() + stream_s
    try:
        while time.monotonic() < deadline:
            module.config_command({"joint1": 0.2}, session)
            time.sleep(0.02)
    except RuntimeError:
        order.append("cut off")
    else:
        order.append("parked")
        module.config_reset()
    return stopper


def test_stopping_waits_for_the_calibration_tool_to_park(
    rig: Callable[..., Rig], mocker: Any
) -> None:
    mocker.patch.object(connection_module, "CONFIG_PARK_GRACE_S", 0.3)
    r = rig(TunableArm, config_sessions=True, deadman_timeout_s=10)
    order: list[str] = []
    halt = r.module.halt

    def halted() -> None:
        order.append("halted")
        halt()

    mocker.patch.object(r.module, "halt", halted)
    warning = mocker.spy(connection_module.logger, "warning")

    stopper = _park_while_stopping(r.module, order, stream_s=0.5)
    stopper.join(timeout=5.0)

    assert order == ["park began", "parked", "halted", "stopped"]
    assert any(
        "waiting up to 10 s for the calibration tool to park the arm" in line
        for line in _messages(warning)
    )


def test_a_park_that_overruns_the_wait_is_cut_off(rig: Callable[..., Rig], mocker: Any) -> None:
    mocker.patch.object(connection_module, "CONFIG_PARK_GRACE_S", 0.2)
    mocker.patch.object(connection_module, "CONFIG_PARK_WAIT_S", 0.3)
    r = rig(TunableArm, config_sessions=True, deadman_timeout_s=10)
    error = mocker.spy(connection_module.logger, "error")
    order: list[str] = []

    stopper = _park_while_stopping(r.module, order, stream_s=2.0)
    stopper.join(timeout=5.0)

    assert order == ["park began", "stopped", "cut off"]
    assert any("did not end its config session within 0.3 s" in line for line in _messages(error))
    assert r.module._config_ended == "the connection halted"


def test_a_stop_without_config_sessions_never_waits(rig: Callable[..., Rig], mocker: Any) -> None:
    mocker.patch.object(connection_module, "CONFIG_PARK_GRACE_S", 5.0)
    r = rig(TunableArm)
    began = time.monotonic()
    r.module.stop()
    assert time.monotonic() - began < 1.0


def test_config_results_cross_processes(rig: Callable[..., Rig]) -> None:
    r = rig(TunableArm, config_sessions=True, deadman_timeout_s=10)
    session = _begin(r.module)
    gains = {"joint1": JointGains(kp=60.0, kd=2.0)}
    results = [
        r.module.config_keys(),
        r.module.config_get(),
        r.module.config_set(ConfigPatch(arm_id=ARM, gains=gains), session_id=session),
        r.module.config_command({"joint1": 0.1}, session),
        r.module.status(),
    ]
    for result in results:
        assert roundtrip(result) == result
    # A patch arrives rebuilt without validation; the connection validates it again.
    bad = roundtrip(ConfigPatch.model_construct(arm_id=ARM, gains={"joint1": {"kp": -1.0}}))
    with pytest.raises(ValidationError):
        r.module.config_set(bad, session_id=session)
