# Copyright 2026 Dimensional Inc.
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

"""Unit tests for Go2CommandModule's operator-command handling.

No robot or signaling: real modules exercise the command plane with only the
external driver/provider and publication boundaries mocked. Covers the safety-relevant paths — sport
allow-list, E-STOP latch + fence, nonce dedup, and the drive guard
(stale/future/reorder).
"""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import ExitStack
from copy import deepcopy
from threading import Event
import time
from typing import Any
from unittest.mock import MagicMock

from dimos_cdrgen.registry import encode
import pytest

from dimos.core.cdr_transport import CdrTransport
from dimos.core.transport import WebRTCTransport
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.geometry_msgs.msg import Twist, TwistStamped, Vector3
from dimos.messages.std_msgs.msg import Header
from dimos.msgs.time import time_from_seconds, to_nanoseconds
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.protocol.pubsub.impl.webrtc.providers.broker import BrokerConfig
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.teleop.hosted.blueprints.cloudflare import (
    teleop_hosted_go2_multicam,
    teleop_hosted_go2_transport,
)
from dimos.teleop.hosted.go2_command import ALLOWED_SPORT_CMDS, Go2CommandModule
from dimos.utils.testing.waiting import wait_until
from dimos.web.relay_bridge.protocol import FrameHeader, encode_data_frame


@pytest.fixture
def module(mocker) -> Iterator[Go2CommandModule]:
    """Real module and config; only publication and the external driver are mocked."""
    instance = Go2CommandModule()
    mocker.patch.object(instance, "go2", MagicMock(), create=True)
    for port in ("cmd_ack", "tele_cmd_vel", "robot_state", "goal_request", "stop_movement"):
        mocker.patch.object(getattr(instance, port), "publish")
    instance._cmd.start()
    yield instance
    instance.stop()


def _twist(ts: float, *, vx: float = 0.3) -> TwistStamped:
    """A drive frame at time ``ts`` (vx=0.3 moving, vx=0 idle-joystick)."""
    t = TwistStamped(
        header=Header(stamp=time_from_seconds(ts), frame_id="operator"),
        twist=Twist(linear=Vector3(vx, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0)),
    )
    return t


# ─── sport allow-list (RPC to driver) ────────────────────────────────

_NON_ACROBATIC = [n for n in ALLOWED_SPORT_CMDS if n not in ("FrontJump", "FrontPounce")]


@pytest.mark.parametrize("name", _NON_ACROBATIC)
def test_allowed_sport_cmd_calls_driver_rpc(
    module: Go2CommandModule, name: str, monkeypatch: pytest.MonkeyPatch
) -> None:
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(module, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))
    module.go2.sport_command.return_value = True

    module._handle_sport_cmd({"name": name, "nonce": 7})
    wait_until(lambda: bool(acks), timeout=2.0)

    module.go2.sport_command.assert_called_once_with(ALLOWED_SPORT_CMDS[name])
    assert acks == [(7, True)]


@pytest.mark.parametrize("name", ["Backflip", "", None, 1013])
def test_disallowed_sport_cmd_rejected(
    module: Go2CommandModule, name: Any, monkeypatch: pytest.MonkeyPatch
) -> None:
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(module, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    module._handle_sport_cmd({"name": name, "nonce": 9})

    module.go2.sport_command.assert_not_called()
    assert acks == [(9, False)]


@pytest.mark.parametrize("name", ["FrontJump", "FrontPounce"])
def test_acrobatics_blocked_by_default(
    module: Go2CommandModule, name: str, monkeypatch: pytest.MonkeyPatch
) -> None:
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(module, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    module._handle_sport_cmd({"name": name, "nonce": 3})

    assert acks == [(3, False)]
    module.go2.sport_command.assert_not_called()


# ─── drive guard (stream filter → tele_cmd_vel) ───────────────────────


def test_drive_drops_stale(module: Go2CommandModule) -> None:
    module._on_cmd_vel_in(_twist(time.time() - 1.0))
    module.tele_cmd_vel.publish.assert_not_called()


def test_drive_drops_future(module: Go2CommandModule) -> None:
    module._on_cmd_vel_in(_twist(time.time() + 5.0))
    module.tele_cmd_vel.publish.assert_not_called()
    assert module._last_cmd_ns == 0.0  # future stamp must not poison the guard


def test_drive_drops_in_window_future_without_poisoning_guard(module: Go2CommandModule) -> None:
    # A future stamp SMALLER than cmd_stale_after_sec (clock skew) must still be
    # rejected and must NOT advance _last_cmd_ns — otherwise every subsequent
    # in-order frame would be dropped as out-of-order until wall-clock catches
    # up, stalling drive. Regression guard for the in-window future case.
    module._on_cmd_vel_in(_twist(time.time() + 0.2))  # +0.2s < 0.5s stale window
    module.tele_cmd_vel.publish.assert_not_called()
    assert module._last_cmd_ns == 0.0
    # a normal fresh frame right after must still be forwarded
    module._on_cmd_vel_in(_twist(time.time()))
    module.tele_cmd_vel.publish.assert_called_once()


def test_drive_drops_out_of_order(module: Go2CommandModule) -> None:
    ts = time.time()
    module._last_cmd_ns = to_nanoseconds(time_from_seconds(ts))
    module._on_cmd_vel_in(_twist(ts - 0.1))
    module.tele_cmd_vel.publish.assert_not_called()


def test_drive_forwards_fresh(module: Go2CommandModule) -> None:
    ts = time.time()
    module._on_cmd_vel_in(_twist(ts))
    module.tele_cmd_vel.publish.assert_called_once()
    assert module._last_cmd_ns == to_nanoseconds(time_from_seconds(ts))


def test_drive_suppresses_idle_zero_stream(module: Go2CommandModule) -> None:
    # Idle-joystick zeros must NOT be forwarded — MovementManager treats any
    # tele_cmd_vel as active manual drive and would cancel the nav plan.
    # Stamps in the recent past (transit delay), monotonically increasing.
    base = time.time() - 0.1
    module._on_cmd_vel_in(_twist(base, vx=0.0))
    module._on_cmd_vel_in(_twist(base + 0.01, vx=0.0))
    module.tele_cmd_vel.publish.assert_not_called()


def test_drive_forwards_release_edge_zero(module: Go2CommandModule) -> None:
    # A zero right after a moving frame IS forwarded (manual stop), then the
    # idle stream goes quiet again. Stamps in the recent past, increasing.
    base = time.time() - 0.1
    module._on_cmd_vel_in(_twist(base, vx=0.3))  # moving → forwarded
    module._on_cmd_vel_in(_twist(base + 0.01, vx=0.0))  # release edge → forwarded (stop)
    module._on_cmd_vel_in(_twist(base + 0.02, vx=0.0))  # idle → suppressed
    assert module.tele_cmd_vel.publish.call_count == 2


def test_estopped_drive_is_dropped(module: Go2CommandModule) -> None:
    module._estopped = True
    module._on_cmd_vel_in(_twist(time.time()))
    module.tele_cmd_vel.publish.assert_not_called()


def test_drive_drops_nan_timestamp(module: Go2CommandModule) -> None:
    # A NaN ts passes every comparison and would poison _last_cmd_ns (ts <= NaN
    # is False forever → reorder guard permanently disabled). Must be rejected.
    frame = _twist(time.time())
    frame.header.stamp.sec = float("nan")  # malformed direct call; impossible on the CDR wire
    module._on_cmd_vel_in(frame)
    module.tele_cmd_vel.publish.assert_not_called()
    assert module._last_cmd_ns == 0.0  # guard not poisoned


def test_drive_drops_non_finite_velocity(module: Go2CommandModule) -> None:
    t = _twist(time.time())
    t.twist.linear.x = float("inf")
    module._on_cmd_vel_in(t)
    module.tele_cmd_vel.publish.assert_not_called()


def test_drive_clamps_excessive_velocity(module: Go2CommandModule) -> None:
    # An untrusted operator sending huge velocities is clamped to the envelope.
    t = _twist(time.time())
    t.twist.linear.x = 99.0  # way over max_linear_mps=1.5
    t.twist.angular.z = -50.0  # way under -max_angular_rps=2.0
    module._on_cmd_vel_in(t)
    published = module.tele_cmd_vel.publish.call_args[0][0]
    assert published.linear.x == 1.5
    assert published.angular.z == -2.0


# ─── E-STOP + fence ──────────────────────────────────────────────────


def test_estop_latches_and_damps(module: Go2CommandModule, monkeypatch: pytest.MonkeyPatch) -> None:
    module.go2.sport_command.return_value = True
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(module, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    module._on_state_json(b'{"type": "estop", "nonce": 1}')
    assert module._estopped is True
    module.stop_movement.publish.assert_called_once()  # nav cancelled
    # robot_state published immediately on latch (not only inside the Damp task),
    # so the UI shows estopped:true even if Damp is slow/fails.
    module.robot_state.publish.assert_called()
    wait_until(lambda: (1, True) in acks, timeout=2.0)
    module.go2.sport_command.assert_called_with(ALLOWED_SPORT_CMDS["Damp"])


def test_estop_clear_publishes_state(
    module: Go2CommandModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    # Clearing E-STOP must publish robot_state immediately so the UI drops
    # estopped:true without waiting for an unrelated update.
    module._estopped = True
    monkeypatch.setattr(module, "_send_ack", lambda nonce, ok: None)
    module.robot_state.publish.reset_mock()
    module._on_state_json(b'{"type": "estop_clear", "nonce": 2}')
    assert module._estopped is False
    module.robot_state.publish.assert_called_once()


def test_repeated_estop_reissues_damp(
    module: Go2CommandModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    module.go2.sport_command.return_value = True
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(module, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    module._on_state_json(b'{"type": "estop", "nonce": 1}')
    wait_until(lambda: module.go2.sport_command.call_count == 1, timeout=2.0)
    module._on_state_json(b'{"type": "estop", "nonce": 1}')  # retransmit, same nonce
    wait_until(lambda: module.go2.sport_command.call_count == 2, timeout=2.0)

    assert acks.count((1, True)) == 2


def test_estop_clear_cancels_plan_and_rearms(
    module: Go2CommandModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    module._estopped = True
    monkeypatch.setattr(module, "_send_ack", lambda nonce, ok: None)

    module._on_state_json(b'{"type": "estop_clear"}')

    assert module._estopped is False
    module.stop_movement.publish.assert_called_once()  # active plan cancelled


def test_nav_cancel_stops_planner(
    module: Go2CommandModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(module, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    module._on_state_json(b'{"type": "nav_cancel", "nonce": 22}')

    (msg,) = module.stop_movement.publish.call_args.args
    assert msg.data is True
    assert acks == [(22, True)]


def test_stand_ready_aborts_on_mid_sequence_estop(
    module: Go2CommandModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    module._posture = "Sit"
    module.go2.standup.return_value = True
    module.go2.sport_command.return_value = True
    module.go2.balance_stand.return_value = True
    module.go2.switch_joystick.return_value = True

    def fake_sleep(_s: float) -> None:
        if not module._estopped:
            module._estopped = True
            module._cmd.bump_safety_epoch()

    monkeypatch.setattr(time, "sleep", fake_sleep)

    assert module._stand_ready_task(module._cmd.safety_epoch) is False
    module.go2.standup.assert_called_once()
    module.go2.balance_stand.assert_not_called()
    module.go2.switch_joystick.assert_not_called()
    assert module._posture == "Sit"


# ─── operator-lost ───────────────────────────────────────────────────


def test_operator_lost_stops_movement_and_fences(
    module: Go2CommandModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(module, "_send_ack", lambda nonce, ok: None)
    epoch_before = module._cmd.safety_epoch
    module._cmd._nonce_results["stale"] = (True, 0.0)

    module._on_state_json(b'{"type": "operator_lost"}')

    assert module._cmd.safety_epoch == epoch_before + 1  # queued cmds fenced
    assert not module._cmd._nonce_results  # reconnect must not re-ack old nonces
    module.stop_movement.publish.assert_called_once()  # nav plan cancelled
    wait_until(lambda: module.go2.stop_movement.called, timeout=2.0)
    module.go2.sport_command.assert_not_called()  # damp_on_operator_lost=False


def test_operator_lost_damps_when_configured(
    module: Go2CommandModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    module.config.damp_on_operator_lost = True
    module.go2.sport_command.return_value = True
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(module, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    module._on_state_json(b'{"type": "operator_lost"}')

    wait_until(lambda: module.go2.sport_command.called, timeout=2.0)
    module.go2.sport_command.assert_called_with(ALLOWED_SPORT_CMDS["Damp"])
    module.go2.stop_movement.assert_called_once()  # one task: stop THEN damp
    assert acks == []  # internal task — no junk ack to the operator


def test_non_urgent_cmd_rejected_while_estopped(
    module: Go2CommandModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(module, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))
    module._estopped = True

    module._handle_sport_cmd({"name": "Sit", "nonce": 4})

    module.go2.sport_command.assert_not_called()
    assert acks == [(4, False)]


# ─── nonce dedup ─────────────────────────────────────────────────────


def test_duplicate_nonce_reacks_without_reexecution(
    module: Go2CommandModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    module.go2.set_light.return_value = True
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(module, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    module._handle_light({"brightness": 1.0, "nonce": 5})
    wait_until(lambda: (5, True) in acks, timeout=2.0)
    module._handle_light({"brightness": 1.0, "nonce": 5})  # duplicate
    wait_until(lambda: acks.count((5, True)) == 2, timeout=2.0)

    module.go2.set_light.assert_called_once()  # executed once, re-acked


# ─── nav goal ────────────────────────────────────────────────────────


def test_nav_goal_publishes_and_acks(
    module: Go2CommandModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(module, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    module._handle_nav_goal({"x": 2.5, "y": -1.0, "nonce": 11})

    (pose,) = module.goal_request.publish.call_args.args
    assert pose.pose.position.x == pytest.approx(2.5)
    assert acks == [(11, True)]


def test_nav_goal_rejects_non_numbers(
    module: Go2CommandModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(module, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    module._handle_nav_goal({"x": "2.5", "y": 1.0, "nonce": 17})

    module.goal_request.publish.assert_not_called()
    assert acks == [(17, False)]


def test_nav_goal_rejected_when_estopped(
    module: Go2CommandModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    module._estopped = True
    acks: list[tuple[Any, bool]] = []
    monkeypatch.setattr(module, "_send_ack", lambda nonce, ok: acks.append((nonce, ok)))

    module._handle_nav_goal({"x": 1, "y": 1, "nonce": 13})

    module.goal_request.publish.assert_not_called()
    assert acks == [(13, False)]


@pytest.mark.parametrize("invalid", [-1, 1_000_000_000, float("nan")])
def test_drive_rejects_unnormalized_time_without_advancing_guard(module, invalid):
    value = _twist(time.time())
    value.header.stamp.nanosec = invalid
    module._on_cmd_vel_in(value)
    assert module._last_cmd_ns == 0
    module.tele_cmd_vel.publish.assert_not_called()


def test_drive_preserves_epoch_nanoseconds_and_does_not_clamp_the_input(module, mocker):
    mocker.patch(
        "dimos.teleop.hosted.go2_command.time.time_ns", return_value=1_789_654_321_000_000_100
    )
    first = TwistStamped(
        Header(Time(1_789_654_321, 1), "operator"),
        Twist(Vector3(99.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)),
    )
    original = deepcopy(first)
    module._on_cmd_vel_in(first)
    second = deepcopy(first)
    second.header.stamp.nanosec = 2
    module._on_cmd_vel_in(second)
    module._on_cmd_vel_in(first)  # same float-second value, but genuinely older
    assert module._last_cmd_ns == 1_789_654_321_000_000_002
    assert module.tele_cmd_vel.publish.call_count == 2
    assert module.tele_cmd_vel.publish.call_args.args[0].linear.x == 1.5
    assert first == original


@pytest.mark.parametrize("speed,moving", [(1e-9, False), (1e-8, False), (1.1e-8, True)])
def test_drive_keeps_existing_numpy_zero_tolerance(module, speed, moving):
    module._on_cmd_vel_in(_twist(time.time(), vx=speed))
    assert module.tele_cmd_vel.publish.call_count == int(moving)


@pytest.mark.parametrize("backend", ["lcm", "zenoh"])
def test_framed_operator_drive_crosses_real_cdr_ports_to_driver_adapter(mocker, backend):
    driver = MagicMock(spec=["move", "stop"])
    mocker.patch("dimos.robot.unitree.go2.connection.make_connection", return_value=driver)
    provider = mocker.Mock()
    subscriptions = {}

    def subscribe(topic, callback):
        subscriptions.setdefault(topic, []).append(callback)
        return lambda: subscriptions[topic].remove(callback)

    provider.subscribe.side_effect = subscribe
    mocker.patch.object(BrokerConfig, "provider", return_value=provider)
    mocker.patch(
        "dimos.teleop.hosted.go2_command.time.time_ns", return_value=1_789_654_321_000_000_100
    )
    delivered = Event()
    commands = []

    def record(value, duration):
        commands.append((value, duration))
        delivered.set()
        return True

    driver.move.side_effect = record
    options = (
        {"url": "memq://"}
        if backend == "lcm"
        else {
            "scouting": False,
            "multicast": False,
            "gossip": False,
            "connect": [],
            "listen": ["tcp/127.0.0.1:0"],
        }
    )
    with ExitStack() as cleanup:
        cleanup.callback(provider.stop)
        command = Go2CommandModule()
        cleanup.callback(command.stop)
        manager = MovementManager()
        cleanup.callback(manager.stop)
        robot = GO2Connection()
        cleanup.callback(robot.stop)
        ingress = WebRTCTransport("cmd_unreliable", TwistStamped, config=BrokerConfig())
        cleanup.callback(ingress.stop)
        cleanup.callback(ingress.subscribe(command._on_cmd_vel_in))
        for module, port in [(command, "tele_cmd_vel"), (manager, "cmd_vel")]:
            wire = CdrTransport("hosted_test_" + port, Twist, backend=backend, **options)
            cleanup.callback(wire.stop)
            module.set_transport(port, wire)
        cleanup.callback(command.tele_cmd_vel.subscribe(manager._on_teleop))
        cleanup.callback(manager.cmd_vel.subscribe(lambda value: robot.move(value, 0.0)))
        source = TwistStamped(
            Header(Time(1_789_654_321, 1), "operator"),
            Twist(Vector3(99.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)),
        )

        def send(value):
            frame = encode_data_frame(
                FrameHeader(
                    ch="twist",
                    seq=0,
                    ts=17.0,
                    delivery="latest",
                    meta={"encoding": "cdr", "type": value.__msgtype__},
                ),
                encode(value),
            )
            for callback in list(subscriptions["cmd_unreliable"]):
                callback(frame, "cmd_unreliable")

        send(source)
        assert delivered.wait(5)
        assert commands[0][0].linear.x == 1.5
        assert commands[0][1] == 0.0
        assert source.twist.linear.x == 99.0
        delivered.clear()
        release = TwistStamped(
            Header(Time(1_789_654_321, 2), "operator"),
            Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)),
        )
        send(release)
        assert delivered.wait(5)
        assert commands[1][0].is_zero()
        release.header.stamp.nanosec = 3
        send(release)  # idle zeros do not cancel another navigation plan
        assert len(commands) == 2
        provider.start.assert_called_once()


def test_hosted_drive_blueprints_share_canonical_types():
    for blueprint in (teleop_hosted_go2_transport, teleop_hosted_go2_multicam):
        assert ("cmd_vel_in", TwistStamped) in blueprint.transport_map
        for name in ("tele_cmd_vel", "nav_cmd_vel", "cmd_vel"):
            assert blueprint.transport_map[(name, Twist)].cls is CdrTransport
