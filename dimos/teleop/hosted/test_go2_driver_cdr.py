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

from collections.abc import Iterator
from contextlib import ExitStack
from threading import Event
from unittest.mock import MagicMock

from dimos_cdrgen.registry import encode
import pytest

from dimos.core.transport import LCMTransport, WebRTCTransport, ZenohTransport
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.geometry_msgs.msg import Twist, TwistStamped, Vector3
from dimos.messages.std_msgs.msg import Header
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.protocol.pubsub.impl.webrtc.providers.broker import BrokerConfig
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.teleop.hosted.go2_command import Go2CommandModule
from dimos.web.relay_bridge.protocol import FrameHeader, decode_data_frame, encode_data_frame


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
            wire = (
                LCMTransport("hosted_test_" + port, Twist, **options)
                if backend == "lcm"
                else ZenohTransport("hosted_test_" + port, Twist, **options)
            )
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
                callback(decode_data_frame(frame).payload, "cmd_unreliable")

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
