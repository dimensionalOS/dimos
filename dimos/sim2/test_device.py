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

"""The simulation device uses normal typed transports and named robot joints."""

from pathlib import Path
from types import SimpleNamespace

import pytest

from dimos.core.global_config import GlobalConfig
from dimos.core.transport import PubSubTransport
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.sim2.device import SimulatedDeviceFactory
from dimos.sim2.spec import Joint, RobotConfig, RobotInstance


@pytest.fixture
def rig(mocker):
    instance = RobotInstance(
        RobotConfig(
            model=Path("not-loaded.xml"),
            root_body="base",
            joints=(
                Joint("arm/elbow", "elbow_joint", "elbow_motor", kp=7, kd=0.7),
                Joint("arm/wrist", "wrist_joint", "wrist_motor", kp=9, kd=0.9),
                Joint("arm/grip", "grip_joint", "grip_servo", mode="position", kp=50, kd=2),
            ),
        ),
        hold_gains={"arm/elbow": (21, 2.1), "arm/grip": (80, 8)},
    )
    topics = {kind: f"private/arm/{kind}" for kind in ("motor_command", "motor_states", "imu")}
    config = SimulatedDeviceFactory(
        instance, topics, GlobalConfig(), ("motor_states", "imu"), halt_damping=4.5
    )
    transports = {
        kind: mocker.create_autospec(PubSubTransport, instance=True, spec_set=True)
        for kind in topics
    }
    unsubscribes = {kind: mocker.Mock() for kind in config.feedback_ports}
    for kind, unsubscribe in unsubscribes.items():
        transports[kind].subscribe.return_value = unsubscribe

    def transport_for(topic, message_type, *, g):
        return transports[next(kind for kind, name in topics.items() if name == topic)]

    make = mocker.patch("dimos.sim2.device.make_transport", side_effect=transport_for)
    publish = mocker.Mock()
    device = config(publish)
    yield SimpleNamespace(
        device=device,
        config=config,
        transports=transports,
        unsubscribes=unsubscribes,
        make=make,
        publish=publish,
    )
    device.stop()


def test_commands_use_declared_joint_order_and_keep_every_pd_field(rig, mocker):
    rig.device.start()
    # Reverse insertion order deliberately: the model's joint order is the wire order.
    values = {
        "arm/grip/position": 0.3,
        "arm/grip/velocity": 0.0,
        "arm/grip/effort": 0.0,
        "arm/grip/kp": 0.0,
        "arm/grip/kd": 0.0,
        "arm/wrist/position": -0.2,
        "arm/wrist/velocity": -0.4,
        "arm/wrist/effort": -1.2,
        "arm/wrist/kp": 30.0,
        "arm/wrist/kd": 3.0,
        "arm/elbow/position": 0.1,
        "arm/elbow/velocity": 0.2,
        "arm/elbow/effort": 1.5,
        "arm/elbow/kp": 20.0,
        "arm/elbow/kd": 2.0,
    }

    rig.device.write(values)

    rig.make.assert_has_calls(
        [
            mocker.call("private/arm/motor_command", MotorCommandArray, g=rig.config.g),
            mocker.call("private/arm/motor_states", JointState, g=rig.config.g),
            mocker.call("private/arm/imu", Imu, g=rig.config.g),
        ]
    )
    command = rig.transports["motor_command"].publish.call_args.args[0]
    assert isinstance(command, MotorCommandArray)
    assert (command.q, command.dq, command.tau, command.kp, command.kd) == (
        [0.1, -0.2, 0.3],
        [0.2, -0.4, 0.0],
        [1.5, -1.2, 0.0],
        [20.0, 30.0, 0.0],
        [2.0, 3.0, 0.0],
    )


def test_feedback_preserves_declared_port_names_and_messages(rig, mocker):
    rig.device.start()
    joints = JointState(
        name=["arm/elbow", "arm/wrist", "arm/grip"], position=[0.1, -0.2, 0.3], ts=12.0
    )
    imu = Imu(frame_id="arm/base", ts=12.0)

    rig.transports["motor_states"].subscribe.call_args.args[0](joints)
    rig.transports["imu"].subscribe.call_args.args[0](imu)

    assert rig.publish.call_args_list == [
        mocker.call("motor_states", joints),
        mocker.call("imu", imu),
    ]


@pytest.mark.parametrize(
    ("names", "positions"),
    [
        (["arm/wrist", "arm/elbow", "arm/grip"], [0.2, 0.1, 0.3]),
        (["arm/elbow", "arm/wrist", "arm/grip"], [0.1]),
    ],
)
def test_mismatched_feedback_is_not_forwarded_or_used_for_a_hold(rig, names, positions):
    rig.device.start()
    receive = rig.transports["motor_states"].subscribe.call_args.args[0]

    with pytest.raises(ValueError, match="does not match the declared joints"):
        receive(JointState(name=names, position=positions))
    rig.device.halt()

    rig.publish.assert_not_called()
    rig.transports["motor_command"].publish.assert_not_called()


def test_halt_damps_effort_motors_and_holds_native_servo_positions(rig):
    rig.device.start()
    receive = rig.transports["motor_states"].subscribe.call_args.args[0]
    receive(JointState(name=["arm/elbow", "arm/wrist", "arm/grip"], position=[0.0, 0.0, 0.0]))
    latest = JointState(name=["arm/elbow", "arm/wrist", "arm/grip"], position=[0.4, -0.3, 0.2])
    receive(latest)
    latest.position[0] = 5.0  # Holding owns a copy of the sample it accepted.

    rig.device.halt()

    command = rig.transports["motor_command"].publish.call_args.args[0]
    assert (command.q, command.dq, command.tau, command.kp, command.kd) == (
        [0.4, -0.3, 0.2],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [4.5, 4.5, 0.0],
    )


def test_halt_before_feedback_leaves_world_startup_hold_in_place(rig):
    rig.device.start()

    rig.device.halt()

    rig.transports["motor_command"].publish.assert_not_called()


def test_partial_start_failure_releases_opened_transports_and_subscriptions(rig):
    rig.transports["imu"].start.side_effect = RuntimeError("cannot open imu stream")

    with pytest.raises(RuntimeError, match="cannot open imu stream"):
        rig.device.start()
    rig.device.stop()

    for transport in rig.transports.values():
        transport.stop.assert_called_once_with()
    rig.unsubscribes["motor_states"].assert_called_once_with()
    rig.unsubscribes["imu"].assert_not_called()
    with pytest.raises(RuntimeError, match="has not started"):
        rig.device.write({})


def test_repeated_start_and_stop_are_harmless_and_late_feedback_is_ignored(rig):
    rig.device.start()
    receive = rig.transports["motor_states"].subscribe.call_args.args[0]

    rig.device.start()
    rig.device.stop()
    rig.device.stop()
    receive(JointState(name=["arm/elbow", "arm/wrist", "arm/grip"], position=[0.1, 0.2, 0.3]))
    rig.device.halt()

    for transport in rig.transports.values():
        transport.start.assert_called_once_with()
        transport.stop.assert_called_once_with()
    for unsubscribe in rig.unsubscribes.values():
        unsubscribe.assert_called_once_with()
    rig.publish.assert_not_called()
    rig.transports["motor_command"].publish.assert_not_called()


def test_stop_failure_still_releases_other_resources_and_disables_commands(rig):
    rig.device.start()
    rig.unsubscribes["imu"].side_effect = RuntimeError("cannot unsubscribe")

    with pytest.raises(RuntimeError, match="cannot unsubscribe"):
        rig.device.stop()
    rig.device.stop()

    for transport in rig.transports.values():
        transport.stop.assert_called_once_with()
    rig.unsubscribes["motor_states"].assert_called_once_with()
    rig.unsubscribes["imu"].assert_called_once_with()
    with pytest.raises(RuntimeError, match="has not started"):
        rig.device.write({})
