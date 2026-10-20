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

"""Canonical whole-body wire bridge, with fake external DDS/ROS endpoints."""

from contextlib import ExitStack
from copy import deepcopy
from functools import partial
from importlib import import_module
import sys
from threading import Event, Thread
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest

from dimos.core import cdr_transport
from dimos.core.cdr_transport import CdrTransport
from dimos.core.global_config import GlobalConfig
from dimos.core.transport_factory import make_transport
from dimos.hardware.whole_body.spec import MotorCommand
from dimos.hardware.whole_body.transport.adapter import TransportWholeBodyAdapter
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.dimos_msgs.msg import MotorCommandArray
from dimos.messages.sensor_msgs.msg import Imu, JointState
from dimos.messages.std_msgs.msg import Header
from dimos.protocol.service.lcmservice import LCMConfig
from dimos.protocol.service.zenohservice import ZenohConfig
from dimos.robot.galaxea.r1pro.connection import R1ProConnection, _stamp_nanoseconds
from dimos.robot.unitree.g1.wholebody_connection import G1LowStateSnapshot, G1WholeBodyConnection
from dimos.utils.testing.waiting import wait_until


@pytest.fixture(params=["lcm", "zenoh"])
def factory(request, monkeypatch):
    monkeypatch.setattr(cdr_transport, "LCMConfig", partial(LCMConfig, url="memq://"))
    monkeypatch.setattr(
        cdr_transport,
        "ZenohConfig",
        partial(
            ZenohConfig,
            scouting=False,
            multicast=False,
            gossip=False,
            connect=[],
            listen=["tcp/127.0.0.1:0"],
        ),
    )
    build = partial(make_transport, g=GlobalConfig(transport=request.param))
    transports = {}

    def shared(topic, kind):
        # ModuleCoordinator shares one transport per logical (name, type) within a worker.
        # memq is endpoint-local, so distinct memq instances cannot act as a network.
        key = (topic, kind)
        if key not in transports:
            transports[key] = build(topic, kind)
        return transports[key]

    try:
        yield shared
    finally:
        for wire in transports.values():
            wire.stop()


def _command(count):
    return MotorCommandArray(
        Header(Time(17, 1), "coordinator"), *[np.arange(count, dtype=np.float64) for _ in range(5)]
    )


def test_adapter_default_factory_reuses_g1_canonical_motor_state_imu_and_command(factory, mocker):
    with ExitStack() as cleanup:
        connection = G1WholeBodyConnection(soft_start_seconds=0.0)
        cleanup.callback(connection.stop)
        publisher = mocker.Mock()
        crc = mocker.Mock()
        crc.Crc.return_value = 17
        low_cmd = SimpleNamespace(
            mode_machine=5,
            crc=0,
            motor_cmd=[SimpleNamespace(mode=1, q=0, dq=0, kp=0, kd=0, tau=0) for _ in range(35)],
        )
        mocker.patch.multiple(
            connection, _publisher=publisher, _low_cmd=low_cmd, _crc=crc, _mode_machine=5
        )
        written = Event()
        values = []

        def write(value):
            values.append(deepcopy(value))
            written.set()

        publisher.Write.side_effect = write
        adapter = TransportWholeBodyAdapter(dof=29, hardware_id="g1", transport_cls=factory)
        cleanup.callback(adapter.disconnect)
        assert adapter.connect()
        wires = {}
        for name, kind in [
            ("motor_states", JointState),
            ("imu", Imu),
            ("motor_command", MotorCommandArray),
        ]:
            wire = factory("/g1/" + name, kind)
            cleanup.callback(wire.stop)
            wires[name] = wire
            connection.set_transport(name, wire)
        cleanup.callback(wires["motor_command"].subscribe(connection._on_motor_command))
        sample = G1LowStateSnapshot(
            [0.25] * 29, [0.5] * 29, [0.75] * 29, (1, 0, 0, 0), (1, 2, 3), (4, 5, 6)
        )
        observed = []
        cleanup.callback(wires["motor_states"].subscribe(observed.append))
        connection._publish_motor_state_and_imu(
            now=17.0, frame_id="g1_pelvis", sample=sample, stamp=Time(1_789_654_321, 1)
        )
        wait_until(
            lambda: (
                adapter.has_motor_states()
                and adapter.read_imu().gyroscope == (1, 2, 3)
                and bool(observed)
            ),
            timeout=5,
        )
        assert adapter.read_motor_states()[0].q == 0.25
        assert adapter.read_motor_states()[0].dq == 0.5
        assert adapter.read_motor_states()[0].tau == 0.75
        assert adapter.read_imu().quaternion == (1, 0, 0, 0)
        assert observed[0].header.stamp == Time(1_789_654_321, 1)
        commands = [MotorCommand(q=1, dq=2, kp=3, kd=4, tau=5) for _ in range(29)]
        assert adapter.write_motor_commands(commands)
        assert written.wait(5)
        assert values[0].mode_machine == 5 and values[0].crc == 17
        assert [(v.q, v.dq, v.kp, v.kd, v.tau) for v in values[0].motor_cmd[:29]] == [
            (1, 2, 3, 4, 5)
        ] * 29


@pytest.mark.parametrize("field", ["q", "dq", "kp", "kd", "tau"])
@pytest.mark.parametrize("count,kind", [(29, G1WholeBodyConnection), (18, R1ProConnection)])
def test_inconsistent_canonical_arrays_are_dropped_before_sdk(field, count, kind, mocker):
    connection = kind()
    with ExitStack() as cleanup:
        cleanup.callback(connection.stop)
        command = _command(count)
        setattr(command, field, np.zeros(count - 1, dtype=np.float64))
        publisher = mocker.Mock()
        if kind is G1WholeBodyConnection:
            low_cmd = SimpleNamespace(
                mode_machine=5,
                crc=0,
                motor_cmd=[
                    SimpleNamespace(mode=1, q=0, dq=0, kp=0, kd=0, tau=0) for _ in range(35)
                ],
            )
            mocker.patch.multiple(
                connection,
                _publisher=publisher,
                _low_cmd=low_cmd,
                _crc=mocker.Mock(),
                _mode_machine=5,
            )
        else:
            mocker.patch.object(connection, "_ros", publisher)
            cleanup.callback(mocker.patch.object, connection, "_ros", None)
        # R1 imports ROS only after validation; G1 has live fake DDS state.
        connection._on_motor_command(command)
        publisher.Write.assert_not_called()
        publisher.publish.assert_not_called()
        assert connection.inputs["motor_command"].type is MotorCommandArray


def test_r1_segment_publication_consumes_generated_command_without_physical_ros(
    factory, mocker, monkeypatch
):
    ros_module = ModuleType("sensor_msgs.msg")
    ros_module.JointState = lambda: SimpleNamespace(header=SimpleNamespace(stamp=None))
    monkeypatch.setitem(sys.modules, "sensor_msgs.msg", ros_module)
    stamp = SimpleNamespace(sec=17, nanosec=1)
    mocker.patch("dimos.robot.galaxea.r1pro.connection._ros_stamp_now", return_value=stamp)
    with ExitStack() as cleanup:
        connection = R1ProConnection()
        cleanup.callback(connection.stop)
        ros = mocker.Mock()
        mocker.patch.multiple(
            connection,
            _ros=ros,
            _cmd_torso_topic="torso",
            _cmd_left_topic="left",
            _cmd_right_topic="right",
        )
        cleanup.callback(mocker.patch.object, connection, "_ros", None)
        wire = factory("/r1pro/motor_command", MotorCommandArray)
        cleanup.callback(wire.stop)
        cleanup.callback(wire.subscribe(connection._on_motor_command))
        written = Event()
        ros.publish.side_effect = lambda *args: (
            written.set() if ros.publish.call_count == 3 else None
        )
        wire.publish(_command(18))
        assert written.wait(5)
        assert [call.args[0] for call in ros.publish.call_args_list] == ["torso", "left", "right"]
        assert [call.args[1].position for call in ros.publish.call_args_list] == [
            list(range(4)),
            list(range(4, 11)),
            list(range(11, 18)),
        ]
        assert all(call.args[1].header.stamp is stamp for call in ros.publish.call_args_list)
        assert connection.outputs["motor_states"].type is JointState
        assert connection.outputs["imu_chassis"].type is Imu


def test_r1_feedback_stamp_preserves_epoch_nanoseconds_and_existing_zero_fallback(mocker):
    mocker.patch("dimos.robot.galaxea.r1pro.connection.time.time_ns", return_value=99)
    source = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=1_789_654_321, nanosec=1))
    )
    assert _stamp_nanoseconds(source) == 1_789_654_321_000_000_001
    source.header.stamp.sec = 0
    source.header.stamp.nanosec = 0
    assert _stamp_nanoseconds(source) == 99


def test_r1_fused_feedback_preserves_oldest_exact_segment_stamp(factory):
    with ExitStack() as cleanup:
        connection = R1ProConnection()
        cleanup.callback(connection.stop)
        wire = factory("/r1pro/motor_states", JointState)
        cleanup.callback(wire.stop)
        connection.set_transport("motor_states", wire)
        observed = []
        ready = Event()

        def receive(value):
            observed.append(value)
            connection._stop_event.set()
            ready.set()

        cleanup.callback(wire.subscribe(receive))
        for callback, count, nanos in [
            (connection._on_feedback_torso, 4, 3),
            (connection._on_feedback_left, 7, 1),
            (connection._on_feedback_right, 7, 2),
        ]:
            message = SimpleNamespace(
                header=SimpleNamespace(stamp=SimpleNamespace(sec=1_789_654_321, nanosec=nanos)),
                position=[float(nanos)] * count,
                velocity=[0.5] * count,
                effort=[0.75] * count,
            )
            callback(message, None)
        thread = Thread(target=connection._publish_loop)

        def close_thread():
            connection._stop_event.set()
            thread.join(timeout=5)
            assert not thread.is_alive()

        cleanup.callback(close_thread)
        thread.start()
        assert ready.wait(5)
        assert observed[0].header.stamp == Time(1_789_654_321, 1)
        assert observed[0].position.tolist() == [3.0] * 4 + [1.0] * 7 + [2.0] * 7
        assert observed[0].velocity.tolist() == [0.5] * 18
        assert observed[0].effort.tolist() == [0.75] * 18


def test_wholebody_blueprints_bind_canonical_types_without_loading_robot_assets(mocker):
    download = mocker.patch(
        "dimos.utils.data.get_data",
        side_effect=AssertionError("blueprint import must not download assets"),
    )
    simple = import_module("dimos.robot.unitree.g1.blueprints.basic.unitree_g1_coordinator")
    groot = import_module("dimos.robot.unitree.g1.blueprints.basic.unitree_g1_groot_wbc")
    r1 = import_module("dimos.robot.galaxea.r1pro.blueprints.basic.r1pro_coordinator")
    for blueprint in (simple.unitree_g1_coordinator, groot.unitree_g1_groot_wbc):
        for name, kind in [
            ("motor_states", JointState),
            ("motor_command", MotorCommandArray),
            ("imu", Imu),
            ("joint_command", JointState),
            ("g1_joints", JointState),
        ]:
            spec = blueprint.transport_map[(name, kind)]
            assert spec.cls is CdrTransport
    assert r1._zenoh_transport("/r1pro/motor_states", JointState).cls is CdrTransport
    assert r1._zenoh_transport("/r1pro/imu", Imu).cls is CdrTransport
    assert r1._zenoh_transport("/r1pro/motor_command", MotorCommandArray).cls is CdrTransport
    download.assert_not_called()
