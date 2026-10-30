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

"""Shared device selection and lifecycle, without hardware or network I/O."""

import pytest

from dimos.control._control_test_helpers import FakeArm
from dimos.control.connection.connection_module import ConnectionModule
from dimos.control.connection.device import ConnectionDevice
from dimos.control.contract.limits import Limits
from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.protocol.rpc.zenohrpc import ZenohRPC


@pytest.fixture
def modules(mocker):
    # Keep the real module lifecycle/event loop; only its RPC network is absent.
    mocker.patch.object(ZenohRPC, "start", autospec=True)
    mocker.patch.object(ZenohRPC, "serve_module_rpc", autospec=True)
    created = []

    def create(cls=ConnectionModule, *, simulation="", **kwargs):
        if cls is ConnectionModule:
            kwargs.setdefault("source", "robot")
            kwargs.setdefault("base_range", Twist(linear=(1.0, 1.0, 0), angular=(0, 0, 1.0)))
        module = cls(
            g=GlobalConfig(simulation=simulation),
            rpc_transport=ZenohRPC,
            deadman_timeout_s=60.0,
            **kwargs,
        )
        created.append(module)
        return module

    yield create
    for module in reversed(created):
        module.stop()


@pytest.fixture
def device(mocker):
    return mocker.create_autospec(ConnectionDevice, instance=True, spec_set=True)


def test_simulation_device_receives_validated_commands_without_opening_hardware(
    modules, device, mocker
):
    factory = mocker.Mock(return_value=device)
    module = modules(simulation="mujoco", simulation_device=factory)
    hardware = mocker.patch.object(module, "create_hardware_device", return_value=device)

    module.start()
    module.command({"robot/base/vx": 2.0})

    device.start.assert_called_once_with()
    device.write.assert_called_once_with({"robot/base/vx": 1.0})
    assert module.status().connected
    factory.assert_called_once_with(module.publish_feedback)
    hardware.assert_not_called()


def test_hardware_mode_ignores_the_simulation_factory(modules, device, mocker):
    factory = mocker.Mock()
    module = modules(simulation_device=factory)
    hardware = mocker.patch.object(module, "create_hardware_device", return_value=device)

    module.start()
    module.command({"robot/base/vx": 0.5})

    hardware.assert_called_once_with()
    factory.assert_not_called()
    device.start.assert_called_once_with()
    device.write.assert_called_once_with({"robot/base/vx": 0.5})


@pytest.mark.parametrize("simulation", ["mujoco", "unsupported"])
def test_missing_simulation_backend_never_opens_hardware(modules, mocker, simulation):
    module = modules(simulation=simulation)
    hardware = mocker.patch.object(module, "create_hardware_device")

    with pytest.raises(ValueError, match="simulation"):
        module.start()

    hardware.assert_not_called()
    assert not module.status().connected
    assert module._loop is None


def test_failed_simulation_factory_never_falls_back_to_hardware(modules, mocker):
    factory = mocker.Mock(side_effect=RuntimeError("world is unavailable"))
    module = modules(simulation="mujoco", simulation_device=factory)
    hardware = mocker.patch.object(module, "create_hardware_device")

    with pytest.raises(RuntimeError, match="world is unavailable"):
        module.start()

    hardware.assert_not_called()
    assert not module.status().connected
    assert module._loop is None


def test_failed_device_start_halts_and_closes_partial_device(modules, device, mocker):
    device.start.side_effect = RuntimeError("cannot read motors")
    module = modules(simulation="mujoco", simulation_device=mocker.Mock(return_value=device))

    with pytest.raises(RuntimeError, match="cannot read motors"):
        module.start()

    device.halt.assert_called_once_with()
    device.stop.assert_called_once_with()
    assert not module.status().connected
    assert module._loop is None


def test_cleanup_errors_do_not_hide_the_device_start_failure(modules, device, mocker):
    device.start.side_effect = RuntimeError("original start failure")
    device.halt.side_effect = RuntimeError("cannot halt")
    device.stop.side_effect = RuntimeError("cannot close")
    module = modules(simulation="mujoco", simulation_device=mocker.Mock(return_value=device))

    with pytest.raises(RuntimeError, match="original start failure"):
        module.start()

    device.stop.assert_called_once_with()
    assert module.status().last_error == "device stop failed: RuntimeError: cannot close"
    assert module._loop is None


def test_stopping_halts_before_closing_and_is_idempotent(modules, device, mocker):
    calls = mocker.Mock()
    calls.attach_mock(device.halt, "halt")
    calls.attach_mock(device.stop, "stop")
    module = modules(simulation="mujoco", simulation_device=mocker.Mock(return_value=device))
    module.start()

    module.stop()
    module.stop()
    module.command({"robot/base/vx": 0.5})

    assert calls.mock_calls == [mocker.call.halt(), mocker.call.stop()]
    device.write.assert_not_called()
    assert not module.status().connected


def test_device_stop_error_still_closes_the_module(modules, device, mocker):
    device.stop.side_effect = RuntimeError("cannot close")
    module = modules(simulation="mujoco", simulation_device=mocker.Mock(return_value=device))
    module.start()

    module.stop()

    assert module.status().last_error == "device stop failed: RuntimeError: cannot close"
    assert not module.status().connected
    assert module._loop is None


def test_device_write_failure_uses_the_existing_halt_and_status_path(modules, device, mocker):
    device.write.side_effect = RuntimeError("device disconnected")
    module = modules(simulation="mujoco", simulation_device=mocker.Mock(return_value=device))
    module.start()

    module.command({"robot/base/vx": 0.5})

    device.halt.assert_called_once_with()
    assert module.status().last_error == "write failed: RuntimeError: device disconnected"


def test_device_feedback_publishes_only_to_declared_outputs(modules, mocker):
    module = modules(FakeArm)
    publish = mocker.patch.object(module.joint_state, "publish")
    message = JointState(name=["mock/joint1"], position=[0.2])

    module.publish_feedback("joint_state", message)
    with pytest.raises(ValueError, match="no output port 'position_command'"):
        module.publish_feedback("position_command", message)
    module.stop()
    module.publish_feedback("joint_state", message)

    publish.assert_called_once_with(message)


def test_start_respects_driver_limits_without_loading_a_model(modules, device, mocker):
    table = {"robot/custom/position": Limits(-1, 1)}
    mocker.patch.object(
        ConnectionModule, "limits", new_callable=mocker.PropertyMock, return_value=table
    )
    module = modules(
        base_range=None, simulation="mujoco", simulation_device=mocker.Mock(return_value=device)
    )

    module.start()
    module.command({"robot/custom/position": 0.25})

    device.write.assert_called_once_with({"robot/custom/position": 0.25})
