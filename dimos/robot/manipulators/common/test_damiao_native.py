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

"""Direct native construction, motor commands and feedback faults."""

from __future__ import annotations

import can_motor_control
import numpy as np
import pytest

from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.common import damiao_connection as common
from dimos.utils.testing.waiting import wait_until


def test_native_partial_commands_keep_targets_and_exact_mit_rows(make_rig, boundary):
    module, commands, readings = make_rig()
    native = boundary[0]
    commands.publish(JointState(name=["arm/joint2", "arm/left_gripper"], position=[-0.5, 0.8]))
    np.testing.assert_equal(
        native.groups["left_arm"].commands[-1], [[80, 5, 0.2, 0, 0], [10, 1.5, -0.5, 0, 0]]
    )
    np.testing.assert_equal(native.groups["right_arm"].commands[-1], [[20, 0.6, 0.2, 0, 0]])
    assert native.groups["left_gripper"].commands == [0.8]
    assert native.groups["right_gripper"].commands == [0.4]
    commands.publish(JointState(name=["arm/joint1"], position=[0.6]))
    assert native.groups["left_arm"].commands[-1][1, 2] == -0.5
    assert readings.sent[-1].name == [
        "arm/joint1",
        "arm/joint2",
        "arm/joint3",
        "arm/left_gripper",
        "arm/right_gripper",
    ]
    assert readings.sent[-1].velocity == [0.3, 0.3, 0.3, 0, 0]
    assert readings.sent[-1].effort == [0.4, 0.4, 0.4, 0, 0]
    assert module.status().connected


def test_feedback_failure_disables_latches_and_blocks_commands(make_rig, boundary):
    module, commands, _ = make_rig()
    boundary[0].refresh_error = RuntimeError("lost feedback")
    wait_until(lambda: module._fault is not None, timeout=2)
    commands.publish(JointState(name=["arm/joint1"], position=[0.7]))
    assert boundary[0].groups["left_arm"].commands == []
    assert "disable" in boundary[0].events
    assert "feedback" in module.status().last_error


@pytest.mark.parametrize("field", ["q", "dq", "tau"])
def test_nonfinite_feedback_disables(make_rig, boundary, field):
    module, _, _ = make_rig()
    getattr(boundary[0].groups["left_arm"], field)[0] = float("nan")
    wait_until(lambda: module._fault is not None, timeout=2)
    assert "disable" in boundary[0].events


def test_gross_feedback_fault_disables_even_if_disable_fails(make_rig, boundary):
    module, _, _ = make_rig()
    boundary[0].disable_error = RuntimeError("unplugged")
    boundary[0].groups["left_arm"].q[0] = 4
    wait_until(lambda: module._fault is not None, timeout=2)
    assert not module._active
    boundary[0].disable_error = None


def test_arm_modes_and_preflight_precede_enable(make_rig, boundary, mocker):
    gravity = mocker.spy(common.PhysicalModel, "gravity")
    make_rig()
    native = boundary[0]
    assert native.groups["left_arm"].modes == ["mit"]
    assert native.groups["right_arm"].modes == ["mit"]
    assert native.events[:3] == ["connect", "refresh", "tick"]
    assert native.events.index("enable") > native.events.index("tick")
    assert gravity.called


def test_gravity_is_added_from_measured_q(make_rig, boundary, mocker):
    module, commands, _ = make_rig()
    gravity = mocker.patch.object(
        module._physical, "gravity", return_value=np.array([1.0, 2.0, 3.0])
    )
    commands.publish(JointState(name=["arm/joint1"], position=[0.8]))
    np.testing.assert_equal(gravity.call_args.args[0], [0.2, 0.2, 0.2])
    np.testing.assert_equal(boundary[0].groups["left_arm"].commands[-1][:, 4], [1.0, 2.0])
    assert boundary[0].groups["right_arm"].commands[-1][0, 4] == 3.0


def test_platform_bus_selectors_preserve_logical_mapping(
    connection_class, boundary, monkeypatch, mocker
):
    module = connection_class()
    module._bus_devices = {"left": "SERIAL", "right": "can0"}
    monkeypatch.setattr(common.sys, "platform", "darwin")
    usb = mocker.patch.object(can_motor_control, "GsUsbBus", create=True)
    module._make_can_bus("left")
    usb.assert_called_once_with(vendor_id=0x1D50, product_id=0x606F, serial_number="SERIAL")
    monkeypatch.setattr(common.sys, "platform", "linux")
    socket = mocker.patch.object(can_motor_control, "SocketCanBus", create=True)
    module._make_can_bus("right")
    socket.assert_called_once_with("can0")
    monkeypatch.setattr(common.sys, "platform", "unsupported")
    with pytest.raises(RuntimeError, match="unsupported"):
        module._make_can_bus("left")
    module.stop()


def test_native_builder_preserves_bus_and_gripper_specs(make_rig, boundary):
    make_rig()
    native = boundary[0]
    assert [(name, bus) for name, bus, _ in native.arm_specs] == [
        ("left_arm", "left"),
        ("right_arm", "right"),
    ]
    assert [(name, transport) for name, transport, _ in native.bus_specs] == [
        ("right", ("socket", "can0")),
        ("left", ("socket", "can1")),
    ]
    assert native.gripper_specs[0][1]["opening_direction"] == "decreasing_position"
    assert native.gripper_specs[0][1]["default_current"] == 0.15
    assert native.gripper_specs[1][1]["opening_direction"] == "increasing_position"
    assert native.gripper_specs[1][1]["default_current"] == 0.1


def test_write_failure_attempts_measured_hold(make_rig, boundary, mocker):
    module, commands, _ = make_rig()
    arm = boundary[0].groups["left_arm"]
    send = mocker.patch.object(arm, "mit_control", side_effect=[RuntimeError("send refused"), None])
    commands.publish(JointState(name=["arm/joint1"], position=[0.8]))
    assert send.call_count == 2
    np.testing.assert_equal(send.call_args.args[0][:, 2], [0.2, 0.2])
    assert "send refused" in module.status().last_error


def test_bad_gripper_feedback_latches(make_rig, boundary):
    module, _, _ = make_rig()
    boundary[0].groups["left_gripper"].opening = float("nan")
    wait_until(lambda: module._fault is not None, timeout=2)
    assert "invalid opening" in module._fault


def test_wrong_feedback_length_latches(make_rig, boundary):
    module, _, _ = make_rig()
    boundary[0].groups["left_arm"].dq = np.zeros(1)
    wait_until(lambda: module._fault is not None, timeout=2)
    assert "wrong state length" in module._fault


def test_tick_failure_latches_feedback_fault(make_rig, boundary):
    module, _, _ = make_rig()
    boundary[0].tick_error = RuntimeError("transport failed")
    wait_until(lambda: module._fault is not None, timeout=2)
    assert "transport failed" in module._fault


def test_gravity_preflight_fails_before_enable(connection_class, boundary, mocker):
    mocker.patch.object(common.PhysicalModel, "gravity", side_effect=ValueError("invalid gravity"))
    module = connection_class()
    with pytest.raises(ValueError, match="invalid gravity"):
        module.start()
    assert "enable" not in boundary[0].events
    assert module._robot is None


def test_normalize_keeps_velocity_and_effort(make_rig, boundary):
    module, _, readings = make_rig()
    upper = float(module._physical._upper[0])
    boundary[0].groups["left_arm"].q[0] = upper + 0.04
    wait_until(lambda: readings.sent[-1].position[0] == upper, timeout=2)
    assert readings.sent[-1].velocity[0] == 0.3
    assert readings.sent[-1].effort[0] == 0.4
    assert module._fault is None


def test_inactive_feedback_uses_gripper_placeholders(connection_class, boundary):
    module = connection_class()
    try:
        module.connect()
        assert module._read_state() == (
            [0.2, 0.2, 0.2, 0.0, 0.0],
            [0.3, 0.3, 0.3, 0.0, 0.0],
            [0.4, 0.4, 0.4, 0.0, 0.0],
        )
    finally:
        module.stop()


@pytest.mark.parametrize("gain", ["kp", "kd"])
@pytest.mark.parametrize("bad", [float("nan"), float("inf")])
def test_invalid_second_arm_gain_prevents_all_native_sends(
    make_rig, boundary, monkeypatch, gain, bad
):
    module, commands, _ = make_rig()
    gains = {name: list(values) for name, values in getattr(module, gain).items()}
    gains["right_arm"][0] = bad
    monkeypatch.setattr(type(module), gain, gains)
    commands.publish(JointState(name=["arm/joint1"], position=[0.8]))
    assert "non-finite" in module.status().last_error
    assert boundary[0].groups["left_arm"].commands == []
    assert boundary[0].groups["right_arm"].commands == []
    assert boundary[0].groups["left_gripper"].commands == []
    assert boundary[0].groups["right_gripper"].commands == []
