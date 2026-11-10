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

from dataclasses import replace
import math

import can_motor_control
import numpy as np
import pytest

from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.damiao import arm as common
from dimos.robot.manipulators.damiao._damiao_native_test_helpers import TEST_ROBOT
from dimos.robot.manipulators.damiao.arm import DamiaoArm
from dimos.utils.testing.waiting import wait_until


def _read_until_fault(module):
    """The hosted cycle is the only reader: one read must latch the fault."""
    try:
        module.read_measured()
    except RuntimeError:
        return
    raise AssertionError("read_measured did not latch a feedback fault")


def test_native_partial_commands_keep_targets_and_exact_mit_rows(make_rig, boundary):
    module, commands, readings = make_rig()
    native = boundary[0]
    commands.publish(JointState(name=["arm/joint2", "arm/left_finger"], position=[-0.5, 0.032]))
    np.testing.assert_equal(
        native.groups["left_arm"].commands[-1], [[80, 5, 0.2, 0, 0], [10, 1.5, -0.5, 0, 0]]
    )
    np.testing.assert_equal(native.groups["right_arm"].commands[-1], [[20, 0.6, 0.2, 0, 0]])
    assert native.groups["left_gripper"].commands == [pytest.approx(0.8)]
    assert native.groups["right_gripper"].commands == [pytest.approx(0.4)]
    commands.publish(JointState(name=["arm/joint1"], position=[0.6]))
    assert native.groups["left_arm"].commands[-1][1, 2] == -0.5
    module.publish_measured(module.read_measured())
    wait_until(lambda: readings.sent[-1].name[0] == "arm/joint1", timeout=2)
    assert readings.sent[-1].name == [
        "arm/joint1",
        "arm/joint2",
        "arm/joint3",
        "arm/left_finger",
        "arm/right_finger",
    ]
    assert readings.sent[-1].velocity == [0.3, 0.3, 0.3, 0, 0]
    assert readings.sent[-1].effort == [0.4, 0.4, 0.4, 0, 0]
    assert module.status().connected


def test_feedback_failure_disables_latches_and_blocks_commands(make_rig, boundary):
    module, commands, _ = make_rig()
    boundary[0].refresh_error = RuntimeError("lost feedback")
    _read_until_fault(module)
    commands.publish(JointState(name=["arm/joint1"], position=[0.7]))
    assert boundary[0].groups["left_arm"].commands == []
    assert "disable" in boundary[0].events
    assert "feedback" in module.status().last_error


@pytest.mark.parametrize("field", ["q", "dq", "tau"])
def test_nonfinite_feedback_disables(make_rig, boundary, field):
    module, _, _ = make_rig()
    getattr(boundary[0].groups["left_arm"], field)[0] = float("nan")
    _read_until_fault(module)
    assert "disable" in boundary[0].events


def test_gross_feedback_fault_disables_even_if_disable_fails(make_rig, boundary):
    module, _, _ = make_rig()
    boundary[0].disable_error = RuntimeError("unplugged")
    boundary[0].groups["left_arm"].q[0] = float("nan")
    _read_until_fault(module)
    assert not module._arm.active
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
        module._arm._physical, "gravity", return_value=np.array([1.0, 2.0, 3.0])
    )
    commands.publish(JointState(name=["arm/joint1"], position=[0.8]))
    np.testing.assert_equal(gravity.call_args.args[0], [0.2, 0.2, 0.2])
    np.testing.assert_equal(boundary[0].groups["left_arm"].commands[-1][:, 4], [1.0, 2.0])
    assert boundary[0].groups["right_arm"].commands[-1][0, 4] == 3.0


def test_platform_bus_selectors_preserve_logical_mapping(monkeypatch, mocker):
    arm = DamiaoArm(TEST_ROBOT, ["SERIAL", "can0"])
    monkeypatch.setattr(common.sys, "platform", "darwin")
    usb = mocker.patch.object(can_motor_control, "GsUsbBus", create=True)
    arm._make_can_bus("SERIAL")
    usb.assert_called_once_with(vendor_id=0x1D50, product_id=0x606F, serial_number="SERIAL")
    monkeypatch.setattr(common.sys, "platform", "linux")
    socket = mocker.patch.object(can_motor_control, "SocketCanBus", create=True)
    arm._make_can_bus("can0")
    socket.assert_called_once_with("can0")
    monkeypatch.setattr(common.sys, "platform", "unsupported")
    with pytest.raises(RuntimeError, match="unsupported"):
        arm._make_can_bus("SERIAL")


def test_native_builder_preserves_bus_and_gripper_specs(make_rig, boundary):
    make_rig()
    native = boundary[0]
    assert [(name, bus) for name, bus, _ in native.arm_specs] == [
        ("left_arm", "left_arm"),
        ("right_arm", "right_arm"),
    ]
    assert [(name, transport) for name, transport, _ in native.bus_specs] == [
        ("left_arm", ("socket", "can1")),
        ("right_arm", ("socket", "can0")),
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
    _read_until_fault(module)
    assert "invalid opening" in module._fault


def test_wrong_feedback_length_latches(make_rig, boundary):
    module, _, _ = make_rig()
    boundary[0].groups["left_arm"].dq = np.zeros(1)
    _read_until_fault(module)
    assert "wrong state length" in module._fault


def test_tick_failure_latches_feedback_fault(make_rig, boundary):
    module, _, _ = make_rig()
    boundary[0].tick_error = RuntimeError("transport failed")
    _read_until_fault(module)
    assert "transport failed" in module._fault


def test_gravity_preflight_fails_before_enable(connection_class, boundary, mocker):
    mocker.patch.object(common.PhysicalModel, "gravity", side_effect=ValueError("invalid gravity"))
    module = connection_class()
    with pytest.raises(ValueError, match="invalid gravity"):
        module.start()
    assert "enable" not in boundary[0].events
    assert module._arm is None


def test_a_reading_past_a_limit_is_published_as_measured(make_rig, boundary):
    module, _, readings = make_rig()
    past = math.pi + 0.001
    boundary[0].groups["left_arm"].q[0] = past
    module.publish_measured(module.read_measured())
    wait_until(lambda: readings.sent[-1].position[0] == past, timeout=2)
    assert module._fault is None
    assert "disable" not in boundary[0].events


def test_inactive_feedback_uses_gripper_placeholders(connection_class, boundary):
    module = connection_class()
    try:
        module.connect()
        reading = module._read_state()
        assert reading.position == [0.2, 0.2, 0.2, 0.0, 0.0]
        assert reading.velocity == [0.3, 0.3, 0.3, 0.0, 0.0]
        assert reading.effort == [0.4, 0.4, 0.4, 0.0, 0.0]
    finally:
        module.stop()


@pytest.mark.parametrize("gain", ["kp", "kd"])
@pytest.mark.parametrize("bad", [float("nan"), float("inf")])
def test_invalid_second_arm_gain_prevents_all_native_sends(make_rig, boundary, gain, bad):
    module, commands, _ = make_rig()
    robot = module._arm._robot
    buses = list(robot.buses)
    values = list(getattr(buses[1], gain))
    values[0] = bad
    buses[1] = replace(buses[1], **{gain: values})
    module._arm._robot = replace(robot, buses=buses)
    commands.publish(JointState(name=["arm/joint1"], position=[0.8]))
    assert "non-finite" in module.status().last_error
    assert boundary[0].groups["left_arm"].commands == []
    assert boundary[0].groups["right_arm"].commands == []
    assert boundary[0].groups["left_gripper"].commands == []
    assert boundary[0].groups["right_gripper"].commands == []


def test_teach_mode_sends_zero_stiffness_teaching_damping_and_gravity(make_rig, boundary):
    module, commands, _ = make_rig()
    robot = module._arm._robot
    module._arm._robot = replace(robot, teach_kd=[2.0, 2.0, 0.5])
    module._arm._teach = True
    gravity = np.array([1.0, 2.0, 3.0])
    import unittest.mock

    with unittest.mock.patch.object(module._arm._physical, "gravity", return_value=gravity):
        commands.publish(JointState(name=["arm/joint1"], position=[0.8]))
    left = boundary[0].groups["left_arm"].commands[-1]
    right = boundary[0].groups["right_arm"].commands[-1]
    np.testing.assert_equal(left[:, 0], [0.0, 0.0])
    np.testing.assert_equal(left[:, 1], [2.0, 2.0])
    np.testing.assert_equal(right[:, 0], [0.0])
    np.testing.assert_equal(right[:, 1], [0.5])
    np.testing.assert_equal(left[:, 4], [1.0, 2.0])
    assert right[0, 4] == 3.0
