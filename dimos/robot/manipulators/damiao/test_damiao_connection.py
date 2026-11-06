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

"""Connection ports, command acceptance and robot shutdown."""

from __future__ import annotations

from dataclasses import replace
import threading

import numpy as np
import pytest

from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.damiao import arm as arm_module, connection as common
from dimos.robot.manipulators.damiao._damiao_native_test_helpers import LocalTransport
from dimos.robot.manipulators.damiao.robot import DamiaoRobot
from dimos.utils.testing.waiting import wait_until


def _teaching_connection(connection_class, **overrides: object):
    robot = replace(
        connection_class.robot,
        teach_kd=[2.0, 2.0, 0.5],
        passive_grippers=frozenset({"left_gripper"}),
        **overrides,
    )

    class Teaching(connection_class):
        pass

    Teaching.robot = robot
    return Teaching


@pytest.mark.parametrize("bad", [4, float("nan"), float("inf")])
def test_invalid_command_is_rejected_as_a_whole(make_rig, boundary, bad):
    module, commands, _ = make_rig()
    commands.publish(JointState(name=["arm/joint1", "arm/joint2"], position=[0.7, bad]))
    assert boundary[0].groups["left_arm"].commands == []
    assert module.status().last_rejection


@pytest.mark.parametrize("bad", [-0.1, 1.01])
def test_gripper_rejects_outside_normalized_range(make_rig, boundary, bad):
    module, commands, _ = make_rig()
    commands.publish(JointState(name=["arm/left_gripper"], position=[bad]))
    assert boundary[0].groups["left_gripper"].commands == []
    assert module.status().last_rejection


def test_measured_hold_replaces_prior_targets_and_next_command_recovers(make_rig, boundary):
    module, commands, _ = make_rig()
    native = boundary[0]
    commands.publish(JointState(name=["arm/joint1"], position=[0.8]))
    native.groups["left_arm"].q[:] = [0.3, 0.4]
    module._on_loop(lambda: module._deadman_expired(module._command_seq))
    wait_until(lambda: module.status().deadman_fired, timeout=2)
    np.testing.assert_equal(native.groups["left_arm"].commands[-1][:, 2], [0.3, 0.4])
    commands.publish(JointState(name=["arm/joint1"], position=[0.7]))
    np.testing.assert_equal(native.groups["left_arm"].commands[-1][:, 2], [0.7, 0.4])
    assert not module.status().deadman_fired


def test_stop_disables_releases_views_and_is_idempotent(make_rig, boundary):
    module, _, _ = make_rig()
    module.stop()
    module.stop()
    assert boundary[0].events.count("disable") == 1
    assert module._arm is None
    assert not module._reader.is_alive()
    assert not module.status().connected


def test_calibration_finishes_before_connected_and_command_acceptance(
    connection_class, boundary, mocker
):
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.start")
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.serve_module_rpc")
    module = connection_class()
    module.set_transport("position_command", LocalTransport())
    module.set_transport("joint_state", LocalTransport())
    native = boundary[0]
    observed = []
    enable = native.enable

    def calibrate():
        observed.append(module.status().connected)
        enable()

    mocker.patch.object(native, "enable", side_effect=calibrate)
    try:
        module.start()
        module.command({"arm/joint1/position": 0.7})
        assert observed == [False]
        assert module.status().connected
        np.testing.assert_equal(native.groups["left_arm"].commands[-1][:, 2], [0.7, 0.2])
    finally:
        module.stop()


def test_failed_enable_rolls_back_and_preserves_error(connection_class, boundary, mocker):
    native = boundary[0]
    native.enable_error = RuntimeError("calibration failed")
    native.disable_error = RuntimeError("disable failed")
    module = connection_class()
    with pytest.raises(RuntimeError, match="calibration failed"):
        module.start()
    assert module._arm is None
    assert module._reader is None
    module.command({"arm/joint1/position": 0.7})
    assert native.groups["left_arm"].commands == []
    assert not module.status().connected
    assert "startup cleanup failed: RuntimeError: disable failed" in module.status().last_error
    assert "disable" in native.events
    module.stop()


def test_initial_bad_feedback_never_enables(connection_class, boundary, mocker):
    mocker.patch.object(
        arm_module.DamiaoArm, "read", side_effect=RuntimeError("invalid initial feedback")
    )
    module = connection_class()
    with pytest.raises(RuntimeError, match="invalid initial feedback"):
        module.start()
    assert "enable" not in boundary[0].events
    assert module._arm is None


def test_connect_does_not_enable_or_accept_commands(connection_class, boundary):
    module = connection_class()
    try:
        module.connect()
        module.command({"arm/joint1/position": 0.7})
        assert "enable" not in boundary[0].events
        assert boundary[0].groups["left_arm"].commands == []
    finally:
        module.stop()


def test_topology_mismatch_fails_before_bus(connection_class, boundary):
    module = connection_class(model=connection_class.robot.model)
    with pytest.raises(ValueError, match="topology"):
        module.start()
    boundary[1].assert_not_called()


def test_disable_failure_still_releases_all_session_handles(make_rig, boundary):
    module, _, _ = make_rig()
    boundary[0].disable_error = RuntimeError("disable failed")
    with pytest.raises(RuntimeError, match="disable failed"):
        module.stop()
    assert module._arm is None
    assert not module.status().connected
    module.stop()


def test_blocked_read_cannot_publish_after_stop_requested(make_rig, monkeypatch):
    module, _, readings = make_rig()
    entered, release, stopped = threading.Event(), threading.Event(), threading.Event()
    failures = []
    read = module._read_state

    def blocked_read():
        entered.set()
        assert release.wait(2)
        return read()

    monkeypatch.setattr(common, "DEFAULT_THREAD_JOIN_TIMEOUT", 0.001)
    monkeypatch.setattr(module, "_read_state", blocked_read)
    assert entered.wait(2)
    publications = len(readings.sent)

    def stop():
        try:
            module.stop()
        except Exception as error:
            failures.append(error)
        finally:
            stopped.set()

    stopper = threading.Thread(target=stop)
    stopper.start()
    try:
        assert module._stop_reading.wait(2)
        assert module._reader.is_alive()
    finally:
        release.set()
        stopper.join(timeout=2)
    assert stopped.is_set()
    assert failures == []
    assert not module._reader.is_alive()
    assert len(readings.sent) == publications


def test_reader_timeout_is_visible_after_native_cleanup(make_rig, boundary, monkeypatch):
    module, _, readings = make_rig()
    entered, release = threading.Event(), threading.Event()

    def block_publication(_message):
        entered.set()
        assert release.wait(2)

    readings.subscribe(block_publication)
    monkeypatch.setattr(common, "DEFAULT_THREAD_JOIN_TIMEOUT", 0.001)
    assert entered.wait(2)
    publications = len(readings.sent)
    try:
        with pytest.raises(RuntimeError, match="reader did not stop"):
            module.stop()
        assert module._arm is None
        assert "disable" in boundary[0].events
        assert not module.status().connected
        assert "shutdown timeout" in module.status().last_error
        assert module._reader.is_alive()
    finally:
        release.set()
        module._reader.join(timeout=2)
    assert not module._reader.is_alive()
    assert len(readings.sent) == publications
    module.stop()


def test_reader_callback_can_stop_without_joining_itself(make_rig):
    module, _, readings = make_rig()
    stopped = threading.Event()
    failures = []

    def stop_from_callback(_message):
        try:
            module.stop()
        except Exception as error:
            failures.append(error)
        finally:
            stopped.set()

    readings.subscribe(stop_from_callback)
    assert stopped.wait(2)
    module._reader.join(timeout=2)
    assert failures == []
    assert not module._reader.is_alive()
    assert module._arm is None


def test_robot_data_rejects_unknown_passive_gripper(connection_class):
    with pytest.raises(ValueError, match="unknown passive grippers"):
        replace(connection_class.robot, passive_grippers=frozenset({"missing"}))


def test_passive_gripper_is_enabled_in_mit_and_never_positioned(connection_class, boundary, mocker):
    teaching = _teaching_connection(connection_class)
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.start")
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.serve_module_rpc")
    module = teaching(deadman_timeout_s=10, teach=True)
    commands, readings = LocalTransport(), LocalTransport()
    module.set_transport("position_command", commands)
    module.set_transport("joint_state", readings)
    try:
        module.start()
        gripper = boundary[0].groups["left_gripper"]
        right = boundary[0].groups["right_gripper"]
        assert gripper.modes == ["mit"]
        assert gripper.enable_count == 1
        assert set(gripper.mit_commands) == {(0.0, 0.0, 0.4, 0.0, 0.0)}
        assert right.modes == right.mit_commands == []

        module.command({"arm/left_gripper/position": 0.9})
        assert gripper.commands == []
        assert gripper.mit_commands[-1] == (0.0, 0.0, 0.4, 0.0, 0.0)

        # A passive jaw follows the hand, so an opening target is ignored
        # rather than validated as an opening.
        module.write({"arm/left_gripper/position": 2.0})
        assert gripper.commands == []
        assert gripper.mit_commands[-1] == (0.0, 0.0, 0.4, 0.0, 0.0)
    finally:
        module.stop()


def test_active_gripper_still_takes_normalized_openings(connection_class, boundary, mocker):
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.start")
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.serve_module_rpc")
    module = connection_class(deadman_timeout_s=10)
    commands, readings = LocalTransport(), LocalTransport()
    module.set_transport("position_command", commands)
    module.set_transport("joint_state", readings)
    try:
        module.start()
        module.command({"arm/right_gripper/position": 0.75})
        assert module._arm._grippers["right_gripper"].commands[-1] == 0.75
    finally:
        module.stop()


def test_teaching_requires_teach_damping(connection_class):
    module = connection_class(teach=True)
    try:
        with pytest.raises(ValueError, match="teach_kd"):
            module.start()
    finally:
        module.stop()


def test_robot_class_attribute_is_the_connection_topology(connection_class):
    robot: DamiaoRobot = connection_class.robot
    module = connection_class()
    try:
        assert module.joint_names == robot.joint_names
        assert module.arm_joints == robot.arm_joints
        assert module.gripper_joints == robot.gripper_joints
    finally:
        module.stop()


def test_connection_config_accepts_position_commands_only():
    from dimos.control.contract.keys import Interface

    with pytest.raises(ValueError, match="position commands only"):
        common.DamiaoConnectionConfig(
            source="arm", joint_commands=(Interface.POSITION, Interface.VELOCITY)
        )
    with pytest.raises(ValueError, match="position commands only"):
        common.DamiaoConnectionConfig(source="arm", pd=True)


def test_can_selectors_must_match_the_robot_buses(connection_class):
    class Missing(connection_class):
        def can_ports(self):
            return ["can0"]

    class Duplicate(connection_class):
        def can_ports(self):
            return ["can0", "can0"]

    for candidate, message in (
        (Missing, "cover exactly"),
        (Duplicate, "must be distinct"),
    ):
        module = candidate()
        try:
            with pytest.raises(ValueError, match=message):
                module.connect()
        finally:
            module.stop()


def test_write_and_read_before_connect_are_refused(connection_class):
    module = connection_class()
    try:
        with pytest.raises(RuntimeError, match="inactive"):
            module.write({"arm/joint1/position": 0.1})
        module.halt()
        with pytest.raises(RuntimeError, match="disconnected"):
            module._read_state()
    finally:
        module.stop()


def test_feedback_fault_reports_a_failing_disable(make_rig, boundary):
    module, _, _ = make_rig()
    boundary[0].disable_error = RuntimeError("unplugged")
    boundary[0].refresh_error = RuntimeError("lost feedback")
    wait_until(lambda: module._fault is not None, timeout=2)
    assert "disable failed" in module.status().last_error
    boundary[0].disable_error = None
    boundary[0].refresh_error = None
