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

"""Connection ports, command acceptance and session shutdown."""

from __future__ import annotations

import threading

import numpy as np
import pytest

from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.common import damiao_connection as common
from dimos.utils.testing.waiting import wait_until


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
    session = module._session
    module.stop()
    module.stop()
    assert boundary[0].events.count("disable") == 1
    assert session.arms == session.grippers == {}
    assert module._session is None
    assert not module._reader.is_alive()
    assert not module.status().connected


def test_failed_enable_rolls_back_and_preserves_error(connection_class, boundary, mocker):
    native = boundary[0]
    native.enable_error = RuntimeError("calibration failed")
    native.disable_error = RuntimeError("disable failed")
    module = connection_class()
    with pytest.raises(RuntimeError, match="calibration failed"):
        module.start()
    assert module._session is None
    assert module._reader is None
    assert not module.status().connected
    assert "startup cleanup failed: RuntimeError: disable failed" in module.status().last_error
    assert "disable" in native.events
    module.stop()


def test_initial_bad_feedback_never_enables(connection_class, boundary, mocker):
    mocker.patch.object(
        common.PhysicalModel, "normalize", side_effect=RuntimeError("invalid initial feedback")
    )
    module = connection_class()
    with pytest.raises(RuntimeError, match="invalid initial feedback"):
        module.start()
    assert "enable" not in boundary[0].events
    assert module._session is None


def test_connect_does_not_enable_or_accept_commands(connection_class, boundary):
    module = connection_class()
    try:
        module.connect()
        module.command({"arm/joint1/position": 0.7})
        assert "enable" not in boundary[0].events
        assert boundary[0].groups["left_arm"].commands == []
    finally:
        module.stop()


def test_mock_session_never_constructs_native_bus(make_rig, boundary):
    module, commands, readings = make_rig(native=False)
    commands.publish(JointState(name=["arm/joint1"], position=[0.8]))
    wait_until(lambda: readings.sent[-1].position[0] == 0.8, timeout=2)
    boundary[1].assert_not_called()
    assert boundary[0].events == []


def test_simulation_ignores_explicit_devices(connection_class, boundary, monkeypatch):
    module = connection_class()
    monkeypatch.setattr(module.config.g, "simulation", "mujoco")
    try:
        module.connect()
        boundary[1].assert_not_called()
    finally:
        module.stop()


def test_topology_mismatch_fails_before_bus(connection_class, boundary, robot_spec):
    module = connection_class(model=robot_spec.planning.model)
    with pytest.raises(ValueError, match="topology"):
        module.start()
    boundary[1].assert_not_called()


def test_disable_failure_still_releases_all_session_handles(make_rig, boundary):
    module, _, _ = make_rig()
    session = module._session
    boundary[0].disable_error = RuntimeError("disable failed")
    with pytest.raises(RuntimeError, match="disable failed"):
        module.stop()
    assert session.arms == session.grippers == {}
    assert module._session is None
    assert not module.status().connected
    module.stop()


def test_no_native_or_pinocchio_import_for_mock_session(connection_class, mocker):
    native = mocker.patch.object(common, "_native", side_effect=AssertionError("native import"))
    physical = mocker.patch.object(
        common, "PhysicalModel", side_effect=AssertionError("physical import")
    )
    module = connection_class(native=False)
    try:
        module.connect()
        assert module._read_state()[0] == [0.2, 0.2, 0.2, 0.4, 0.4]
        native.assert_not_called()
        physical.assert_not_called()
    finally:
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
        assert module._session is None
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
    assert module._session is None
