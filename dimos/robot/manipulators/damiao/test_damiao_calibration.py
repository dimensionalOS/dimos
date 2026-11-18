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

"""Calibration files applied at start, and the tuning RPCs a calibration tool drives."""

from __future__ import annotations

from dataclasses import replace
from datetime import datetime, timezone
import hashlib
import importlib
import json
import math
from pathlib import Path
import threading
import time
from typing import Any

import numpy as np
import pytest

from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.calibration._calibration_test_helpers import (
    offline_robot_assets,
    require_model,
)
from dimos.robot.manipulators.calibration._inertial import LinkInertial
from dimos.robot.manipulators.calibration.hooks import ArmTuningHooks
from dimos.robot.manipulators.calibration.schema import (
    ArmCalibration,
    ArmCalibrationError,
    GravityCalibration,
    JointGains,
    LinkGravity,
    RobotModelRef,
    gravity_links_from_inertial,
    save_calibration,
)
from dimos.robot.manipulators.calibration.spec import model_sha256
from dimos.robot.manipulators.damiao import connection as common
from dimos.robot.manipulators.damiao._damiao_native_test_helpers import (
    LocalTransport,
    drive_and_record,
    make_calibrated_test_robot,
)
from dimos.robot.manipulators.damiao._damiao_test_helpers import _local_test_model
from dimos.robot.manipulators.damiao.arm import PhysicalModel
from dimos.robot.manipulators.dual_openyam.calibration_workspace import (
    dual_openyam_calibration_specs,
)
from dimos.robot.manipulators.dual_openyam.config import DUAL_OPENYAM
from dimos.robot.manipulators.dual_openyam.connection import DualOpenYamConnection
from dimos.utils.testing.waiting import wait_until

RIG = make_calibrated_test_robot()
CREATED = datetime(2026, 11, 17, 9, 30, tzinfo=timezone.utc)


def _calibration(
    arm_id: str = "rig/left",
    *,
    sha256: str | None = None,
    gains: dict[str, tuple[float, float]] | None = None,
    masses: dict[str, float] | None = None,
    friction: dict[str, float] | None = None,
) -> ArmCalibration:
    return ArmCalibration(
        arm_id=arm_id,
        robot_model=RobotModelRef(name="rig", sha256=sha256 or model_sha256(RIG.gravity_model)),
        created=CREATED,
        gains={joint: JointGains(kp=kp, kd=kd) for joint, (kp, kd) in gains.items()}
        if gains
        else None,
        gravity=GravityCalibration(
            links={
                link: LinkGravity(mass_kg=mass, com_m=(0.1, 0.0, 0.0))
                for link, mass in masses.items()
            }
        )
        if masses
        else None,
        friction_nm=friction,
    )


@pytest.fixture
def rig_class(connection_class):
    class Rig(connection_class):
        pass

    Rig.robot = RIG
    return Rig


@pytest.fixture
def start(rig_class, boundary, mocker):
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.start")
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.serve_module_rpc")
    # No calibration tool parks here; a stop need not wait a second for one.
    mocker.patch.object(common, "TUNING_PARK_GRACE_S", 0.02)
    made = []

    def make(connection_type=None, *, commands=None, **kwargs):
        kwargs.setdefault("deadman_timeout_s", 10)
        module = (connection_type or rig_class)(**kwargs)
        readings = LocalTransport()
        module.set_transport("position_command", commands or LocalTransport())
        module.set_transport("joint_state", readings)
        made.append(module)
        module.start()
        wait_until(lambda: bool(readings.sent), timeout=2)
        return module

    yield make
    for module in made:
        module.stop()


def _rows(boundary, bus: str) -> np.ndarray:
    return boundary[0].groups[bus].commands[-1]


def _messages(spy) -> list[str]:
    return [str(call.args[0]) for call in spy.call_args_list]


def test_auto_applies_each_arms_saved_file_and_logs_one_line(
    start, boundary, calibration_root, mocker
):
    path = save_calibration(
        calibration_root / "rig/left",
        _calibration(gains={"joint1": (60.0, 3.0)}, masses={"link2": 2.0}),
    )
    info = mocker.spy(common.logger, "info")
    module = start()
    module.command({"arm/joint1/position": 0.2})
    expected = PhysicalModel(
        RIG.gravity_model,
        RIG.arm_joint_names,
        gravity_comp=True,
        inertial_overrides={"link2": LinkInertial(mass_kg=2.0, com_m=(0.1, 0.0, 0.0))},
    ).gravity(np.full(3, 0.2))
    np.testing.assert_equal(_rows(boundary, "left_arm")[:, :2], [[60, 3], [10, 1.5]])
    np.testing.assert_allclose(_rows(boundary, "left_arm")[:, 4], expected[:2])
    np.testing.assert_allclose(_rows(boundary, "right_arm")[:, 4], expected[2:])
    assert module.status().calibration == {
        "rig/left": {
            "path": str(path),
            "created": "2026-11-17T09:30:00+00:00",
            "gains": ["joint1"],
            "links": ["link2"],
        },
        "rig/right": None,
    }
    lines = [line for line in _messages(info) if "rig/left" in line]
    assert len(lines) == 1
    assert str(path) in lines[0] and "2026-11-17 09:30 UTC" in lines[0]
    assert "['joint1']" in lines[0] and "['link2']" in lines[0]
    assert "No calibration for rig/right, using URDF defaults" in _messages(info)


# What each connection sent before calibration existed, recorded with
# ``drive_and_record`` from the code at the commit in "recorded_from".
ROWS_BEFORE_CALIBRATION: dict[str, Any] = json.loads(
    Path(__file__).with_name("damiao_rows_before_calibration.json").read_text()
)["cases"]


@pytest.mark.parametrize("case", list(ROWS_BEFORE_CALIBRATION))
def test_auto_without_a_saved_file_sends_what_was_sent_before_calibration(
    start, boundary, tmp_path, mocker, case
):
    recorded = ROWS_BEFORE_CALIBRATION[case]
    module_name, _, qualname = recorded["connection"].partition(":")
    connection_type = getattr(importlib.import_module(module_name), qualname)
    robot = connection_type.robot
    info = mocker.spy(common.logger, "info")
    boundary[0].default_position = 0.0
    with offline_robot_assets():
        require_model(robot.gravity_model, case)
        module = start(
            connection_type,
            **recorded["ports"],
            teach=recorded["teach"],
            # One reading at start, then only the ones the sequence takes.
            state_rate_hz=0.001,
            source="arm",
            model=_local_test_model(connection_type, tmp_path / "control.urdf"),
        )
        for arm_id in robot.arm_ids():
            assert f"No calibration for {arm_id}, using URDF defaults" in _messages(info)
        assert module.status().calibration == dict.fromkeys(robot.arm_ids())
        sent = drive_and_record(module, boundary[0], recorded["pose"])
    expected = recorded["sent"]
    assert list(sent) == list(expected)
    # Every native call in order: the same sends, ticks and reads.
    assert sent["events"] == expected["events"]
    for name, group in expected.items():
        if name == "events":
            continue
        if "rows" not in group:
            # A jaw: its openings, or its passive MIT frames while teaching.
            assert sent[name] == group, name
            continue
        assert sent[name]["modes"] == group["modes"]
        assert len(sent[name]["rows"]) == len(group["rows"]), name
        for rows, want in zip(sent[name]["rows"], group["rows"], strict=True):
            rows, want = np.asarray(rows), np.asarray(want)
            # kp, kd, target and velocity exactly as recorded; gravity to rounding.
            np.testing.assert_equal(rows[:, :4], want[:, :4])
            np.testing.assert_allclose(rows[:, 4], want[:, 4], rtol=1e-12, atol=1e-12)


def test_an_explicit_file_applies_to_the_arm_it_names(start, boundary, tmp_path):
    path = save_calibration(
        tmp_path / "trial.json", _calibration("rig/right", gains={"joint3": (30.0, 1.0)})
    )
    module = start(calibration=str(path))
    module.command({"arm/joint1/position": 0.2})
    np.testing.assert_equal(_rows(boundary, "right_arm")[0, :2], [30, 1])
    np.testing.assert_equal(_rows(boundary, "left_arm")[:, :2], [[80, 5], [10, 1.5]])
    calibration = module.status().calibration
    assert calibration["rig/left"] is None
    assert calibration["rig/right"]["path"] == str(path)


def _write_text(path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)


@pytest.mark.parametrize(
    "write,reason",
    [
        (
            lambda root: save_calibration(
                root / "rig/left", _calibration(sha256="0" * 64, gains={"joint1": (60.0, 3.0)})
            ),
            "different robot model",
        ),
        (
            lambda root: _write_text(root / "rig/left/calibration.json", "{not json"),
            "not valid JSON",
        ),
        (
            lambda root: save_calibration(
                root / "rig/left", _calibration("rig/right", gains={"joint3": (60.0, 3.0)})
            ),
            "is for arm 'rig/right'",
        ),
        (
            lambda root: save_calibration(
                root / "rig/left", _calibration(gains={"joint3": (60.0, 3.0)})
            ),
            "joints of no arm 'rig/left' drives: ['joint3']",
        ),
        (
            lambda root: save_calibration(
                root / "rig/left",
                _calibration(gains={"joint1": (60.0, 3.0)}, masses={"ghost": 1.0}),
            ),
            "links arm 'rig/left' does not move: ['ghost']",
        ),
        (
            # link3 hangs off joint3, which the right arm drives.
            lambda root: save_calibration(
                root / "rig/left", _calibration(masses={"link2": 1.2, "link3": 1.0})
            ),
            "links arm 'rig/left' does not move: ['link3']; it moves ['link1', 'link2']",
        ),
        (
            # 50 kg on link2 would need about 140 N m from joint1, a 28 N m motor.
            lambda root: save_calibration(root / "rig/left", _calibration(masses={"link2": 50.0})),
            "N m of gravity torque on joint1 at the measured pose, more than its motor's "
            "rated 28 N m",
        ),
    ],
    ids=[
        "other-model",
        "corrupt",
        "other-arm",
        "other-arms-joint",
        "unknown-link",
        "other-arms-link",
        "too-heavy",
    ],
)
def test_a_file_that_does_not_fit_is_ignored_with_a_warning(
    start, boundary, calibration_root, mocker, write, reason
):
    write(calibration_root)
    warning = mocker.spy(common.logger, "warning")
    module = start()
    module.command({"arm/joint1/position": 0.2})
    np.testing.assert_equal(_rows(boundary, "left_arm")[:, :2], [[80, 5], [10, 1.5]])
    assert module.status().calibration == {"rig/left": None, "rig/right": None}
    lines = [line for line in _messages(warning) if "rig/left" in line]
    assert len(lines) == 1
    assert lines[0].startswith("Ignoring the calibration for rig/left, using URDF defaults: ")
    assert reason in lines[0]


def test_off_never_reads_a_saved_file(start, boundary, calibration_root, mocker):
    save_calibration(calibration_root / "rig/left", _calibration(gains={"joint1": (60.0, 3.0)}))
    load = mocker.spy(common, "load_calibration")
    module = start(calibration="off")
    module.command({"arm/joint1/position": 0.2})
    load.assert_not_called()
    np.testing.assert_equal(_rows(boundary, "left_arm")[0, :2], [80, 5])
    assert module.status().calibration == {"rig/left": None, "rig/right": None}


def test_an_unnamed_robot_reads_no_calibration(make_rig, mocker):
    load = mocker.spy(common, "load_calibration")
    module, _, _ = make_rig(tuning=True)
    load.assert_not_called()
    assert module.status().calibration == {}
    assert module.tuning_arm_ids() == []


def test_friction_in_a_file_is_not_applied_yet(start, boundary, calibration_root, mocker):
    save_calibration(
        calibration_root / "rig/left",
        _calibration(gains={"joint1": (60.0, 3.0)}, friction={"joint1": 0.3}),
    )
    info = mocker.spy(common.logger, "info")
    module = start()
    assert module._arm.feedforward == {}
    assert any("friction is not applied yet" in line for line in _messages(info))


def _active(module) -> bool:
    return module._tuning_session is not None


def test_tuning_hooks_round_trip(start, boundary):
    module = start(tuning=True)
    native = boundary[0]
    assert isinstance(module, ArmTuningHooks)
    assert module.tuning_arm_ids() == ["rig/left", "rig/right"]
    assert module.tuning_joint_names() == ["joint1", "joint2", "joint3"]
    session = module.tuning_begin()
    assert session["lease_s"] == 2.0 and len(session["session_id"]) == 32

    # Each setter returns how the joint is driven now, as clamped.
    assert module.tuning_set_gains("joint2", 900, 2) == {
        "kp": 500.0,
        "kd": 2.0,
        "floating": False,
        "float_kd": None,
        "feedforward_nm": 0.0,
    }
    np.testing.assert_equal(_rows(boundary, "left_arm")[1, :2], [500, 2])
    assert module.tuning_set_float("joint3", True, kd=0.4)["float_kd"] == 0.4
    # A fifth of joint1's rated 28 N m.
    assert module.tuning_set_feedforward("joint1", 9.0)["feedforward_nm"] == pytest.approx(5.6)
    gains = module.tuning_get_gains()
    assert gains["joint3"]["floating"] and gains["joint1"]["feedforward_nm"] == pytest.approx(5.6)
    np.testing.assert_equal(_rows(boundary, "right_arm")[0, :2], [0, 0.4])
    module.tuning_set_float("joint3", False)
    module.tuning_set_feedforward("joint1", 0.0)
    module.tuning_reset_gains()
    assert module.tuning_get_gains()["joint2"]["kp"] == 10.0
    np.testing.assert_equal(_rows(boundary, "right_arm")[0, :2], [20, 0.6])

    native.groups["left_arm"].tau[:] = [0.7, -0.3]
    wait_until(lambda: module.tuning_read()["joint1"]["effort"] == 0.7, timeout=2)
    sample = module.tuning_read()
    assert list(sample) == ["joint1", "joint2", "joint3", "left_finger", "right_finger"]
    assert sample["joint2"]["effort"] == -0.3 and sample["joint2"]["position"] == 0.2
    assert sample["joint1"]["ts"] > 0

    assert module.tuning_command({"joint1": 0.3, "left_finger": 0.02}) == {
        "ok": True,
        "error": None,
    }
    assert _rows(boundary, "left_arm")[0, 2] == 0.3
    assert native.groups["left_gripper"].commands[-1] == pytest.approx(0.5)
    assert module.status().last_command_time is not None
    module.tuning_end()
    assert not _active(module)
    # Ending a session that is not there does nothing.
    module.tuning_end()


# Every tuning RPC, with arguments it would accept.
TUNING_CALLS: dict[str, tuple[Any, ...]] = {
    "tuning_begin": (),
    "tuning_end": (),
    "tuning_arm_ids": (),
    "tuning_joint_names": (),
    "tuning_get_gains": (),
    "tuning_set_gains": ("joint1", 60.0, 3.0),
    "tuning_reset_gains": (),
    "tuning_set_float": ("joint3", True),
    "tuning_set_feedforward": ("joint1", 0.1),
    "tuning_reset_all": (),
    "tuning_read": (),
    "tuning_command": ({"joint1": 0.1},),
    "tuning_apply_calibration": ("off",),
    "tuning_applied_calibration": (),
}


def test_every_tuning_rpc_is_refused_unless_the_connection_is_deployed_for_tuning(start, boundary):
    assert set(TUNING_CALLS) == {name for name in dir(ArmTuningHooks) if name.startswith("tuning_")}
    module = start()
    assert not module.config.tuning
    sent = len(boundary[0].groups["left_arm"].commands)
    for name, args in TUNING_CALLS.items():
        with pytest.raises(RuntimeError, match="tuning is off"):
            getattr(module, name)(*args)
    assert len(boundary[0].groups["left_arm"].commands) == sent


def test_setters_and_commands_need_a_tuning_session(start, boundary):
    module = start(tuning=True)
    for name in (
        "tuning_set_gains",
        "tuning_set_float",
        "tuning_set_feedforward",
        "tuning_command",
    ):
        with pytest.raises(RuntimeError, match="no tuning session is active; call tuning_begin"):
            getattr(module, name)(*TUNING_CALLS[name])
    # Reading, resetting and applying a calibration do not.
    module.tuning_get_gains()
    module.tuning_reset_all()
    module.tuning_reset_gains()
    module.tuning_apply_calibration("off")
    module.tuning_begin()
    with pytest.raises(RuntimeError, match="already active"):
        module.tuning_begin()
    # Like any tuning call that raises, that ended the session.
    with pytest.raises(RuntimeError, match="the last one ended: tuning_begin raised"):
        module.tuning_set_gains("joint1", 60.0, 3.0)


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
def test_a_lease_is_a_positive_number_of_seconds_up_to_thirty(start, boundary, lease, error):
    module = start(tuning=True)
    with pytest.raises(error, match="lease_s"):
        module.tuning_begin(lease)
    assert not _active(module)


def test_bad_arguments_are_refused_and_end_the_session(start, boundary):
    module = start(tuning=True)
    for call, error, match in (
        (lambda: module.tuning_command({"joint1": 4.0}), ValueError, "limit"),
        (lambda: module.tuning_command({"nope": 0.0}), ValueError, "not joints"),
        (lambda: module.tuning_command({"joint1": "0.1"}), TypeError, "must be a number"),
        (lambda: module.tuning_set_gains("left_finger", 1.0, 1.0), ValueError, "not an arm joint"),
        (lambda: module.tuning_set_gains("joint1", "80", 1.0), TypeError, "must be a number"),
        (lambda: module.tuning_set_float("joint3", "false"), TypeError, "True or False"),
        (lambda: module.tuning_set_float("joint3", 0), TypeError, "True or False"),
        (lambda: module.tuning_set_feedforward("joint1", float("nan")), ValueError, "finite"),
        (lambda: module.tuning_apply_calibration(force="yes"), TypeError, "True or False"),
    ):
        module.tuning_begin()
        module.tuning_set_float("joint3", True)
        with pytest.raises(error, match=match):
            call()
        assert not _active(module) and module._arm.floating == {}
    assert _rows(boundary, "left_arm")[0, 2] == 0.2


def test_a_tuning_session_ignores_position_command_and_says_so_once(start, boundary, mocker):
    commands = LocalTransport()
    module = start(tuning=True, commands=commands)
    warning = mocker.spy(common.logger, "warning")
    info = mocker.spy(common.logger, "info")
    commands.publish(JointState(name=["arm/joint1"], position=[0.5]))
    assert _rows(boundary, "left_arm")[0, 2] == 0.5
    module.tuning_begin()
    for target in (0.6, 0.7, 0.8):
        commands.publish(JointState(name=["arm/joint1"], position=[target]))
    assert _rows(boundary, "left_arm")[0, 2] == 0.5
    assert len([line for line in _messages(warning) if "ignoring position_command" in line]) == 1
    module.tuning_end()
    ended = [line for line in _messages(info) if "ended (tuning_end)" in line]
    assert len(ended) == 1 and "3 position commands ignored" in ended[0]
    commands.publish(JointState(name=["arm/joint1"], position=[0.9]))
    assert _rows(boundary, "left_arm")[0, 2] == 0.9


def test_a_released_joint_stays_where_the_hand_left_it_though_a_coordinator_streams(
    start, boundary
):
    commands = LocalTransport()
    module = start(tuning=True, commands=commands)
    module.tuning_begin()
    module.tuning_set_float("joint3", True)
    boundary[0].groups["right_arm"].q[:] = [0.9]
    # A coordinator still streaming the target it had before the float.
    commands.publish(JointState(name=["arm/joint3"], position=[-0.5]))
    module.tuning_set_float("joint3", False)
    commands.publish(JointState(name=["arm/joint3"], position=[-0.5]))
    np.testing.assert_equal(_rows(boundary, "right_arm")[0, :3], [20, 0.6, 0.9])


def test_a_lapsed_lease_undoes_every_live_change_and_logs_one_line(start, boundary, mocker):
    module = start(tuning=True)
    warning = mocker.spy(common.logger, "warning")
    module.tuning_begin(lease_s=0.2)
    module.tuning_set_gains("joint2", 40.0, 2.0)
    module.tuning_set_float("joint3", True)
    module.tuning_set_feedforward("joint1", 1.0)
    boundary[0].groups["right_arm"].q[:] = [0.7]
    wait_until(lambda: not _active(module), timeout=2)
    arm = module._arm
    assert arm.floating == {} and arm.feedforward == {}
    assert arm.get_gains()["joint2"] == (10.0, 1.5)
    np.testing.assert_equal(_rows(boundary, "right_arm")[0, :3], [20, 0.6, 0.7])
    np.testing.assert_equal(_rows(boundary, "left_arm")[:, :2], [[80, 5], [10, 1.5]])
    lines = [line for line in _messages(warning) if "ended" in line]
    assert len(lines) == 1
    assert "no tuning call within its lease of 0.2 s" in lines[0]
    assert "held ['joint3'] where they are" in lines[0]
    assert "dropped the feed-forward of ['joint1']" in lines[0]
    assert "gains of ['joint2'] back to the calibration" in lines[0]
    with pytest.raises(RuntimeError, match="the last one ended: no tuning call within its lease"):
        module.tuning_set_gains("joint1", 60.0, 3.0)


def test_every_tuning_call_renews_the_lease(start, boundary):
    module = start(tuning=True)
    module.tuning_begin(lease_s=0.3)
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        module.tuning_read()
        time.sleep(0.05)
    assert _active(module)
    module.tuning_set_gains("joint1", 60.0, 3.0)


def test_the_deadman_halting_the_arm_ends_the_session(start, boundary):
    module = start(tuning=True, deadman_timeout_s=0.05)
    module.tuning_begin()
    module.tuning_set_float("joint3", True)
    assert module.tuning_command({"joint1": 0.3})["ok"]
    # No more targets: the deadman halts the arm, and a halt ends the session.
    wait_until(lambda: module.status().deadman_fired, timeout=2)
    assert not _active(module) and module._arm.floating == {}
    with pytest.raises(RuntimeError, match="the last one ended: the connection halted"):
        module.tuning_set_float("joint3", True)


def test_a_command_the_motors_did_not_get_is_not_ok_and_ends_the_session(start, boundary):
    module = start(tuning=True)
    module.tuning_begin()
    boundary[0].groups["left_arm"].error = RuntimeError("bus off")
    result = module.tuning_command({"joint1": 0.3})
    assert not result["ok"] and "bus off" in result["error"]
    assert not _active(module)
    boundary[0].groups["left_arm"].error = None


def test_a_fault_ends_the_session(start, boundary):
    module = start(tuning=True)
    module.tuning_begin()
    module.tuning_set_feedforward("joint1", 1.0)
    boundary[0].refresh_error = RuntimeError("lost feedback")
    wait_until(lambda: module._fault is not None, timeout=2)
    assert not _active(module) and module._arm.feedforward == {}
    boundary[0].refresh_error = None


def test_stopping_ends_the_session(start, boundary):
    module = start(tuning=True)
    module.tuning_begin()
    module.tuning_set_float("joint3", True)
    arm = module._arm
    module.stop()
    assert not _active(module) and arm.floating == {}


def test_tuning_refuses_a_faulted_or_stopped_connection(start, boundary):
    module = start(tuning=True)
    boundary[0].refresh_error = RuntimeError("lost feedback")
    wait_until(lambda: module._fault is not None, timeout=2)
    with pytest.raises(RuntimeError, match="lost feedback"):
        module.tuning_set_gains("joint1", 1.0, 1.0)
    boundary[0].refresh_error = None
    module.stop()
    for call in (
        module.tuning_get_gains,
        module.tuning_read,
        module.tuning_begin,
        lambda: module.tuning_command({"joint1": 0.1}),
    ):
        with pytest.raises(RuntimeError, match="disconnected"):
            call()


def test_tuning_applies_a_calibration_live_and_off_restores_the_defaults(
    start, boundary, calibration_root, tmp_path
):
    module = start(tuning=True)
    trial = save_calibration(
        tmp_path / "trial.json", _calibration(gains={"joint1": (60.0, 3.0)}, masses={"link2": 1.5})
    )
    applied = module.tuning_apply_calibration(str(trial))["arms"]
    assert applied["rig/left"]["links"] == ["link2"] and applied["rig/right"] is None
    # Sent at once, without a command.
    np.testing.assert_equal(_rows(boundary, "left_arm")[0, :2], [60, 3])
    assert module.status().calibration == applied

    mismatched = save_calibration(tmp_path / "bad.json", _calibration(sha256="0" * 64))
    with pytest.raises(ArmCalibrationError, match="different robot model"):
        module.tuning_apply_calibration(str(mismatched))
    np.testing.assert_equal(_rows(boundary, "left_arm")[0, :2], [60, 3])
    assert module.status().calibration == applied

    result = module.tuning_apply_calibration("off")
    assert result["arms"] == {"rig/left": None, "rig/right": None}
    np.testing.assert_equal(_rows(boundary, "left_arm")[0, :2], [80, 5])
    assert module._arm.inertial_overrides == {}

    saved = save_calibration(
        calibration_root / "rig/right", _calibration("rig/right", gains={"joint3": (30.0, 1.0)})
    )
    applied = module.tuning_apply_calibration()["arms"]
    assert applied["rig/right"]["path"] == str(saved) and applied["rig/left"] is None
    np.testing.assert_equal(_rows(boundary, "right_arm")[0, :2], [30, 1])


def test_applying_one_arms_file_live_keeps_what_the_other_arm_runs(start, boundary, tmp_path):
    module = start(tuning=True)
    right = save_calibration(
        tmp_path / "right.json", _calibration("rig/right", gains={"joint3": (30.0, 1.0)})
    )
    left = save_calibration(tmp_path / "left.json", _calibration(gains={"joint1": (60.0, 3.0)}))
    module.tuning_apply_calibration(str(right))
    applied = module.tuning_apply_calibration(str(left))["arms"]
    assert applied["rig/right"]["path"] == str(right) and applied["rig/left"]["path"] == str(left)
    np.testing.assert_equal(_rows(boundary, "right_arm")[0, :2], [30, 1])
    np.testing.assert_equal(_rows(boundary, "left_arm")[0, :2], [60, 3])
    assert module.status().calibration == applied


def test_a_live_calibration_that_steps_gravity_more_than_2_nm_needs_force(
    start, boundary, tmp_path
):
    module = start(tuning=True)
    heavier = save_calibration(tmp_path / "heavier.json", _calibration(masses={"link2": 3.0}))
    sent = len(boundary[0].groups["left_arm"].commands)
    with pytest.raises(
        ValueError, match=r"more than 2 N m: \{'joint1': .*near home, or pass force=True"
    ):
        module.tuning_apply_calibration(str(heavier))
    assert len(boundary[0].groups["left_arm"].commands) == sent
    assert module.status().calibration["rig/left"] is None

    result = module.tuning_apply_calibration(str(heavier), force=True)
    pose = np.full(3, 0.2)
    joints = RIG.arm_joint_names
    before = PhysicalModel(RIG.gravity_model, joints, gravity_comp=True).gravity(pose)
    after = PhysicalModel(
        RIG.gravity_model,
        joints,
        gravity_comp=True,
        inertial_overrides={"link2": LinkInertial(mass_kg=3.0, com_m=(0.1, 0.0, 0.0))},
    ).gravity(pose)
    assert result["gravity_step_nm"] == pytest.approx(
        dict(zip(joints, after - before, strict=True))
    )
    # joint3 carries none of link2.
    assert result["gravity_step_nm"]["joint3"] == 0.0
    assert result["arms"]["rig/left"]["links"] == ["link2"]
    np.testing.assert_allclose(_rows(boundary, "left_arm")[:, 4], after[:2])


def test_masses_over_a_motors_rating_at_home_are_refused(start, boundary, tmp_path, rig_class):
    class HomeRig(rig_class):
        robot = replace(RIG, home_joints=[0.0, 0.0, 0.0])

    module = start(HomeRig, tuning=True)
    # Every link vertical: no gravity torque at the measured pose.
    boundary[0].groups["left_arm"].q[:] = [math.pi / 2, 0.0]
    boundary[0].groups["right_arm"].q[:] = [0.0]
    heavy = save_calibration(tmp_path / "heavy.json", _calibration(masses={"link2": 10.0}))
    with pytest.raises(
        ArmCalibrationError, match="on joint1 at home, more than its motor's rated 28 N m"
    ):
        module.tuning_apply_calibration(str(heavy))
    assert module.status().calibration == {"rig/left": None, "rig/right": None}


def test_files_from_the_calibration_tools_load_into_dual_openyam(
    start, boundary, calibration_root, tmp_path
):
    with offline_robot_assets():
        require_model(DUAL_OPENYAM.gravity_model, "Dual OpenYAM")
        specs = dual_openyam_calibration_specs()
        for spec in specs:
            first, link = spec.joint_names[0], spec.gravity_links[-1]
            save_calibration(
                calibration_root / spec.arm_id,
                ArmCalibration.for_spec(
                    spec,
                    gains={first: JointGains(kp=60.0, kd=4.0)},
                    gravity=GravityCalibration(
                        links=gravity_links_from_inertial(
                            spec, {link: LinkInertial(mass_kg=0.5, com_m=(0.0, 0.0, 0.05))}
                        )
                    ),
                ),
            )
        boundary[0].default_position = 0.0
        module = start(
            DualOpenYamConnection,
            left_can_port="can0",
            right_can_port="can1",
            source="arm",
            model=_local_test_model(DualOpenYamConnection, tmp_path / "control.urdf"),
        )
    calibration = module.status().calibration
    assert list(calibration) == ["dual_openyam/left", "dual_openyam/right"]
    for spec in specs:
        assert calibration[spec.arm_id]["gains"] == [spec.joint_names[0]]
        assert calibration[spec.arm_id]["links"] == [spec.gravity_links[-1]]
    module.command({"arm/left_joint1/position": 0.0})
    np.testing.assert_equal(_rows(boundary, "left_arm")[0, :2], [60, 4])
    np.testing.assert_equal(_rows(boundary, "right_arm")[0, :2], [60, 4])


def test_the_applied_calibration_reports_the_masses_and_file_hash_it_runs(
    start, boundary, calibration_root
):
    path = save_calibration(calibration_root / "rig/left", _calibration(masses={"link2": 2.0}))
    sha = hashlib.sha256(path.read_bytes()).hexdigest()
    module = start(tuning=True)

    applied = module.tuning_applied_calibration()

    assert applied == {
        "rig/left": {
            "path": str(path),
            "created": "2026-11-17T09:30:00+00:00",
            "sha256": sha,
            "links": {"link2": {"mass_kg": 2.0, "com_m": [0.1, 0.0, 0.0]}},
        },
        "rig/right": None,
    }
    # A file rewritten on disk changes nothing the arm runs until it is applied.
    save_calibration(path, _calibration(masses={"link2": 2.5}))
    assert module.tuning_applied_calibration() == applied
    assert json.loads(json.dumps(applied)) == applied


def _park_while_stopping(module, order: list[str], stream_s: float) -> threading.Thread:
    """Stop ``module`` on a thread while this one parks, as a calibration tool does."""

    def stop() -> None:
        module.stop()
        order.append("stopped")

    stopper = threading.Thread(target=stop)
    stopper.start()
    time.sleep(0.05)
    module.tuning_begin()
    order.append("park began")
    deadline = time.monotonic() + stream_s
    try:
        while time.monotonic() < deadline:
            module.tuning_command({"joint1": 0.2})
            time.sleep(0.02)
    except RuntimeError:
        order.append("cut off")
    else:
        order.append("parked")
        module.tuning_end()
    return stopper


def test_stopping_waits_for_the_calibration_tool_to_park_before_disabling(start, boundary, mocker):
    module = start(tuning=True)
    mocker.patch.object(common, "TUNING_PARK_GRACE_S", 0.3)
    order: list[str] = []
    disable = module._arm.disable

    def disabled() -> None:
        order.append("disabled")
        disable()

    mocker.patch.object(module._arm, "disable", disabled)
    warning = mocker.spy(common.logger, "warning")

    stopper = _park_while_stopping(module, order, stream_s=0.5)
    stopper.join(timeout=5.0)

    assert order == ["park began", "parked", "disabled", "stopped"]
    assert any(
        "waiting up to 10 s for the calibration tool to park the arm" in line
        for line in _messages(warning)
    )


def test_a_park_that_overruns_the_wait_is_cut_off_and_the_arm_disabled(start, boundary, mocker):
    module = start(tuning=True)
    mocker.patch.object(common, "TUNING_PARK_GRACE_S", 0.2)
    mocker.patch.object(common, "TUNING_PARK_WAIT_S", 0.3)
    error = mocker.spy(common.logger, "error")
    order: list[str] = []

    stopper = _park_while_stopping(module, order, stream_s=2.0)
    stopper.join(timeout=5.0)

    assert order == ["park began", "stopped", "cut off"]
    assert any("did not end its tuning session within 0.3 s" in line for line in _messages(error))
    assert module._tuning_ended == "the connection halted"
