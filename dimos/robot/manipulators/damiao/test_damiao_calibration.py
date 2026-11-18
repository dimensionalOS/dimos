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

"""Calibration files applied at start, and the arm config contract a calibration tool drives."""

from __future__ import annotations

from dataclasses import replace
from datetime import datetime, timezone
import hashlib
import importlib
import json
import math
from pathlib import Path
import re
import threading
import time
from typing import Any

import numpy as np
from pydantic import ValidationError
import pytest

from dimos.control.connection import connection_module
from dimos.control.connection.arm_config import AppliedFile, CommandResult, ConfigPatch
from dimos.control.connection.connection_module import ConnectionStatus
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.calibration._calibration_test_helpers import (
    offline_robot_assets,
    require_model,
)
from dimos.robot.manipulators.calibration._inertial import LinkInertial
from dimos.robot.manipulators.calibration.robots import calibration_keys
from dimos.robot.manipulators.calibration.schema import (
    ArmCalibration,
    ArmCalibrationError,
    GravityCalibration,
    JointGains,
    LinkGravity,
    RobotModelRef,
    gravity_links_from_inertial,
    load_calibration,
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
from dimos.utils.testing.cbor import roundtrip
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
    mocker.patch.object(connection_module, "CONFIG_PARK_GRACE_S", 0.02)
    made = []

    def make(connection_type=None, *, commands=None, readings=None, **kwargs):
        kwargs.setdefault("deadman_timeout_s", 10)
        module = (connection_type or rig_class)(**kwargs)
        readings = readings or LocalTransport()
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


def _files(module) -> dict[str, AppliedFile | None]:
    """The calibration file each arm runs, by arm id, as config_get reports it."""
    return {arm_id: state.file for arm_id, state in module.config_get().items()}


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
    assert _files(module) == {
        "rig/left": AppliedFile(
            path=str(path),
            created="2026-11-17T09:30:00+00:00",
            sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        ),
        "rig/right": None,
    }
    assert set(module.config_get()["rig/left"].links) == {"link2"}
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
        assert _files(module) == dict.fromkeys(robot.arm_ids())
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
    files = _files(module)
    assert files["rig/left"] is None
    assert files["rig/right"] is not None and files["rig/right"].path == str(path)


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
        (
            # kp 0 would float joint1 for good, and a calibration tool would restore it.
            lambda root: save_calibration(
                root / "rig/left", _calibration(gains={"joint1": (0.0, 1.0)})
            ),
            "gains outside 0 < kp <= 500 and 0 <= kd <= 5: {'joint1': {'kp': 0.0, 'kd': 1.0}}",
        ),
        (
            lambda root: save_calibration(
                root / "rig/left", _calibration(gains={"joint2": (900.0, 9.0)})
            ),
            "gains outside 0 < kp <= 500 and 0 <= kd <= 5: {'joint2': {'kp': 900.0, 'kd': 9.0}}",
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
        "kp-zero",
        "past-the-mit-range",
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
    assert _files(module) == {"rig/left": None, "rig/right": None}
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
    assert _files(module) == {"rig/left": None, "rig/right": None}


def test_an_unnamed_robot_reads_no_calibration(make_rig, mocker):
    load = mocker.spy(common, "load_calibration")
    module, _, _ = make_rig(config_sessions=True)
    load.assert_not_called()
    assert module.config_keys().arms == {} and module.config_get() == {}


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
    return module._config_session is not None


def _begin(module, **kwargs: Any) -> str:
    """Start a config session; returns its id."""
    session = module.config_set(ConfigPatch(arm_id="rig/left"), **kwargs).session_id
    assert session is not None
    return session


def _left(module, session: str | None, **fields: Any):
    return module.config_set(ConfigPatch(arm_id="rig/left", **fields), session_id=session)


def _right(module, session: str | None, **fields: Any):
    return module.config_set(ConfigPatch(arm_id="rig/right", **fields), session_id=session)


def _float(module, session: str, kd: float = 0.5):
    return _right(module, session, gains={"joint3": JointGains(kp=0.0, kd=kd)})


def _base_messages(mocker, level: str):
    return mocker.spy(connection_module.logger, level)


def test_the_config_contract_round_trip(start, boundary, calibration_root):
    readings = LocalTransport()
    module = start(config_sessions=True, readings=readings)
    native = boundary[0]
    keys = module.config_keys()
    assert keys.arms == {"rig/left": ["joint1", "joint2"], "rig/right": ["joint3"]}
    assert keys.session == ["gains", "float", "torque_offset_nm"] and keys.gravity_compensation
    assert keys.persist_paths["rig/left"] == str(calibration_root / "rig/left/calibration.json")
    assert keys.source == "arm" and keys.sessions
    started = module.config_set(ConfigPatch(arm_id="rig/left"))
    session = started.session_id
    assert started.lease_s == 2.0 and session is not None and len(session) == 32

    # Each change returns what took effect, as clamped.
    applied = _left(module, session, gains={"joint2": JointGains(kp=900, kd=2)})
    assert applied.applied.gains == {"joint2": JointGains(kp=500.0, kd=2.0)}
    assert applied.clamped == ["gains.joint2.kp"] and applied.session_id == session
    np.testing.assert_equal(_rows(boundary, "left_arm")[1, :2], [500, 2])
    # kp 0 floats the joint with that damping, not a gain of 0.
    assert _float(module, session, kd=0.4).applied.gains == {"joint3": JointGains(kp=0.0, kd=0.4)}
    # A fifth of joint1's rated 28 N m.
    offset = _left(module, session, torque_offset_nm={"joint1": 9.0})
    assert offset.applied.torque_offset_nm == {"joint1": pytest.approx(5.6)}
    assert offset.clamped == ["torque_offset_nm.joint1"]
    state = module.config_get()
    assert state["rig/right"].floating == {"joint3": 0.4}
    assert state["rig/right"].gains["joint3"] == JointGains(kp=20.0, kd=0.6)
    assert state["rig/left"].torque_offset_nm == {"joint1": pytest.approx(5.6)}
    assert state["rig/left"].session_id == session and state["rig/left"].file is None
    np.testing.assert_equal(_rows(boundary, "right_arm")[0, :2], [0, 0.4])
    _right(module, session, gains={"joint3": JointGains(kp=20.0, kd=0.6)})
    assert module._arm.floating == {}
    _left(module, session, torque_offset_nm={"joint1": 0.0})
    module.config_reset(keep_session=True)
    assert module.config_get()["rig/left"].gains["joint2"].kp == 10.0
    np.testing.assert_equal(_rows(boundary, "right_arm")[0, :2], [20, 0.6])

    # Readings come from the joint_state stream, effort included.
    native.groups["left_arm"].tau[:] = [0.7, -0.3]
    wait_until(lambda: readings.sent[-1].effort[0] == 0.7, timeout=2)
    sample = readings.sent[-1]
    assert sample.name == [
        f"arm/{name}" for name in ("joint1", "joint2", "joint3", "left_finger", "right_finger")
    ]
    assert sample.effort[1] == -0.3 and sample.position[1] == 0.2 and sample.ts > 0

    result = module.config_command({"joint1": 0.3, "left_finger": 0.02}, session)
    assert result == CommandResult(ok=True)
    assert _rows(boundary, "left_arm")[0, 2] == 0.3
    assert native.groups["left_gripper"].commands[-1] == pytest.approx(0.5)
    assert module.status().last_command_time is not None
    # The base status, which an RPC client decodes without any Damiao code.
    assert type(module.status()) is ConnectionStatus
    assert module.status().config_session_id == session
    module.config_reset()
    assert not _active(module)
    # Ending a session that is not there does nothing.
    module.config_reset()


def test_a_plan_made_before_the_run_expects_what_the_connection_reports(start, boundary):
    # dimos arm calibrate --dry-run plans from calibration_keys, asking no connection.
    expected = calibration_keys(RIG)
    keys = start(config_sessions=True).config_keys()
    assert keys.model_copy(update={"persist_paths": {}}) == expected


def test_the_calls_that_change_the_arm_are_refused_unless_deployed_for_calibration(start, boundary):
    module = start()
    assert not module.config.config_sessions
    assert not any(name.startswith("tuning_") for name in module.rpcs)
    sent = len(boundary[0].groups["left_arm"].commands)
    for call in (
        lambda: module.config_set(ConfigPatch(arm_id="rig/left")),
        lambda: module.config_set(ConfigPatch(arm_id="rig/left", notes="x"), scope="persist"),
        lambda: module.config_reset(),
        lambda: module.config_command({"joint1": 0.1}, "0" * 32),
    ):
        with pytest.raises(RuntimeError, match="config sessions are off"):
            call()
    assert len(boundary[0].groups["left_arm"].commands) == sent
    # Reading still answers.
    assert not module.config_keys().sessions
    assert set(module.config_get()) == {"rig/left", "rig/right"}


def test_commands_need_a_config_session(start, boundary):
    module = start(config_sessions=True)
    with pytest.raises(RuntimeError, match="no config session is live"):
        module.config_command({"joint1": 0.1}, "0" * 32)
    # Reading, resetting and saving do not.
    module.config_get()
    module.config_reset()
    module.config_reset(keep_session=True)
    module.config_set(ConfigPatch(arm_id="rig/left", notes="x"), scope="persist")
    session = _begin(module)
    with pytest.raises(RuntimeError, match="a config session is live"):
        _begin(module)
    # Like any config call that raises, that ended the session.
    with pytest.raises(RuntimeError, match=r"the config session ended \(config_set raised"):
        _left(module, session, gains={"joint1": JointGains(kp=60.0, kd=3.0)})


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
    module = start(config_sessions=True)
    with pytest.raises(error, match="lease_s"):
        _begin(module, lease_s=lease)
    assert not _active(module)


def test_bad_arguments_are_refused_and_end_the_session(start, boundary):
    module = start(config_sessions=True)
    for call, error, match in (
        (lambda s: module.config_command({"joint1": 4.0}, s), ValueError, "limit"),
        (lambda s: module.config_command({"nope": 0.0}, s), ValueError, "not position-"),
        (lambda s: module.config_command({"joint1": "0.1"}, s), TypeError, "must be a number"),
        (
            lambda s: _left(module, s, gains={"left_finger": JointGains(kp=1.0, kd=1.0)}),
            ValueError,
            "not arm joints of rig/left",
        ),
        (
            lambda s: _left(module, s, gains={"joint3": JointGains(kp=1.0, kd=1.0)}),
            ValueError,
            "not arm joints of rig/left",
        ),
        (
            lambda s: module.config_set(
                {"arm_id": "rig/left", "gains": {"joint1": {"kp": "80", "kd": 1.0}}}, session_id=s
            ),
            ValidationError,
            "valid number",
        ),
        (
            lambda s: module.config_set(
                {"arm_id": "rig/left", "torque_offset_nm": {"joint1": float("nan")}},
                session_id=s,
            ),
            ValidationError,
            "finite",
        ),
        (
            lambda s: module.config_set(ConfigPatch(arm_id="rig/nope"), session_id=s),
            ValueError,
            "arms",
        ),
        (
            lambda s: module.config_set(
                ConfigPatch(arm_id="rig/left"), scope="persist", force="yes", session_id=s
            ),
            TypeError,
            "True or False",
        ),
    ):
        session = _begin(module)
        _float(module, session)
        with pytest.raises(error, match=match):
            call(session)
        assert not _active(module) and module._arm.floating == {}
    assert _rows(boundary, "left_arm")[0, 2] == 0.2


def test_a_config_session_drops_position_command_and_says_so_once(start, boundary, mocker):
    commands = LocalTransport()
    module = start(config_sessions=True, commands=commands)
    warning = _base_messages(mocker, "warning")
    info = _base_messages(mocker, "info")
    commands.publish(JointState(name=["arm/joint1"], position=[0.5]))
    assert _rows(boundary, "left_arm")[0, 2] == 0.5
    _begin(module)
    for target in (0.6, 0.7, 0.8):
        commands.publish(JointState(name=["arm/joint1"], position=[target]))
    assert _rows(boundary, "left_arm")[0, 2] == 0.5
    assert len([line for line in _messages(warning) if "dropping position commands" in line]) == 1
    module.config_reset()
    ended = [line for line in _messages(info) if "ended (config_reset)" in line]
    assert len(ended) == 1 and "3 position commands dropped" in ended[0]
    commands.publish(JointState(name=["arm/joint1"], position=[0.9]))
    assert _rows(boundary, "left_arm")[0, 2] == 0.9


def test_a_released_joint_stays_where_the_hand_left_it_though_a_coordinator_streams(
    start, boundary
):
    commands = LocalTransport()
    module = start(config_sessions=True, commands=commands)
    held = module.config_get()["rig/right"].gains["joint3"]
    session = _begin(module)
    _float(module, session)
    boundary[0].groups["right_arm"].q[:] = [0.9]
    # A coordinator still streaming the target it had before the float.
    commands.publish(JointState(name=["arm/joint3"], position=[-0.5]))
    _right(module, session, gains={"joint3": held})
    commands.publish(JointState(name=["arm/joint3"], position=[-0.5]))
    np.testing.assert_equal(_rows(boundary, "right_arm")[0, :3], [20, 0.6, 0.9])


def test_a_lapsed_lease_undoes_every_live_change_and_logs_one_line(start, boundary, mocker):
    module = start(config_sessions=True)
    warning = _base_messages(mocker, "warning")
    session = _begin(module, lease_s=0.2)
    _left(module, session, gains={"joint2": JointGains(kp=40.0, kd=2.0)})
    _float(module, session)
    _left(module, session, torque_offset_nm={"joint1": 1.0})
    boundary[0].groups["right_arm"].q[:] = [0.7]
    wait_until(lambda: not _active(module), timeout=2)
    # The session is gone before its undo finishes, both under the hardware lock.
    with module._hardware_lock:
        arm = module._arm
    assert arm.floating == {} and arm.feedforward == {}
    assert arm.get_gains()["joint2"] == (10.0, 1.5)
    np.testing.assert_equal(_rows(boundary, "right_arm")[0, :3], [20, 0.6, 0.7])
    np.testing.assert_equal(_rows(boundary, "left_arm")[:, :2], [[80, 5], [10, 1.5]])
    lines = [line for line in _messages(warning) if "ended" in line]
    assert len(lines) == 1
    assert "no config call within its lease of 0.2 s" in lines[0]
    assert "held ['joint3'] where they are" in lines[0]
    assert "dropped the feed-forward of ['joint1']" in lines[0]
    assert "gains of ['joint2'] back to the calibration" in lines[0]
    with pytest.raises(RuntimeError, match="the config session ended \\(no config call within"):
        _left(module, session, gains={"joint1": JointGains(kp=60.0, kd=3.0)})


def test_a_lapsed_lease_holds_a_softly_held_joint_where_the_hand_pushed_it(start, boundary):
    module = start(config_sessions=True)
    session = _begin(module, lease_s=0.2)
    _left(module, session, gains={"joint1": JointGains(kp=0.5, kd=0.1)})
    # A hand pushes the softly held joint1 far from its target of 0.2.
    boundary[0].groups["left_arm"].q[:] = [0.9, 0.2]
    wait_until(lambda: not _active(module), timeout=2)
    # Its file gains are back, holding it where it is, not pulling it back to 0.2.
    np.testing.assert_equal(_rows(boundary, "left_arm")[0, :3], [80, 5, 0.9])


def test_a_reset_after_a_command_holds_the_arm_where_it_is(start, boundary):
    module = start(config_sessions=True)
    session = _begin(module)
    _left(module, session, gains={"joint1": JointGains(kp=0.5, kd=0.1)})
    assert module.config_command({"joint1": 0.3}, session).ok
    # joint1 never reached 0.3: it is at 1.0.
    boundary[0].groups["left_arm"].q[:] = [1.0, 0.2]
    module.config_reset()
    np.testing.assert_equal(_rows(boundary, "left_arm")[0, :3], [80, 5, 1.0])
    # The same with no live change to undo: the commanded target goes too.
    session = _begin(module)
    assert module.config_command({"joint1": 0.3}, session).ok
    module.config_reset()
    np.testing.assert_equal(_rows(boundary, "left_arm")[0, :3], [80, 5, 1.0])


def test_every_config_call_renews_the_lease(start, boundary):
    module = start(config_sessions=True)
    session = _begin(module, lease_s=0.3)
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        module.config_get()
        time.sleep(0.05)
    assert _active(module)
    _left(module, session, gains={"joint1": JointGains(kp=60.0, kd=3.0)})


def test_the_deadman_halting_the_arm_ends_the_session(start, boundary):
    module = start(config_sessions=True, deadman_timeout_s=0.05)
    session = _begin(module)
    _float(module, session)
    assert module.config_command({"joint1": 0.3}, session).ok
    # No more targets: the deadman halts the arm, and a halt ends the session.
    wait_until(lambda: module.status().deadman_fired, timeout=2)
    assert not _active(module) and module._arm.floating == {}
    with pytest.raises(RuntimeError, match="the config session ended \\(the connection halted"):
        _float(module, session)


def test_a_command_the_motors_did_not_get_is_not_ok_and_ends_the_session(start, boundary):
    module = start(config_sessions=True)
    session = _begin(module)
    boundary[0].groups["left_arm"].error = RuntimeError("bus off")
    result = module.config_command({"joint1": 0.3}, session)
    assert not result.ok and result.error is not None and "bus off" in result.error
    assert not _active(module)
    boundary[0].groups["left_arm"].error = None


def test_a_fault_ends_the_session(start, boundary):
    module = start(config_sessions=True)
    session = _begin(module)
    _left(module, session, torque_offset_nm={"joint1": 1.0})
    boundary[0].refresh_error = RuntimeError("lost feedback")
    wait_until(lambda: module._fault is not None, timeout=2)
    assert not _active(module) and module._arm.feedforward == {}
    boundary[0].refresh_error = None


def test_stopping_ends_the_session(start, boundary, mocker):
    # Nobody parks: the stop gives up waiting at once.
    mocker.patch.object(connection_module, "CONFIG_PARK_WAIT_S", 0.1)
    module = start(config_sessions=True)
    session = _begin(module)
    _float(module, session)
    arm = module._arm
    module.stop()
    assert not _active(module) and arm.floating == {}
    assert module._config_ended == "the connection halted"


def test_a_faulted_or_stopped_connection_is_refused(start, boundary):
    module = start(config_sessions=True)
    boundary[0].refresh_error = RuntimeError("lost feedback")
    wait_until(lambda: module._fault is not None, timeout=2)
    with pytest.raises(RuntimeError, match="lost feedback"):
        _left(module, None, gains={"joint1": JointGains(kp=1.0, kd=1.0)})
    assert not _active(module)
    boundary[0].refresh_error = None
    module.stop()
    for call in (
        module.config_get,
        lambda: _begin(module),
        lambda: module.config_command({"joint1": 0.1}, "0" * 32),
    ):
        with pytest.raises(RuntimeError, match="disconnected|not running"):
            call()


def test_a_save_merges_into_the_arms_file_and_applies_it_live(start, boundary, calibration_root):
    module = start(config_sessions=True)
    path = calibration_root / "rig/left/calibration.json"
    links = {"link2": LinkGravity(mass_kg=1.5, com_m=(0.1, 0.0, 0.0))}
    patch = ConfigPatch(
        arm_id="rig/left", gains={"joint1": JointGains(kp=60.0, kd=3.0)}, links=links
    )
    saved = module.config_set(patch, scope="persist")
    assert saved.path == str(path) and saved.applied_now and saved.refused is None
    assert saved.applied == patch and saved.unsupported == []
    # Sent at once, without a command.
    np.testing.assert_equal(_rows(boundary, "left_arm")[0, :2], [60, 3])
    stored = load_calibration(path, expected_model_sha256=model_sha256(RIG.gravity_model))
    assert stored.gains == patch.gains and stored.gravity is not None
    assert stored.gravity.links == links
    state = module.config_get()["rig/left"]
    assert state.links == links and state.file is not None
    assert state.file.path == str(path)
    assert state.file.sha256 == hashlib.sha256(path.read_bytes()).hexdigest()
    assert _files(module)["rig/right"] is None

    # A second save merges: joint2's gains join joint1's, the links stay.
    more = ConfigPatch(arm_id="rig/left", gains={"joint2": JointGains(kp=30.0, kd=1.0)})
    module.config_set(more, scope="persist")
    stored = load_calibration(path)
    assert stored.gains is not None and set(stored.gains) == {"joint1", "joint2"}
    assert stored.gravity is not None and stored.gravity.links == links
    np.testing.assert_equal(_rows(boundary, "left_arm")[:, :2], [[60, 3], [30, 1]])
    assert len(list((path.parent / "history").iterdir())) == 1

    # A check alone is written, not applied, and the record follows the file.
    module.config_set(
        ConfigPatch(arm_id="rig/left", checks={"float": {"passed": True}}), scope="persist"
    )
    assert load_calibration(path).checks == {"float": {"passed": True}}
    state = module.config_get()["rig/left"]
    assert state.file is not None
    assert state.file.sha256 == hashlib.sha256(path.read_bytes()).hexdigest()


def test_a_save_leaves_out_what_a_file_cannot_hold(start, boundary, calibration_root):
    module = start(config_sessions=True)
    patch = ConfigPatch(
        arm_id="rig/left",
        gains={"joint1": JointGains(kp=0.0, kd=0.5), "joint2": JointGains(kp=30.0, kd=1.0)},
        torque_offset_nm={"joint1": 0.2},
    )
    saved = module.config_set(patch, scope="persist")
    assert saved.unsupported == ["torque_offset_nm.joint1", "gains.joint1.kp", "gains.joint1.kd"]
    assert "session" in saved.why["torque_offset_nm.joint1"]
    stored = load_calibration(calibration_root / "rig/left/calibration.json")
    assert stored.gains == {"joint2": JointGains(kp=30.0, kd=1.0)}
    assert module._arm.floating == {} and module._arm.feedforward == {}


def test_a_save_of_nothing_a_file_holds_writes_nothing(start, boundary, calibration_root):
    module = start(config_sessions=True)
    path = calibration_root / "rig/left/calibration.json"
    module.config_set(ConfigPatch(arm_id="rig/left", notes="a"), scope="persist")
    before = path.read_bytes()
    for patch in (
        ConfigPatch(arm_id="rig/left", torque_offset_nm={"joint1": 1.0}),
        ConfigPatch(arm_id="rig/left", gains={"joint1": JointGains(kp=0.0, kd=0.5)}),
        ConfigPatch(arm_id="rig/left"),
    ):
        saved = module.config_set(patch, scope="persist")
        assert saved.path is None and not saved.applied_now
        assert saved.unsupported == patch.keys()
    assert path.read_bytes() == before
    assert not (path.parent / "history").exists()


def test_a_check_is_recorded_only_in_the_file_the_arm_runs(start, boundary, calibration_root):
    module = start(config_sessions=True)
    path = calibration_root / "rig/left/calibration.json"
    check = ConfigPatch(arm_id="rig/left", checks={"float": {"passed": True}})
    # The file the arm runs takes a check.
    module.config_set(
        ConfigPatch(arm_id="rig/left", gains={"joint1": JointGains(kp=60.0, kd=3.0)}),
        scope="persist",
    )
    assert module.config_set(check, scope="persist").unsupported == []
    # New masses saved without applying them: the arm still runs the old file.
    heavier = ConfigPatch(
        arm_id="rig/left", links={"link2": LinkGravity(mass_kg=1.5, com_m=(0.1, 0.0, 0.0))}
    )
    module.config_set(heavier, scope="persist", apply_now=False)
    refused = module.config_set(
        check.model_copy(update={"notes": "after the check"}), scope="persist"
    )
    assert refused.unsupported == ["checks.float"]
    assert refused.why["checks.float"].startswith(f"the arm does not run {path} as it is on disk")
    stored = load_calibration(path)
    assert stored.checks is None and stored.notes == "after the check"
    # Nothing left to save but the check: nothing is written.
    written = path.read_bytes()
    assert module.config_set(check, scope="persist").path is None
    assert path.read_bytes() == written


def test_a_check_is_refused_while_the_arm_runs_no_file(start, boundary, calibration_root):
    path = save_calibration(
        calibration_root / "rig/left", _calibration(gains={"joint1": (0.0, 1.0)})
    )
    module = start(config_sessions=True)
    assert _files(module)["rig/left"] is None
    refused = module.config_set(
        ConfigPatch(arm_id="rig/left", checks={"float": {"passed": True}}), scope="persist"
    )
    assert refused.unsupported == ["checks.float"] and refused.path is None
    assert "it runs no calibration file" in refused.why["checks.float"]
    assert load_calibration(path).checks is None


def test_a_session_leaves_out_what_only_a_file_holds(start, boundary, calibration_root):
    module = start(config_sessions=True)
    session = _begin(module)
    links = {"link2": LinkGravity(mass_kg=1.5, com_m=(0.1, 0.0, 0.0))}
    applied = _left(module, session, links=links, notes="x")
    assert applied.unsupported == ["links.link2", "notes"]
    assert module._arm.inertial_overrides == {}
    assert not (calibration_root / "rig/left").exists()


@pytest.mark.parametrize("apply_now", [True, False])
def test_a_save_stores_and_reports_its_gains_as_clamped(
    start, boundary, calibration_root, apply_now
):
    module = start(config_sessions=True)
    patch = ConfigPatch(arm_id="rig/left", gains={"joint2": JointGains(kp=900.0, kd=9.0)})
    saved = module.config_set(patch, scope="persist", apply_now=apply_now)
    assert saved.applied.gains == {"joint2": JointGains(kp=500.0, kd=5.0)}
    assert saved.clamped == ["gains.joint2.kp", "gains.joint2.kd"]
    stored = load_calibration(calibration_root / "rig/left/calibration.json")
    assert stored.gains == {"joint2": JointGains(kp=500.0, kd=5.0)}
    if apply_now:
        assert module._arm.get_gains()["joint2"] == (500.0, 5.0)
    # The file it wrote loads at start like any other.
    module.stop()
    assert start().config_get()["rig/left"].gains["joint2"] == JointGains(kp=500.0, kd=5.0)


def test_a_saved_file_that_does_not_fit_is_replaced_not_merged(
    start, boundary, calibration_root, mocker
):
    path = save_calibration(
        calibration_root / "rig/left", _calibration(sha256="0" * 64, gains={"joint2": (5.0, 1.0)})
    )
    module = start(config_sessions=True)
    warning = mocker.spy(common.logger, "warning")
    module.config_set(
        ConfigPatch(arm_id="rig/left", gains={"joint1": JointGains(kp=60.0, kd=3.0)}),
        scope="persist",
    )
    stored = load_calibration(path)
    assert stored.gains == {"joint1": JointGains(kp=60.0, kd=3.0)}
    assert stored.robot_model.sha256 == model_sha256(RIG.gravity_model)
    assert any("afresh" in line and "different robot model" in line for line in _messages(warning))
    assert len(list((path.parent / "history").iterdir())) == 1


def test_saving_one_arms_file_live_keeps_what_the_other_arm_runs(start, boundary, calibration_root):
    module = start(config_sessions=True)
    right = ConfigPatch(arm_id="rig/right", gains={"joint3": JointGains(kp=30.0, kd=1.0)})
    left = ConfigPatch(arm_id="rig/left", gains={"joint1": JointGains(kp=60.0, kd=3.0)})
    module.config_set(right, scope="persist")
    module.config_set(left, scope="persist")
    files = _files(module)
    assert {arm_id: None if file is None else file.path for arm_id, file in files.items()} == {
        "rig/left": str(calibration_root / "rig/left/calibration.json"),
        "rig/right": str(calibration_root / "rig/right/calibration.json"),
    }
    np.testing.assert_equal(_rows(boundary, "right_arm")[0, :2], [30, 1])
    np.testing.assert_equal(_rows(boundary, "left_arm")[0, :2], [60, 3])


def test_a_save_that_steps_gravity_more_than_2_nm_is_refused_unless_forced(
    start, boundary, calibration_root
):
    module = start(config_sessions=True)
    path = calibration_root / "rig/left/calibration.json"
    heavier = ConfigPatch(
        arm_id="rig/left", links={"link2": LinkGravity(mass_kg=3.0, com_m=(0.1, 0.0, 0.0))}
    )
    sent = len(boundary[0].groups["left_arm"].commands)
    refused = module.config_set(heavier, scope="persist")
    assert refused.refused is not None and not refused.applied_now and refused.path is None
    assert re.search(
        r"more than 2 N m: \{'joint1': .*near home, or pass force=True", refused.refused
    )
    assert abs(refused.gravity_step_nm["joint1"]) > 2.0
    assert not path.exists()
    assert len(boundary[0].groups["left_arm"].commands) == sent
    assert _files(module)["rig/left"] is None

    # Skipping writes the file without applying it.
    skipped = module.config_set(heavier, scope="persist", apply_now=False)
    assert skipped.path == str(path) and not skipped.applied_now and skipped.refused is None
    assert path.exists() and _files(module)["rig/left"] is None
    assert len(boundary[0].groups["left_arm"].commands) == sent

    result = module.config_set(heavier, scope="persist", force=True)
    pose = np.full(3, 0.2)
    joints = RIG.arm_joint_names
    before = PhysicalModel(RIG.gravity_model, joints, gravity_comp=True).gravity(pose)
    after = PhysicalModel(
        RIG.gravity_model,
        joints,
        gravity_comp=True,
        inertial_overrides={"link2": LinkInertial(mass_kg=3.0, com_m=(0.1, 0.0, 0.0))},
    ).gravity(pose)
    assert result.applied_now and result.gravity_step_nm == pytest.approx(
        dict(zip(joints, after - before, strict=True))
    )
    # joint3 carries none of link2.
    assert result.gravity_step_nm["joint3"] == 0.0
    assert set(module.config_get()["rig/left"].links) == {"link2"}
    np.testing.assert_allclose(_rows(boundary, "left_arm")[:, 4], after[:2])


def test_masses_over_a_motors_rating_at_home_are_refused(
    start, boundary, calibration_root, rig_class
):
    class HomeRig(rig_class):
        robot = replace(RIG, home_joints=[0.0, 0.0, 0.0])

    module = start(HomeRig, config_sessions=True)
    # Every link vertical: no gravity torque at the measured pose.
    boundary[0].groups["left_arm"].q[:] = [math.pi / 2, 0.0]
    boundary[0].groups["right_arm"].q[:] = [0.0]
    heavy = ConfigPatch(
        arm_id="rig/left", links={"link2": LinkGravity(mass_kg=10.0, com_m=(0.1, 0.0, 0.0))}
    )
    with pytest.raises(
        ArmCalibrationError, match="on joint1 at home, more than its motor's rated 28 N m"
    ):
        module.config_set(heavy, scope="persist")
    assert _files(module) == {"rig/left": None, "rig/right": None}
    assert not (calibration_root / "rig/left/calibration.json").exists()


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
    states = module.config_get()
    assert list(states) == ["dual_openyam/left", "dual_openyam/right"]
    for spec in specs:
        state = states[spec.arm_id]
        assert state.file is not None and state.file.path.startswith(str(calibration_root))
        assert state.gains[spec.joint_names[0]] == JointGains(kp=60.0, kd=4.0)
        assert list(state.links) == [spec.gravity_links[-1]]
    module.command({"arm/left_joint1/position": 0.0})
    np.testing.assert_equal(_rows(boundary, "left_arm")[0, :2], [60, 4])
    np.testing.assert_equal(_rows(boundary, "right_arm")[0, :2], [60, 4])


def test_the_state_reports_the_masses_and_file_hash_each_arm_runs(
    start, boundary, calibration_root
):
    path = save_calibration(calibration_root / "rig/left", _calibration(masses={"link2": 2.0}))
    sha = hashlib.sha256(path.read_bytes()).hexdigest()
    module = start(config_sessions=True)

    state = module.config_get()

    assert state["rig/left"].file == AppliedFile(
        path=str(path), created="2026-11-17T09:30:00+00:00", sha256=sha
    )
    assert state["rig/left"].links == {"link2": LinkGravity(mass_kg=2.0, com_m=(0.1, 0.0, 0.0))}
    assert state["rig/right"].file is None and state["rig/right"].links == {}
    # A file rewritten on disk changes nothing the arm runs until it is applied.
    save_calibration(path, _calibration(masses={"link2": 2.5}))
    assert module.config_get() == state
    assert roundtrip(state) == state


def _park_while_stopping(module, order: list[str], stream_s: float) -> threading.Thread:
    """Stop ``module`` on a thread while this one parks, as a calibration tool does."""

    def stop() -> None:
        module.stop()
        order.append("stopped")

    stopper = threading.Thread(target=stop)
    stopper.start()
    time.sleep(0.05)
    session = _begin(module)
    order.append("park began")
    deadline = time.monotonic() + stream_s
    try:
        while time.monotonic() < deadline:
            module.config_command({"joint1": 0.2}, session)
            time.sleep(0.02)
    except RuntimeError:
        order.append("cut off")
    else:
        order.append("parked")
        module.config_reset()
    return stopper


def test_stopping_waits_for_the_calibration_tool_to_park_before_disabling(start, boundary, mocker):
    module = start(config_sessions=True)
    mocker.patch.object(connection_module, "CONFIG_PARK_GRACE_S", 0.3)
    order: list[str] = []
    disable = module._arm.disable

    def disabled() -> None:
        order.append("disabled")
        disable()

    mocker.patch.object(module._arm, "disable", disabled)
    warning = _base_messages(mocker, "warning")

    stopper = _park_while_stopping(module, order, stream_s=0.5)
    stopper.join(timeout=5.0)

    assert order == ["park began", "parked", "disabled", "stopped"]
    assert any(
        "waiting up to 10 s for the calibration tool to park the arm" in line
        for line in _messages(warning)
    )


def test_a_park_that_overruns_the_wait_is_cut_off_and_the_arm_disabled(start, boundary, mocker):
    module = start(config_sessions=True)
    mocker.patch.object(connection_module, "CONFIG_PARK_GRACE_S", 0.2)
    mocker.patch.object(connection_module, "CONFIG_PARK_WAIT_S", 0.3)
    error = _base_messages(mocker, "error")
    order: list[str] = []

    stopper = _park_while_stopping(module, order, stream_s=2.0)
    stopper.join(timeout=5.0)

    assert order == ["park began", "stopped", "cut off"]
    assert any("did not end its config session within 0.3 s" in line for line in _messages(error))
    assert module._config_ended == "the connection halted"
