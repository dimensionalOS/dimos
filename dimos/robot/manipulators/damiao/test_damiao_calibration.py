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

from datetime import datetime, timezone

import numpy as np
import pytest

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
    make_calibrated_test_robot,
)
from dimos.robot.manipulators.damiao._damiao_test_helpers import _local_test_model
from dimos.robot.manipulators.damiao.arm import PhysicalModel
from dimos.robot.manipulators.dual_openyam.calibration_workspace import (
    dual_openyam_calibration_specs,
)
from dimos.robot.manipulators.dual_openyam.config import DUAL_OPENYAM
from dimos.robot.manipulators.dual_openyam.connection import DualOpenYamConnection
from dimos.robot.manipulators.openarm.connection import OpenArmConnection
from dimos.robot.manipulators.seeed_b601_dm.connection import B601DMConnection
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
    made = []

    def make(connection_type=None, **kwargs):
        module = (connection_type or rig_class)(deadman_timeout_s=10, **kwargs)
        readings = LocalTransport()
        module.set_transport("position_command", LocalTransport())
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


# Pre-calibration rows of two gravity-compensated arms at fixed poses: the
# first joint commanded 0.05 rad past its measured position, then held.
OPENARM_POSE = [0.3, -0.2, 0.1, 0.8, -0.4, 0.2, 0.1, -0.3, 0.2, -0.1, 0.9, 0.3, -0.2, 0.0]
OPENARM_GRAVITY = [
    0.02827939416356918,
    -1.6977429614268749,
    -0.5199258389030557,
    2.0722063841959617,
    0.007253902348079567,
    0.3703900053177113,
    -0.010301118765709147,
    0.3434260145103905,
    1.5705788373393517,
    0.5552199409942215,
    2.455153606787638,
    0.025091085300970303,
    -0.4038377548046186,
    0.035626667442231134,
]
B601_POSE = [0.2, 0.9, 0.7, -0.3, 0.4, 0.1]
B601_GRAVITY = [
    3.6897080811677806e-16,
    1.1071731002501215,
    -6.401352732269811,
    -1.5759525903706573,
    -0.1325144358484307,
    0.0002572368260063199,
]


@pytest.mark.parametrize(
    "connection_type,ports,pose,gravity",
    [
        (
            OpenArmConnection,
            {"left_can_port": "can0", "right_can_port": "can1"},
            OPENARM_POSE,
            OPENARM_GRAVITY,
        ),
        (B601DMConnection, {"can_port": "can0"}, B601_POSE, B601_GRAVITY),
    ],
    ids=["openarm", "b601"],
)
def test_auto_without_a_saved_file_sends_the_rows_sent_before_calibration(
    start, boundary, tmp_path, mocker, connection_type, ports, pose, gravity
):
    info = mocker.spy(common.logger, "info")
    boundary[0].default_position = 0.0
    module = start(
        connection_type,
        **ports,
        source="arm",
        model=_local_test_model(connection_type, tmp_path / "control.urdf"),
    )
    robot = connection_type.robot
    for arm_id in robot.arm_ids():
        assert f"No calibration for {arm_id}, using URDF defaults" in _messages(info)
    assert module.status().calibration == dict.fromkeys(robot.arm_ids())
    offset = 0
    for bus in robot.buses:
        boundary[0].groups[bus.name].q = np.asarray(pose[offset : offset + len(bus.arm)])
        offset += len(bus.arm)
    module.command({f"arm/{robot.arm_joint_names[0]}/position": pose[0] + 0.05})
    commanded = {bus.name: _rows(boundary, bus.name).copy() for bus in robot.buses}
    module.halt()
    held = {bus.name: _rows(boundary, bus.name).copy() for bus in robot.buses}
    offset = 0
    for bus in robot.buses:
        count = len(bus.arm)
        targets = [0.0] * count
        if offset == 0:
            targets[0] = pose[0] + 0.05
        for rows, expected_targets in (
            (commanded[bus.name], targets),
            (held[bus.name], pose[offset : offset + count]),
        ):
            np.testing.assert_equal(rows[:, 0], bus.kp)
            np.testing.assert_equal(rows[:, 1], bus.kd)
            np.testing.assert_allclose(rows[:, 2], expected_targets, rtol=0, atol=1e-15)
            np.testing.assert_equal(rows[:, 3], 0.0)
            np.testing.assert_allclose(
                rows[:, 4], gravity[offset : offset + count], rtol=1e-12, atol=1e-12
            )
        offset += count


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
            "lacks: ['ghost']",
        ),
    ],
    ids=["other-model", "corrupt", "other-arm", "other-arms-joint", "unknown-link"],
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
    module, _, _ = make_rig()
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


def test_tuning_hooks_round_trip(start, boundary, mocker):
    module = start()
    native = boundary[0]
    assert isinstance(module, ArmTuningHooks)
    assert module.tuning_arm_ids() == ["rig/left", "rig/right"]
    assert module.tuning_joint_names() == ["joint1", "joint2", "joint3"]

    module.tuning_set_gains("joint2", 900, 2)
    np.testing.assert_equal(_rows(boundary, "left_arm")[1, :2], [500, 2])
    module.tuning_set_float("joint3", True, kd=0.4)
    module.tuning_set_feedforward("joint1", 0.5)
    gains = module.tuning_get_gains()
    assert gains["joint2"] == {
        "kp": 500.0,
        "kd": 2.0,
        "floating": False,
        "float_kd": None,
        "feedforward_nm": 0.0,
    }
    assert gains["joint3"]["floating"] and gains["joint3"]["float_kd"] == 0.4
    assert gains["joint1"]["feedforward_nm"] == 0.5
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

    module.tuning_command({"joint1": 0.3, "left_finger": 0.02})
    assert _rows(boundary, "left_arm")[0, 2] == 0.3
    assert native.groups["left_gripper"].commands[-1] == pytest.approx(0.5)
    assert module.status().last_command_time is not None
    with pytest.raises(ValueError, match="limit"):
        module.tuning_command({"joint1": 4.0})
    with pytest.raises(ValueError, match="not joints"):
        module.tuning_command({"nope": 0.0})
    with pytest.raises(ValueError, match="not an arm joint"):
        module.tuning_set_gains("left_finger", 1.0, 1.0)
    assert _rows(boundary, "left_arm")[0, 2] == 0.3


def test_tuning_refuses_a_faulted_or_stopped_connection(start, boundary):
    module = start()
    boundary[0].refresh_error = RuntimeError("lost feedback")
    wait_until(lambda: module._fault is not None, timeout=2)
    with pytest.raises(RuntimeError, match="lost feedback"):
        module.tuning_set_gains("joint1", 1.0, 1.0)
    boundary[0].refresh_error = None
    module.stop()
    for call in (
        module.tuning_get_gains,
        module.tuning_read,
        lambda: module.tuning_command({"joint1": 0.1}),
    ):
        with pytest.raises(RuntimeError, match="disconnected"):
            call()


def test_tuning_applies_a_calibration_live_and_off_restores_the_defaults(
    start, boundary, calibration_root, tmp_path
):
    module = start()
    trial = save_calibration(
        tmp_path / "trial.json", _calibration(gains={"joint1": (60.0, 3.0)}, masses={"link2": 2.0})
    )
    applied = module.tuning_apply_calibration(str(trial))
    assert applied["rig/left"]["links"] == ["link2"] and applied["rig/right"] is None
    # Sent at once, without a command.
    np.testing.assert_equal(_rows(boundary, "left_arm")[0, :2], [60, 3])
    assert module.status().calibration == applied

    mismatched = save_calibration(tmp_path / "bad.json", _calibration(sha256="0" * 64))
    with pytest.raises(ArmCalibrationError, match="different robot model"):
        module.tuning_apply_calibration(str(mismatched))
    np.testing.assert_equal(_rows(boundary, "left_arm")[0, :2], [60, 3])
    assert module.status().calibration == applied

    assert module.tuning_apply_calibration("off") == {"rig/left": None, "rig/right": None}
    np.testing.assert_equal(_rows(boundary, "left_arm")[0, :2], [80, 5])
    assert module._arm.inertial_overrides == {}

    saved = save_calibration(
        calibration_root / "rig/right", _calibration("rig/right", gains={"joint3": (30.0, 1.0)})
    )
    applied = module.tuning_apply_calibration()
    assert applied["rig/right"]["path"] == str(saved) and applied["rig/left"] is None
    np.testing.assert_equal(_rows(boundary, "right_arm")[0, :2], [30, 1])


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
