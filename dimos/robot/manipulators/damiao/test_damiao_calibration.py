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

"""Calibration files applied at connect, and the get_config/set_config a calibration tool drives."""

from __future__ import annotations

from dataclasses import replace
from datetime import datetime, timezone
import hashlib
import importlib
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from dimos.control.calibration._calibration_test_helpers import (
    offline_robot_assets,
    require_model,
)
from dimos.control.calibration._inertial import LinkInertial
from dimos.control.calibration.schema import (
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
from dimos.control.calibration.spec import model_sha256, specs_from_damiao_robot
from dimos.robot.manipulators.damiao._damiao_native_test_helpers import (
    LocalTransport,
    drive_and_record,
    make_calibrated_test_robot,
)
from dimos.robot.manipulators.damiao._damiao_test_helpers import _local_test_model
from dimos.robot.manipulators.dual_openyam.config import DUAL_OPENYAM
from dimos.robot.manipulators.dual_openyam.connection import DualOpenYamConnection
from dimos.utils.testing.cbor import roundtrip
from dimos.utils.testing.waiting import wait_until

RIG = make_calibrated_test_robot()
CREATED = datetime(2026, 11, 17, 9, 30, tzinfo=timezone.utc)
HEAVY_LINK2 = {"link2": {"mass_kg": 1.5, "com_m": [0.1, 0.0, 0.0]}}


def _calibration(
    arm_id: str = "rig/left",
    *,
    sha256: str | None = None,
    gains: dict[str, tuple[float, float]] | None = None,
    masses: dict[str, float] | None = None,
) -> ArmCalibration:
    return ArmCalibration(
        arm_id=arm_id,
        robot_model=RobotModelRef(name="rig", sha256=sha256 or model_sha256(RIG.gravity_model)),
        created=CREATED,
        gains={joint: JointGains(kp=kp, kd=kd) for joint, (kp, kd) in (gains or {}).items()}
        or None,
        gravity=GravityCalibration(
            links={
                link: LinkGravity(mass_kg=mass, com_m=(0.1, 0.0, 0.0))
                for link, mass in masses.items()
            }
        )
        if masses
        else None,
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
        kwargs.setdefault("deadman_timeout_s", 10)
        module = (connection_type or rig_class)(**kwargs)
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


def test_auto_applies_each_arms_saved_file(start, boundary, calibration_root):
    path = save_calibration(
        calibration_root / "rig/left",
        _calibration(gains={"joint1": (60.0, 3.0)}, masses={"link2": 2.0}),
    )
    module = start()
    module.command({"arm/joint1/position": 0.2})
    np.testing.assert_equal(_rows(boundary, "left_arm")[:, :2], [[60, 3], [10, 1.5]])
    expected = module._arm.gravity({"link2": LinkInertial(mass_kg=2.0, com_m=(0.1, 0.0, 0.0))})
    np.testing.assert_allclose(_rows(boundary, "left_arm")[:, 4], list(expected.values())[:2])
    state = module.get_config()
    assert state["rig/left"]["file"] == {
        "path": str(path),
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "created": "2026-11-17T09:30:00Z",
    }
    assert state["rig/left"]["links"] == {
        "link2": {"mass_kg": 2.0, "com_m": [0.1, 0.0, 0.0], "merged_links": []}
    }
    assert state["rig/right"] == {
        "gains": {"joint3": {"kp": 20.0, "kd": 0.6}},
        "floating": [],
        "links": {},
        "file": None,
        "can_float": True,
    }
    assert roundtrip(state) == state


# What each connection sent before calibration existed, recorded with
# ``drive_and_record`` from the code at the commit in "recorded_from".
ROWS_BEFORE_CALIBRATION: dict[str, Any] = json.loads(
    Path(__file__).with_name("damiao_rows_before_calibration.json").read_text()
)["cases"]


@pytest.mark.parametrize("case", list(ROWS_BEFORE_CALIBRATION))
def test_auto_without_a_saved_file_sends_what_was_sent_before_calibration(
    start, boundary, tmp_path, case
):
    recorded = ROWS_BEFORE_CALIBRATION[case]
    module_name, _, qualname = recorded["connection"].partition(":")
    connection_type = getattr(importlib.import_module(module_name), qualname)
    robot = connection_type.robot
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
        assert {arm_id: state["file"] for arm_id, state in module.get_config().items()} == (
            dict.fromkeys(robot.arm_ids())
        )
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
    assert module.get_config()["rig/left"]["file"] is None


def _write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)


@pytest.mark.parametrize(
    "calibration,reason",
    [
        (_calibration(sha256="0" * 64), "different robot model"),
        ("{not json", "not valid JSON"),
        (_calibration("rig/right"), "is for arm 'rig/right'"),
        (_calibration(gains={"joint3": (60.0, 3.0)}), "joints of no arm 'rig/left' drives"),
        (_calibration(masses={"ghost": 1.0}), "does not move: ['ghost']"),
        # link3 hangs off joint3, which the right arm drives.
        (_calibration(masses={"link3": 1.0}), "does not move: ['link3']"),
        # 50 kg on link2 would need about 140 N m from joint1, a 28 N m motor.
        (_calibration(masses={"link2": 50.0}), "more than its motor's rated 28 N m"),
        (_calibration(gains={"joint1": (0.0, 1.0)}), "gains outside 0 < kp <= 500"),
        (_calibration(gains={"joint2": (900.0, 9.0)}), "gains outside 0 < kp <= 500"),
    ],
)
def test_a_saved_file_that_does_not_fit_makes_connect_raise(
    start, boundary, calibration_root, calibration, reason
):
    path = calibration_root / "rig/left/calibration.json"
    if isinstance(calibration, str):
        _write_text(path, calibration)
    else:
        _write_text(path, calibration.model_dump_json())
    with pytest.raises(ArmCalibrationError, match="calibration: off") as raised:
        start()
    assert reason in str(raised.value) and str(path) in str(raised.value)
    assert boundary[0].groups["left_arm"].commands == []


def test_off_never_reads_a_saved_file(start, boundary, calibration_root):
    _write_text(calibration_root / "rig/left/calibration.json", "{not json")
    module = start(calibration="off")
    assert module.get_config()["rig/left"]["file"] is None


def test_an_explicit_file_that_is_missing_makes_connect_raise(start, tmp_path):
    with pytest.raises(ArmCalibrationError, match="No arm calibration at"):
        start(calibration=str(tmp_path / "missing.json"))


def test_set_config_needs_config_writes(start):
    module = start()
    with pytest.raises(RuntimeError, match="config_writes is off"):
        module.set_config({"rig/left": {"gains": {"joint1": {"kp": 1.0, "kd": 1.0}}}})


def test_a_patch_with_one_bad_entry_changes_nothing_and_names_every_bad_entry(
    start, boundary, calibration_root
):
    module = start(config_writes=True)
    before = module.get_config()
    sent = len(boundary[0].groups["left_arm"].commands)
    patch = {
        "rig/left": {
            "gains": {"joint1": {"kp": 40.0, "kd": 1.0}, "joint3": {"kp": 1.0, "kd": 1.0}}
        },
        "rig/right": {"gains": {"joint3": {"kp": 900.0, "kd": 1.0}}, "links": HEAVY_LINK2},
        "rig/ghost": {},
    }
    with pytest.raises(ValueError, match="changed nothing") as raised:
        module.set_config(patch)
    message = str(raised.value)
    for bad in (
        "rig/left.gains.joint3: not a joint of rig/left",
        "rig/right.gains.joint3: joint3 gains kp=900.0",
        "rig/right.links: saved only",
        "rig/ghost: not an arm",
    ):
        assert bad in message
    assert "gains.joint1" not in message
    assert module.get_config() == before
    assert len(boundary[0].groups["left_arm"].commands) == sent
    with pytest.raises(ValueError, match=r"rig/left.gains.joint1.kd: Input should be a valid"):
        module.set_config({"rig/left": {"gains": {"joint1": {"kp": 40.0, "kd": "1"}}}}, save=True)
    with pytest.raises(ValueError, match="kp 0 floats a joint, which is live only"):
        module.set_config({"rig/left": {"gains": {"joint1": {"kp": 0.0, "kd": 1.0}}}}, save=True)
    assert not (calibration_root / "rig").exists()


def test_a_floated_joint_gets_its_gains_back_when_the_deadman_fires(start, boundary):
    module = start(config_writes=True, deadman_timeout_s=0.05)
    state = module.set_config(
        {
            "rig/left": {"gains": {"joint2": {"kp": 40.0, "kd": 2.0}}},
            "rig/right": {"gains": {"joint3": {"kp": 0.0, "kd": 0.4}}},
        }
    )
    assert state["rig/right"]["floating"] == ["joint3"]
    assert state["rig/right"]["gains"]["joint3"] == {"kp": 0.0, "kd": 0.4}
    np.testing.assert_equal(_rows(boundary, "right_arm")[0, :2], [0, 0.4])
    np.testing.assert_equal(_rows(boundary, "left_arm")[1, :2], [40, 2])
    # No command follows: the deadman halts, which undoes both and holds where the arm is.
    boundary[0].groups["right_arm"].q[:] = [0.5]
    wait_until(lambda: module.status().deadman_fired, timeout=2)
    np.testing.assert_equal(_rows(boundary, "right_arm")[0, :3], [20, 0.6, 0.5])
    np.testing.assert_equal(_rows(boundary, "left_arm")[1, :2], [10, 1.5])
    assert module.get_config()["rig/right"]["floating"] == []
    # The next command resumes.
    module.command({"arm/joint3/position": 0.3})
    np.testing.assert_equal(_rows(boundary, "right_arm")[0, :3], [20, 0.6, 0.3])


def test_a_failed_write_undoes_the_live_changes(start, boundary):
    module = start(config_writes=True)
    module.set_config({"rig/right": {"gains": {"joint3": {"kp": 0.0, "kd": 0.4}}}})
    left = boundary[0].groups["left_arm"]
    left.error = RuntimeError("bus off")
    module.command({"arm/joint1/position": 0.3})
    left.error = None
    assert "bus off" in (module.status().last_error or "")
    assert module.get_config()["rig/right"]["floating"] == []


def test_a_save_merges_into_the_file_keeps_history_and_applies_it(
    start, boundary, calibration_root
):
    module = start(config_writes=True)
    path = calibration_root / "rig/left/calibration.json"
    module.set_config({"rig/right": {"gains": {"joint3": {"kp": 0.0, "kd": 0.4}}}})
    state = module.set_config(
        {"rig/left": {"gains": {"joint1": {"kp": 60.0, "kd": 3.0}}, "links": HEAVY_LINK2}},
        save=True,
    )
    # Applied at once, without a command, and every live change dropped.
    np.testing.assert_equal(_rows(boundary, "left_arm")[0, :2], [60, 3])
    assert state["rig/right"]["floating"] == []
    assert state["rig/left"]["file"]["sha256"] == hashlib.sha256(path.read_bytes()).hexdigest()
    assert set(state["rig/left"]["links"]) == {"link2"}
    module.set_config(
        {"rig/left": {"gains": {"joint2": {"kp": 30.0, "kd": 1.0}}, "notes": "bench"}}, save=True
    )
    stored = load_calibration(path)
    assert stored.gains is not None and set(stored.gains) == {"joint1", "joint2"}
    assert stored.gravity is not None and set(stored.gravity.links) == {"link2"}
    assert stored.notes == "bench"
    np.testing.assert_equal(_rows(boundary, "left_arm")[:, :2], [[60, 3], [30, 1]])
    assert len(list((path.parent / "history").iterdir())) == 1


@pytest.mark.parametrize(
    "stored,reason",
    [
        (_calibration(sha256="0" * 64).model_dump_json(), "different robot model"),
        ("{not json", "not valid JSON"),
    ],
)
def test_a_save_over_a_file_that_does_not_fit_raises_and_writes_nothing(
    start, calibration_root, stored, reason
):
    module = start(config_writes=True, calibration="off")
    path = calibration_root / "rig/left/calibration.json"
    _write_text(path, stored)
    with pytest.raises(ValueError, match=reason):
        module.set_config({"rig/left": {"gains": {"joint1": {"kp": 60.0, "kd": 3.0}}}}, save=True)
    assert path.read_text() == stored and not (path.parent / "history").exists()


def test_save_calibration_refuses_to_replace_another_models_file(tmp_path):
    path = save_calibration(tmp_path, _calibration(sha256="0" * 64))
    with pytest.raises(ArmCalibrationError, match="Not saving over it"):
        save_calibration(path, _calibration())
    assert load_calibration(path).robot_model.sha256 == "0" * 64


def test_masses_over_a_motors_rating_at_home_are_refused(
    start, boundary, calibration_root, rig_class
):
    class HomeRig(rig_class):
        robot = replace(RIG, home_joints=[0.0, 0.0, 0.0])

    module = start(HomeRig, config_writes=True)
    # Every link vertical: no gravity torque at the measured pose.
    boundary[0].groups["left_arm"].q[:] = [math.pi / 2, 0.0]
    boundary[0].groups["right_arm"].q[:] = [0.0]
    heavy = {"link2": {"mass_kg": 10.0, "com_m": [0.1, 0.0, 0.0]}}
    with pytest.raises(ValueError, match="on joint1 at home, more than its motor's rated 28 N m"):
        module.set_config({"rig/left": {"links": heavy}}, save=True)
    assert not (calibration_root / "rig").exists()


def test_files_from_the_calibration_tools_load_into_dual_openyam(
    start, boundary, calibration_root, tmp_path
):
    with offline_robot_assets():
        require_model(DUAL_OPENYAM.gravity_model, "Dual OpenYAM")
        specs = specs_from_damiao_robot(DUAL_OPENYAM)
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
    states = module.get_config()
    assert list(states) == ["dual_openyam/left", "dual_openyam/right"]
    for spec in specs:
        state = states[spec.arm_id]
        assert state["file"]["path"].startswith(str(calibration_root))
        assert state["gains"][spec.joint_names[0]] == {"kp": 60.0, "kd": 4.0}
        assert list(state["links"]) == [spec.gravity_links[-1]]
    module.command({"arm/left_joint1/position": 0.0})
    np.testing.assert_equal(_rows(boundary, "left_arm")[0, :2], [60, 4])
    np.testing.assert_equal(_rows(boundary, "right_arm")[0, :2], [60, 4])
