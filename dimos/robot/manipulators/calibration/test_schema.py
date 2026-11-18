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

from datetime import datetime, timedelta, timezone
import json
import os
from pathlib import Path
from unittest import mock
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin
import pytest

from dimos.constants import STATE_DIR
from dimos.robot.manipulators.calibration._inertial import LinkInertial, apply_inertial_overrides
from dimos.robot.manipulators.calibration.schema import (
    ArmCalibration,
    ArmCalibrationError,
    GravityCalibration,
    JointGains,
    LinkGravity,
    RobotModelRef,
    bake_into_urdf,
    calibration_from_legacy_gravity,
    calibration_path,
    load_calibration,
    save_calibration,
    to_inertial_overrides,
)
from dimos.robot.manipulators.calibration.spec import ArmCalibrationSpec

REFERENCE = Path(__file__).resolve().parent / "reference" / "gravity_both_2026-10-16.json"
SHA = "a" * 64
CREATED = datetime(2026, 11, 18, 9, 30, tzinfo=timezone.utc)

# A base, one moving link and a tool fixed to it.
TOY_URDF = """<?xml version="1.0"?>
<robot name="toy">
  <!-- kept -->
  <link name="base"/>
  <link name="arm">
    <inertial>
      <origin xyz="0.1 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <link name="tool">
    <inertial>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>
  <link name="other">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.3"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>
  <joint name="shoulder" type="revolute">
    <parent link="base"/>
    <child link="arm"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1" upper="1" effort="10" velocity="1"/>
  </joint>
  <joint name="tool_mount" type="fixed">
    <parent link="arm"/>
    <child link="tool"/>
    <origin xyz="0.3 0 0" rpy="0 0 1.5707963267948966"/>
  </joint>
  <joint name="other_mount" type="fixed">
    <parent link="base"/>
    <child link="other"/>
    <origin xyz="0 0.2 0" rpy="0 0 0"/>
  </joint>
</robot>
"""


def calibration(**fields: object) -> ArmCalibration:
    values: dict[str, object] = {
        "arm_id": "toy/arm",
        "robot_model": RobotModelRef(name="toy", sha256=SHA),
        "created": CREATED,
        "gravity": GravityCalibration(
            links={"arm": LinkGravity(mass_kg=2.0, com_m=(0.2, 0.0, 0.01), merged_links=["tool"])},
            fit={"ridge_nm": 0.1, "holdout_poses": ["pose_05"]},
        ),
        "gains": {"shoulder": JointGains(kp=80.0, kd=5.0)},
        "friction_nm": {"shoulder": 0.21},
        "checks": {"float": {"passed": True, "drift_mm": 1.2}},
        "notes": "bench run",
    }
    values.update(fields)
    return ArmCalibration.model_validate(values)


def test_calibration_round_trips_through_its_file(tmp_path: Path) -> None:
    original = calibration()
    path = save_calibration(tmp_path / "toy" / "arm", original)
    assert path == tmp_path / "toy" / "arm" / "calibration.json"
    raw = json.loads(path.read_text())
    assert raw["schema_version"] == 1 and raw["created"] == "2026-11-18T09:30:00Z"
    assert load_calibration(path) == original
    assert load_calibration(path.parent, expected_model_sha256=SHA, expected_arm_id="toy/arm") == (
        original
    )
    assert to_inertial_overrides(original) == {
        "arm": LinkInertial(mass_kg=2.0, com_m=(0.2, 0.0, 0.01))
    }
    assert to_inertial_overrides(calibration(gravity=None)) == {}


def test_created_is_stored_in_utc_and_must_carry_a_zone() -> None:
    local = datetime(2026, 11, 18, 11, 30, tzinfo=timezone(timedelta(hours=2)))
    assert calibration(created=local).created == CREATED
    with pytest.raises(ValueError, match="time zone"):
        calibration(created=datetime(2026, 11, 18, 9, 30))


def test_calibration_path_is_per_arm_under_the_state_dir() -> None:
    assert calibration_path("dual_openyam/left") == (
        STATE_DIR / "arms" / "dual_openyam" / "left" / "calibration.json"
    )
    with pytest.raises(ValueError):
        calibration_path("../etc")
    with pytest.raises(ValueError):
        calibration_path("/abs")


def test_save_keeps_the_replaced_file_in_history(tmp_path: Path) -> None:
    first = calibration()
    second = calibration(created=CREATED + timedelta(days=1), notes="second run")
    path = save_calibration(tmp_path, first)
    save_calibration(tmp_path, second)
    assert load_calibration(path) == second
    history = sorted((tmp_path / "history").iterdir())
    assert [p.name for p in history] == ["2026-11-18T09-30-00Z.json"]
    assert load_calibration(history[0]) == first
    # Saving the same created time twice never overwrites history.
    save_calibration(tmp_path, second)
    save_calibration(tmp_path, second)
    assert len(list((tmp_path / "history").iterdir())) == 3
    assert not [p for p in tmp_path.iterdir() if p.name.endswith(".tmp")]


def test_a_failed_save_leaves_the_old_file_whole(tmp_path: Path) -> None:
    path = save_calibration(tmp_path, calibration())
    before = path.read_text()
    with mock.patch("os.replace", side_effect=OSError("disk full")):
        with pytest.raises(OSError, match="disk full"):
            save_calibration(tmp_path, calibration(notes="never written"))
    assert path.read_text() == before
    assert not [p for p in tmp_path.iterdir() if p.name.endswith(".tmp")]
    assert os.path.exists(path)


def test_load_refuses_a_calibration_for_another_model(tmp_path: Path) -> None:
    path = save_calibration(tmp_path, calibration())
    with pytest.raises(ArmCalibrationError) as error:
        load_calibration(path, expected_model_sha256="b" * 64)
    message = str(error.value)
    assert "different robot model" in message
    assert "'toy'" in message and "aaaaaaaaaaaa" in message and "bbbbbbbbbbbb" in message
    assert "calibrate the arm again" in message
    with pytest.raises(ArmCalibrationError, match="is for arm 'toy/arm', not 'toy/other'"):
        load_calibration(path, expected_arm_id="toy/other")


def test_load_explains_bad_files(tmp_path: Path) -> None:
    with pytest.raises(ArmCalibrationError, match="No arm calibration"):
        load_calibration(tmp_path / "missing.json")
    bad = tmp_path / "bad.json"
    bad.write_text("{not json")
    with pytest.raises(ArmCalibrationError, match="not valid JSON"):
        load_calibration(bad)
    future = tmp_path / "future.json"
    future.write_text(
        json.dumps({**json.loads(calibration().model_dump_json()), "schema_version": 2})
    )
    with pytest.raises(ArmCalibrationError, match="schema_version 2.*reads schema_version 1"):
        load_calibration(future)
    broken = tmp_path / "broken.json"
    raw = json.loads(calibration().model_dump_json())
    raw["gravity"]["links"]["arm"]["mass_kg"] = -1.0
    raw["surprise"] = True
    broken.write_text(json.dumps(raw))
    with pytest.raises(ArmCalibrationError, match="gravity.links.arm.mass_kg") as error:
        load_calibration(broken)
    assert "surprise" in str(error.value)


def test_bake_into_urdf_changes_exactly_the_listed_links() -> None:
    baked = bake_into_urdf(calibration(), TOY_URDF)
    before = {link.get("name"): link for link in ET.fromstring(TOY_URDF).iter("link")}
    after = {link.get("name"): link for link in ET.fromstring(baked).iter("link")}
    assert set(before) == set(after)
    for name in ("base", "tool", "other"):
        assert ET.tostring(after[name]) == ET.tostring(before[name])
    for joint_before, joint_after in zip(
        ET.fromstring(TOY_URDF).iter("joint"), ET.fromstring(baked).iter("joint"), strict=True
    ):
        assert ET.tostring(joint_before) == ET.tostring(joint_after)
    inertial = after["arm"].find("inertial")
    assert inertial is not None
    mass = inertial.find("mass")
    origin = inertial.find("origin")
    assert mass is not None and origin is not None
    # The tool keeps 0.5 kg at (0.3, 0, 0.05) in the arm frame; the arm gets the rest.
    assert float(mass.get("value", "nan")) == pytest.approx(1.5)
    com = [float(v) for v in origin.get("xyz", "").split()]
    assert com == pytest.approx(
        [(2.0 * 0.2 - 0.5 * 0.3) / 1.5, 0.0, (2.0 * 0.01 - 0.5 * 0.05) / 1.5]
    )
    assert origin.get("rpy") == "0 0 0"
    assert ET.tostring(inertial.find("inertia")) == ET.tostring(
        before["arm"].find("inertial").find("inertia")  # type: ignore[union-attr]
    )
    assert "<!-- kept -->" in baked
    # The body pinocchio builds from the baked file is the fitted one.
    model = pin.buildModelFromXML(baked)
    body = model.inertias[model.getJointId("shoulder")]
    assert body.mass == pytest.approx(2.0)
    assert np.allclose(body.lever, [0.2, 0.0, 0.01])


def test_bake_matches_the_inertial_overrides_on_pinocchio() -> None:
    cal = calibration()
    model = pin.buildModelFromXML(TOY_URDF)
    apply_inertial_overrides(model, to_inertial_overrides(cal))
    baked = pin.buildModelFromXML(bake_into_urdf(cal, TOY_URDF))
    for q in (np.array([-0.7]), np.array([0.3])):
        assert np.allclose(
            pin.computeGeneralizedGravity(baked, baked.createData(), q),
            pin.computeGeneralizedGravity(model, model.createData(), q),
        )


def test_bake_refuses_impossible_or_unknown_links() -> None:
    light = calibration(
        gravity=GravityCalibration(
            links={"arm": LinkGravity(mass_kg=0.4, com_m=(0.0, 0.0, 0.0), merged_links=["tool"])}
        )
    )
    with pytest.raises(ArmCalibrationError, match="lighter than its merged links"):
        bake_into_urdf(light, TOY_URDF)
    unknown = calibration(
        gravity=GravityCalibration(links={"wing": LinkGravity(mass_kg=1.0, com_m=(0, 0, 0))})
    )
    with pytest.raises(ArmCalibrationError, match="no link 'wing'"):
        bake_into_urdf(unknown, TOY_URDF)
    assert bake_into_urdf(calibration(gravity=None), TOY_URDF) == TOY_URDF


# The bench reference fit, converted per arm


@pytest.mark.parametrize(
    ("side", "masses"),
    [
        (
            "left",
            {
                "left_link2": 1.469,
                "left_link3": 1.728,
                "left_link4": 0.712,
                "left_link5": 0.573,
                "left_gripper": 0.885,
            },
        ),
        (
            "right",
            {
                "right_link2": 1.469,
                "right_link3": 1.399,
                "right_link4": 0.739,
                "right_link5": 0.628,
                "right_gripper": 0.938,
            },
        ),
    ],
)
def test_reference_gravity_converts_into_each_arms_calibration(
    dual_openyam_specs: list[ArmCalibrationSpec],
    side: str,
    masses: dict[str, float],
    tmp_path: Path,
) -> None:
    spec = next(s for s in dual_openyam_specs if s.arm_id == f"dual_openyam/{side}")
    converted = calibration_from_legacy_gravity(REFERENCE, spec)
    assert converted.arm_id == f"dual_openyam/{side}"
    assert converted.robot_model == RobotModelRef(name="dual_openyam", sha256=spec.model_sha256())
    assert converted.created == datetime(2026, 10, 16, 2, 46, 34, tzinfo=timezone.utc)
    assert converted.gravity is not None
    links = converted.gravity.links
    assert {name: round(link.mass_kg, 3) for name, link in links.items()} == masses
    assert links[f"{side}_gripper"].merged_links == [
        f"{side}_tip_left",
        f"{side}_tip_right",
        f"{side}_grasp_frame",
    ]
    assert links[f"{side}_link2"].merged_links == []
    # It saves and loads as that arm's calibration file for that model.
    path = save_calibration(tmp_path, converted)
    assert (
        load_calibration(
            path, expected_model_sha256=spec.model_sha256(), expected_arm_id=spec.arm_id
        )
        == converted
    )
    # Baked into the rig URDF, the arm's bodies carry the reference masses.
    model = pin.buildModelFromXML(bake_into_urdf(converted, spec.model.load().xml))
    for name, mass in masses.items():
        frame = model.frames[model.getFrameId(name, pin.FrameType.BODY)]
        assert model.inertias[frame.parentJoint].mass == pytest.approx(mass, abs=5e-4)
