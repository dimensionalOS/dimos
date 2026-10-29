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

"""Flattening the xArm7 MJCF for robosuite must not change the robot."""

from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

from dimos.simulation.libero import xarm_robot
from dimos.utils.data import LfsPath


@pytest.fixture(scope="module")
def xarm_xml() -> Path:
    return Path(LfsPath("xarm7")) / "xarm7.xml"


@pytest.fixture(scope="module")
def flat(xarm_xml: Path) -> ET.Element:
    return xarm_robot.flatten(xarm_xml)


@pytest.mark.self_hosted  # needs the xarm7 LFS asset
def test_flat_mjcf_has_no_classes_and_names_every_geom(flat: ET.Element) -> None:
    assert flat.find("default") is None
    for element in flat.iter():
        assert "class" not in element.attrib and "childclass" not in element.attrib
    assert all(geom.get("name") for geom in flat.iter("geom"))
    assert all(mesh.get("name") for mesh in flat.iter("mesh"))
    assert flat.find(".//body[@name='right_hand']") is not None
    assert flat.find("worldbody/body").get("pos") == "0 0 0"


@pytest.mark.self_hosted  # needs the xarm7 LFS asset
def test_flat_mjcf_keeps_the_xarm_dynamics(xarm_xml: Path, flat: ET.Element) -> None:
    # The file's keyframe is sized for the full scene that includes it; drop it here.
    source = ET.parse(xarm_xml).getroot()
    source.remove(source.find("keyframe"))
    source.find("compiler").set("meshdir", str(xarm_xml.parent / "assets"))
    original = mujoco.MjModel.from_xml_string(ET.tostring(source, encoding="unicode"))
    flattened = mujoco.MjModel.from_xml_string(ET.tostring(flat, encoding="unicode"))
    assert (flattened.njnt, flattened.nu, flattened.ngeom) == (
        original.njnt,
        original.nu,
        original.ngeom,
    )
    for field in ("dof_damping", "dof_armature", "dof_frictionloss", "jnt_range", "jnt_limited"):
        np.testing.assert_allclose(
            getattr(flattened, field), getattr(original, field), err_msg=field
        )
    for field in (
        "actuator_gainprm",
        "actuator_biasprm",
        "actuator_ctrlrange",
        "actuator_forcerange",
    ):
        np.testing.assert_allclose(
            getattr(flattened, field), getattr(original, field), err_msg=field
        )
    for field in ("geom_type", "geom_size", "geom_friction", "geom_contype", "geom_group"):
        np.testing.assert_allclose(
            getattr(flattened, field), getattr(original, field), err_msg=field
        )


def test_forward_moves_fixed_and_table_length_offsets() -> None:
    assert xarm_robot._forward((-0.6, 0.0, 0.0), 0.15) == pytest.approx((-0.45, 0.0, 0.0))
    table = xarm_robot._forward(lambda length: (-0.16 - length / 2, 0.0, 0.0), 0.15)
    assert table(1.0) == pytest.approx((-0.51, 0.0, 0.0))
