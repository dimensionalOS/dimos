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


"""The M20 model every M20 blueprint shows comes from the vendor MJCF the sim steps, under the robot's own joint names."""

from pathlib import Path
from xml.etree import ElementTree as ET

import mujoco

from dimos.robot.deeprobotics.m20.model import M20_MJCF, M20_MODEL, STAND_JOINT_POSITIONS


def test_the_model_has_every_joint_the_robot_reports() -> None:
    loaded = M20_MODEL.load()
    assert loaded.root_link == "base_link"
    names = {joint.name for joint in loaded.joints}
    assert set(STAND_JOINT_POSITIONS) <= names
    assert {f"{leg}_wheel_joint" for leg in ("fl", "fr", "hl", "hr")} <= names


def test_the_model_matches_the_mjcf_and_its_meshes_exist() -> None:
    mjcf = mujoco.MjModel.from_xml_path(str(M20_MJCF))
    hinges = {
        mjcf.joint(j).name
        for j in range(mjcf.njnt)
        if mjcf.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE
    }
    assert {joint.name for joint in M20_MODEL.load().joints} == hinges
    meshes = [m.get("filename") for m in ET.fromstring(M20_MODEL.load().xml).iter("mesh")]
    assert len(meshes) == 17 and all(Path(str(m)).exists() for m in meshes)
