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

from pathlib import Path

import pytest
import rerun as rr
import trimesh

pytest.importorskip("yourdfpy", reason="URDF visualization extra is not installed")

from dimos.visualization.rerun.urdf_robot import UrdfRobotStaticRerunFactory


def test_static_robot_logs_nothing_without_components(tmp_path: Path) -> None:
    trimesh.creation.box().export(tmp_path / "link.stl")
    urdf_path = tmp_path / "arm.urdf"
    urdf_path.write_text(
        '<robot name="arm"><link name="base"><visual><geometry>'
        '<mesh filename="link.stl"/></geometry></visual></link></robot>'
    )

    entities = UrdfRobotStaticRerunFactory(urdf_path=urdf_path, root_path="world/arm")(rr)

    assert {type(archetype) for _, archetype in entities} == {rr.Transform3D, rr.Mesh3D}
    # Rerun warns "Dropping chunk without components" for an archetype with no field set.
    assert all(archetype.as_component_batches() for _, archetype in entities)
