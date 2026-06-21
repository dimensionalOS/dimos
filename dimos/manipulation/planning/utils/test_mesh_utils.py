# Copyright 2025-2026 Dimensional Inc.
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

from dimos.manipulation.planning.utils import mesh_utils
import dimos.robot.assets.processing as processing


def test_prepare_urdf_for_drake_uses_rendered_urdf_and_keeps_drake_cleanup(
    tmp_path: Path,
    monkeypatch,
) -> None:
    monkeypatch.setattr(processing, "_RENDERED_URDF_CACHE_ROOT", tmp_path / "rendered")
    monkeypatch.setattr(mesh_utils, "_CACHE_DIR", tmp_path / "drake")
    package_root = tmp_path / "pkg"
    mesh = package_root / "meshes" / "link.stl"
    mesh.parent.mkdir(parents=True)
    mesh.write_text("solid link\nendsolid link\n")
    urdf = tmp_path / "robot.urdf"
    urdf.write_text(
        "<robot name='r'>"
        "<link name='base'><visual><geometry>"
        "<mesh filename='package://pkg/meshes/link.stl'/>"
        "</geometry></visual></link>"
        "<transmission name='ignore_me'><type>bad</type></transmission>"
        "</robot>"
    )

    prepared = Path(
        mesh_utils.prepare_urdf_for_drake(
            urdf,
            {"pkg": package_root},
        )
    )

    prepared_text = prepared.read_text()
    assert prepared.is_relative_to(tmp_path / "drake")
    assert "package://" not in prepared_text
    assert str(mesh) in prepared_text
    assert "<transmission" not in prepared_text
