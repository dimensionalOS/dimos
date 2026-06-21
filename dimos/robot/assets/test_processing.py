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

import dimos.robot.assets.processing as processing


def test_render_urdf_preserves_package_uris_by_default(
    tmp_path: Path,
    monkeypatch,
) -> None:
    monkeypatch.setattr(processing, "_RENDERED_URDF_CACHE_ROOT", tmp_path / "rendered")
    urdf = tmp_path / "robot.urdf"
    urdf.write_text(
        "<robot name='r'><link name='base'><visual><geometry>"
        "<mesh filename='package://pkg/meshes/link.stl'/>"
        "</geometry></visual></link></robot>"
    )

    rendered = processing.render_urdf(urdf, {"pkg": tmp_path / "pkg"})

    assert rendered.is_relative_to(tmp_path / "rendered")
    assert "package://pkg/meshes/link.stl" in rendered.read_text()


def test_render_urdf_can_rewrite_package_uris_to_absolute_paths(
    tmp_path: Path,
    monkeypatch,
) -> None:
    monkeypatch.setattr(processing, "_RENDERED_URDF_CACHE_ROOT", tmp_path / "rendered")
    package_root = tmp_path / "pkg"
    mesh = package_root / "meshes" / "link.stl"
    mesh.parent.mkdir(parents=True)
    mesh.write_text("solid link\nendsolid link\n")
    urdf = tmp_path / "robot.urdf"
    urdf.write_text(
        "<robot name='r'><link name='base'><visual><geometry>"
        "<mesh filename='package://pkg/meshes/link.stl'/>"
        "</geometry></visual></link></robot>"
    )

    rendered = processing.render_urdf(
        urdf,
        {"pkg": package_root},
        package_uri_mode="absolute",
    )

    rendered_text = rendered.read_text()
    assert "package://" not in rendered_text
    assert str(mesh) in rendered_text


def test_render_urdf_cache_key_tracks_package_root_changes(
    tmp_path: Path,
    monkeypatch,
) -> None:
    monkeypatch.setattr(processing, "_RENDERED_URDF_CACHE_ROOT", tmp_path / "rendered")
    package_root = tmp_path / "pkg"
    mesh = package_root / "meshes" / "link.stl"
    mesh.parent.mkdir(parents=True)
    mesh.write_text("solid old\nendsolid old\n")
    urdf = tmp_path / "robot.urdf"
    urdf.write_text(
        "<robot name='r'><link name='base'><visual><geometry>"
        "<mesh filename='package://pkg/meshes/link.stl'/>"
        "</geometry></visual></link></robot>"
    )

    first = processing.render_urdf(urdf, {"pkg": package_root})
    mesh.write_text("solid new\nendsolid new\n")
    second = processing.render_urdf(urdf, {"pkg": package_root})

    assert second != first


def test_render_urdf_strips_nested_urdf_suffix_from_cache_name(
    tmp_path: Path,
    monkeypatch,
) -> None:
    monkeypatch.setattr(processing, "_RENDERED_URDF_CACHE_ROOT", tmp_path / "rendered")
    xacro = tmp_path / "robot.urdf.xacro"
    xacro.write_text("<robot name='r'/>")
    monkeypatch.setattr(
        processing,
        "_process_xacro",
        lambda _path, _package_paths, _xacro_args: "<robot name='r'/>",
    )

    rendered = processing.render_urdf(xacro)

    assert rendered.name == "robot.urdf"
