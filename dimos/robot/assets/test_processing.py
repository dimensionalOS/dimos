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
import xml.etree.ElementTree as ET

import pytest

import dimos.robot.assets.processing as processing


def test_render_urdf_preserves_package_uris_by_default(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
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
    monkeypatch: pytest.MonkeyPatch,
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
    monkeypatch: pytest.MonkeyPatch,
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
    monkeypatch: pytest.MonkeyPatch,
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


def test_rendered_robot_description_is_lazy(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    rendered = tmp_path / "rendered" / "robot.urdf"
    calls: list[object] = []

    def fake_render(*args, **kwargs) -> Path:
        calls.append((args, kwargs))
        return rendered

    monkeypatch.setattr(processing, "render_urdf", fake_render)

    lazy_path = processing.rendered_robot_description(tmp_path / "robot.urdf.xacro")

    assert lazy_path.name == "robot.urdf"
    assert lazy_path.suffix == ".urdf"
    assert calls == []
    assert str(lazy_path) == str(rendered)
    assert len(calls) == 1
    assert str(lazy_path) == str(rendered)
    assert len(calls) == 1


def test_render_urdf_can_remove_joint_subtrees(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(processing, "_RENDERED_URDF_CACHE_ROOT", tmp_path / "rendered")
    urdf = tmp_path / "robot.urdf"
    urdf.write_text(
        """
        <robot name="r">
          <link name="base"/>
          <link name="wrist"/>
          <link name="finger"/>
          <link name="finger_tip"/>
          <joint name="arm" type="revolute">
            <parent link="base"/><child link="wrist"/>
          </joint>
          <joint name="gripper" type="prismatic">
            <parent link="wrist"/><child link="finger"/>
          </joint>
          <joint name="tip" type="fixed">
            <parent link="finger"/><child link="finger_tip"/>
          </joint>
        </robot>
        """
    )

    rendered = processing.render_urdf(
        urdf,
        removed_joint_names=frozenset({"gripper"}),
    )

    root = ET.parse(rendered).getroot()
    assert [link.get("name") for link in root.findall("link")] == ["base", "wrist"]
    assert [joint.get("name") for joint in root.findall("joint")] == ["arm"]


def test_render_urdf_rejects_unknown_removed_joint(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(processing, "_RENDERED_URDF_CACHE_ROOT", tmp_path / "rendered")
    urdf = tmp_path / "robot.urdf"
    urdf.write_text("<robot name='r'><link name='base'/></robot>")

    try:
        processing.render_urdf(
            urdf,
            removed_joint_names=frozenset({"missing"}),
        )
    except ValueError as exc:
        assert "missing" in str(exc)
    else:
        raise AssertionError("Expected an unknown joint to fail rendering")
