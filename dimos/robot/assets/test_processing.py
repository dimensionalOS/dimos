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


def test_load_robot_description_preserves_package_uris_without_writing_output(
    tmp_path: Path,
) -> None:
    urdf = tmp_path / "robot.urdf"
    urdf.write_text(
        "<robot name='r'><link name='base'><visual><geometry>"
        "<mesh filename='package://pkg/meshes/link.stl'/>"
        "</geometry></visual></link></robot>"
    )

    description = processing.load_robot_description(urdf, {"pkg": tmp_path / "pkg"})

    assert description.source_path == urdf
    assert "package://pkg/meshes/link.stl" in description.xml
    assert list(tmp_path.iterdir()) == [urdf]


def test_load_robot_description_can_rewrite_package_uris_to_absolute_paths(
    tmp_path: Path,
) -> None:
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

    description = processing.load_robot_description(
        urdf,
        {"pkg": package_root},
        package_uri_mode="absolute",
    )

    assert "package://" not in description.xml
    assert str(mesh) in description.xml


def test_load_robot_description_reruns_xacro_each_time(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    xacro = tmp_path / "robot.urdf.xacro"
    xacro.write_text("<robot name='r'/>")
    calls = iter(("first", "second"))
    monkeypatch.setattr(
        processing,
        "_process_xacro",
        lambda _path, _package_paths, _xacro_args: f"<robot name='{next(calls)}'/>",
    )

    first = processing.load_robot_description(xacro)
    second = processing.load_robot_description(xacro)

    assert first.xml == "<robot name='first'/>"
    assert second.xml == "<robot name='second'/>"
    assert list(tmp_path.iterdir()) == [xacro]


def test_load_robot_description_adds_ordered_fixed_frames(
    tmp_path: Path,
) -> None:
    urdf = tmp_path / "robot.urdf"
    urdf.write_text("<robot name='r'><link name='base'/></robot>")

    description = processing.load_robot_description(
        urdf,
        additional_fixed_frames=(
            processing.FixedFrameDefinition("tool", "base", xyz=(0.1, 0.2, 0.3)),
            processing.FixedFrameDefinition("camera", "tool", rpy=(0.0, 0.0, 1.57)),
        ),
    )

    root = ET.fromstring(description.xml)
    assert [link.get("name") for link in root.findall("link")] == ["base", "tool", "camera"]
    joints = root.findall("joint")
    assert [joint.get("name") for joint in joints] == ["tool_joint", "camera_joint"]
    assert joints[0].find("origin").attrib == {"xyz": "0.1 0.2 0.3", "rpy": "0.0 0.0 0.0"}
    assert joints[1].find("parent").attrib == {"link": "tool"}


@pytest.mark.parametrize(
    ("frames", "message"),
    [
        ((processing.FixedFrameDefinition("base", "base"),), "link already exists"),
        ((processing.FixedFrameDefinition("tool", "missing"),), "unknown parent link"),
    ],
)
def test_load_robot_description_rejects_invalid_fixed_frames(
    tmp_path: Path,
    frames: tuple[processing.FixedFrameDefinition, ...],
    message: str,
) -> None:
    urdf = tmp_path / "robot.urdf"
    urdf.write_text("<robot name='r'><link name='base'/></robot>")

    with pytest.raises(ValueError, match=message):
        processing.load_robot_description(urdf, additional_fixed_frames=frames)


def test_load_robot_description_rejects_fixed_frames_for_mjcf(tmp_path: Path) -> None:
    mjcf = tmp_path / "robot.xml"
    mjcf.write_text("<mujoco/>")

    with pytest.raises(ValueError, match="only for URDF/Xacro"):
        processing.load_robot_description(
            mjcf,
            additional_fixed_frames=(processing.FixedFrameDefinition("tool", "base"),),
        )
