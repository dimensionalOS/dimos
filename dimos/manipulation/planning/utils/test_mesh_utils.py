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
import re
from urllib.parse import unquote, urlparse

import numpy as np
import pytest

from dimos.manipulation.planning.utils import mesh_utils
from dimos.robot.assets.model import LoadedRobotModel


def test_prepare_urdf_for_drake_keeps_xml_in_memory_and_drake_cleanup(
    tmp_path: Path,
) -> None:
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

    description = LoadedRobotModel(
        xml=urdf.read_text().replace("package://pkg", str(package_root)),
        source_path=urdf,
        package_paths={"pkg": package_root},
    )
    prepared = mesh_utils.prepare_urdf_for_drake(description)

    assert "package://" not in prepared.xml
    assert str(mesh) in prepared.xml
    assert "<transmission" not in prepared.xml
    assert not (tmp_path / "drake").exists()


def test_mesh_conversion_cache_is_keyed_by_mesh_content(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    mesh = tmp_path / "link.stl"
    mesh.write_text(
        "solid link\n"
        "facet normal 0 0 1\nouter loop\n"
        "vertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\n"
        "endloop\nendfacet\nendsolid link\n"
    )
    description = LoadedRobotModel(
        xml=f'<robot name="r"><link name="base"><visual><geometry><mesh filename="{mesh}"/>'
        "</geometry></visual></link></robot>",
        source_path=tmp_path / "robot.urdf",
        package_paths={},
    )
    monkeypatch.setattr(mesh_utils, "_CACHE_DIR", tmp_path / "derived" / "drake_meshes")

    first = mesh_utils.prepare_urdf_for_drake(description, convert_meshes=True)
    first_match = re.search(r'filename="([^"]+\.obj)"', first.xml)
    assert first_match is not None
    first_uri = first_match.group(1)
    first_obj = Path(unquote(urlparse(first_uri).path))
    mesh.write_text(mesh.read_text().replace("vertex 1 0 0", "vertex 2 0 0"))
    second = mesh_utils.prepare_urdf_for_drake(description, convert_meshes=True)
    second_match = re.search(r'filename="([^"]+\.obj)"', second.xml)
    assert second_match is not None
    second_uri = second_match.group(1)
    second_obj = Path(unquote(urlparse(second_uri).path))

    assert first_uri.startswith("file://")
    assert second_uri.startswith("file://")
    assert first_obj.exists()
    assert second_obj.exists()
    assert first_obj != second_obj
    assert not list(tmp_path.glob("*.urdf"))


def test_mesh_conversion_keeps_different_same_stem_meshes_distinct(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    visual_mesh = tmp_path / "visual" / "link.stl"
    collision_mesh = tmp_path / "collision" / "link.stl"
    visual_mesh.parent.mkdir()
    collision_mesh.parent.mkdir()
    mesh_template = (
        "solid link\n"
        "facet normal 0 0 1\nouter loop\n"
        "vertex 0 0 0\nvertex {extent} 0 0\nvertex 0 1 0\n"
        "endloop\nendfacet\nendsolid link\n"
    )
    visual_mesh.write_text(mesh_template.format(extent=1))
    collision_mesh.write_text(mesh_template.format(extent=2))
    description = LoadedRobotModel(
        xml=(
            '<robot name="r"><link name="base">'
            f'<visual><geometry><mesh filename="{visual_mesh}"/></geometry></visual>'
            f'<collision><geometry><mesh filename="{collision_mesh}"/></geometry></collision>'
            "</link></robot>"
        ),
        source_path=tmp_path / "robot.urdf",
        package_paths={},
    )
    monkeypatch.setattr(mesh_utils, "_CACHE_DIR", tmp_path / "derived" / "drake_meshes")

    prepared = mesh_utils.prepare_urdf_for_drake(description, convert_meshes=True)

    obj_uris = re.findall(r'filename="([^"]+\.obj)"', prepared.xml)
    obj_paths = [Path(unquote(urlparse(uri).path)) for uri in obj_uris]
    assert len(obj_paths) == 2
    assert obj_paths[0] != obj_paths[1]
    assert all(path.exists() for path in obj_paths)
    assert obj_paths[0].read_text() != obj_paths[1].read_text()


def _cube_points(scale: float) -> np.ndarray:
    return np.array(
        [[x, y, z] for x in (0.0, scale) for y in (0.0, scale) for z in (0.0, scale)],
        dtype=np.float64,
    )


def _hull_for_scale(scale: float) -> str | None:
    # Separate frame so the array is freed on return and CPython reuses its
    # address, the aliasing the old id(points) name turned into shared files.
    points = _cube_points(scale)
    return mesh_utils.pointcloud_to_convex_hull_obj(points)


def test_convex_hull_default_path_is_unique_per_call(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(mesh_utils, "_CACHE_DIR", tmp_path / "derived" / "drake_meshes")

    paths = [_hull_for_scale(0.1 * (i + 1)) for i in range(4)]

    assert all(path is not None for path in paths)
    assert len(set(paths)) == len(paths)
    assert len({Path(path).read_text() for path in paths}) == len(paths)


def test_convex_hull_cache_key_reuses_one_file(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(mesh_utils, "_CACHE_DIR", tmp_path / "derived" / "drake_meshes")

    first = mesh_utils.pointcloud_to_convex_hull_obj(_cube_points(0.1), cache_key="object_a")
    assert first is not None
    before = Path(first).read_text()

    second = mesh_utils.pointcloud_to_convex_hull_obj(_cube_points(0.4), cache_key="object_a")
    assert second == first
    assert Path(first).read_text() != before

    hull_dir = tmp_path / "derived" / "drake_meshes" / "convex_hulls"
    assert len(list(hull_dir.glob("*.obj"))) == 1


def test_convex_hull_cache_keys_that_sanitize_alike_stay_distinct(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(mesh_utils, "_CACHE_DIR", tmp_path / "derived" / "drake_meshes")

    first = mesh_utils.pointcloud_to_convex_hull_obj(_cube_points(0.1), cache_key="object a/1")
    second = mesh_utils.pointcloud_to_convex_hull_obj(_cube_points(0.4), cache_key="object_a_1")

    assert first is not None
    assert second is not None
    assert first != second
    assert Path(first).read_text() != Path(second).read_text()
