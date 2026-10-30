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

from copy import deepcopy
import json
import math
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

from native.unreal.DimosRenderer.Scripts.export_level_collisions import (
    collision_vertices,
    export_level,
)


@pytest.fixture
def audit():
    transform = {"translation": [0, 0, 0], "rotation_xyzw": [0, 0, 0, 1], "scale": [1, 1, 1]}
    asset = "/Game/Fixtures/Box.Box"
    component = {
        "path": "/Game/Room.Room:PersistentLevel.Box.Mesh",
        "transform": transform,
        "collision_enabled": "<CollisionEnabled.QUERY_AND_PHYSICS: 3>",
        "collision_profile": "BlockAll",
        "simulating_physics": False,
        "mobility": "<ComponentMobility.STATIC: 0>",
        "asset": asset,
    }
    return {
        "map": "/Game/Room",
        "errors": [],
        "actors": [{"path": "Box", "label": "Box", "meshes": [component]}],
        "assets": {
            asset: {
                "collision_trace": "CTF_UseSimpleAndComplex",
                "colliders": [{"type": "box", "size": [100, 200, 20], "transform": transform}],
            }
        },
    }


def test_nested_collision_transform_preserves_reflection_and_nonuniform_scale():
    collider = {
        "type": "box",
        "size": [2, 4, 6],
        "transform": {
            "translation": [10, 20, 30],
            "rotation_xyzw": [0, 0, math.sqrt(0.5), math.sqrt(0.5)],
            "scale": [1, 2, 1],
        },
    }

    points = np.asarray(collision_vertices(collider, (-2, 3, 0.5)))

    assert points.min(axis=0) == pytest.approx((-0.28, -0.63, 0.135))
    assert points.max(axis=0) == pytest.approx((-0.12, -0.57, 0.165))


def test_static_instances_reuse_collision_meshes_without_visual_or_body_duplicates(audit, tmp_path):
    second = deepcopy(audit["actors"][0])
    second["path"] = "Box2"
    second["meshes"][0]["path"] += "2"
    second["meshes"][0]["transform"]["translation"] = [300, 400, 50]
    audit["actors"].append(second)

    manifest = export_level(audit, tmp_path, spawn=(0, 0, 0.1), source_sha256="source")
    model = mujoco.MjModel.from_xml_path(str(tmp_path / "scene.xml"))
    root = ET.parse(tmp_path / "scene.xml").getroot()
    metadata = json.loads(root.find("./custom/text").attrib["data"])

    assert model.nbody == 1
    assert model.ngeom == 2
    assert model.nmesh == 1
    assert model.geom_pos[1] == pytest.approx((3, -4, 0.5))
    assert metadata["geoms"] == {}
    assert len(metadata["hidden_geoms"]) == 2
    assert [item["actor"] for item in manifest["components"]] == ["Box", "Box2"]


def test_hidden_unreal_collision_geometry_remains_visible_to_physics_rays(audit, tmp_path):
    export_level(audit, tmp_path, spawn=(0, 0, 0.1), source_sha256="source")
    model = mujoco.MjModel.from_xml_path(str(tmp_path / "scene.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    hit = np.full(1, -1, dtype=np.int32)

    distance = mujoco.mj_ray(
        model,
        data,
        np.array([0.0, 0.0, 1.0]),
        np.array([0.0, 0.0, -1.0]),
        np.ones(6, dtype=np.uint8),
        True,
        -1,
        hit,
    )

    assert distance == pytest.approx(0.9)
    assert hit[0] == 0
    sidecar = json.loads((tmp_path / "scene.json").read_text())
    assert not sidecar.get("hidden_geom_groups")
    root = ET.parse(tmp_path / "scene.xml").getroot()
    metadata = json.loads(root.find("./custom/text").attrib["data"])
    assert metadata["hidden_geoms"] == [model.geom(0).name]


def test_explicit_plane_thickness_preserves_finite_bounds_and_original_upper_surface(audit):
    collider = audit["assets"]["/Game/Fixtures/Box.Box"]["colliders"][0]
    collider["size"] = [100, 100, 0]

    points = np.asarray(collision_vertices(collider, (15, 15, 1), surface_thickness=0.001))

    assert points.min(axis=0) == pytest.approx((-7.5, -7.5, -0.001))
    assert points.max(axis=0) == pytest.approx((7.5, 7.5, 0))


@pytest.mark.parametrize("problem", ["missing", "complex", "dynamic", "movable", "building"])
def test_export_rejects_unfaithful_collision_before_writing(audit, tmp_path, problem):
    component = audit["actors"][0]["meshes"][0]
    source = audit["assets"][component["asset"]]
    if problem == "missing":
        source["colliders"] = []
    elif problem == "complex":
        source["collision_trace"] = "CTF_UseComplexAsSimple"
    elif problem == "dynamic":
        component["simulating_physics"] = True
    elif problem == "movable":
        component["mobility"] = "<ComponentMobility.MOVABLE: 2>"
    else:
        component["asset"] = "/Game/Konbini/Assets/Geo/SM_KombiniGeo.SM_KombiniGeo"
        audit["assets"] = {component["asset"]: source}

    with pytest.raises(ValueError):
        export_level(audit, tmp_path, spawn=(0, 0, 0), source_sha256="source")

    assert not list(tmp_path.iterdir())
