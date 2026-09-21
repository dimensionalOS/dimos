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

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from dimos.experimental.scene_cooking import planning as plan_module
from dimos.experimental.scene_cooking.package_config import browser_visual_spec_for_target
from dimos.experimental.scene_cooking.sidecar import SceneCookSidecar
from dimos.experimental.scene_cooking.source_assets.mesh import ScenePrimMesh
from dimos.simulation.scene_assets.spec import (
    ARTIFACT_FRAMES,
    SceneMeshAlignment,
    ScenePackage,
    load_scene_package,
)


def _metadata(tmp_path: Path) -> dict[str, Any]:
    return {
        "source_path": str(tmp_path / "source.glb"),
        "package_dir": str(tmp_path),
        "alignment": {
            "scale": 1.0,
            "rotation_zyx_deg": [0.0, 0.0, 0.0],
            "translation": [0.0, 0.0, 0.0],
            "y_up": True,
        },
        "artifact_frames": ARTIFACT_FRAMES,
        "artifacts": {
            "browser_visual": str(tmp_path / "visual.glb"),
            "browser_visuals": {
                "rerun": str(tmp_path / "visual.rerun.glb"),
                "babylon": str(tmp_path / "visual.babylon.glb"),
            },
            "browser_collision": str(tmp_path / "collision.glb"),
            "objects": str(tmp_path / "objects.json"),
            "mujoco_scene": str(tmp_path / "wrapper.xml"),
            "mujoco_binary": str(tmp_path / "wrapper.mjb"),
        },
        "stats": {},
    }


def test_load_scene_package_rejects_missing_artifact_frames(tmp_path: Path) -> None:
    raw = _metadata(tmp_path)
    raw.pop("artifact_frames")
    metadata_path = tmp_path / "scene.meta.json"
    metadata_path.write_text(json.dumps(raw))

    with pytest.raises(ValueError, match="missing artifact frame metadata"):
        load_scene_package(metadata_path)


def test_load_scene_package_rejects_mismatched_artifact_frames(tmp_path: Path) -> None:
    raw = _metadata(tmp_path)
    raw["artifact_frames"] = {
        "browser_visual": "dimos_world",
        "browser_collision": "source",
        "mujoco": "dimos_world",
    }
    metadata_path = tmp_path / "scene.meta.json"
    metadata_path.write_text(json.dumps(raw))

    with pytest.raises(ValueError, match="artifact frame mismatch"):
        load_scene_package(metadata_path)


def test_entity_collision_paths_round_trip(tmp_path: Path) -> None:
    raw = _metadata(tmp_path)
    raw["entities"] = [
        {
            "id": "chair",
            "visual_path": "entities/chair/visual.glb",
            "collision_paths": [
                "entities/chair/mujoco_collision/hull_000.obj",
                "entities/chair/mujoco_collision/hull_001.obj",
            ],
            "descriptor": {"shape_hint": "mesh"},
        }
    ]
    metadata_path = tmp_path / "scene.meta.json"
    metadata_path.write_text(json.dumps(raw))

    package = load_scene_package(metadata_path)

    assert package.entities[0]["collision_paths"] == [
        str(tmp_path / "entities/chair/mujoco_collision/hull_000.obj"),
        str(tmp_path / "entities/chair/mujoco_collision/hull_001.obj"),
    ]
    assert package.to_json_dict()["entities"][0]["collision_paths"] == [
        "entities/chair/mujoco_collision/hull_000.obj",
        "entities/chair/mujoco_collision/hull_001.obj",
    ]


def test_load_scene_package_accepts_expected_artifact_frames(tmp_path: Path) -> None:
    metadata_path = tmp_path / "scene.meta.json"
    metadata_path.write_text(json.dumps(_metadata(tmp_path)))

    package = load_scene_package(metadata_path)

    assert package.visual_path == tmp_path / "visual.glb"
    assert package.browser_visual_path("rerun") == tmp_path / "visual.rerun.glb"
    assert package.browser_visual_path("babylon") == tmp_path / "visual.babylon.glb"
    assert package.browser_collision_path == tmp_path / "collision.glb"
    assert package.objects_path == tmp_path / "objects.json"
    assert package.mujoco_scene_path == tmp_path / "wrapper.xml"
    assert package.mujoco_binary_path == tmp_path / "wrapper.mjb"


def test_scene_package_metadata_uses_package_relative_paths(tmp_path: Path) -> None:
    package = ScenePackage(
        package_dir=tmp_path,
        source_path=tmp_path / "source.glb",
        alignment=SceneMeshAlignment(),
        visual_path=tmp_path / "browser" / "visual.glb",
        browser_visuals={
            "rerun": tmp_path / "browser" / "visual.rerun.glb",
            "babylon": tmp_path / "browser" / "visual.babylon.glb",
        },
        browser_collision_path=tmp_path / "browser" / "collision.glb",
        objects_path=tmp_path / "browser" / "objects.json",
        mujoco_scene_path=tmp_path / "mujoco" / "abc123" / "wrapper.xml",
        mujoco_binary_path=tmp_path / "mujoco" / "abc123" / "wrapper.mjb",
        entities=[
            {
                "id": "chair_001",
                "visual_path": str(tmp_path / "entities" / "chair_001" / "visual.glb"),
            }
        ],
    )

    metadata_path = package.write_metadata()
    raw = json.loads(metadata_path.read_text())

    assert raw["package_dir"] == "."
    assert raw["artifacts"]["browser_visual"] == "browser/visual.glb"
    assert raw["artifacts"]["browser_visuals"] == {
        "babylon": "browser/visual.babylon.glb",
        "rerun": "browser/visual.rerun.glb",
    }
    assert raw["artifacts"]["browser_collision"] == "browser/collision.glb"
    assert raw["artifacts"]["objects"] == "browser/objects.json"
    assert raw["artifacts"]["mujoco_scene"] == "mujoco/abc123/wrapper.xml"
    assert raw["artifacts"]["mujoco_binary"] == "mujoco/abc123/wrapper.mjb"
    assert raw["entities"][0]["visual_path"] == "entities/chair_001/visual.glb"

    loaded = load_scene_package(metadata_path)
    assert loaded.package_dir == tmp_path
    assert loaded.visual_path == tmp_path / "browser" / "visual.glb"
    assert loaded.browser_visual_path("rerun") == tmp_path / "browser" / "visual.rerun.glb"
    assert loaded.browser_visual_path("babylon") == tmp_path / "browser" / "visual.babylon.glb"
    assert loaded.mujoco_scene_path == tmp_path / "mujoco" / "abc123" / "wrapper.xml"
    assert loaded.mujoco_binary_path == tmp_path / "mujoco" / "abc123" / "wrapper.mjb"
    assert loaded.entities[0]["visual_path"] == str(
        tmp_path / "entities" / "chair_001" / "visual.glb"
    )


def test_load_scene_package_tolerates_missing_objects_sidecar(tmp_path: Path) -> None:
    raw = _metadata(tmp_path)
    # Older cooked packages without the semantic sidecar should still load.
    raw["artifacts"].pop("objects")
    metadata_path = tmp_path / "scene.meta.json"
    metadata_path.write_text(json.dumps(raw))

    package = load_scene_package(metadata_path)

    assert package.objects_path is None


def test_browser_visual_profiles_are_backend_specific() -> None:
    rerun = browser_visual_spec_for_target("rerun")
    babylon = browser_visual_spec_for_target("babylon")

    assert rerun.artifact_name == "visual.rerun.glb"
    assert rerun.quantize is False
    assert rerun.normalize_textures is True
    assert rerun.demote_required_extensions == ("KHR_texture_transform",)

    assert babylon.artifact_name == "visual.babylon.glb"
    assert babylon.quantize is True
    assert babylon.normalize_textures is False
    assert babylon.demote_required_extensions == ()


def test_extract_scene_objects_emits_per_prim_aabb() -> None:
    # Inlined so importing this test module doesn't pull heavy open3d/trimesh
    # for the many tests here that don't need browser.collision.
    from dimos.experimental.scene_cooking.browser.collision import extract_scene_objects

    triangles = np.array([[0, 1, 2]], dtype=np.int32)
    prims = [
        ScenePrimMesh(
            name="Sectional_seat",
            prim_path="/Apt/Living/Sectional/seat",
            vertices=np.array(
                [[-1.0, -2.0, 0.0], [2.0, -2.0, 0.5], [-1.0, 1.0, 1.0]],
                dtype=np.float32,
            ),
            triangles=triangles,
        ),
        ScenePrimMesh(
            name="Empty_prim",
            prim_path="/Apt/Living/Empty",
            vertices=np.empty((0, 3), dtype=np.float32),
            triangles=np.empty((0, 3), dtype=np.int32),
        ),
    ]

    objects = extract_scene_objects(prims)

    assert len(objects) == 1  # empty prim filtered
    entry = objects[0]
    assert entry["id"] == "Sectional_seat"
    assert entry["prim_path"] == "/Apt/Living/Sectional/seat"
    assert entry["aabb"]["min"] == [-1.0, -2.0, 0.0]
    assert entry["aabb"]["max"] == [2.0, 1.0, 1.0]


def test_load_scene_package_preserves_packaged_entities(tmp_path: Path) -> None:
    raw = _metadata(tmp_path)
    raw["entities"] = [
        {
            "id": "chair_016",
            "descriptor": {"entity_id": "chair_016", "shape_hint": "box"},
        }
    ]
    metadata_path = tmp_path / "scene.meta.json"
    metadata_path.write_text(json.dumps(raw))

    package = load_scene_package(metadata_path)

    assert package.entities[0]["id"] == "chair_016"


def test_scene_cook_plan_maps_collision_prims_to_blender_visual_nodes(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    def fake_load_scene_prims(
        path: str | Path,
        alignment: SceneMeshAlignment | None = None,
    ) -> list[ScenePrimMesh]:
        del path, alignment
        triangles = np.array([[0, 1, 2]], dtype=np.int32)
        return [
            ScenePrimMesh(
                name="Chair_seat",
                prim_path="Chair_a1b2c3",
                vertices=np.array(
                    [[-1.0, -1.0, 0.2], [-0.5, -1.0, 0.2], [-1.0, -0.5, 0.8]],
                    dtype=np.float32,
                ),
                triangles=triangles,
            ),
            ScenePrimMesh(
                name="Chair.016_seat",
                prim_path="Chair.016_a1b2c3",
                vertices=np.array(
                    [[1.0, 2.0, 0.2], [2.0, 2.0, 0.2], [1.0, 3.0, 0.8]],
                    dtype=np.float32,
                ),
                triangles=triangles,
            ),
            ScenePrimMesh(
                name="Chair.016_back",
                prim_path="Chair.016_d4e5f6",
                vertices=np.array(
                    [[1.0, 2.0, 0.8], [2.0, 3.0, 1.4], [1.5, 2.5, 1.2]],
                    dtype=np.float32,
                ),
                triangles=triangles,
            ),
        ]

    monkeypatch.setattr(plan_module, "load_scene_prims", fake_load_scene_prims)
    sidecar = SceneCookSidecar.from_dict(
        {
            "interactables": [
                {
                    "id": "chair_000",
                    "source_prim_paths": ["Chair_*"],
                    "physics": {"shape": "box"},
                },
                {
                    "id": "chair_016",
                    "source_prim_paths": ["Chair.016_*"],
                    "physics": {"shape": "box"},
                },
            ]
        }
    )

    plan = plan_module.build_scene_cook_plan(
        tmp_path / "office.glb",
        sidecar=sidecar,
        alignment=SceneMeshAlignment(scale=2.0, y_up=False),
        output_dir=tmp_path,
    )

    base_entity = plan.entities[0]
    assert base_entity.matched_prim_paths == ("Chair_a1b2c3",)
    assert base_entity.visual_node_patterns == ("Chair",)
    assert base_entity.descriptor["mesh_ref"] == "entities/chair_000/visual.glb"

    entity = plan.entities[1]
    assert entity.matched_prim_paths == ("Chair.016_a1b2c3", "Chair.016_d4e5f6")
    assert entity.visual_node_patterns == ("Chair.016",)
    assert entity.descriptor["mesh_ref"] == "entities/chair_016/visual.glb"
    assert plan.collision_spec.resolve("Chair_a1b2c3")["type"] == "skip"
    assert plan.collision_spec.resolve("Chair.016_a1b2c3")["type"] == "skip"
    assert plan.collision_spec.resolve("Chair.001_a1b2c3")["type"] == "auto"


def test_synthetic_entity_uses_pose_and_extents(tmp_path: Path) -> None:
    sidecar = SceneCookSidecar.from_dict(
        {
            "interactables": [
                {
                    "id": "manip_cube",
                    "pose": {"x": 0.0, "y": 0.75, "z": 0.69},
                    "kind": "dynamic",
                    "mass": 0.15,
                    "physics": {"shape": "box", "extents": [0.08, 0.08, 0.08]},
                    "visual": {"rgba": [0.85, 0.20, 0.20, 1.0]},
                    "tags": ["manipulation"],
                },
            ]
        }
    )
    plan = plan_module.build_scene_cook_plan(
        tmp_path / "office.glb",
        sidecar=sidecar,
        alignment=SceneMeshAlignment(),
        output_dir=tmp_path,
    )

    entity = plan.entities[0]
    assert entity.spec.is_synthetic
    assert entity.matched_prim_paths == ()
    assert entity.visual_path is None
    assert entity.center == (0.0, 0.75, 0.69)
    assert entity.descriptor["shape_hint"] == "box"
    assert entity.descriptor["extents"] == [0.08, 0.08, 0.08]
    assert entity.descriptor["rgba"] == [0.85, 0.20, 0.20, 1.0]
    assert entity.descriptor["mesh_ref"] == ""


def test_entity_group_expands_per_prim_with_shared_prototypes(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    def fake_load_scene_prims(
        path: str | Path,
        alignment: SceneMeshAlignment | None = None,
    ) -> list[ScenePrimMesh]:
        del path, alignment
        triangles = np.array([[0, 1, 2], [1, 2, 3]], dtype=np.int32)
        return [
            ScenePrimMesh(
                name="cereal_1",
                prim_path="Grocery_Scatter_Left_Aligned__2_CerealBlue.0001_2_CerealBlue_Mesh",
                vertices=np.array(
                    [
                        [0.0, 0.0, 0.0],
                        [0.2, 0.0, 0.0],
                        [0.0, 0.1, 0.3],
                        [0.2, 0.1, 0.3],
                    ],
                    dtype=np.float32,
                ),
                triangles=triangles,
            ),
            ScenePrimMesh(
                name="cereal_2",
                prim_path="Grocery_Scatter_Right_Aligned__2_CerealBlue.0002_2_CerealBlue_Mesh",
                vertices=np.array(
                    [
                        [1.0, 0.0, 0.0],
                        [1.0, 0.2, 0.0],
                        [0.9, 0.0, 0.3],
                        [0.9, 0.2, 0.3],
                    ],
                    dtype=np.float32,
                ),
                triangles=triangles,
            ),
            ScenePrimMesh(
                name="apple",
                prim_path="Produce_Scatter__4_RedApple.0001_4_RedApple_Mesh",
                vertices=np.array(
                    [
                        [2.0, 0.0, 0.0],
                        [2.1, 0.0, 0.0],
                        [2.0, 0.1, 0.1],
                        [2.1, 0.1, 0.1],
                    ],
                    dtype=np.float32,
                ),
                triangles=triangles,
            ),
            ScenePrimMesh(
                name="cereal_no_instance_suffix",
                prim_path="Grocery_Scatter_Left_Aligned__2_CerealBlue_2_CerealBlue_Mesh",
                vertices=np.array(
                    [
                        [3.0, 0.0, 0.0],
                        [3.2, 0.0, 0.0],
                        [3.0, 0.1, 0.3],
                        [3.2, 0.1, 0.3],
                    ],
                    dtype=np.float32,
                ),
                triangles=triangles,
            ),
        ]

    monkeypatch.setattr(plan_module, "load_scene_prims", fake_load_scene_prims)
    sidecar = SceneCookSidecar.from_dict(
        {
            "entity_groups": [
                {
                    "id_prefix": "product",
                    "source_prim_paths": ["Grocery_Scatter_*", "Produce_Scatter__*"],
                    "kind": "dynamic",
                    "mass": 0.05,
                    "physics": {"shape": "mesh", "friction": [0.6, 0.02, 0.001]},
                    "tags": ["shelf_product"],
                }
            ]
        }
    )

    plan = plan_module.build_scene_cook_plan(
        tmp_path / "supermarket.glb",
        sidecar=sidecar,
        alignment=SceneMeshAlignment(),
        output_dir=tmp_path,
    )

    assert len(plan.entities) == 4
    assert len(plan.prototypes) == 3
    assert {prototype.id for prototype in plan.prototypes} == {
        "product_2_CerealBlue",
        "product_4_RedApple",
        "product_2_CerealBlue_2_CerealBlue",
    }
    entity_prototype_ids = [entity.prototype_id for entity in plan.entities]
    assert entity_prototype_ids.count("product_2_CerealBlue") == 2
    assert entity_prototype_ids.count("product_4_RedApple") == 1
    assert entity_prototype_ids.count("product_2_CerealBlue_2_CerealBlue") == 1
    assert plan.entities[0].descriptor["shape_hint"] == "mesh"
    assert plan.entities[0].descriptor["mesh_ref"] == ""
    assert plan.entities[0].descriptor["prototype_id"] == "product_2_CerealBlue"
    assert plan.entities[0].initial_quat[0] != 0.0
    assert (
        plan.collision_spec.resolve(
            "Grocery_Scatter_Left_Aligned__2_CerealBlue.0001_2_CerealBlue_Mesh"
        )["type"]
        == "skip"
    )
    assert plan.collision_spec.resolve("Wall")["type"] == "auto"


def test_interactable_requires_prims_or_pose() -> None:
    with pytest.raises(ValueError, match="source_prim_paths.*or pose"):
        SceneCookSidecar.from_dict({"interactables": [{"id": "ghost"}]})


def test_entity_group_requires_source_prim_paths() -> None:
    with pytest.raises(ValueError, match="source_prim_paths required"):
        SceneCookSidecar.from_dict({"entity_groups": [{"id_prefix": "product"}]})
