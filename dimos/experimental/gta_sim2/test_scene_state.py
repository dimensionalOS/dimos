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

"""Inactive variants cannot reenter via spatial queries, persistence or stale caches."""

import json
from pathlib import Path
import sqlite3

import numpy as np
from pxr import Gf, Usd, UsdGeom
import pytest

from dimos.experimental.gta_sim2.prepare_m20 import BASE
from dimos.experimental.gta_sim2.scene_state import bind_catalog
from dimos.experimental.gta_sim2.spawns import load_spawns, prepare
from dimos.experimental.gta_sim2.world_geometry import WorldCatalog
from dimos.experimental.gta_sim2.world_index import add_instance, file_digest


def test_state_filters_before_admission_and_keeps_shared_sources(tmp_path: Path) -> None:
    archive, decoder = tmp_path / "levels.rpf", tmp_path / "decoder"
    archive.write_bytes(b"fixture")
    decoder.write_bytes(b"decoder")
    database = tmp_path / "world.sqlite"
    with sqlite3.connect(database) as db:
        db.executescript("""
            CREATE TABLE instances(id INTEGER PRIMARY KEY, identity TEXT UNIQUE, source TEXT,
                transform TEXT, bbox TEXT, kind TEXT, metadata TEXT);
            CREATE VIRTUAL TABLE spatial USING rtree(id,minx,maxx,miny,maxy,minz,maxz);
            CREATE TABLE objects(identity TEXT PRIMARY KEY,definition TEXT);
        """)
        for index in range(10005):
            add_instance(
                db,
                f"levels.rpf@1/entity/{index}",
                "shared-source",
                np.eye(4),
                np.array([[-1.0, -1, -1], [1.0, 1, 1]]),
                "entity",
                {},
            )
        for identity in ("levels.rpf@1/interior/0", "levels.rpf@10/entity/0"):
            add_instance(
                db,
                identity,
                "shared-source",
                np.eye(4),
                np.array([[-1.0, -1, -1], [1.0, 1, 1]]),
                "entity",
                {},
            )
    manifest = {
        "version": 1,
        "database": database.name,
        "database_sha256": file_digest(database),
        "decoder": str(decoder),
        "decoder_sha256": file_digest(decoder),
        "game": str(tmp_path),
        "workspace": str(tmp_path),
        "origin_rage_m": [0, 0, 0],
        "coverage_bounds": [[-100] * 3, [100] * 3],
        "archive_size": archive.stat().st_size,
        "archive_mtime_ns": archive.stat().st_mtime_ns,
        "geometry_cache_bytes": 1200,
        "disk_cache_bytes": 100000,
        "include_mask": 1,
    }
    source = tmp_path / "original.json"
    source.write_text(json.dumps(manifest))
    profile = tmp_path / "state.json"
    profile.write_text(json.dumps({"disabled_maps": {"levels.rpf@1": "destroyed"}}))
    bound = tmp_path / "intact/catalog.json"
    bind_catalog(source, profile, bound)
    catalog = WorldCatalog(bound)
    try:
        selected = catalog.instances(np.zeros(3), 12)
        assert [entry["identity"] for entry in selected] == ["levels.rpf@10/entity/0"]
        assert catalog.instance("levels.rpf@10/entity/0")["source"] == "shared-source"
        with pytest.raises(ValueError, match="Unknown world instance"):
            catalog.instance("levels.rpf@1/entity/0")
        assert catalog.stats["inactive_instances"] == 10006
        assert catalog.digest != file_digest(source)
        assert file_digest(database) == manifest["database_sha256"]
    finally:
        catalog.close()
    profile.write_text(json.dumps({"disabled_maps": {}}))
    with pytest.raises(ValueError, match="Scene-state profile changed"):
        WorldCatalog(bound)


def test_spawn_uses_world_origin_and_fresh_scene_identity(tmp_path: Path) -> None:
    scene = tmp_path / "world.usda"
    stage = Usd.Stage.CreateNew(str(scene))
    body = UsdGeom.Xform.Define(stage, "/" + BASE)
    body.AddTransformOp().Set(Gf.Matrix4d().SetTranslate(Gf.Vec3d(0, 0, 0.531599)))
    stage.GetRootLayer().Save()
    original = scene.read_bytes()
    catalog = tmp_path / "catalog.json"
    catalog.write_text("{}")
    manifest = {
        "origin_rage_m": [-458, -66, 1.3192443216876168],
        "streaming_catalog_sha256": file_digest(catalog),
    }
    scene.with_name("manifest.json").write_text(json.dumps(manifest))
    scene.with_name("surfaces.npz").write_bytes(b"fixture")
    definitions = tmp_path / "definitions.json"
    definitions.write_text(
        json.dumps(
            {
                "locations": [
                    {
                        "id": "house",
                        "label": "House",
                        "floor_rage_m": [-2349, -5530, 5.984],
                        "yaw_degrees": 180,
                        "speed_mps": 0.4,
                    }
                ]
            }
        )
    )
    output = tmp_path / "spawns"
    prepare(scene, definitions, output)
    locations = load_spawns(output / "spawns.json", catalog)
    moved = Usd.Stage.Open(str(locations["house"]["scene"]))
    transform = UsdGeom.Xformable(moved.GetPrimAtPath("/" + BASE)).GetLocalTransformation()
    np.testing.assert_allclose(transform.ExtractTranslation(), [-1891, -5464, 5.196354678312383])
    np.testing.assert_allclose(transform.TransformDir(Gf.Vec3d(1, 0, 0)), [-1, 0, 0], atol=1e-12)
    assert scene.read_bytes() == original
    with pytest.raises(ValueError, match="original base scene"):
        prepare(locations["house"]["scene"], definitions, tmp_path / "nested-spawns")
    locations["house"]["scene"].write_text("changed")
    with pytest.raises(ValueError, match="Invalid or changed spawn"):
        load_spawns(output / "spawns.json", catalog)
