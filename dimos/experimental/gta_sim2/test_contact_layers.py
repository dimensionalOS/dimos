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

"""Game query helpers must never replace rigid-body stair contact surfaces."""

from collections.abc import Iterator
from contextlib import ExitStack
import json
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import pytest

from dimos.experimental.gta_sim2.contact_layers import bind_catalog
from dimos.experimental.gta_sim2.world_compile import compile_world_window
from dimos.experimental.gta_sim2.world_geometry import SourceGeometry, WorldCatalog
from dimos.experimental.gta_sim2.world_index import build, file_digest


@pytest.fixture
def contact_catalogs(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> Iterator[tuple[WorldCatalog, WorldCatalog]]:
    archive, decoder = tmp_path / "levels.rpf", tmp_path / "decoder"
    archive.write_bytes(b"synthetic archive")
    decoder.write_bytes(b"synthetic decoder")
    dependencies, interiors, bounds = (
        tmp_path / n for n in ("deps.jsonl", "interiors.jsonl", "bounds.jsonl")
    )
    dependencies.write_text("")
    interiors.write_text("")
    bounds.write_text(json.dumps({"id": "stairs", "bbox": [[99, -1, -0.1], [103, 1, 2]]}) + "\n")
    output = tmp_path / "built"
    build(dependencies, bounds, interiors, output, [100, 0, 0], decoder, tmp_path, tmp_path)
    source = output / "catalog.json"
    legacy = json.loads(source.read_text())
    legacy["include_mask"] = 0x02002000
    legacy.pop("contact_policy")
    profile = tmp_path / "scene-state.json"
    profile.write_text(json.dumps({"disabled_maps": {"levels.rpf@123": "destroyed variant"}}))
    legacy["scene_state"] = {"path": str(profile), "sha256": file_digest(profile)}
    legacy["database"] = str(output / "world.sqlite")
    legacy_path = tmp_path / "legacy.json"
    legacy_path.write_text(json.dumps(legacy))
    # These are the authored INCLUDE flags, not TYPE flags. The vehicle query
    # category occurs in both helper records, even on the character stair ramp.
    floor_and_treads = [
        [[99, -1, -0.1], [103, 1, 0]],
        [[100, -1, 0], [100.3, 1, 0.18]],
        [[100.3, -1, 0], [100.6, 1, 0.36]],
        [[100.6, -1, 0], [100.9, 1, 0.54]],
    ]
    shapes = [(0x2007D33E40, box) for box in floor_and_treads]
    shapes += [(0x02000040, [[99.5, -1, 0], [99.8, 1, 1.5]])]
    ramp = np.array(
        [
            [[100.0, -1, 0.18], [100.9, -1, 0.72], [100.9, 1, 0.72]],
            [[100.0, -1, 0.18], [100.9, 1, 0.72], [100.0, 1, 0.18]],
        ]
    )

    def decode_fixture(self: WorldCatalog, source: str) -> SourceGeometry:
        mask = self.manifest["include_mask"]
        primitives: list[dict[str, Any]] = [
            {"type": "box", "bbox": box, "matrix": np.eye(4).ravel().tolist()}
            for include, box in shapes
            if include & mask
        ]
        triangles = ramp if mask & 0x03E00A00 else np.empty((0, 3, 3))
        return SourceGeometry(triangles, primitives, {}, triangles.nbytes + 1000)

    monkeypatch.setattr(WorldCatalog, "_source", decode_fixture)
    with ExitStack() as resources:
        old, new = WorldCatalog(legacy_path), WorldCatalog(source)
        resources.callback(old.close)
        resources.callback(new.close)
        yield old, new


def test_physical_contacts_preserve_treads_and_remove_query_blocker(
    contact_catalogs: tuple[WorldCatalog, WorldCatalog],
) -> None:
    heights = []
    for catalog in contact_catalogs:
        model, data, _, _ = compile_world_window(mujoco.MjSpec(), catalog, np.zeros(3), 4)
        mujoco.mj_forward(model, data)
        samples = []
        for x in [-0.35, 0.15, 0.45, 0.75]:
            geom = np.array([-1], dtype=np.int32)
            distance = mujoco.mj_ray(
                model, data, np.array([x, 0, 2.0]), np.array([0.0, 0.0, -1.0]), None, 1, -1, geom
            )
            assert geom[0] >= 0
            samples.append(2 - distance)
        heights.append(samples)
    np.testing.assert_allclose(heights[0], [1.5, 0.27, 0.45, 0.63], atol=1e-6)
    np.testing.assert_allclose(heights[1], [0, 0.18, 0.36, 0.54], atol=1e-6)


def test_rebinding_invalidates_cache_and_preserves_source(
    contact_catalogs: tuple[WorldCatalog, WorldCatalog], tmp_path: Path
) -> None:
    old, _ = contact_catalogs
    original = old.path.read_bytes()
    target = tmp_path / "rebound" / "catalog.json"
    result = bind_catalog(old.path, target)
    assert old.path.read_bytes() == original
    assert result["database_sha256"] == old.manifest["database_sha256"]
    assert result["decoder_sha256"] == old.manifest["decoder_sha256"]
    assert result["scene_state"] == old.manifest["scene_state"]
    assert result["parent_catalog_sha256"] == file_digest(old.path)
    assert file_digest(target) != old.digest
    assert int(result["include_mask"]) & 0x02000040 == 0
    assert int(result["include_mask"]) & 0x03E00A00 == 0
    assert int(result["include_mask"]) & 0x2007D33E40 != 0
    with pytest.raises(ValueError, match="new catalog"):
        bind_catalog(old.path, target)
