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

"""Extraction must remove every part from the static scene without moving it."""

import json

import numpy as np
import trimesh

from dimos.sim2.scenes.sf555.pickables import BOTTLES, prepare


def test_complete_assemblies_are_removed_and_rebased_without_changing_world_geometry(tmp_path):
    source, output = tmp_path / "source", tmp_path / "output"
    source.mkdir()
    parts = []
    mesh = source / "part.stl"
    trimesh.creation.box((0.05, 0.05, 0.1)).export(mesh)
    for index, prefix in enumerate(BOTTLES.values()):
        for part, z in (("body", 0.8), ("cap", 0.9)):
            parts.append(
                {
                    "name": prefix + part,
                    "origin": [index + 10, 8, z],
                    "visual": str(mesh),
                    "collisions": [str(mesh)],
                }
            )
    table = {
        "name": "table",
        "origin": [10, 8, 0.7],
        "visual": str(mesh),
        "collisions": [str(mesh)],
    }
    (source / "collision-report.json").write_text(
        json.dumps(
            {
                "source_sha256": "fixture",
                "objects": [*parts, table],
            }
        )
    )
    prepare(source, output)
    office = json.loads((output / "collision-report.json").read_text())
    assert office["objects"] == [table]
    rows = json.loads((output / "pickables.json").read_text())["bottles"]
    assert {r["id"] for r in rows} == set(BOTTLES)
    for bottle in rows:
        assert len(bottle["parts"]) == 2
        hull = trimesh.load_mesh(bottle["collision"])
        assert hull.is_watertight and hull.volume > 0
        np.testing.assert_allclose(hull.bounds.mean(axis=0), 0, atol=1e-7)
        for part in bottle["parts"]:
            np.testing.assert_allclose(
                np.asarray(part["local_origin"]) + bottle["position"],
                part["origin"],
                atol=1e-8,
            )
