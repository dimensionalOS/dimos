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

"""Topology failures must not become plausible but false walking routes."""

import json
from pathlib import Path

import pytest

from dimos.experimental.gta_sim2.navigation import NavTile


def graph_file(tmp_path: Path) -> Path:
    # Two adjacent road quads plus a two-vertex source record between their IDs.
    vertices = [[0, 0, 0], [2, 0, 0], [2, 2, 0], [0, 2, 0], [4, 0, 0], [4, 2, 0]]
    ids = [[0, 1, 2, 3], [0, 1], [1, 4, 5, 2]]
    polygons = []
    for i, face in enumerate(ids):
        polygons.append(
            {
                "id": i,
                "vertices": face,
                "raw": [0] * 6 + [0x2000] + [0] * 4,
                "edges": [{"area": 0xFFFFFFFF, "polygon": 0xFFFF} for _ in face],
            }
        )
    polygons[0]["edges"][1] = {"area": 7, "polygon": 2}
    polygons[2]["edges"][3] = {"area": 7, "polygon": 0}
    path = tmp_path / "graph.json"
    path.write_text(
        json.dumps(
            {"version": 1, "area_id": 7, "vertices": vertices, "polygons": polygons, "portals": []}
        )
    )
    return path


def test_route_retains_ids_across_non_surface_records(tmp_path: Path) -> None:
    tile = NavTile(graph_file(tmp_path))
    route = tile.route((1, 1, 0), (3, 1, 0))
    assert route["polygon_ids"] == [0, 2]
    assert route["waypoints_rage_m"] == [[1, 1, 0], [2, 1, 0], [3, 1, 0]]
    assert tile.audit["non_surface_records"] == 1
    assert tile.audit["geometry_matches"] == tile.audit["reciprocal_links"] == 2


def test_unverified_reciprocity_or_clearance_cannot_route(tmp_path: Path) -> None:
    path = graph_file(tmp_path)
    tile = NavTile(path)
    with pytest.raises(ValueError, match="No verified"):
        tile.route((1, 1, 0), (3, 1, 0), minimum_width=2.1)
    graph = json.loads(path.read_text())
    graph["polygons"][2]["edges"][3]["polygon"] = 2
    path.write_text(json.dumps(graph))
    with pytest.raises(ValueError, match="No verified"):
        NavTile(path).route((1, 1, 0), (3, 1, 0))


def test_height_cannot_snap_to_another_storey(tmp_path: Path) -> None:
    tile = NavTile(graph_file(tmp_path))
    with pytest.raises(ValueError, match="3D position"):
        tile.locate((1, 1, 10))


def test_reordered_ids_are_rejected(tmp_path: Path) -> None:
    path = graph_file(tmp_path)
    graph = json.loads(path.read_text())
    graph["polygons"].pop(1)
    path.write_text(json.dumps(graph))
    with pytest.raises(ValueError, match="reordered"):
        NavTile(path)
