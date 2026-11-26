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

"""Conservative routing over source-indexed GTA navigation polygons.

Local neighbours are accepted only when their stored IDs agree with shared
geometry and reciprocal records. Portals and opaque flags are retained in the
export but are not assumed to be walkable robot transitions.
"""

import argparse
from collections import Counter
import heapq
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray
from shapely.geometry import Point, Polygon


class NavTile:
    def __init__(self, source: Path) -> None:
        self.source = source
        self.raw = json.loads(source.read_text())
        if self.raw["version"] != 1:
            raise ValueError("Unsupported nav graph version")
        self.area = int(self.raw["area_id"])
        self.vertices = np.asarray(self.raw["vertices"], dtype=np.float64)
        self.polygons = self.raw["polygons"]
        self.centers = np.zeros((len(self.polygons), 3), dtype=np.float64)
        self.shapes: dict[int, Polygon] = {}
        self.adjacency: dict[int, list[tuple[int, float, NDArray[np.float64]]]] = {}
        self.audit: Counter[str] = Counter()
        for i, poly in enumerate(self.polygons):
            if poly["id"] != i:
                raise ValueError("Original polygon IDs were reordered")
            ids = poly["vertices"]
            if len(ids) != len(poly["edges"]) or any(v >= len(self.vertices) or v < 0 for v in ids):
                raise ValueError("Invalid source polygon indexing")
            self.audit["source_polygon_records"] += 1
            if len(ids) < 3:
                self.audit["non_surface_records"] += 1
                continue
            points = self.vertices[ids]
            self.centers[i] = points.mean(axis=0)
            shape = Polygon(points[:, :2])
            if shape.is_valid and shape.area > 1e-6:
                self.shapes[i] = shape
        for i, poly in enumerate(self.polygons):
            ids = poly["vertices"]
            neighbours = []
            for k, edge in enumerate(poly["edges"]):
                self.audit["edge_records"] += 1
                if edge["area"] != self.area:
                    self.audit["boundary_or_external_edges"] += 1
                    continue
                self.audit["local_edges"] += 1
                other = edge["polygon"]
                if not 0 <= other < len(self.polygons):
                    self.audit["invalid_target"] += 1
                    continue
                pair = {ids[k], ids[(k + 1) % len(ids)]}
                target = self.polygons[other]
                matched = False
                reciprocal = False
                for j, vertex in enumerate(target["vertices"]):
                    if pair == {vertex, target["vertices"][(j + 1) % len(target["vertices"])]}:
                        matched = True
                        back = target["edges"][j]
                        reciprocal = back["area"] == self.area and back["polygon"] == i
                        if reciprocal:
                            break
                self.audit["geometry_matches" if matched else "geometry_mismatches"] += 1
                self.audit["reciprocal_links" if reciprocal else "nonreciprocal_links"] += 1
                if matched and reciprocal and i in self.shapes and other in self.shapes:
                    endpoints = self.vertices[[ids[k], ids[(k + 1) % len(ids)]]]
                    width = float(np.linalg.norm(endpoints[1, :2] - endpoints[0, :2]))
                    neighbours.append((other, width, endpoints.mean(axis=0)))
            self.adjacency[i] = neighbours
        self.audit["portal_records_not_used_for_routing"] = len(self.raw["portals"])

    def traversable(self, polygon: int, max_slope_degrees: float = 12) -> bool:
        if polygon not in self.shapes or not self.polygons[polygon]["raw"][6] & 0x2000:
            return False
        points = self.vertices[self.polygons[polygon]["vertices"]]
        _, _, vh = np.linalg.svd(points - points.mean(axis=0))
        normal = vh[-1]
        return bool(abs(normal[2]) >= math.cos(math.radians(max_slope_degrees)))

    def locate(self, xyz: tuple[float, float, float]) -> int:
        candidates = []
        for i, shape in self.shapes.items():
            if self.traversable(i) and shape.covers(Point(xyz[:2])):
                candidates.append((abs(self.centers[i, 2] - xyz[2]), i))
        if not candidates or min(candidates)[0] > 0.5:
            raise ValueError(f"No road polygon at requested 3D position {xyz}")
        return min(candidates)[1]

    def route(
        self,
        start: tuple[float, float, float],
        goal: tuple[float, float, float],
        minimum_width: float = 0.7,
    ) -> dict[str, Any]:
        """A* through verified shared edges; output a corridor, not a footstep plan."""
        source, target = self.locate(start), self.locate(goal)
        todo = [(0.0, source)]
        costs = {source: 0.0}
        previous: dict[int, tuple[int, NDArray[np.float64]]] = {}
        closed: set[int] = set()
        while todo:
            _, current = heapq.heappop(todo)
            if current in closed:
                continue
            closed.add(current)
            if current == target:
                break
            for neighbour, width, portal in self.adjacency[current]:
                if width < minimum_width or not self.traversable(neighbour):
                    continue
                cost = costs[current] + float(
                    np.linalg.norm(self.centers[neighbour] - self.centers[current])
                )
                if cost < costs.get(neighbour, math.inf):
                    costs[neighbour] = cost
                    previous[neighbour] = current, portal
                    estimate = float(np.linalg.norm(self.centers[neighbour] - self.centers[target]))
                    heapq.heappush(todo, (cost + estimate, neighbour))
        if target not in costs:
            raise ValueError("No verified road corridor with required shared-edge width")
        ids = [target]
        portals = []
        while ids[-1] != source:
            parent, portal = previous[ids[-1]]
            ids.append(parent)
            portals.append(portal.tolist())
        points = [list(start), *reversed(portals), list(goal)]
        return {
            "area_id": self.area,
            "polygon_ids": list(reversed(ids)),
            "waypoints_rage_m": points,
            "required_shared_edge_width_m": minimum_width,
            "length_m": float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum()),
            "source": str(self.source),
            "audit": dict(self.audit),
            "limitations": [
                "Road flag 0x2000 is a community interpretation",
                "Portals, boundary transitions and non-surface records are excluded",
                "Shared-edge width is not full-body clearance; collision checks are still required",
            ],
        }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--graph", type=Path, required=True)
    parser.add_argument("--start", type=float, nargs=3, required=True)
    parser.add_argument("--goal", type=float, nargs=3, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    tile = NavTile(args.graph)
    result = tile.route(tuple(args.start), tuple(args.goal))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
