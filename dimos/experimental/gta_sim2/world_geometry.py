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

"""Bounded on-demand source decoding and spatial queries for the world catalog."""

from collections import OrderedDict
from dataclasses import dataclass
import fcntl
import hashlib
import json
from pathlib import Path
import selectors
import sqlite3
import subprocess
import time
from typing import Any

import numpy as np
from numpy.typing import NDArray

from dimos.experimental.gta_sim2.prepare import glb_triangles
from dimos.experimental.gta_sim2.scene_state import disabled_maps
from dimos.experimental.gta_sim2.world_index import file_digest, placed_bounds


@dataclass
class SourceGeometry:
    triangles: NDArray[np.float64]
    primitives: list[dict[str, Any]]
    metadata: dict[str, Any]
    bytes: int


@dataclass
class InstanceGeometry:
    identity: str
    number: int
    transform: NDArray[np.float64]
    source: SourceGeometry
    metadata: dict[str, Any]


class WorldCatalog:
    def __init__(self, path: Path, *, validate: bool = True) -> None:
        self.path = path.resolve()
        self.manifest = json.loads(self.path.read_text())
        if self.manifest["version"] != 1:
            raise ValueError("Unsupported world catalog")
        self.digest = file_digest(self.path)
        database = self.path.parent / self.manifest["database"]
        if validate and file_digest(database) != self.manifest["database_sha256"]:
            raise ValueError("World dependency database changed")
        archive = Path(self.manifest["game"]) / "levels.rpf"
        stat = archive.stat()
        if (stat.st_size, stat.st_mtime_ns) != (
            self.manifest["archive_size"],
            self.manifest["archive_mtime_ns"],
        ):
            raise ValueError("Source archive changed; rebuild the catalog")
        if (
            validate
            and file_digest(Path(self.manifest["decoder"])) != self.manifest["decoder_sha256"]
        ):
            raise ValueError("Source decoder changed; rebuild the catalog identity")
        self.origin = np.asarray(self.manifest["origin_rage_m"], dtype=np.float64)
        self.coverage = np.asarray(self.manifest["coverage_bounds"], dtype=np.float64)
        profile = self.manifest.get("scene_state")
        inactive_maps = disabled_maps(Path(profile["path"]), profile["sha256"]) if profile else {}
        self._db = sqlite3.connect(database.as_uri() + "?mode=ro", uri=True)
        self._db.row_factory = sqlite3.Row
        # Filter before spatial admission and decoding, including interior rows
        # whose identity retains the owning map. The source database stays immutable.
        self._db.execute("CREATE TEMP TABLE inactive_instances(id INTEGER PRIMARY KEY)")
        if inactive_maps:
            for resource in inactive_maps:
                self._db.execute(
                    "INSERT OR IGNORE INTO inactive_instances SELECT id FROM instances WHERE identity=? OR identity GLOB ?",
                    (resource, resource + "/*"),
                )
        inactive_count = self._db.execute("SELECT count(*) FROM inactive_instances").fetchone()[0]
        self._cache: OrderedDict[str, SourceGeometry] = OrderedDict()
        self._bytes = 0
        self._limit = int(self.manifest["geometry_cache_bytes"])
        self._disk_limit = int(self.manifest["disk_cache_bytes"])
        self._directory = self.path.parent / "cooked" / self.digest[:16]
        self._directory.mkdir(parents=True, exist_ok=True)
        self._disk: OrderedDict[str, int] = OrderedDict()
        self._decoder: subprocess.Popen[str] | None = None
        self.stats: dict[str, Any] = {
            "memory_hits": 0,
            "disk_hits": 0,
            "decoded": 0,
            "evictions": 0,
            "unresolved": 0,
            "unsupported": {},
            "query_ms": 0.0,
            "inactive_instances": inactive_count,
        }
        with (self._directory / ".lock").open("a") as lock:
            fcntl.flock(lock.fileno(), fcntl.LOCK_EX)
            self._refresh_disk()
            self._trim_disk()

    def _refresh_disk(self) -> None:
        self._disk.clear()
        for metadata in sorted(self._directory.glob("*.json"), key=lambda p: p.stat().st_mtime_ns):
            geometry = metadata.with_suffix(".glb")
            self._disk[metadata.stem] = metadata.stat().st_size + (
                geometry.stat().st_size if geometry.exists() else 0
            )

    def instances(self, center: NDArray[np.float64], half_extent: float) -> list[dict[str, Any]]:
        world = np.asarray(center) + self.origin
        lo, hi = world - half_extent - 0.05, world + half_extent + 0.05
        started = time.monotonic()
        rows = self._db.execute(
            """SELECT i.*,o.definition AS object FROM spatial s JOIN instances i ON i.id=s.id
            LEFT JOIN objects o ON o.identity=i.identity
            WHERE s.minx<=? AND s.maxx>=? AND s.miny<=? AND s.maxy>=? AND s.minz<=? AND s.maxz>=?
            AND i.id NOT IN (SELECT id FROM inactive_instances)
            ORDER BY i.id LIMIT 10001""",
            tuple(float(v) for v in (hi[0], lo[0], hi[1], lo[1], hi[2], lo[2])),
        ).fetchall()
        if len(rows) > 10000:
            raise MemoryError("Spatial query exceeds the 10,000-instance admission budget")
        self.stats["query_ms"] = (time.monotonic() - started) * 1000
        return [dict(row) for row in rows]

    def instance(self, identity: str) -> dict[str, Any]:
        row = self._db.execute(
            "SELECT i.*,o.definition AS object FROM instances i LEFT JOIN objects o ON o.identity=i.identity WHERE i.identity=? AND i.id NOT IN (SELECT id FROM inactive_instances)",
            (identity,),
        ).fetchone()
        if row is None:
            raise ValueError(f"Unknown world instance: {identity}")
        return dict(row)

    def geometry(self, record: dict[str, Any]) -> InstanceGeometry:
        transform = np.asarray(json.loads(record["transform"]), dtype=np.float64)
        transform[:3, 3] -= self.origin
        return InstanceGeometry(
            record["identity"],
            record["id"],
            transform,
            self._source(record["source"]),
            json.loads(record["metadata"]),
        )

    def _trim_disk(self) -> None:
        total = sum(self._disk.values())
        while total > self._disk_limit and self._disk:
            key, size = self._disk.popitem(last=False)
            for suffix in (".json", ".glb"):
                (self._directory / (key + suffix)).unlink(missing_ok=True)
            total -= size

    def _decode(self, source: str, path: Path) -> dict[str, Any]:
        if self._decoder is None:
            self._decoder = subprocess.Popen(
                [self.manifest["decoder"], self.manifest["game"], self.manifest["workspace"]],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                bufsize=1,
            )
        process = self._decoder
        assert process.stdin is not None and process.stdout is not None
        process.stdin.write(
            json.dumps(
                {"id": source, "output": str(path), "include_mask": self.manifest["include_mask"]}
            )
            + "\n"
        )
        process.stdin.flush()
        with selectors.DefaultSelector() as selector:
            selector.register(process.stdout, selectors.EVENT_READ)
            if not selector.select(timeout=30):
                self._stop_decoder()
                raise TimeoutError(f"Source decoder timed out: {source}")
        line = process.stdout.readline()
        if not line:
            raise RuntimeError("Source decoder exited")
        result: dict[str, Any] = json.loads(line)
        if "error" in result:
            # A drawable can intentionally have no embedded collision; preserve the unresolved dependency.
            if result["error"] in (
                "drawable bound missing",
                "no identified embedded collider",
                "fragment bound missing",
            ):
                return {"id": source, "unresolved": result["error"], "primitives": [], "stats": {}}
            raise ValueError(f"Source decoding failed for {source}: {result['error']}")
        result["glb_sha256"] = file_digest(path)
        if result.get("id") != source:
            raise ValueError("Source decoder response identity mismatch")
        return result

    def _source(self, source: str) -> SourceGeometry:
        if source in self._cache:
            value = self._cache.pop(source)
            self._cache[source] = value
            self.stats["memory_hits"] += 1
            return value
        # Multiple independent simulations may share cooked sources. Serialize disk
        # admission/eviction, never the physics owner or memory-cache hits.
        with (self._directory / ".lock").open("a") as lock:
            fcntl.flock(lock.fileno(), fcntl.LOCK_EX)
            self._refresh_disk()
            return self._source_disk(source)

    def _source_disk(self, source: str) -> SourceGeometry:
        key = hashlib.sha256(source.encode()).hexdigest()[:24]
        geometry_path = self._directory / (key + ".glb")
        metadata_path = self._directory / (key + ".json")
        if metadata_path.exists():
            metadata = json.loads(metadata_path.read_text())
            if metadata.get("id") != source:
                raise ValueError("Cooked source identity mismatch")
            if (
                not metadata.get("unresolved")
                and file_digest(geometry_path) != metadata["glb_sha256"]
            ):
                raise ValueError("Cooked collider changed")
            self.stats["disk_hits"] += 1
            metadata_path.touch()
        else:
            try:
                metadata = self._decode(source, geometry_path)
            except (OSError, RuntimeError, ValueError, MemoryError):
                geometry_path.unlink(missing_ok=True)
                raise
            pending = metadata_path.with_suffix(".tmp")
            pending.write_text(json.dumps(metadata))
            pending.replace(metadata_path)
            self.stats["decoded"] += 1
        self._disk.pop(key, None)
        self._disk[key] = metadata_path.stat().st_size + (
            geometry_path.stat().st_size if geometry_path.exists() else 0
        )
        groups = [] if metadata.get("unresolved") else glb_triangles(geometry_path)
        triangles = (
            np.concatenate([group[1] for group in groups]) if groups else np.empty((0, 3, 3))
        )
        size = triangles.nbytes + metadata_path.stat().st_size * 4
        value = SourceGeometry(triangles, metadata["primitives"], metadata, size)
        if metadata.get("unresolved"):
            self.stats["unresolved"] += 1
        for kind, count in metadata.get("stats", {}).get("other_bound_types", {}).items():
            self.stats["unsupported"][kind] = self.stats["unsupported"].get(kind, 0) + count
        if size > self._limit:
            raise MemoryError(f"One collider source exceeds the decoded-cache budget: {source}")
        while self._bytes + size > self._limit and self._cache:
            _, removed = self._cache.popitem(last=False)
            self._bytes -= removed.bytes
            self.stats["evictions"] += 1
        self._cache[source] = value
        self._bytes += size
        self.stats["decoded_cache_bytes"] = self._bytes
        self.stats["decoded_cache_entries"] = len(self._cache)
        self._trim_disk()
        return value

    def _stop_decoder(self) -> None:
        if self._decoder is not None:
            self._decoder.terminate()
            try:
                self._decoder.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self._decoder.kill()
                self._decoder.wait()
            if self._decoder.stdin:
                self._decoder.stdin.close()
            if self._decoder.stdout:
                self._decoder.stdout.close()
            self._decoder = None

    def close(self) -> None:
        self._stop_decoder()
        self._db.close()
        self._cache.clear()


def transformed_triangles(instance: InstanceGeometry) -> NDArray[np.float64]:
    return instance.source.triangles @ instance.transform[:3, :3].T + instance.transform[:3, 3]


def primitive_vertices(primitive: dict[str, Any]) -> NDArray[np.float64] | None:
    if primitive["type"] == "box_vertices":
        corners = np.asarray(primitive["corners"], dtype=np.float64)
        return np.asarray(
            np.concatenate((corners, 2 * corners.mean(axis=0) - corners)), dtype=np.float64
        )
    if primitive["type"] == "box":
        box = np.asarray(primitive["bbox"])
        corners = np.array(
            [[box[i, 0], box[j, 1], box[k, 2]] for i in (0, 1) for j in (0, 1) for k in (0, 1)]
        )
        matrix = np.asarray(primitive["matrix"]).reshape(4, 4).T
        return np.asarray(corners @ matrix[:3, :3].T + matrix[:3, 3], dtype=np.float64)
    return None


def primitive_bounds(
    primitive: dict[str, Any], transform: NDArray[np.float64]
) -> NDArray[np.float64]:
    vertices = primitive_vertices(primitive)
    if vertices is not None:
        world = vertices @ transform[:3, :3].T + transform[:3, 3]
        return np.array([world.min(axis=0), world.max(axis=0)])
    endpoints = (
        np.array([primitive["centre"]])
        if primitive["type"] == "sphere"
        else np.array([primitive["a"], primitive["b"]])
    )
    box = np.array(
        [endpoints.min(axis=0) - primitive["radius"], endpoints.max(axis=0) + primitive["radius"]]
    )
    return placed_bounds(box, transform)
