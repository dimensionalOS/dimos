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

"""Build an immutable spatial/dependency catalog from authored source records.

The database contains instances and provenance, not resident triangle arrays.
Source metadata and generated geometry stay outside the repository.
"""

import argparse
from collections import Counter
import hashlib
import itertools
import json
from pathlib import Path
import sqlite3
import time
from typing import Any

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from dimos.experimental.gta_sim2.contact_layers import (
    CONTACT_LIMIT,
    CONTACT_POLICY,
    ROBOT_CONTACT_MASK,
)


def file_digest(path: Path) -> str:
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def instance_number(identity: str) -> int:
    return int.from_bytes(hashlib.sha256(identity.encode()).digest()[:8], "little") & (
        (1 << 63) - 1
    )


def placed_bounds(box: Any, matrix: NDArray[np.float64]) -> NDArray[np.float64]:
    bounds = np.asarray(box, dtype=np.float64)
    center = matrix[:3, :3] @ bounds.mean(axis=0) + matrix[:3, 3]
    radius = np.abs(matrix[:3, :3]) @ ((bounds[1] - bounds[0]) / 2)
    return np.array([center - radius, center + radius])


def placement_matrix(entity: dict[str, Any]) -> NDArray[np.float64]:
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(entity["rotation"]).as_matrix() @ np.diag(entity["scale"])
    result[:3, 3] = entity["position"]
    if not np.isfinite(result).all() or abs(np.linalg.det(result[:3, :3])) < 1e-10:
        raise ValueError("Degenerate instance transform")
    return result


def add_instance(
    db: sqlite3.Connection,
    identity: str,
    source: str,
    matrix: NDArray[np.float64],
    box: NDArray[np.float64],
    kind: str,
    extra: dict[str, Any],
) -> None:
    number = instance_number(identity)
    db.execute(
        "INSERT INTO instances VALUES (?,?,?,?,?,?,?)",
        (
            number,
            identity,
            source,
            json.dumps(matrix.tolist()),
            json.dumps(box.tolist()),
            kind,
            json.dumps(extra),
        ),
    )
    db.execute(
        "INSERT INTO spatial VALUES (?,?,?,?,?,?,?)", (number, *box[:, 0], *box[:, 1], *box[:, 2])
    )


def build(
    dependencies: Path,
    collision_index: Path,
    interior_index: Path,
    output: Path,
    origin: list[float],
    decoder: Path,
    game: Path,
    workspace: Path,
    objects: Path | None = None,
) -> dict[str, Any]:
    output.mkdir(parents=True, exist_ok=True)
    pending = output / "world.pending.sqlite"
    if pending.exists() or (output / "world.sqlite").exists():
        raise ValueError("Choose a new output catalog directory")
    started = time.monotonic()
    counts: Counter[str] = Counter()
    archetypes: dict[int, dict[str, Any]] = {}
    models: dict[tuple[int, str], str | None] = {}
    with sqlite3.connect(pending) as db:
        db.executescript("""
            PRAGMA journal_mode=DELETE;
            CREATE TABLE instances(id INTEGER PRIMARY KEY, identity TEXT UNIQUE, source TEXT,
                transform TEXT, bbox TEXT, kind TEXT, metadata TEXT);
            CREATE VIRTUAL TABLE spatial USING rtree(id,minx,maxx,miny,maxy,minz,maxz);
            CREATE TABLE maps(id TEXT PRIMARY KEY, name INTEGER, space TEXT, parents TEXT);
            CREATE INDEX map_names ON maps(name);
            CREATE TABLE entities(identity TEXT PRIMARY KEY,map_id TEXT,ordinal INTEGER,offset INTEGER,
                archetype INTEGER,archetype_key TEXT,model TEXT,lod TEXT,parent_map INTEGER,parent_ordinal INTEGER,metadata TEXT);
            CREATE INDEX entity_parent ON entities(parent_map,parent_ordinal);
            CREATE INDEX entity_ordinal ON entities(map_id,ordinal);
            CREATE TABLE assets(id TEXT PRIMARY KEY,hash INTEGER,name TEXT,key TEXT,type TEXT);
            CREATE TABLE source_bounds(id TEXT PRIMARY KEY,bbox TEXT,classification TEXT);
            CREATE TABLE issues(identity TEXT,reason TEXT,detail TEXT);
            CREATE TABLE objects(identity TEXT PRIMARY KEY,definition TEXT NOT NULL);
        """)
        with dependencies.open() as stream:
            for line in stream:
                record = json.loads(line)
                if record["kind"] == "archetype":
                    box = record["bbox"]
                    if record["name"] is None or any(v is None for v in box):
                        continue
                    key = int(record["name"])
                    previous = archetypes.get(key)
                    if previous and previous["bbox"] != box:
                        # Index the union conservatively; collider geometry still comes from the asset.
                        record["bbox"] = np.array(
                            [
                                np.minimum(previous["bbox"][0], box[0]),
                                np.maximum(previous["bbox"][1], box[1]),
                            ]
                        ).tolist()
                        counts["archetype_bbox_variants"] += 1
                    archetypes[key] = record
                elif record["kind"] == "model":
                    model_key = (record["hash"], record["key"])
                    if model_key in models and models[model_key] != record["id"]:
                        models[model_key] = None
                    else:
                        models[model_key] = record["id"]
                    db.execute(
                        "INSERT INTO assets VALUES (?,?,?,?,?)",
                        (
                            record["id"],
                            record["hash"],
                            record["name"],
                            record["key"],
                            record["type"],
                        ),
                    )
        counts["archetypes"] = len(archetypes)
        counts["models"] = len(models)
        counts["ambiguous_model_keys"] = sum(value is None for value in models.values())
        with dependencies.open() as stream:
            for line in stream:
                record = json.loads(line)
                if record["kind"] != "map":
                    continue
                map_id, space = record["id"], record["space"]
                counts[space + "_maps"] += 1
                db.execute(
                    "INSERT INTO maps VALUES (?,?,?,?)",
                    (map_id, record["name"], space, json.dumps(record["parent_maps"])),
                )
                for entity in record["entities"]:
                    counts[space + "_entities"] += 1
                    ordinal = entity["ordinal"]
                    identity = (
                        f"{map_id}/entity/{ordinal}"
                        if ordinal is not None
                        else f"{map_id}/record/{entity['offset']}"
                    )
                    model = models.get((entity["archetype"], entity["archetype_key"]))
                    lod = entity["lod"]
                    db.execute(
                        "INSERT INTO entities VALUES (?,?,?,?,?,?,?,?,?,?,?)",
                        (
                            identity,
                            map_id,
                            ordinal,
                            entity["offset"],
                            entity["archetype"],
                            entity["archetype_key"],
                            model,
                            lod["level"],
                            entity["parent_map"],
                            lod["parent_index"],
                            json.dumps(entity),
                        ),
                    )
                    if entity["parent_map"] is not None:
                        counts["authored_parent_links"] += 1
                    if space != "world" or lod["level"] not in ("HD", "ORPHAN_HD"):
                        continue
                    archetype = archetypes.get(entity["archetype"])
                    if not model or not archetype:
                        counts["unresolved_world_instances"] += 1
                        db.execute(
                            "INSERT INTO issues VALUES (?,?,?)",
                            (
                                identity,
                                "missing_model_or_archetype",
                                json.dumps({"model": model, "archetype": entity["archetype"]}),
                            ),
                        )
                        continue
                    try:
                        transform = placement_matrix(entity)
                    except ValueError:
                        counts["degenerate_instances"] += 1
                        continue
                    add_instance(
                        db,
                        identity,
                        model,
                        transform,
                        placed_bounds(archetype["bbox"], transform),
                        "entity",
                        {
                            "name": entity["name"],
                            "physics_dictionary": archetype["physics_dictionary"],
                            "map": map_id,
                            "ordinal": ordinal,
                        },
                    )
                    counts["indexed_entity_instances"] += 1
                if (
                    sum(counts[k] for k in ("world_maps", "interior_maps", "group_maps")) % 5000
                    == 0
                ):
                    print(json.dumps(dict(counts)), flush=True)
        # Resolve authored links only. Preserve missing/ambiguous targets explicitly.
        db.executescript("""
            CREATE TABLE parent_links AS
                SELECT child.identity AS child,parent.identity AS parent
                FROM entities child JOIN maps map ON child.parent_map=map.name AND map.space='world'
                JOIN entities parent ON parent.map_id=map.id AND parent.ordinal=child.parent_ordinal
                WHERE child.parent_map IS NOT NULL;
            CREATE INDEX parent_link_child ON parent_links(child);
        """)
        counts["resolved_parent_links"] = db.execute(
            "SELECT COUNT(DISTINCT child) FROM parent_links"
        ).fetchone()[0]
        counts["ambiguous_parent_links"] = db.execute(
            "SELECT COUNT(*) FROM (SELECT child FROM parent_links GROUP BY child HAVING COUNT(*) != 1)"
        ).fetchone()[0]
        db.execute("""INSERT INTO issues
            SELECT child.identity,'unresolved_parent',json_object('map',child.parent_map,'ordinal',child.parent_ordinal)
            FROM entities child LEFT JOIN parent_links link ON link.child=child.identity
            WHERE child.parent_map IS NOT NULL AND link.child IS NULL""")
        db.execute("""INSERT INTO issues
            SELECT child,'ambiguous_parent',json_group_array(parent) FROM parent_links
            GROUP BY child HAVING COUNT(*)>1""")
        # Interior-shell bounds have local coordinates. Only exact links are instantiated.
        interiors = (
            [json.loads(line) for line in interior_index.open()] if interior_index.exists() else []
        )
        local_sources = {
            row["collision_linked"]["id"] for row in interiors if row.get("collision_linked")
        }
        local_sources.update(
            itertools.chain.from_iterable(row.get("collision", []) for row in interiors)
        )
        boxes: dict[str, Any] = {}
        with collision_index.open() as stream:
            for line in stream:
                row = json.loads(line)
                box = np.asarray(row["bbox"], dtype=np.float64)
                source = row["id"]
                if not np.isfinite(box).all() or box.shape != (2, 3):
                    raise ValueError(f"Invalid source bound: {source}")
                classification = "world"
                if source in local_sources:
                    classification = "interior_local"
                    boxes[source] = box
                elif np.max(np.abs(box[:, :2])) <= 50:
                    classification = "unclassified_origin_local"
                db.execute(
                    "INSERT INTO source_bounds VALUES (?,?,?)",
                    (source, json.dumps(box.tolist()), classification),
                )
                counts[classification + "_bounds"] += 1
                if classification == "world":
                    add_instance(db, source, source, np.eye(4), box, "terrain", {})
        for row in interiors:
            link = row.get("collision_linked", {})
            if link.get("method") != "exact":
                counts["unresolved_interior_instances"] += 1
                continue
            source = link["id"]
            transform = placement_matrix(
                {"position": row["position"], "rotation": row["rotation_world"], "scale": [1, 1, 1]}
            )
            identity = f"{row['map']}/interior/{row['instance']}"
            add_instance(
                db,
                identity,
                source,
                transform,
                placed_bounds(boxes[source], transform),
                "interior",
                {"method": "exact", "archetype": row["archetype"]},
            )
            counts["indexed_interior_instances"] += 1
        if objects is not None:
            definitions = json.loads(objects.read_text())
            for identity, definition in definitions.items():
                if not db.execute(
                    "SELECT 1 FROM instances WHERE identity=?", (identity,)
                ).fetchone():
                    raise ValueError(
                        f"Object definition references an unknown placement: {identity}"
                    )
                if definition.get("kind") not in ("free", "hinge") or not definition.get(
                    "provenance"
                ):
                    raise ValueError("Object definitions need an explicit kind and provenance")
                db.execute(
                    "INSERT INTO objects VALUES (?,?)",
                    (identity, json.dumps(definition, allow_nan=False)),
                )
            counts["explicit_objects"] = len(definitions)
        db.commit()
    pending.replace(output / "world.sqlite")
    archive = game / "levels.rpf"
    stat = archive.stat()
    manifest = {
        "version": 1,
        "database": "world.sqlite",
        "database_sha256": file_digest(output / "world.sqlite"),
        "origin_rage_m": origin,
        "coverage_bounds": [
            [-12288 - origin[0], -9856 - origin[1], -100 - origin[2]],
            [5120 - origin[0], 13184 - origin[1], 500 - origin[2]],
        ],
        "decoder": str(decoder.resolve()),
        "decoder_sha256": file_digest(decoder),
        "game": str(game.resolve()),
        "workspace": str(workspace.resolve()),
        "archive_size": stat.st_size,
        "archive_mtime_ns": stat.st_mtime_ns,
        "include_mask": ROBOT_CONTACT_MASK,
        "contact_policy": CONTACT_POLICY,
        "counts": dict(counts),
        "dependencies_sha256": file_digest(dependencies),
        "source_bounds_sha256": file_digest(collision_index),
        "objects_sha256": file_digest(objects) if objects else None,
        "build_seconds": time.monotonic() - started,
        "geometry_cache_bytes": 256 << 20,
        "disk_cache_bytes": 2 << 30,
        "limits": [
            "Unresolved dependencies are recorded; the data extent is not a guarantee of physical coverage.",
            "Interior links based only on heuristics are withheld.",
            CONTACT_LIMIT,
        ],
    }
    (output / "catalog.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    for name in (
        "dependencies",
        "collision-index",
        "interior-index",
        "output",
        "decoder",
        "game",
        "workspace",
    ):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--origin", nargs=3, type=float, required=True)
    parser.add_argument(
        "--objects",
        type=Path,
        help="Reviewed per-instance free/hinge definitions, keyed by stable placement ID",
    )
    args = parser.parse_args()
    print(
        json.dumps(
            build(
                args.dependencies,
                args.collision_index,
                args.interior_index,
                args.output,
                args.origin,
                args.decoder,
                args.game,
                args.workspace,
                args.objects,
            ),
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
