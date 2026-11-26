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

"""Derive an experimental catalog of structurally verified single-hinge GTA doors.

Only cached assets matching the requested glob are examined. The source catalog
is immutable; a separate output directory receives a copy and admission report.
Admission does not establish clearance against surrounding world colliders.
"""

import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path
import shutil
import sqlite3
from typing import Any

import numpy as np
from numpy.typing import NDArray

from dimos.experimental.gta_sim2.articulation import rigid_hinge, verify_collision
from dimos.experimental.gta_sim2.world_geometry import WorldCatalog
from dimos.experimental.gta_sim2.world_index import file_digest


def renderer_placements(
    work: Path, sources: set[str]
) -> dict[tuple[str, str], NDArray[np.float32]]:
    """Mirror the renderer's exact map/model/HD-position admission key."""
    strings = json.loads((work / "map/strings.json").read_text())
    dtype = np.dtype(
        {
            "names": ["position", "model", "file", "lod"],
            "formats": [("<f4", 3), "<u4", "<u4", "u1"],
            "offsets": [0, 44, 48, 56],
            "itemsize": 72,
        }
    )
    path = work / "map/entities.bin"
    if path.stat().st_size % dtype.itemsize:
        raise ValueError("Invalid legacy placement table")
    entities = np.memmap(path, dtype=dtype, mode="r")
    indices = [i for i, name in enumerate(strings["strings"]) if name in sources]
    selected = entities[np.isin(entities["model"], indices) & (entities["lod"] == 0)]
    grouped: dict[tuple[str, str], list[Any]] = defaultdict(list)
    for row in selected:
        key = strings["files"][row["file"]], strings["strings"][row["model"]]
        grouped[key].append(row["position"])
    return {key: np.array(values, dtype=np.float32) for key, values in grouped.items()}


def discover(catalog_path: Path, work: Path, output: Path, pattern: str) -> dict[str, Any]:
    if output.exists():
        raise ValueError("Use a new output directory; existing catalogs are never overwritten")
    catalog = WorldCatalog(catalog_path)
    models: dict[str, tuple[dict[str, Any], dict[str, Any]]] = {}
    rejected: dict[str, str] = {}
    definitions: dict[str, Any] = {}
    try:
        for path in sorted((work / "gui_cache_v4/models/levels").glob(pattern + "/model.json")):
            try:
                meta = json.loads(path.read_text())
                definition = rigid_hinge(meta, path.with_name("model.glb"))
                definition["provenance"]["metadata_sha256"] = file_digest(path)
                source = meta["source"]["id"]
                if source in models:
                    raise ValueError("Duplicate cached source identity")
                models[source] = meta, definition
            except (ValueError, KeyError, IndexError, OSError) as error:
                rejected[str(path)] = str(error)
        if not models:
            raise ValueError("No eligible cached door models")
        baked = renderer_placements(work, set(models))
        source_database = catalog_path.parent / catalog.manifest["database"]
        with sqlite3.connect(source_database.resolve().as_uri() + "?mode=ro", uri=True) as db:
            db.row_factory = sqlite3.Row
            rows = db.execute(
                "SELECT i.*,o.definition AS object FROM instances i LEFT JOIN objects o "
                "ON i.identity=o.identity WHERE i.source IN ("
                + ",".join("?" for _ in models)
                + ")",
                tuple(models),
            ).fetchall()
        checked: dict[str, str | None] = {}
        for row in rows:
            identity, source = row["identity"], row["source"]
            try:
                if row["object"]:
                    raise ValueError("Existing explicit object preserved")
                transform = np.array(json.loads(row["transform"]))
                scale = np.linalg.norm(transform[:3, :3], axis=0)
                if (
                    not np.isfinite(transform).all()
                    or np.any(scale <= 0)
                    or not np.allclose(scale, scale[0])
                    or not np.allclose(
                        transform[:3, :3].T @ transform[:3, :3], np.eye(3) * scale[0] ** 2
                    )
                    or np.linalg.det(transform[:3, :3]) <= 0
                ):
                    raise ValueError("Non-rigid or nonuniform placement")
                positions = baked.get((json.loads(row["metadata"]).get("map", ""), source))
                if (
                    positions is None
                    or np.count_nonzero(np.all(np.abs(positions - transform[:3, 3]) < 0.01, axis=1))
                    != 1
                ):
                    raise ValueError("No unique HD renderer placement")
                if source not in checked:
                    try:
                        verify_collision(models[source][0], catalog.geometry(dict(row)).source)
                        checked[source] = None
                    except ValueError as error:
                        checked[source] = str(error)
                if checked[source]:
                    raise ValueError(checked[source])
                definitions[identity] = models[source][1]
            except ValueError as error:
                rejected[identity] = str(error)
        if not definitions:
            raise ValueError("No door placements passed collision and renderer admission")
        output.mkdir(parents=True)
        objects_path = output / "objects.json"
        objects_path.write_text(json.dumps(definitions, indent=2, allow_nan=False) + "\n")
        shutil.copyfile(source_database, output / "world.sqlite")
        with sqlite3.connect(output / "world.sqlite") as db:
            db.executemany(
                "INSERT INTO objects VALUES (?,?)",
                [(identity, json.dumps(d, allow_nan=False)) for identity, d in definitions.items()],
            )
        manifest = catalog.manifest.copy()
        manifest["database"] = "world.sqlite"
        manifest["database_sha256"] = file_digest(output / "world.sqlite")
        manifest["objects_sha256"] = file_digest(objects_path)
        manifest["derived_from_sha256"] = catalog.digest
        manifest["counts"] = dict(manifest["counts"], automatic_single_hinge_doors=len(definitions))
        manifest["limits"] = manifest["limits"] + [
            "Door admission verifies source structure, not world contact clearance. Known floor/frame overlaps jam some hinges; this catalog is for evaluation.",
            "Automatic doors are intact rigid single-hinge assemblies; no latches, motors, fracture or inferred joint damping.",
            "This admission pass examines cached models only; other doors and props remain static.",
        ]
        report = {
            "models_eligible": len(models),
            "models_admitted": len({d["provenance"]["model"] for d in definitions.values()}),
            "placements_admitted": len(definitions),
            "placements_examined": len(rows),
            "rejection_counts": dict(Counter(rejected.values())),
            "rejected": rejected,
        }
        (output / "admission.json").write_text(json.dumps(report, indent=2) + "\n")
        (output / "catalog.json").write_text(json.dumps(manifest, indent=2) + "\n")
        return {key: value for key, value in report.items() if key != "rejected"}
    finally:
        catalog.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("catalog", "work", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--pattern", default="*door*", help="Cached model directory glob")
    args = parser.parse_args()
    print(json.dumps(discover(args.catalog, args.work, args.output, args.pattern), indent=2))


if __name__ == "__main__":
    main()
