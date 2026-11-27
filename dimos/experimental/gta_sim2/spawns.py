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

"""Prepare and validate named M20 deployments; browser inputs select IDs only."""

import argparse
import json
import math
from pathlib import Path
import re
from typing import Any

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom

from dimos.experimental.gta_sim2.prepare_m20 import BASE
from dimos.experimental.gta_sim2.world_index import file_digest
from dimos.robot.assets.usd_utils.assets import resolve_usd_assets


def load_spawns(path: Path, catalog: Path) -> dict[str, dict[str, Any]]:
    data = json.loads(path.read_text())
    if data.get("version") != 1 or data.get("catalog_sha256") != file_digest(catalog):
        raise ValueError("Spawn list belongs to a different collision catalog")
    result: dict[str, dict[str, Any]] = {}
    for item in data["locations"]:
        identity = item["id"]
        scene = (path.parent / item["scene"]).resolve()
        if (
            re.fullmatch(r"[a-z0-9-]{1,48}", identity) is None
            or identity in result
            or not isinstance(item["label"], str)
            or not 0 < item["speed_mps"] <= 3
            or item.get("camera", "follow") not in ("follow", "head")
            or file_digest(scene) != item["scene_sha256"]
        ):
            raise ValueError("Invalid or changed spawn definition")
        manifest = json.loads(scene.with_name("manifest.json").read_text())
        if manifest["streaming_catalog_sha256"] != data["catalog_sha256"]:
            raise ValueError("Spawn scene and catalog identities differ")
        result[identity] = {"camera": "follow", **item, "scene": scene}
    if not result or len(result) > 32:
        raise ValueError("Provide between one and 32 named spawns")
    return result


def prepare(scene: Path, definitions: Path, output: Path) -> dict[str, Any]:
    """Create isolated scenes at nominated floor coordinates for subsequent testing."""
    if output.exists():
        raise ValueError("Choose a new spawn output directory")
    manifest = json.loads(scene.with_name("manifest.json").read_text())
    if "spawn" in manifest:
        raise ValueError("Prepare named spawns from the original base scene")
    origin = np.asarray(manifest["origin_rage_m"])
    source = resolve_usd_assets(scene.read_text(), str(scene.resolve().parent))
    output.mkdir(parents=True)
    result: dict[str, Any] = {
        "version": 1,
        "catalog_sha256": manifest["streaming_catalog_sha256"],
        "locations": [],
    }
    for item in json.loads(definitions.read_text())["locations"]:
        identity = item["id"]
        floor = np.asarray(item["floor_rage_m"], dtype=np.float64)
        heading = float(item["yaw_degrees"])
        if (
            re.fullmatch(r"[a-z0-9-]{1,48}", identity) is None
            or floor.shape != (3,)
            or not np.isfinite(floor).all()
            or not math.isfinite(heading)
            or any(p["id"] == identity for p in result["locations"])
        ):
            raise ValueError("Invalid spawn pose or duplicate ID")
        layer = Sdf.Layer.CreateAnonymous()
        layer.ImportFromString(source)
        stage = Usd.Stage.Open(layer)
        body = UsdGeom.Xformable(stage.GetPrimAtPath("/" + BASE))
        base = body.GetLocalTransformation().ExtractTranslation()
        if abs(base[0]) > 1e-6 or abs(base[1]) > 1e-6 or not 0 < base[2] < 2:
            raise ValueError("Base robot must stand above the scene origin")
        clearance = base[2]
        matrix = Gf.Matrix4d(1.0)
        matrix.SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), heading))
        matrix.SetTranslateOnly(Gf.Vec3d(*(floor - origin + [0, 0, clearance])))
        body.MakeMatrixXform().Set(matrix)
        directory = output / identity
        directory.mkdir()
        target = directory / "world.usda"
        stage.GetRootLayer().Export(str(target))
        # The legacy floor-query fallback is not used by the world backend,
        # but retaining the source proxy preserves the older inspection path.
        (directory / "surfaces.npz").symlink_to(scene.with_name("surfaces.npz").resolve())
        (directory / "manifest.json").write_text(
            json.dumps({**manifest, "spawn": item}, indent=2) + "\n"
        )
        result["locations"].append(
            {**item, "scene": str(target.relative_to(output)), "scene_sha256": file_digest(target)}
        )
    (output / "spawns.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("scene", "definitions", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(prepare(args.scene, args.definitions, args.output)))


if __name__ == "__main__":
    main()
