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

"""Bind a robot scene to the immutable global world catalog."""

import argparse
import json
from pathlib import Path
import shutil
from typing import Any

import numpy as np
from pxr import Sdf, Usd

from dimos.experimental.gta_sim2.world_index import file_digest


def prepare(scene: Path, catalog: Path, output: Path) -> dict[str, Any]:
    source = json.loads(scene.with_name("manifest.json").read_text())
    world = json.loads(catalog.read_text())
    if not np.allclose(source["origin_rage_m"], world["origin_rage_m"]):
        raise ValueError("Robot scene and catalog origins differ")
    output.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.Open(str(scene.resolve()))
    patch = stage.GetPrimAtPath("/world/gta_patch")
    if not patch:
        raise ValueError("Scene has no GTA world anchor")
    digest = file_digest(catalog)
    patch.CreateAttribute(
        "gta:collision_catalog_sha256", Sdf.ValueTypeNames.String, custom=True
    ).Set(digest)
    patch.CreateAttribute("gta:world_streaming", Sdf.ValueTypeNames.Bool, custom=True).Set(True)
    patch.CreateAttribute("gta:scene_state_sha256", Sdf.ValueTypeNames.String, custom=True).Set(
        world.get("scene_state", {}).get("sha256", "")
    )
    stage.GetRootLayer().Export(str(output / "world.usda"))
    shutil.copyfile(scene.with_name("surfaces.npz"), output / "surfaces.npz")
    source.setdefault("optical_proxy_bounds_rage_m", source["bounds_rage_m"])
    source["bounds_rage_m"] = (
        np.asarray(world["coverage_bounds"]) + world["origin_rage_m"]
    ).tolist()
    source["streaming_catalog_sha256"] = digest
    source["world_catalog"] = str(catalog.resolve())
    source["world_counts"] = world["counts"]
    source["limitations"] = world["limits"] + [
        "The default MuJoCo optical proxy covers the initial patch; the interactive M20 uses SixNG for streamed Sim2 RGB-D.",
        "Uniform friction 0.8 remains an assumption.",
        "Dynamic objects require admitted physical definitions in the catalog's objects table.",
    ]
    source.pop("excluded_overlapping_sources", None)
    (output / "manifest.json").write_text(json.dumps(source, indent=2) + "\n")
    return {"scene": str(output / "world.usda"), "catalog_sha256": digest}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("scene", "catalog", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(prepare(args.scene, args.catalog, args.output)))


if __name__ == "__main__":
    main()
