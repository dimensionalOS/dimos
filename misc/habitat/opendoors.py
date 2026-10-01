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
"""The `hssd-opendoors` dataset the navigation cases run in: HSSD scene instances with their
door objects removed, next to `hssd-hab`, sharing its stages, objects and semantics.

    python misc/habitat/opendoors.py --hssd /data/habitat/hssd-hab --out /data/habitat/hssd-opendoors \\
        --ground-truth misc/habitat/ground_truth/hssd [scene_id ...]

Door ids come from the ground truth (label `door`); a scene without ground truth is skipped.
The dataset config points at `../hssd-hab`, so both live under the same data directory.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--hssd", type=Path, required=True, help="hssd-hab dataset directory")
    ap.add_argument("--out", type=Path, required=True, help="hssd-opendoors directory to write")
    ap.add_argument(
        "--ground-truth", type=Path, required=True, help="directory of <scene>.json boxes"
    )
    ap.add_argument("scenes", nargs="*", help="scene ids; default: every scene with ground truth")
    a = ap.parse_args()
    (a.out / "scenes").mkdir(parents=True, exist_ok=True)
    cfg = json.loads((a.hssd / "hssd-hab.scene_dataset_config.json").read_text())
    rel = Path("..") / a.hssd.name
    cfg["stages"]["paths"][".json"] = [str(rel / "stages")]
    cfg["objects"]["paths"][".json"] = [str(rel / "objects/*"), str(rel / "objects/decomposed/*")]
    cfg["scene_instances"]["paths"][".json"] = ["scenes"]
    cfg["semantic_scene_descriptor_instances"]["hssd_ssd_map"] = str(
        rel / "semantics/hssd-hab_semantic_lexicon.json"
    )
    (a.out / "hssd-opendoors.scene_dataset_config.json").write_text(json.dumps(cfg, indent=1))
    ids = a.scenes or sorted(
        f.name.removesuffix(".scene_instance.json")
        for f in (a.hssd / "scenes").glob("*.scene_instance.json")
    )
    removed: dict[str, int] = {}
    for sid in ids:
        gt = a.ground_truth / f"{sid}.json"
        if not gt.exists():
            continue
        doors = {
            d["id"].split("@")[0]
            for d in json.loads(gt.read_text())["detections"]
            if d["label"] == "door"
        }
        scene = json.loads((a.hssd / "scenes" / f"{sid}.scene_instance.json").read_text())
        keep = [
            o for o in scene["object_instances"] if not any(h in o["template_name"] for h in doors)
        ]
        removed[sid] = len(scene["object_instances"]) - len(keep)
        scene["object_instances"] = keep
        (a.out / "scenes" / f"{sid}.scene_instance.json").write_text(json.dumps(scene))
    (a.out / "removed_doors.json").write_text(json.dumps(removed, indent=1))
    print(f"{len(removed)} scenes written; doors removed: {sum(removed.values())}")


if __name__ == "__main__":
    main()
