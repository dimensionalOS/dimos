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

"""Flatten 3D ground-truth files (``<scene_id>.json``) to ``<scene_id>.top_down.json``.

The 3D to 2D step of the pipeline on its own: ``hssd_ground_truth.py`` already
writes both files in one run, but that needs the HSSD dataset. This script only
needs the checked-in 3D file, so the 2D view can be regenerated (or the
projection re-tuned) anywhere.

    uv run python misc/habitat/project_top_down.py misc/habitat/ground_truth/hssd/*[0-9].json
"""

from __future__ import annotations

import argparse
from pathlib import Path

from dimos.simulation.object_detections import (
    COVERING_FOOTPRINT_FRACTION,
    read_detection3d_json,
    top_down,
    write_detection2d_json,
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("paths", nargs="+", type=Path, help="3D ground-truth JSON files")
    parser.add_argument("--out", type=Path, help="output directory (default: next to each input)")
    parser.add_argument(
        "--covering-fraction",
        type=float,
        default=COVERING_FOOTPRINT_FRACTION,
        help="drop boxes spanning this fraction of the scene on both axes (floor slabs, roofs); "
        "pass 0 to keep everything",
    )
    args = parser.parse_args()
    fraction = args.covering_fraction or None
    for path in args.paths:
        if path.name.endswith(".top_down.json"):
            continue
        detections, provenance = read_detection3d_json(path)
        flat = top_down(detections, covering_fraction=fraction)
        out_dir = args.out or path.parent
        out_dir.mkdir(parents=True, exist_ok=True)
        out = write_detection2d_json(
            flat, out_dir / f"{path.name[: -len('.json')]}.top_down.json", provenance=provenance
        )
        print(
            f"{path.name}: {detections.detections_length} boxes -> {flat.detections_length}; wrote {out}"
        )


if __name__ == "__main__":
    main()
