#!/usr/bin/env python3
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

"""Launch the VR Memory World against a SQLite memory store + pickled map.

Usage:
    python scripts/run_memory_world.py [--db PATH] [--map PATH] [--port 8443]

Then in the Quest browser, navigate to ``https://<host>:8443/memory_world``
and tap Connect. Left thumbstick walks, right thumbstick X snap-turns,
right trigger teleports, bimanual pinch scales the world.
"""

from __future__ import annotations

import argparse
from pathlib import Path

from dimos.teleop.memory_world import MemoryWorldModule
from dimos.utils.data import get_data

# Defaults match the bigoffice dataset shipped via LFS. ``get_data()`` will
# auto-pull and decompress these on first use.
_DEFAULT_DB_NAME = "go2_bigoffice.db"
_DEFAULT_MAP_NAME = "unitree_go2_bigoffice_map.pickle"


def _resolve(name_or_path: str, kind: str) -> Path:
    """Treat an explicit path as-is; otherwise pull via get_data()."""
    p = Path(name_or_path).expanduser()
    if p.is_absolute() or p.parts[:1] == ("data",):
        if not p.exists():
            raise SystemExit(f"{kind} not found at {p}")
        return p.resolve()
    return get_data(name_or_path)


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--db",
        default=_DEFAULT_DB_NAME,
        help="LFS-managed name (e.g. 'go2_bigoffice.db') or a path under data/",
    )
    p.add_argument(
        "--map",
        default=_DEFAULT_MAP_NAME,
        help="LFS-managed name or a path; e.g. 'unitree_go2_bigoffice_map.pickle'",
    )
    p.add_argument("--port", type=int, default=8443, help="HTTPS port to serve on")
    p.add_argument(
        "--cloud-source",
        choices=["pickle", "lidar"],
        default="pickle",
        help="pickle: prebuilt map (RGB). lidar: voxel map accumulated live "
        "from the lidar stream (height-coloured, no pickle needed).",
    )
    p.add_argument(
        "--lidar-world-frame",
        action="store_true",
        help="lidar mode: scans are already map/world-registered, so don't "
        "re-apply pose. Use this if voxels look scattered everywhere.",
    )
    p.add_argument(
        "--voxel-scans",
        type=int,
        default=150,
        help="lidar mode: how many lidar scans to accumulate. 0 = use ALL "
        "frames (densest map, slowest build).",
    )
    p.add_argument(
        "--image-markers",
        type=int,
        default=200,
        help="How many capture-pose image markers to sample across the run.",
    )
    p.add_argument(
        "--voxel-size",
        type=float,
        default=0.05,
        help="Voxel size (m) for downsampling before sending to the headset",
    )
    p.add_argument(
        "--max-points",
        type=int,
        default=250_000,
        help="Hard cap on points shipped to the client",
    )
    args = p.parse_args()

    db_path = _resolve(args.db, "memory store")
    # The pickle is only needed for the top-down map and the "pickle" cloud
    # source. In lidar mode we still try to resolve it (for the minimap) but
    # don't hard-fail if it's missing.
    try:
        map_path = _resolve(args.map, "global map")
    except SystemExit:
        if args.cloud_source == "pickle":
            raise
        map_path = None

    module = MemoryWorldModule(
        store_path=str(db_path),
        global_map_path=str(map_path) if map_path else "",
        cloud_source=args.cloud_source,
        lidar_world_frame=args.lidar_world_frame,
        n_voxel_scans=args.voxel_scans,
        n_image_markers=args.image_markers,
        voxel_size=args.voxel_size,
        max_points=args.max_points,
        server_port=args.port,
    )
    module.start()
    print(f"open https://<host>:{args.port}{module.config.client_route} in the Quest browser")
    try:
        while True:
            input("press enter to stop...\n")
            break
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        module.stop()


if __name__ == "__main__":
    main()
