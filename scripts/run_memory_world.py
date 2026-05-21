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
    map_path = _resolve(args.map, "global map")

    module = MemoryWorldModule(
        store_path=str(db_path),
        global_map_path=str(map_path),
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
