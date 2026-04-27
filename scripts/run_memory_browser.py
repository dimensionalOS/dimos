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

"""Launch the VR Memory Browser against a SQLite memory store.

Usage:
    python scripts/run_memory_browser.py [--db PATH] [--port 8443] [--n 50]

Then in the Quest browser, navigate to ``https://<host>:8443/memory_browser``
and tap Connect. Hold the right grip and rotate your wrist to scrub the
timeline; flick laterally to dismiss the focus highlight.
"""

from __future__ import annotations

import argparse
from pathlib import Path

from dimos.teleop.memory_browser import MemoryBrowserModule


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--db",
        default="data/go2_bigoffice.db",
        help="Path to the SQLite memory store",
    )
    p.add_argument("--port", type=int, default=8443, help="HTTPS port to serve on")
    p.add_argument(
        "--stream",
        default="color_image",
        help="Stream name in the store to populate the ribbon",
    )
    p.add_argument(
        "--n",
        type=int,
        default=50,
        help="Number of thumbnails to sample across the timeline",
    )
    args = p.parse_args()

    db_path = Path(args.db).expanduser().resolve()
    if not db_path.exists():
        raise SystemExit(f"memory store not found: {db_path}")

    module = MemoryBrowserModule(
        store_path=str(db_path),
        stream_name=args.stream,
        n_thumbnails=args.n,
        server_port=args.port,
    )
    module.start()
    print(f"open https://<host>:{args.port}{module.config.client_route} in the Quest browser")
    try:
        # Keep the process alive; the module runs its own threads.
        while True:
            input("press enter to stop...\n")
            break
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        module.stop()


if __name__ == "__main__":
    main()
