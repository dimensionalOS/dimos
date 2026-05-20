# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

"""Verify a memory2 SqliteStore — open the .db and print stream summaries.

The agent reads recorded streams directly from the .db; no copy step.
"""

from __future__ import annotations

import argparse
from pathlib import Path

from dimos.memory2.store.sqlite import SqliteStore


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--db", type=Path, required=True,
                        help="Recording .db path (required).")
    args = parser.parse_args()

    if not args.db.exists():
        raise SystemExit(f"recording not found: {args.db}")

    print(f"[verify] opening {args.db} ({args.db.stat().st_size / 1e6:.0f} MB)")
    store = SqliteStore(path=str(args.db))
    print(f"[verify] streams: {store.list_streams()}")
    for name in store.list_streams():
        s = store.stream(name)
        try:
            print(f"  {s.summary()}")
        except Exception as e:  # noqa: BLE001
            print(f"  {name}: <summary error: {e}>")
    store.stop()
    print("[verify] done")


if __name__ == "__main__":
    main()
