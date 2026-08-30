#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
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

"""Fold a store's sightings into the views questions are asked against.

    python -m scripts.build_views recording.db

Reads ``belief_observation`` from the store and writes ``belief_identity``,
``belief_entity`` and ``belief_frame_annotation`` back into the same file, next
to the camera, lidar and odometry they were derived from. One file goes in, the
same file comes out fuller.

**Why in place.** Every tool downstream takes one store: ``dimos mem rerun``
renders one, a belief query reads one, and putting streams on a shared timeline
requires them to share a file. Deriving into a second file and merging it back
was a step that existed only to undo the split it had just made.

Re-running replaces the derived streams rather than appending to them, so the
same command twice gives the same store rather than doubled entities.

The folds themselves live in ``dimos.experimental.memory_belief.pipeline``,
because a robot deriving the same views while it drives has to run identical
code. Anything worth tuning is a field on ``ViewParams``, not a constant here.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time

from dimos.experimental.memory_belief.annotate import ANNOTATION_STREAM_NAME
from dimos.experimental.memory_belief.entity import ENTITY_STREAM_NAME
from dimos.experimental.memory_belief.identity import IDENTITY_STREAM_NAME
from dimos.experimental.memory_belief.pipeline import ViewParams, build_views
from dimos.experimental.memory_belief.types import STREAM_NAME as BELIEF_STREAM_NAME
from dimos.memory.store.sqlite import SqliteStore

#: Read the defaults off an instance, not off the class: ViewParams uses slots,
#: so its class attributes are descriptors rather than the values.
DEFAULTS = ViewParams()

#: Written by this script, and therefore safe to replace on a re-run. The raw
#: streams are not in this list and are never touched.
DERIVED = (IDENTITY_STREAM_NAME, ENTITY_STREAM_NAME, ANNOTATION_STREAM_NAME)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("store", type=Path, help="a recording holding belief_observation")
    parser.add_argument(
        "--static-threshold",
        type=float,
        default=DEFAULTS.static_threshold_m,
        help="metres an entity may move and still count as static",
    )
    args = parser.parse_args(argv)

    params = ViewParams(static_threshold_m=args.static_threshold)
    started = time.perf_counter()
    store = SqliteStore(path=str(args.store), must_exist=True)
    try:
        if BELIEF_STREAM_NAME not in store.list_streams():
            print(
                f"{args.store} holds no {BELIEF_STREAM_NAME!r}; run detect_recording first",
                file=sys.stderr,
            )
            return 1

        for name in DERIVED:
            if name in store.list_streams():
                store.delete_stream(name)
                print(f"replacing {name}")

        build_views(
            belief=store,
            out=store,
            session=args.store.stem,
            params=params,
            report=print,
        )
    finally:
        store.stop()

    print(f"wrote views into {args.store} in {time.perf_counter() - started:.0f}s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
