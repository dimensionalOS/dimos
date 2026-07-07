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

"""Offline entrypoint: scan a recording (or recall over it) from the command line.

    # fold the whole recording and print the believed-present objects (world coords)
    uv run python -m dimos.perception.scan_recording REC.db --prompt cup bottle box

    # bound the fold to the most-recent 30 s
    uv run python -m dimos.perception.scan_recording REC.db --prompt cup --window 30

    # recall "when/where did I last see X" (builds the CLIP index once if missing)
    uv run python -m dimos.perception.scan_recording REC.db --recall "coffee cup" --snapshot out.png

A thin driver over :class:`~dimos.perception.scene_scan.SceneScanner` and
:mod:`dimos.perception.recall`; all the load-bearing logic is unit-tested.
"""

from __future__ import annotations

import argparse

from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.recall import build_frame_clip_index, index_stream_name, recall
from dimos.perception.scene_scan import SceneScanner
from dimos.robot.manipulators.xarm.worldbelief_recorder import xarm6_worldbelief_history_path


def _run_scan(store: SqliteStore, args: argparse.Namespace) -> None:
    history = xarm6_worldbelief_history_path() if args.history else None
    scanner = SceneScanner(
        target_frame="world",
        history_path=history,
        min_frames=args.min_frames,
        detector_conf=args.conf,
    )
    prompt = args.prompt or None
    # --window bounds the (single, fresh-scanner) scan's lookback; default = whole recording.
    present = scanner.scan_recent(store, window=args.window, prompt=prompt)
    print(f"present objects: {len(present)}")
    belief = scanner._belief
    for o in present:
        c = o.center
        print(
            f"  {o.name:14s} id={o.object_id[:6]} trust={belief.trust_of(o.object_id)} "
            f"basis={belief.basis_of(o.object_id)} world=({c.x:+.2f},{c.y:+.2f},{c.z:+.2f})"
        )


def _run_recall(store: SqliteStore, args: argparse.Namespace) -> None:
    if store.stream(index_stream_name("openai/clip-vit-base-patch32"), Image).count() == 0:
        print("no CLIP index yet — building it once (this scans every frame)...")
        build_frame_clip_index(store)
    hit, _ = recall(store, args.recall)  # bare index search — no source recordings to verify against
    if hit is None:
        print(f"recall('{args.recall}'): no match")
        return
    where = "unknown" if hit.pose_stamped is None else (
        f"({hit.pose_stamped.x:+.2f},{hit.pose_stamped.y:+.2f},{hit.pose_stamped.z:+.2f})"
    )
    print(f"recall('{args.recall}'): when_ts={hit.ts:.2f} where={where} similarity={hit.similarity:.3f}")
    if args.snapshot is not None:
        import cv2

        # to_opencv() is format-aware and already returns BGR — exactly what imwrite wants.
        # (An extra RGB2BGR here double-swaps and turns red objects blue.)
        cv2.imwrite(args.snapshot, hit.data.to_opencv())
        print(f"  snapshot -> {args.snapshot}")


def main() -> None:
    p = argparse.ArgumentParser(description="Scan / recall over a WorldBelief recording (.db).")
    p.add_argument("recording", help="path to the recording .db")
    p.add_argument("--prompt", nargs="*", help="open-vocab detection prompts for a scan")
    p.add_argument("--window", type=float, default=0.0, help="scan only the last N seconds (0 = whole)")
    p.add_argument("--recall", help="recall text query instead of a scan")
    p.add_argument("--min-frames", type=int, default=3, dest="min_frames")
    p.add_argument("--conf", type=float, default=0.6, help="detector confidence threshold")
    p.add_argument("--history", action="store_true", help="use the shared cross-session history db")
    p.add_argument("--snapshot", help="save the recalled frame to this path")
    args = p.parse_args()

    with SqliteStore(path=args.recording, must_exist=True) as store:
        if args.recall:
            _run_recall(store, args)
        else:
            _run_scan(store, args)


if __name__ == "__main__":
    main()
