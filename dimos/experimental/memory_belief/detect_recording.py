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

"""Turn a recording's camera frames into belief, in the recording itself.

    python -m dimos.experimental.memory_belief.detect_recording recording.db --vocabulary lvis

Writes ``belief_observation`` back into the store it read the frames from, so
the sightings sit on the same timeline as the camera and lidar they came from.
``python -m scripts.build_views`` then folds them into entities in that same
file, and ``dimos mem rerun`` plays the result. One store throughout.

The vocabulary is an argument, not a constant, and whatever it is gets written
onto every record -- so a question about a term outside it answers
OUT_OF_VOCABULARY instead of a confident "no". ``--vocabulary none`` runs
prompt-free and records ``None``, which reads as "covers everything".

Nothing here names a class, a room, or a robot.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time
from typing import TYPE_CHECKING, Any

from dimos.experimental.memory_belief.identity import (
    IDENTITY_STREAM_NAME,
    IdentityClaim,
    append_identity,
    claims_from_tracks,
)
from dimos.experimental.memory_belief.locate import (
    PinholeFisheye,
)
from dimos.experimental.memory_belief.produce import DetectParams, detect_stream
from dimos.experimental.memory_belief.vocabulary import from_file, resolve as resolve_vocabulary
from dimos.experimental.memory_belief.write import append_belief, belief_stream, derived_stream
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

# The one robot-specific import. The camera mount belongs in the recording's
# `tf` stream; reading it from there would make this producer platform-neutral.
from dimos.robot.unitree.go2.connection import GO2Connection

#: Read the defaults off an instance: DetectParams uses slots, so its class
#: attributes are descriptors rather than the values.
_D = DetectParams()

if TYPE_CHECKING:
    from collections.abc import Sequence


def build_detector(
    model_name: str | None,
    conf: float,
    device: str | None,
    vocabulary: tuple[str, ...] | None = None,
) -> Any:
    """YOLO-E, open-vocabulary or constrained to a named list.

    Open-vocabulary keeps the model's own tag list, which on this recording
    contained scenes and concepts alongside objects. A vocabulary removes that
    class of noise structurally, at the cost of being unable to see what it was
    not given -- which is why the list is recorded on every observation.
    """
    from dimos.perception.detection.detectors.yoloe import Yoloe2DDetector, YoloePromptMode

    if vocabulary:
        # Text prompting needs the non-"-pf" weights; the prompt-free
        # checkpoints have the vocabulary baked in and ignore set_classes.
        detector = Yoloe2DDetector(
            prompt_mode=YoloePromptMode.PROMPT,
            model_name=model_name or "yoloe-11l-seg.pt",
            device=device,
            conf=conf,
            exclude_class_ids=[],
            max_area_ratio=None,
        )
        detector.set_prompts(text=list(vocabulary))
        return detector

    return Yoloe2DDetector(
        prompt_mode=YoloePromptMode.LRPC,  # LRPC == prompt-free
        model_name=model_name,
        device=device,
        conf=conf,
        # Both filters off on purpose. `exclude_class_ids` would silently drop
        # classes the run never gets to reconsider, and `max_area_ratio` would
        # drop the large nearby objects that matter most indoors. A detection
        # kept with its confidence is recoverable; one never recorded is not.
        exclude_class_ids=[],
        max_area_ratio=None,
    )


def main(argv: Sequence[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path, help="path to a .db recording")
    parser.add_argument(
        "--out",
        type=Path,
        default=None,
        help="write to a separate store instead of into the recording. Splitting "
        "them means every later tool needs both files and a join; the default "
        "keeps sightings beside the frames they came from",
    )
    parser.add_argument("--image-stream", default="color_image")
    parser.add_argument("--lidar-stream", default="lidar")
    parser.add_argument(
        "--no-locate",
        action="store_true",
        help="skip 3D placement; records keep capture_place_ref only",
    )
    parser.add_argument(
        "--min-points", type=int, default=_D.min_points, help="lidar returns needed to place"
    )
    parser.add_argument(
        "--depth-band", type=float, default=_D.depth_band_m, help="depth inlier band, metres"
    )
    parser.add_argument(
        "--align-tolerance", type=float, default=0.3, help="frame-to-scan pairing window, seconds"
    )
    parser.add_argument(
        "--identity",
        action="store_true",
        help="also write tracker-derived identity claims to the belief_identity stream",
    )
    parser.add_argument("--stride", type=int, default=1, help="use every Nth frame")
    parser.add_argument("--limit", type=int, default=None, help="stop after N frames")
    parser.add_argument("--model", default=None, help="YOLO-E weights (default: prompt-free 11s)")
    parser.add_argument("--device", default=None, help="cuda / cpu (default: auto)")
    parser.add_argument("--conf", type=float, default=0.25, help="detector confidence threshold")
    parser.add_argument(
        "--min-confidence",
        type=float,
        default=0.0,
        help="drop detections below this before recording; a record that exists "
        "at all counts as evidence downstream",
    )
    parser.add_argument("--min-brightness", type=float, default=_D.min_brightness)
    parser.add_argument(
        "--place-size", type=float, default=_D.place_size_m, help="place cell size, metres"
    )
    parser.add_argument(
        "--vocabulary",
        default=None,
        help="'lvis', a path to a one-term-per-line file, or omit for open-vocabulary. "
        "Whatever is chosen is recorded on every observation so a later question "
        "about an unlisted thing answers OUT_OF_VOCABULARY, not 'no'.",
    )
    parser.add_argument(
        "--vocabulary-extra",
        default=None,
        help="additional terms appended to --vocabulary, one per line",
    )
    args = parser.parse_args(argv)

    # In place by default: `align()` and tag pushdown work against the frames
    # the sightings came from only when they share a file, and every downstream
    # tool takes one store.
    out_path = args.out or args.recording

    vocab = resolve_vocabulary(args.vocabulary)
    if vocab and args.vocabulary_extra:
        vocab = tuple(dict.fromkeys(vocab + from_file(args.vocabulary_extra)))
    detector = build_detector(args.model, args.conf, args.device, vocab)
    mode = "prompt" if vocab else "lrpc"
    source = f"yoloe-{mode}:{Path(getattr(detector, 'model_name', 'yoloe')).stem}"

    store = SqliteStore(path=str(args.recording), must_exist=True)
    out_store = store if out_path == args.recording else SqliteStore(path=str(out_path))
    try:
        # No payload type: recordings disagree about what their streams hold,
        # and the store already knows what this one was created with. Consumed
        # as an iterator and never materialised, so this same producer runs
        # against a live robot, where the stream does not end.
        frames: Any = store.stream(args.image_stream)
        locator = None
        if not args.no_locate:
            frames = frames.align(
                store.stream(args.lidar_stream, PointCloud2), tolerance=args.align_tolerance
            )
            locator = PinholeFisheye(GO2Connection.camera_info_static)

        print(f"{args.recording.name}: streaming {args.image_stream!r}, stride {args.stride}")
        vlabel = f"{len(vocab)} terms" if vocab else "open (vocabulary=None)"
        print(f"detector: {source}  vocabulary: {vlabel}  conf>={args.conf}")
        print(f"placement: {'off' if args.no_locate else f'on, min_points={args.min_points}'}")

        out_stream = belief_stream(out_store)
        ident_stream = (
            derived_stream(out_store, IDENTITY_STREAM_NAME, IdentityClaim)
            if args.identity
            else None
        )
        started = time.perf_counter()
        written = placed = 0
        dark = [0]
        labels: dict[str, int] = {}
        sightings: list[tuple[str, Any, float]] = []

        params = DetectParams(
            min_confidence=args.min_confidence,
            min_brightness=args.min_brightness,
            min_points=args.min_points,
            depth_band_m=args.depth_band,
            place_size_m=args.place_size,
        )
        for record in detect_stream(
            frames,
            detector,
            stream_name=args.image_stream,
            vocabulary=vocab,  # None means open-vocabulary: "covers everything"
            source=source,
            params=params,
            camera=locator,
            stride=args.stride,
            limit=args.limit,
            on_skip=lambda: dark.__setitem__(0, dark[0] + 1),
        ):
            if record.target_pose is not None:
                placed += 1
            if ident_stream is not None and record.identity_basis:
                sightings.append(
                    (record.target_ref, record.identity_basis.split(":")[-1], record.valid_ts)
                )
            append_belief(out_stream, record)
            written += 1
            if record.label:
                labels[record.label] = labels.get(record.label, 0) + 1
            if written % 250 == 0:
                rate = written / (time.perf_counter() - started)
                print(f"  {written} records  ({rate:.0f}/s)", file=sys.stderr)

        elapsed = time.perf_counter() - started
        print(
            f"\nwrote {written} belief records to {out_path} in {elapsed:.0f}s "
            f"({dark[0]} frames skipped as too dark)"
        )
        pct = placed / written * 100 if written else 0.0
        print(f"placed in 3D: {placed} / {written} records ({pct:.0f}%)")
        if ident_stream is not None:
            claims = 0
            for claim in claims_from_tracks(sightings, session=f"{args.recording.stem}"):
                append_identity(ident_stream, claim)
                claims += 1
            entities = len({s[1] for s in sightings})
            print(f"identity: {claims} claims over {entities} tracker entities")
        if vocab:
            named = len(labels)
            print(f"labels used: {named} of {len(vocab)} offered ({named / len(vocab) * 100:.1f}%)")
        else:
            print(f"discovered vocabulary: {len(labels)} distinct labels (nothing was prompted)")
        for label, count in sorted(labels.items(), key=lambda kv: -kv[1])[:25]:
            print(f"  {label:<28} {count}")
    finally:
        if out_store is not store:
            out_store.stop()
        store.stop()


if __name__ == "__main__":
    main()
