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

"""Run Hyperspace's ingest over a recording the memory world can open.

Hyperspace's own CLI opens an mcap raw, so its ``camera_info`` arrives as
undecoded bytes and its streams are picked by name; this entry point opens
the recording through :func:`recording.open_recording` (ROS 2 CDR decoded by
schema) and names the streams by payload type, then hands the store to the
same :func:`dimos.mapping.hyperspace.cli.ingest`. Output goes to
``<recording>.hyperspace.db`` (see :func:`hyperspace_search.memory_db_for`).

    python -m dimos.teleop.memory_world.hyperspace_ingest <recording.db|.mcap> \
        [--model-name DIR_OR_HF_ID] [--device mps|cuda|cpu] [--hz 5]
"""

from __future__ import annotations

import argparse
from collections.abc import Iterator
import heapq
import logging
from pathlib import Path
import sys
import time
from typing import Any

import numpy as np

from dimos.teleop.memory_world.hyperspace_search import memory_db_for
from dimos.teleop.memory_world.recording import depth_info_stream_for

logger = logging.getLogger(__name__)


def ingest_command(
    recording: str | Path, *, model_name: str, device: str, hz: float, novelty: float = 0.02
) -> list[str]:
    """The subprocess that runs this module on *recording*, with the current interpreter."""
    return [
        sys.executable,
        "-m",
        "dimos.teleop.memory_world.hyperspace_ingest",
        str(recording),
        "--model-name",
        model_name,
        "--device",
        device,
        "--hz",
        str(hz),
        "--novelty",
        str(novelty),
    ]


def ingest_recording(
    recording: str | Path,
    *,
    model_name: str,
    device: str = "auto",
    hz: float = 5.0,
    max_seconds: float = 1e9,
    max_depth_m: float = 10.0,
    novelty: float = 0.02,
) -> dict[str, Any]:
    """Embed *recording*'s keyframes into its Hyperspace memory db. Returns the ingest stats."""
    from dimos.mapping.hyperspace import patches as hs
    from dimos.mapping.hyperspace.cli import pick_device
    from dimos.mapping.hyperspace.embedder import SigLIP2Patches
    from dimos.mapping.hyperspace.ingest import IngestConfig
    from dimos.memory.store.sqlite import SqliteStore
    from dimos.teleop.memory_world.recording import detect_streams, open_recording

    recording = Path(recording)
    store = open_recording(recording)
    store.start()
    detected = detect_streams(store)
    missing = [role for role in ("image", "depth", "camera_info", "tf") if not detected.get(role)]
    if missing:
        raise SystemExit(f"{recording.name} has no {', '.join(missing)} stream; cannot ingest")
    streams = set(store.list_streams())
    depth_info = depth_info_stream_for(streams, detected["depth"], detected["camera_info"])
    print(
        f"streams: color={detected['image']} depth={detected['depth']} "
        f"info={detected['camera_info']}/{depth_info} tf={detected['tf']}",
        flush=True,
    )

    memory_path = memory_db_for(recording)
    # Built beside the final name and moved into place at the end: a rerun (or an
    # interrupted run) must not append a second copy of every keyframe.
    building = memory_path.with_name(memory_path.name + ".building")
    for stale in (
        building,
        building.with_name(building.name + "-wal"),
        building.with_name(building.name + "-shm"),
    ):
        stale.unlink(missing_ok=True)
    memory = SqliteStore(path=str(building), must_exist=False)
    memory.start()
    chosen = pick_device(device)
    print(f"embedding with {model_name} on {chosen} -> {memory_path}", flush=True)
    model = SigLIP2Patches(model_name=model_name, device=chosen, towers="vision")
    model.start()
    started = time.monotonic()
    try:
        stats = _ingest(
            store,
            memory,
            model,
            streams=detected,
            depth_info=depth_info,
            hz=hz,
            max_seconds=max_seconds,
            config=IngestConfig(
                gate=hs.KeyframeGateConfig(novelty_threshold=novelty), max_depth_m=max_depth_m
            ),
        )
    finally:
        for obj in (model, memory, store):
            try:
                obj.stop()
            except Exception:
                logger.exception("stopping %s", type(obj).__name__)
    for suffix in ("-wal", "-shm"):
        memory_path.with_name(memory_path.name + suffix).unlink(missing_ok=True)
    building.replace(memory_path)
    summary: dict[str, Any] = {**stats, "seconds": round(time.monotonic() - started, 1)}
    print(f"done: {summary}", flush=True)
    return summary


TF_TOLERANCE_S = 0.1


def _ingest(
    store: Any,
    memory: Any,
    model: Any,
    *,
    streams: dict[str, Any],
    depth_info: str,
    hz: float,
    max_seconds: float,
    config: Any,
) -> dict[str, int]:
    """Hyperspace's ingest loop (``dimos.mapping.hyperspace.cli.ingest``), with two
    changes for stitched recordings: keyframes are placed through the corrected tf
    tree (:func:`build_tf_tree`), and that corrected ``world -> base_link`` is what
    goes into the memory db's tf, so query-time placement matches the map."""
    from dimos.mapping.hyperspace.ingest import PatchIngestor
    from dimos.msgs.geometry_msgs.Quaternion import Quaternion
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage
    from dimos.teleop.memory_world.recording import build_tf_tree

    tree = build_tf_tree(store, streams["tf"])
    # The one decision is build_tf_tree's: the corrected poses replace world -> base_link
    # here exactly when they did in the tree, and never for an empty stream.
    world, corrected = tree.substituted or (None, None)

    def lookup(target: str, source: str, ts: float) -> Any:
        matrix = tree.lookup(target, source, ts, TF_TOLERANCE_S)
        return None if matrix is None else np.asarray(matrix, dtype=np.float64)

    ingestor = PatchIngestor(memory, model, config, lookup=lookup)
    for name in (streams["camera_info"], depth_info):
        first = next(iter(store.streams[name].order_by("ts")), None)
        if first is None:
            raise SystemExit(f"stream {name!r} is empty")
        ingestor.add_camera_info(first.data)

    colors = store.streams[streams["image"]].order_by("ts")
    depths = store.streams[streams["depth"]].order_by("ts")
    start_ts = float(colors.first().ts)

    # Only the tf the slice can use; a whole recording's tf is hundreds of
    # thousands of messages the query side would otherwise decode.
    # Hyperspace reads tf back in time order but skips ids it has passed, so the
    # originals and the corrected poses are merged chronologically, streamed.
    def in_window(observation: Any) -> bool:
        return start_ts - 5.0 <= float(observation.ts) <= start_ts + max_seconds + 5.0

    def originals() -> Iterator[tuple[float, TFMessage]]:
        for observation in store.streams[streams["tf"]].order_by("ts"):
            if not in_window(observation):
                continue
            message = observation.data
            if corrected is not None:
                kept = [
                    t
                    for t in message.transforms
                    if not (t.frame_id == world and t.child_frame_id == "base_link")
                ]
                if not kept:
                    continue
                message = TFMessage(*kept)
            yield float(observation.ts), message

    def corrected_poses() -> Iterator[tuple[float, TFMessage]]:
        if corrected is None:
            return
        for observation in store.streams[corrected].order_by("ts"):
            if not in_window(observation):
                continue
            pose = observation.data.pose
            p, q = pose.position, pose.orientation
            stamp = float(
                getattr(observation.data, "ts", 0.0) or observation.ts
            )  # as build_tf_tree
            yield (
                stamp,
                TFMessage(
                    Transform(
                        translation=Vector3(float(p.x), float(p.y), float(p.z)),
                        rotation=Quaternion(float(q.x), float(q.y), float(q.z), float(q.w)),
                        frame_id=world,
                        child_frame_id="base_link",
                        ts=stamp,
                    )
                ),
            )

    def statics() -> Iterator[tuple[float, TFMessage]]:
        if streams.get("tf_static") is None:
            return
        held = [t for obs in store.streams[streams["tf_static"]] for t in obs.data.transforms]
        if held:  # once, before everything: Hyperspace holds the last sample of an edge
            yield start_ts - 5.0, TFMessage(*held)

    transforms = 0
    for stamp, message in heapq.merge(
        statics(), originals(), corrected_poses(), key=lambda item: item[0]
    ):
        ingestor.add_tf(message, ts=stamp)
        transforms += 1
    print(
        f"tf: {transforms} messages{' (corrected base poses from ' + corrected + ')' if corrected else ''}",
        flush=True,
    )

    min_interval = 1.0 / hz if hz > 0 else 0.0
    ingestor.config.min_frame_interval_s = max(min_interval, config.min_frame_interval_s)
    first_ts: float | None = None
    started = time.monotonic()
    for pair in colors.align(depths, tolerance=config.depth_max_dt):
        color_obs, depth_obs = pair.data[0], pair.data[1]
        stamp = float(color_obs.ts)
        if first_ts is None:
            first_ts = stamp
        if stamp - first_ts > max_seconds:
            break
        ingestor.add_depth(depth_obs.data)
        ingestor.add_image(color_obs.data)
        if ingestor.stats["images"] % 200 == 0:
            print(
                f"{stamp - first_ts:.0f}s: {ingestor.stats['embedded']} embedded, "
                f"{ingestor.stats['kept']} kept ({time.monotonic() - started:.0f}s)",
                flush=True,
            )
    ingestor.flush()
    return dict(ingestor.stats)


def main(argv: list[str] | None = None) -> None:
    from dimos.mapping.hyperspace.embedder import SIGLIP2_MODEL_NAME

    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("recording", help="A memory2 .db or a ROS 2 .mcap")
    parser.add_argument("--model-name", default=SIGLIP2_MODEL_NAME)
    parser.add_argument("--device", default="auto", help="cuda, mps, cpu or auto")
    parser.add_argument(
        "--hz", type=float, default=5.0, help="colour frames per second to consider"
    )
    parser.add_argument("--max-seconds", type=float, default=1e9)
    parser.add_argument("--max-depth", type=float, default=10.0)
    parser.add_argument(
        "--novelty", type=float, default=0.02, help="keyframe gate novelty threshold"
    )
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    ingest_recording(
        args.recording,
        model_name=args.model_name,
        device=args.device,
        hz=args.hz,
        max_seconds=args.max_seconds,
        max_depth_m=args.max_depth,
        novelty=args.novelty,
    )


if __name__ == "__main__":
    main()
