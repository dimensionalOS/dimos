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
import logging
from pathlib import Path
import sys
import time
from typing import Any

from dimos.teleop.memory_world.hyperspace_search import memory_db_for

logger = logging.getLogger(__name__)


def depth_info_stream_for(streams: set[str], depth_stream: str, camera_info: str) -> str:
    """The depth camera's own ``camera_info`` when the recording has one, else the colour one."""
    candidate = f"{depth_stream}_camera_info"
    return candidate if candidate in streams else camera_info


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
    from dimos.mapping.hyperspace.cli import ingest, pick_device
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
    memory = SqliteStore(path=str(memory_path), must_exist=False)
    memory.start()
    chosen = pick_device(device)
    print(f"embedding with {model_name} on {chosen} -> {memory_path}", flush=True)
    model = SigLIP2Patches(model_name=model_name, device=chosen, towers="vision")
    model.start()
    started = time.monotonic()
    try:
        stats = ingest(
            store,
            memory,
            model,
            color_stream=detected["image"],
            depth_stream=detected["depth"],
            color_info_stream=detected["camera_info"],
            depth_info_stream=depth_info,
            tf_stream=detected["tf"],
            hz=hz,
            max_seconds=max_seconds,
            # A lower novelty threshold than Hyperspace's 0.05 keeps consecutive keyframes,
            # which its "support" refinement needs (two keyframes agreeing on a voxel).
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
    summary: dict[str, Any] = {**stats, "seconds": round(time.monotonic() - started, 1)}
    print(f"done: {summary}", flush=True)
    return summary


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
