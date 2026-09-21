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

"""Build everything a recording needs before it is served, with no server running.

    python -m dimos.teleop.memory_world.prebuild <recording.db|.mcap> [--image-index-stride 5]

Two things are written into the recording (into ``<name>.derived.db`` beside a
read-only mcap): the voxel replay streams the timeline scrubs, and the SigLIP frame
index the questions are answered from. Both are the module's own builds, called
directly, so what this leaves behind is exactly what a first server start would have
built -- and a later start finds both and is ready in seconds. Running it again on a
finished recording is cheap: the replay is found, the index adds nothing.

Exits non-zero, with the reason, when either build refuses.
"""

from __future__ import annotations

import argparse
import sys
import time

from dimos.teleop.memory_world.module import MemoryWorldModule
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def prebuild(store_path: str, **config: object) -> MemoryWorldModule:
    """Build the replay streams and the frame index of *store_path*; raise on refusal."""
    module = MemoryWorldModule(store_path=store_path, build_image_index_on_start=True, **config)
    try:
        started = time.monotonic()
        module._ensure_replay()  # raises when nothing could be placed
        logger.info("replay ready in %.0f s", time.monotonic() - started)
        started = time.monotonic()
        module._build_visual_index()  # logs its failure and leaves it in the progress
        if not module._index_progress.startswith("ready"):
            raise RuntimeError(f"visual index: {module._index_progress}")
        logger.info("%s in %.0f s", module._index_progress, time.monotonic() - started)
    finally:
        module.stop()
    return module


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description="Build a recording's replay streams and SigLIP frame index ahead of serving it"
    )
    parser.add_argument("store_path")
    parser.add_argument(
        "--image-index-stride",
        type=int,
        default=5,
        help="index every Nth frame; 5 is what ~/Commands/memworld serves with",
    )
    parser.add_argument("--image-stream", default=None, help="default: the module's guess")
    parser.add_argument("--lidar-stream", default=None)
    parser.add_argument("--world-frame", default=None)
    args = parser.parse_args(argv)
    config: dict[str, object] = {"image_index_stride": args.image_index_stride}
    for key, value in (
        ("image_stream_name", args.image_stream),
        ("lidar_stream_name", args.lidar_stream),
        ("world_frame", args.world_frame),
    ):
        if value is not None:
            config[key] = value
    try:
        prebuild(args.store_path, **config)
    except (Exception, SystemExit) as error:  # a refusal is a SystemExit
        sys.exit(f"prebuild failed: {error}")


if __name__ == "__main__":
    main()
