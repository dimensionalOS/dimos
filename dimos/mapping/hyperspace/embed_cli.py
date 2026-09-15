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

"""``dimos map embed`` -- put an embedding index into a recording, and nothing else.

    dimos map embed bike.db                                  # the default trio
    dimos map embed bike.db --models google/siglip2-base-patch16-224
    dimos map embed bike.db --models facebook/PE-Core-L14-336 --index-name pe

One model per pass, written into the recording itself. Run it again with another
checkpoint and the new index lands beside the ones already there rather than over
them -- which is the whole point of it being its own command. Comparing two encoders
means holding both at once, and a tool that can only replace makes that impossible.

`dimos map query` also builds an index, as a step on the way to answering something.
This does only the building, so a long pass is not tangled with a question, and so a
recording can be prepared on one machine and asked on another.
"""

from __future__ import annotations

from pathlib import Path
import time
from typing import Any

import typer

from dimos.mapping.hyperspace import patches as hs
from dimos.mapping.hyperspace.cli import (
    WAL_CAP_BYTES,
    drop_index,
    fold_the_wal,
    hold_the_wal,
    index_is_finished,
    ingest,
    memory_db_for,
    open_store,
    pick_device,
    pick_index,
    pick_stream,
    refuse_unless_readable,
    wal_bytes,
)
from dimos.mapping.hyperspace.embedder import DEFAULT_TRIO, PatchEnsemble
from dimos.mapping.hyperspace.ingest import (
    IngestConfig,
    index_slug,
    stream_names,
)
from dimos.mapping.hyperspace.module import depth2depth_model_of
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def report_indexes(memory: Any) -> None:
    """What this recording already answers from, before anything is touched.

    Counted off the PATCH streams, not off `indexes_in`. The flat layout writes no
    keyframe stream at all, and `indexes_in` looks for one -- so asking it reports
    "none yet" about an index that has two million vectors in it.
    """
    from dimos.mapping.hyperspace.frames import member_streams

    found = list(member_streams(memory))
    if not found:
        typer.echo("indexes: none yet")
        return
    rows = [f"{tag}={memory.stream(stream, dict).count():,}" for tag, stream in found]
    typer.echo(f"indexes: {', '.join(rows)}")


def main(
    recording_path: Path = typer.Argument(..., help="the .db recording to index, in place"),
    models: str = typer.Option(
        ",".join(DEFAULT_TRIO),
        "--models",
        help="comma separated checkpoints (ids or dirs; NaFlex ones as id@budget, "
        "tiled ones as id#RxC). The default is the trio the agreement step wants.",
    ),
    index_name: str = typer.Option(
        "",
        "--index-name",
        help="name this index instead of deriving it from the checkpoints, to keep two "
        "indexes of the same model side by side",
    ),
    memory_db: Path = typer.Option(
        None, "--memory-db", help="write the index here instead of into the recording"
    ),
    replace: bool = typer.Option(
        False, "--replace", help="rebuild this index if the recording already has one"
    ),
    keep_frames: bool = typer.Option(
        False,
        "--keep-frames",
        help="also keep the colour frame behind each embedding frame, as a live run "
        "does. Unnecessary for a recording, which already holds its own colour.",
    ),
    hz: float = typer.Option(
        hs.MAX_KEYFRAME_HZ,
        "--hz",
        help="frames a second to EMBED, which is the ceiling on the keyframe rate. This "
        "is the cost knob: every frame it looks at is a forward pass through every "
        "checkpoint, whether or not the gate then keeps it.",
    ),
    max_seconds: float = typer.Option(
        0.0, "--seconds", help="stop after this much of the recording (0 = all of it)"
    ),
    novelty: float = typer.Option(
        -1.0,
        "--novelty",
        help="mean patch change needed to keep a frame; 0 keeps every frame it looks at, "
        "-1 uses the tuned default",
    ),
    depth2depth: str = typer.Option(
        "default",
        "--depth2depth",
        help="fill the depth holes before measuring patch depth: 'default', a "
        "depth-anything checkpoint, or '' for raw stereo",
    ),
    max_depth: float = typer.Option(10.0, "--max-depth", help="depth past this (m) is a hole"),
    device: str = typer.Option("auto", "--device"),
    color_stream: str = typer.Option("", "--color-stream"),
    depth_stream: str = typer.Option("", "--depth-stream"),
    color_info_stream: str = typer.Option("", "--color-info-stream"),
    depth_info_stream: str = typer.Option("", "--depth-info-stream"),
    tf_stream: str = typer.Option("tf", "--tf-stream"),
) -> None:
    """Embed a recording's frames and write the patch vectors into it."""
    recording_path = recording_path.expanduser()
    source = open_store(recording_path)
    color = pick_stream(source, color_stream or None, "color", "image")
    depth = pick_stream(source, depth_stream or None, "depth", "image")
    color_info = pick_stream(source, color_info_stream or None, "camera_info")
    depth_info = pick_stream(source, depth_info_stream or None, "depth", "camera_info")

    memory_path = (memory_db or memory_db_for(recording_path)).expanduser()
    memory = source if memory_path == recording_path else open_store(memory_path, must_exist=False)
    if memory_path.suffix == ".db":
        hold_the_wal(memory)

    specs = [spec.strip() for spec in models.split(",") if spec.strip()]
    if not specs:
        raise typer.BadParameter("--models needs at least one checkpoint")
    slug = index_name or pick_index(memory, specs)
    report_indexes(memory)
    typer.echo(f"streams: color={color} depth={depth} info={color_info}/{depth_info}")
    typer.echo(f"writing {index_name or index_slug(specs)} into {stream_names(slug)[1]}__m_*")

    if index_is_finished(memory, slug) and not replace:
        typer.echo(
            f"{recording_path.name} already holds this index. Nothing was changed; "
            "pass --replace to rebuild it, or --index-name to keep both."
        )
        raise typer.Exit(0)

    # Every stream the pass reads, checked BEFORE the old index is dropped: an empty
    # one must not cost the index it was going to replace.
    refuse_unless_readable(source, (color, depth, color_info, depth_info, tf_stream))
    model = PatchEnsemble(specs, device=pick_device(device), towers="vision")
    model.start()
    typer.echo(f"embedding with {model.tags} on {pick_device(device)}")
    drop_index(memory, slug)

    started = time.monotonic()
    stats = ingest(
        source,
        memory,
        model,
        color_stream=color,
        depth_stream=depth,
        color_info_stream=color_info,
        depth_info_stream=depth_info,
        tf_stream=tf_stream,
        hz=hz,
        # `ingest` stops once the recording is this far in, so "all of it" has to be a
        # number rather than a zero -- which read as "stop immediately" and embedded
        # exactly one frame before anyone noticed.
        max_seconds=max_seconds if max_seconds > 0 else 1e9,
        config=IngestConfig(
            gate=hs.KeyframeGateConfig(
                max_angular_velocity=None,
                **({} if novelty < 0 else {"novelty_threshold": novelty}),
            ),
            max_depth_m=max_depth,
            depth2depth_model=depth2depth_model_of(depth2depth),
            # The flat layout, always: it is the only one the frames-first query reads,
            # and a fresh index has no old reader to keep happy.
            flat=True,
            keep_frames=keep_frames,
        ),
        slug=slug,
    )
    took = time.monotonic() - started
    kept = stats.get("kept", 0)
    typer.echo(
        f"embedded {kept} frames of {stats.get('images', 0)} in {took:.0f}s "
        f"({kept / max(took, 1e-9):.1f} frames/s)"
    )
    report_indexes(memory)
    if memory_path.suffix == ".db":
        if wal_bytes(memory) > WAL_CAP_BYTES:
            typer.echo("folding a large write-ahead log back")
        fold_the_wal(memory)
    memory.stop()


if __name__ == "__main__":
    typer.run(main)
