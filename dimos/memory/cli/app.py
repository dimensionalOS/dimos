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

"""``dimos mem`` — memory store commands."""

from __future__ import annotations

from pathlib import Path

import typer

from dimos.memory.cli.summary import main as _summary_main

mem_app = typer.Typer(help="memory store commands", no_args_is_help=True)
mem_app.command("summary")(_summary_main)


@mem_app.command()
def rerun(
    path: str = typer.Argument(
        ...,
        help=(
            "Store .db/.mcap: path, cwd/data/LFS name, recording folder, or downloaded name. "
            "Upload ID prefixes match only the first 8 characters stored in download filenames."
        ),
    ),
    out: str = typer.Option(None, "--out", help="Output .rrd (default: alongside the source)"),
    seconds: float = typer.Option(None, "--seconds", help="Only the first N seconds"),
    no_gui: bool = typer.Option(False, "--no-gui", help="Write the .rrd but don't open the viewer"),
    root: str = typer.Option(
        None, "--root", help="Nest every stream under this entity path (<root>/<name>)"
    ),
) -> None:
    """Render a memory store into rerun (writes a .rrd, then opens the viewer).

    The detached viewer stays open after this command exits.
    Close the viewer window separately; Ctrl+C and dimos stop will not close it.
    """
    from dimos.memory.cli.dataset import AmbiguousDatasetError, open_dataset
    from dimos.memory.cli.render import render_store

    try:
        store = open_dataset(path)
    except (FileNotFoundError, AmbiguousDatasetError) as exc:
        raise typer.BadParameter(str(exc), param_hint="PATH") from exc
    render_store(store, out=out, seconds=seconds, no_gui=no_gui, root=root)


@mem_app.command()
def embed(
    path: str = typer.Argument(..., help="Store .db that already has a color image stream."),
    hz: float = typer.Option(2.0, "--hz", help="Max frames per second to embed."),
    stream: str = typer.Option("color_image", "--stream", help="Source image stream."),
    dest: str = typer.Option("color_image_embedded", "--dest", help="Embedding stream to write."),
) -> None:
    """Write CLIP embeddings for a color image stream into the same store."""
    from dimos.memory.cli.dataset import AmbiguousDatasetError, open_dataset
    from dimos.memory.query import embed_color_stream
    from dimos.models.embedding.clip import CLIPModel

    if hz <= 0:
        raise typer.BadParameter("hz must be > 0", param_hint="--hz")
    try:
        store = open_dataset(path)
    except (FileNotFoundError, AmbiguousDatasetError) as exc:
        raise typer.BadParameter(str(exc), param_hint="PATH") from exc
    try:
        written = embed_color_stream(store, CLIPModel(), hz=hz, stream=stream, dest=dest)
    except LookupError as exc:
        raise typer.BadParameter(str(exc), param_hint="--stream") from exc
    except ValueError as exc:
        raise typer.BadParameter(str(exc)) from exc
    finally:
        store.stop()
    typer.echo(f"wrote {written} embeddings to {dest}")


@mem_app.command()
def caption(
    path: str = typer.Argument(..., help="Store .db that already has a color image stream."),
    jsonl: str = typer.Option(
        ..., "--jsonl", help="JSONL rows with t and caption, parsed, or raw."
    ),
    dest: str = typer.Option("color_image_caption", "--dest", help="Caption stream to write."),
) -> None:
    """Write 1 Hz visual-logger JSON onto a color_image_caption stream."""
    from dimos.memory.cli.dataset import AmbiguousDatasetError, open_dataset
    from dimos.memory.query import captions_from_jsonl, write_caption_stream

    try:
        store = open_dataset(path)
    except (FileNotFoundError, AmbiguousDatasetError) as exc:
        raise typer.BadParameter(str(exc), param_hint="PATH") from exc
    try:
        rows = captions_from_jsonl(Path(jsonl))
        written = write_caption_stream(store, rows, dest=dest)
    except LookupError as exc:
        raise typer.BadParameter(str(exc), param_hint="PATH") from exc
    except ValueError as exc:
        raise typer.BadParameter(str(exc)) from exc
    finally:
        store.stop()
    typer.echo(f"wrote {written} captions to {dest}")


@mem_app.command("dump-captions")
def dump_captions(
    path: str = typer.Argument(..., help="Store .db that already has color_image_caption."),
    out: str = typer.Option(
        ..., "--out", help="Directory for captions.jsonl and captions/YYYY-MM-DD.jsonl."
    ),
) -> None:
    """Write 1 Hz captions as one JSONL file per UTC day."""
    from dimos.memory.cli.dataset import AmbiguousDatasetError, open_dataset
    from dimos.memory.query import dump_caption_jsonl

    dest = Path(out)
    dest.mkdir(parents=True, exist_ok=True)
    try:
        store = open_dataset(path)
    except (FileNotFoundError, AmbiguousDatasetError) as exc:
        raise typer.BadParameter(str(exc), param_hint="PATH") from exc
    try:
        files = dump_caption_jsonl(store, dest)
    except LookupError as exc:
        raise typer.BadParameter(str(exc), param_hint="PATH") from exc
    finally:
        store.stop()
    typer.echo(f"wrote {sum(row['n'] for row in files)} captions in {len(files)} day file(s)")


@mem_app.command("track")
def track(
    path: str = typer.Argument(..., help="Store .db that already has a color_image stream."),
    out: str = typer.Option(
        ..., "--out", help="Directory for tracks.jsonl and tracks/YYYY-MM-DD.jsonl."
    ),
    queries: str = typer.Option(
        "person,car,backpack,handbag,suitcase,chair,bench,dog,bicycle,motorcycle",
        "--queries",
        help="Comma-separated OmDet class names.",
    ),
    reid: bool = typer.Option(
        True, "--reid/--no-reid", help="Crop-embedding ReID on track breaks."
    ),
    embedder: str = typer.Option("siglip", "--embedder", help="clip, siglip, or dino."),
    det_threshold: float = typer.Option(0.3, "--det-threshold"),
    reid_threshold: float = typer.Option(0.75, "--reid-threshold"),
    seconds: float | None = typer.Option(
        None, "--seconds", help="Only the first N relative seconds."
    ),
) -> None:
    """Run OmDet + BoT-SORT on the same 1 Hz frames as captions."""
    from dimos.memory.cli.dataset import AmbiguousDatasetError, open_dataset
    from dimos.perception import identity as vis_id

    names = tuple(name.strip() for name in queries.split(",") if name.strip())
    if not names:
        raise typer.BadParameter("queries must not be empty", param_hint="--queries")
    dest = Path(out)
    dest.mkdir(parents=True, exist_ok=True)
    try:
        store = open_dataset(path)
    except (FileNotFoundError, AmbiguousDatasetError) as exc:
        raise typer.BadParameter(str(exc), param_hint="PATH") from exc
    tracker = None
    try:
        tracker = vis_id.build_identity_tracker(
            queries=names,
            det_threshold=det_threshold,
            reid=reid,
            embedder=embedder,
            reid_threshold=reid_threshold,
        )
        files = vis_id.dump_track_jsonl(store, dest, tracker.update, until=seconds)
    except (LookupError, ImportError, ValueError) as exc:
        raise typer.BadParameter(str(exc)) from exc
    finally:
        if tracker is not None:
            tracker.stop()
        store.stop()
    typer.echo(f"wrote {sum(row['n'] for row in files)} track seconds in {len(files)} day file(s)")
