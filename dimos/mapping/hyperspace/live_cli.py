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

"""``dimos map live`` -- ask a recording the way the robot would, and watch it happen.

    dimos map live bike.db -q "a traffic cone" -q "a stop sign" -o ~/out

Nothing here is a simulation of the live system: it builds the same `LiveQuery` the
`Hyperspace` module holds, warms the same models, and calls the same `ask`. The only
difference is where the store came from. That is the point -- a page drawn by a
reimplementation would be a picture of something the robot does not do.

Each query gets a self-contained 3D page -- the same one `dimos map find` writes, and
for the same reason: an answer is a box somewhere, and a list of coordinates is not
something anyone can judge. The recording's geometry is there in grey, the answers in
orange, and the frames the detector was shown hang on their own view frustums so the
evidence sits beside the claim. The clock still replays the answering: each box appears
at the second it actually arrived. The pages, an index and a `queries.zip` of all of
them land in the output directory.
"""

from __future__ import annotations

import json
from pathlib import Path
import time
from typing import Any
import zipfile

import typer

from dimos.mapping.hyperspace.cli import open_store, pick_device, pick_stream
from dimos.mapping.hyperspace.detect import (
    DetectConfig,
    detector_dtype_for,
    gpu_preprocess_for,
)
from dimos.mapping.hyperspace.frames import member_streams, spec_of
from dimos.mapping.hyperspace.live import LiveConfig, LiveQuery
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def slug_of(text: str) -> str:
    """A filename from a query: ``"a stop sign"`` -> ``a_stop_sign``."""
    kept = [character if character.isalnum() else "_" for character in text.strip().lower()]
    return "_".join(part for part in "".join(kept).split("_") if part) or "query"


def scene_of(store: Any, frames: Any, world_frame: str) -> tuple[Any, Any]:
    """The recording's geometry and route, read once: every query stands in the same room."""
    from dimos.mapping.hyperspace import render

    started = time.monotonic()
    frames.load_tf()
    points = render.scene_points(store, frames.tf, world_frame)
    route = render.trajectory(store, frames.tf, world_frame)
    typer.echo(
        f"scene: {len(points)} points, {len(route)} poses ({time.monotonic() - started:.1f}s)"
    )
    return points, route


def write_page(
    path: Path,
    query: str,
    result: Any,
    answers: list[Any],
    live: Any,
    scene: tuple[Any, Any],
    recording: str,
) -> Path:
    """One query's page: the answers where they are, in the room they are in.

    The same page `dimos map find` writes, because there is one thing worth looking at
    and it is the boxes in the scene. A list of coordinates is not a result you can
    judge; a box floating in an aisle is. The frames the detector was shown hang on
    their own view frustums, so the evidence sits beside the claim in the same space,
    and the clock still replays the answers at the seconds they actually arrived.
    """
    from dimos.mapping.hyperspace import render

    timings = result.timings
    stats = {
        "took": f"{result.ms:.0f} ms",
        "search": f"{timings.get('search', 0):.2f}s",
        "detect": f"{timings.get('detect', 0):.2f}s",
        "frames matched": int(timings.get("frames_matched", 0)),
        "refused": result.refused,
        "models": ", ".join(live.config.models),
    }
    return render.boxes_html(
        path,
        query,
        answers,
        scene[0],
        scene[1],
        recording=recording,
        views=render.camera_views(answers, live.frames, live.config.detect.world_frame),
        stats=stats,
    )


def main(
    recording_path: Path = typer.Argument(..., help="the .db recording to ask"),
    query: list[str] = typer.Option(..., "-q", "--query", help="what to look for; repeatable"),
    out: Path = typer.Option(None, "--out", "-o", help="where the pages and the zip land"),
    models: str = typer.Option(
        "", "--models", help="comma separated member tags to search (default: all of them)"
    ),
    threshold: float = typer.Option(
        DetectConfig.threshold, "--threshold", help="OWLv2's per-box acceptance score"
    ),
    max_episodes: int = typer.Option(12, "--max-episodes"),
    merge_m: float = typer.Option(0.75, "--merge", help="answers this close are one place (m)"),
    world_frame: str = typer.Option("odom", "--world-frame"),
    depth2depth: str = typer.Option(
        "auto",
        "--depth2depth",
        help="fill stereo's holes as boxes are placed: 'auto' only when the recording "
        "has no filled-depth stream, '' off, or a checkpoint by name",
    ),
    device: str = typer.Option("auto", "--device"),
    index_device: str = typer.Option(
        LiveConfig.index_device,
        "--index-device",
        help="where the patch index lives: 'auto' is the accelerator with the spill in "
        "RAM, 'cpu' the numpy path, or name a device",
    ),
    tower_device: str = typer.Option(
        LiveConfig.tower_device,
        "--tower-device",
        help="where the text towers run; 'auto' asks what is left once the detector and "
        "the index are placed, 'cpu' keeps the card for the detector",
    ),
    dtype: str = typer.Option(
        DetectConfig.dtype,
        "--dtype",
        help="detector precision: 'auto' is fp16 on CUDA and float32 elsewhere; "
        "'' forces float32. bf16 moves scores and is not worth it",
    ),
    rank_with: str = typer.Option(
        DetectConfig.rank_with,
        "--rank-with",
        help="score one member over everything and let it choose the frames the others "
        "confirm; 'auto' picks the cheapest member, '' searches every member in full",
    ),
    rank_frames: int = typer.Option(
        DetectConfig.rank_frames,
        "--rank-frames",
        help="how many of the ranking member's best frames the others confirm; 0 = no cut, "
        "which narrows nothing and so saves nothing",
    ),
    contrast: bool = typer.Option(
        DetectConfig.contrast,
        "--contrast/--no-contrast",
        help="subtract generic floor/wall/ceiling prompts from every patch score",
    ),
    gpu_preprocess: str = typer.Option(
        DetectConfig.gpu_preprocess,
        "--gpu-preprocess",
        help="prepare the detector's images on the card (347 ms a frame becomes 3.5 ms): "
        "'auto' on CUDA only, or on/off. MPS has no antialiased resize, so 'on' raises there",
    ),
) -> None:
    """Answer each query the way the live module would, and write a page per query."""
    recording_path = recording_path.expanduser()
    out = (out or recording_path.parent / f"{recording_path.stem}_live").expanduser()
    out.mkdir(parents=True, exist_ok=True)
    store = open_store(recording_path)

    available = [tag for tag, _ in member_streams(store)]
    if not available:
        raise typer.BadParameter(
            f"{recording_path} holds no patch streams; build one with `dimos map embed`"
        )
    wanted = [tag.strip() for tag in models.split(",") if tag.strip()] or available
    missing = [tag for tag in wanted if tag not in available]
    if missing:
        raise typer.BadParameter(f"no such model(s) {missing}; the index holds {available}")

    live = LiveQuery(
        store,
        LiveConfig(
            detect=DetectConfig(
                threshold=threshold,
                device=pick_device(device),
                max_episodes=max_episodes,
                world_frame=world_frame,
                depth2depth=depth2depth,
                dtype=dtype,
                gpu_preprocess=gpu_preprocess,
                rank_with=rank_with,
                rank_frames=rank_frames,
                contrast=contrast,
            ),
            models=wanted,
            merge_m=merge_m,
            tower_device=tower_device,
            index_device=index_device,
            # Named at construction, not patched afterwards: `RecordingFrames` reads the
            # camera intrinsics in its constructor, so a name set later is set too late.
            color_stream=pick_stream(store, None, "color", "image"),
            depth_stream=pick_stream(store, None, "depth", "image"),
            color_info_stream=pick_stream(store, None, "camera", "info"),
            depth_info_stream=pick_stream(store, None, "depth", "camera", "info"),
        ),
    )

    # What it actually ran on, not what was asked for: "auto" is three different machines
    # and a page that compares two of them should not leave the reader inferring which.
    detect = live.config.detect
    typer.echo(f"index: {recording_path}  models {wanted} of {available}")
    typer.echo(
        f"detector: {detect.device} dtype "
        f"{detector_dtype_for(detect.dtype, detect.device) or 'float32'} "
        f"gpu_preprocess {gpu_preprocess_for(detect.gpu_preprocess, detect.device)} "
        f"index_device {live.config.index_device} "
        f"towers {live.tower_device()} "
        f"rank_with {detect.rank_with or 'every member'} rank_frames {detect.rank_frames}"
    )
    loaded = live.warm([spec_of(tag) for tag in wanted])
    typer.echo(
        f"warm: detector {loaded['detector']:.1f}s, text towers {loaded['towers']:.1f}s, "
        f"recording {loaded['recording']:.1f}s, {int(loaded['index'])} patches in "
        f"{loaded['index_s']:.1f}s, first pass {loaded.get('first_pass_s', 0):.1f}s"
    )

    scene = scene_of(store, live.frames, world_frame)

    written = []
    summary: dict[str, Any] = {
        "recording": str(recording_path),
        "ran_on": {
            "device": detect.device,
            "dtype": detector_dtype_for(detect.dtype, detect.device) or "float32",
            "gpu_preprocess": gpu_preprocess_for(detect.gpu_preprocess, detect.device),
            "rank_with": detect.rank_with,
            "rank_frames": detect.rank_frames,
            "models": list(wanted),
            "warm": loaded,
        },
        "queries": {},
    }
    for text in query:
        typer.echo(f"\n{text!r}")
        started = time.monotonic()
        result = live.ask(text)
        for found in result.objects:
            typer.echo(
                f"  place {found.place_id:<3} owl {found.confidence:.2f}  "
                f"at ({found.centre[0]:6.1f}, {found.centre[1]:6.1f}, {found.centre[2]:5.1f}) "
                f"{found.frame}  {found.depth_m:.1f} m away  {found.views} view(s)"
            )
        typer.echo(
            f"  {len(result)} place(s), {result.refused} refused, "
            f"{time.monotonic() - started:.1f}s  {result.timings}"
        )
        path = write_page(
            out / f"{slug_of(text)}.html",
            text,
            result,
            live.answers,
            live,
            scene,
            recording_path.name,
        )
        written.append(path)
        typer.echo(f"  {path}")
        summary["queries"][text] = result.as_dict()

    index = out / "index.html"
    index.write_text(_index_page(recording_path.name, query, written, summary))
    (out / "answers.json").write_text(json.dumps(summary, indent=2, default=str))

    bundle = out / "queries.zip"
    with zipfile.ZipFile(bundle, "w", zipfile.ZIP_DEFLATED) as archive:
        for path in [index, *written]:
            archive.write(path, path.name)
    typer.echo(f"\n{len(written)} page(s) + {index.name} -> {bundle}")
    live.close()


def _index_page(recording: str, queries: list[str], written: list[Path], summary: dict) -> str:
    items = []
    for text, path in zip(queries, written, strict=False):
        answer = summary["queries"][text]
        items.append(
            f"<li><a href='{path.name}'>{text}</a> &mdash; {len(answer['objects'])} place(s), "
            f"{answer['refused']} refused, {answer['ms']:.0f} ms</li>"
        )
    return (
        "<!doctype html><meta charset='utf-8'><title>hyperspace, live</title>"
        "<style>body{font:15px/1.6 ui-sans-serif,-apple-system,sans-serif;max-width:60ch;"
        "margin:40px auto;padding:0 20px;background:#0a0b10;color:#e8e8ef}"
        "a{color:#ff8a2e}li{margin:6px 0}</style>"
        f"<h1>{recording}</h1><ul>{''.join(items)}</ul>"
    )


if __name__ == "__main__":
    typer.run(main)
