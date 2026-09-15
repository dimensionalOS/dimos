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

"""``dimos map find`` -- ask a recording where something is, and get boxes back.

The frames-first path: the patch index ranks *frames*, the frames become episodes,
each episode's best image goes to OWLv2 with the same words, and its 2D box plus that
frame's depth becomes a 3D box. Answers print as they are found, because a detector
frame costs the better part of a second and the last episode should not hold up the
first.

    dimos map find grocery.db -q "a basket" -q "bread"
"""

from __future__ import annotations

import json
from pathlib import Path
import time
from typing import Any

import numpy as np
import typer

from dimos.mapping.hyperspace.cli import memory_db_for, open_store, pick_device, pick_stream
from dimos.mapping.hyperspace.detect import (
    DetectConfig,
    Owlv2Boxes,
    RecordingFrames,
    find,
    merge_duplicates,
)
from dimos.mapping.hyperspace.frames import TextTowers, member_streams, spec_of
from dimos.mapping.hyperspace.live import LiveConfig
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def slug_of(text: str) -> str:
    """A filename from a query: ``"a basket"`` -> ``a_basket``."""
    kept = [character if character.isalnum() else "_" for character in text.strip().lower()]
    return "".join(kept).strip("_").replace("__", "_") or "query"


def report(detection: Any) -> str:
    """One line a person can read, for a result that has just arrived."""
    head = f"  #{detection.rank:<2} {detection.episode_frames:4} frames / {detection.episode_span:5.1f}s"
    if not detection.found:
        return f"{head}  --  {detection.note or 'nothing found'}"
    if detection.box3d is None:
        return f"{head}  owl {detection.score:.2f}  2D only ({detection.note})"
    box = detection.box3d
    return (
        f"{head}  owl {detection.score:.2f}  "
        f"at ({box.centre[0]:6.1f}, {box.centre[1]:6.1f}, {box.centre[2]:5.1f}) {box.frame}  "
        f"{box.extent[0]:.2f}x{box.extent[1]:.2f}x{box.extent[2]:.2f} m  "
        f"{box.depth_m:.1f} m away"
    )


def main(
    recording_path: Path = typer.Argument(..., help="the .db or .mcap recording"),
    query: list[str] = typer.Option(..., "-q", "--query", help="what to look for; repeatable"),
    memory_db: Path = typer.Option(
        None, "--memory-db", help="where the index lives (default: the recording itself)"
    ),
    out: Path = typer.Option(
        None, "--out", "-o", help="directory for the evidence sheet, the 3D page and the json"
    ),
    models: str = typer.Option(
        "",
        "--models",
        help="comma-separated member tags to search (default: the cheapest one in the index). "
        "'all' searches every model.",
    ),
    max_episodes: int = typer.Option(12, "--max-episodes", help="episodes to run the detector on"),
    min_episode_frames: int = typer.Option(2, "--min-frames", help="drop runs shorter than this"),
    gap_s: float = typer.Option(1.0, "--gap", help="quiet this long ends an episode (s)"),
    threshold: float = typer.Option(0.5, "--threshold", help="OWLv2's per-box acceptance score"),
    attempts: int = typer.Option(3, "--attempts", help="frames of an episode to try"),
    batch: int = typer.Option(1, "--batch", help="frames per detector forward pass"),
    checkpoint: str = typer.Option("", "--owl", help="an OWLv2 checkpoint other than base"),
    band_m: float = typer.Option(
        0.5, "--depth-band", help="depth spread that is still the object (m)"
    ),
    merge_m: float = typer.Option(
        0.75, "--merge", help="answers whose centres are this close are one place (m)"
    ),
    world_frame: str = typer.Option("odom", "--world-frame"),
    depth2depth: str = typer.Option(
        "auto",
        "--depth2depth",
        help="fill stereo's holes as boxes are placed: 'auto' only when the recording "
        "has no filled-depth stream, '' off, or a checkpoint by name",
    ),
    device: str = typer.Option("auto", "--device"),
    tower_device: str = typer.Option(
        LiveConfig.tower_device,
        "--tower-device",
        help="where the text towers run; 'cpu' keeps the card for the detector",
    ),
    contrast: bool = typer.Option(
        DetectConfig.contrast,
        "--contrast/--no-contrast",
        help="subtract generic floor/wall/ceiling prompts from every patch score",
    ),
    no_scene: bool = typer.Option(False, "--no-scene", help="skip the point cloud in the 3D page"),
) -> None:
    recording_path = recording_path.expanduser()
    index_path = (memory_db or memory_db_for(recording_path)).expanduser()
    out = (out or recording_path.parent / "hyperspace_find").expanduser()

    recording = open_store(recording_path)
    store = recording if index_path == recording_path else open_store(index_path)

    available = [tag for tag, _ in member_streams(store)]
    if not available:
        raise typer.BadParameter(
            f"{index_path} holds no per-model patch streams; build one with "
            "`dimos map query <recording> --flat`"
        )
    if models == "all":
        wanted = available
    elif models:
        wanted = [tag.strip() for tag in models.split(",") if tag.strip()]
        missing = [tag for tag in wanted if tag not in available]
        if missing:
            raise typer.BadParameter(f"no such model(s) {missing}; the index holds {available}")
    else:
        # The patches only have to rank frames, and the cheapest model does that as well
        # as four of them did -- for a tenth of the search. Ask for more with --models.
        wanted = [min(available, key=len)]

    config = DetectConfig(
        threshold=threshold,
        checkpoint=checkpoint or DetectConfig.checkpoint,
        device=pick_device(device),
        attempts=attempts,
        batch=batch,
        max_episodes=max_episodes,
        min_episode_frames=min_episode_frames,
        episode_gap_s=gap_s,
        depth_band_m=band_m,
        depth2depth=depth2depth,
        world_frame=world_frame,
        contrast=contrast,
    )
    frames = RecordingFrames(
        recording,
        color_stream=pick_stream(recording, None, "color", "image"),
        depth_stream=pick_stream(recording, None, "depth", "image"),
        color_info_stream=pick_stream(recording, None, "camera", "info"),
        depth_info_stream=pick_stream(recording, None, "depth", "camera", "info"),
        config=config,
    )
    boxes = Owlv2Boxes(config)
    # The towers stay off the card on purpose -- see `LiveConfig.tower_device`. Three of
    # them beside OWLv2 does not fit in 8 GB, and they are a second of CPU per query.
    towers = TextTowers(tower_device or config.device or "cpu")

    typer.echo(f"index: {index_path}  models {wanted} of {available}")
    typer.echo(f"detector: {config.checkpoint} on {config.device or 'auto'}")

    # Everything a query needs is loaded before one is asked, so the first answer costs
    # what the tenth does. All three are lazy by default and would otherwise land on
    # whoever asked first -- and the third is not a model at all, it is the recording's
    # transforms and by-stamp index.
    at = time.monotonic()
    for tag in wanted:
        towers.background(spec_of(tag))
    text_warm = time.monotonic() - at
    typer.echo(
        f"warm: text towers {text_warm:.1f}s, detector {boxes.warm():.1f}s, "
        f"recording {frames.warm():.1f}s"
    )

    from dimos.mapping.hyperspace.resident import ResidentIndex

    # The index is read once, here, and every query after is a matrix multiply. There
    # is no second way to search: going through sqlite was fifty times slower.
    held = ResidentIndex()
    members = [(tag, name) for tag, name in member_streams(store) if tag in wanted]
    spent = held.warm(store, members)
    loaded = [held.of(store, tag, name) for tag, name in members]
    typer.echo(
        f"resident: {sum(patches.rows for patches in loaded)} patches, "
        f"{sum(patches.megabytes for patches in loaded):.0f} MB, loaded in {spent:.1f}s"
    )

    summary: dict[str, Any] = {"recording": str(recording_path), "queries": {}}
    # Read once: the same scene backs every query's page, and it is ~700 thumbnail
    # rows placed by tf.
    scene: tuple[Any, Any] | None = None
    for text in query:
        typer.echo(f"\n{text!r}")
        started = time.monotonic()
        found = []
        spent: dict[str, float] = {}
        for detection in find(
            store,
            recording,
            text,
            config=config,
            models=wanted,
            towers=towers,
            frames=frames,
            boxes=boxes,
            keep_images=True,
            resident=held,
            timings=spent,
        ):
            typer.echo(report(detection))
            found.append(detection)
        took = time.monotonic() - started
        placed = sum(1 for detection in found if detection.box3d is not None)
        places = merge_duplicates(found, merge_m)
        typer.echo(
            f"  {len(found)} episodes, {placed} placed in {places} distinct place(s), "
            f"{sum(1 for d in found if not d.found)} refused, {took:.1f}s"
        )
        typer.echo(
            f"  time: search {spent.get('search', 0):.3f}s"
            f" + episodes {spent.get('episodes', 0):.3f}s"
            f" + detector {spent.get('detect', 0):.3f}s"
            f"  (first answer at {spent.get('first_result', 0):.3f}s,"
            f" {int(spent.get('frames_matched', 0))} frames matched)"
        )
        for detection in found:
            if detection.duplicate_of is not None:
                typer.echo(f"    #{detection.rank} is another look at #{detection.duplicate_of}")

        name = slug_of(text)
        if scene is None:
            scene = _scene(store, frames, config, no_scene)
        sheet = _write_artifacts(
            out, name, text, found, scene, recording_path, frames, config.world_frame
        )
        summary["queries"][text] = {
            "seconds": took,
            "episodes": len(found),
            "placed": placed,
            "places": places,
            "detections": [detection.as_dict() for detection in found],
            "artifacts": {key: str(value) for key, value in sheet.items() if value},
        }
        for key, value in sheet.items():
            if value:
                typer.echo(f"  {key}: {value}")

    towers.close()
    out.mkdir(parents=True, exist_ok=True)
    written = out / f"{recording_path.stem}_find.json"
    written.write_text(json.dumps(summary, indent=2))
    typer.echo(f"\nsummary: {written}")


def _scene(
    store: Any, frames: RecordingFrames, config: DetectConfig, no_scene: bool
) -> tuple[Any, Any]:
    """The recording's geometry and route, from the depth thumbnails the index holds."""
    from dimos.mapping.hyperspace import render

    empty = np.zeros((0, 3), dtype=np.float32)
    if no_scene:
        return empty, empty
    frames.load_tf()
    started = time.monotonic()
    points = render.scene_points(store, frames.tf, config.world_frame)
    route = render.trajectory(store, frames.tf, config.world_frame)
    typer.echo(
        f"  scene: {len(points)} points, {len(route)} poses ({time.monotonic() - started:.1f}s)"
    )
    return points, route


def _write_artifacts(
    out: Path,
    name: str,
    text: str,
    found: list[Any],
    scene: tuple[Any, Any],
    recording_path: Path,
    frames: Any = None,
    world_frame: str = "odom",
) -> dict[str, Path | None]:
    from dimos.mapping.hyperspace import render

    out.mkdir(parents=True, exist_ok=True)
    sheet = render.evidence_sheet(out / f"{name}_frames.png", text, found)
    views = [] if frames is None else render.camera_views(found, frames, world_frame)
    page = render.boxes_html(
        out / f"{name}_boxes.html",
        text,
        found,
        scene[0],
        scene[1],
        recording=recording_path.name,
        views=views,
    )
    return {"frames": sheet, "boxes": page}


if __name__ == "__main__":
    typer.run(main)
