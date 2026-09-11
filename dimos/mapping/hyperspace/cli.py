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

"""Ask a recording a question and get an rrd back.

    dimos map query RECORDING -q "a traffic cone" [-q "a chair"] [-o out.rrd]

Reads a memory2 ``.db`` or an ``.mcap``, runs the same ingest the live module
runs over its colour + depth + tf (keyframe gate, SigLIP2 patches, depth
pairing) into a memory db next to the recording, answers each query against
that db, and writes an rrd with the scene in grey and the answer highlighted.
Without ``-o`` the rrd goes to a temp file and opens in Rerun. The memory db
is kept, so a second run with new questions skips the embedding.
"""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import tempfile
import time
from typing import TYPE_CHECKING, Any

import numpy as np
import typer

from dimos.mapping.hyperspace import patches as hs
from dimos.mapping.hyperspace.embedder import DEFAULT_MEMBERS, PatchEnsemble
from dimos.mapping.hyperspace.ingest import (
    KEYFRAME_STREAM,
    IngestConfig,
    PatchIngestor,
    transform_to_matrix,
)
from dimos.mapping.hyperspace.module import store_members
from dimos.mapping.hyperspace.query import HyperspaceQuery
from dimos.mapping.hyperspace.refine import METHODS, refine_config_of
from dimos.memory.tf import StreamTF

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from dimos.memory.store.base import Store

app = typer.Typer(add_completion=False, help=__doc__)

TIMELINE = "ts"
SCENE_COLOR = (110, 120, 135)
SCENE_ALPHA_MIN = 40


def open_store(path: Path, *, must_exist: bool = True) -> Store:
    """Open a recording, picking the store from the file extension."""
    if path.suffix == ".mcap":
        from dimos.memory.store.mcap import McapStore

        store = McapStore(path=str(path))
    elif path.suffix == ".db":
        from dimos.memory.store.sqlite import SqliteStore

        store = SqliteStore(path=str(path), must_exist=must_exist)
    else:
        raise typer.BadParameter(
            f"expected a .db or .mcap recording, got {path.suffix or path.name!r}"
        )
    store.start()
    return store


def pick_stream(store: Store, wanted: str | None, *keywords: str) -> str:
    """Name the stream to use: the one asked for, else the best keyword match."""
    names = store.list_streams()
    if wanted:
        if wanted not in names:
            raise typer.BadParameter(f"no stream {wanted!r} in the recording; have {sorted(names)}")
        return wanted
    matches = [name for name in names if all(word in name.lower() for word in keywords)]
    if not matches:
        raise typer.BadParameter(
            f"no stream matching {'+'.join(keywords)} in the recording; have {sorted(names)}"
        )
    # Prefer the shortest match: "color_image" over "color_image_camera_info".
    return min(matches, key=len)


def pick_device(device: str) -> str:
    if device != "auto":
        return device
    import torch

    if torch.cuda.is_available():
        return "cuda"
    if torch.backends.mps.is_available():
        return "mps"
    return "cpu"


def ingest(
    recording: Store,
    memory: Store,
    model: PatchEnsemble,
    *,
    color_stream: str,
    depth_stream: str,
    color_info_stream: str,
    depth_info_stream: str,
    tf_stream: str,
    hz: float,
    max_seconds: float,
    config: IngestConfig,
) -> dict[str, int]:
    """Run the live module's ingest over a recording. Returns its stats."""
    recorded_tf = StreamTF.from_store(recording, tf_stream)

    def lookup(target: str, source: str, ts: float) -> NDArray[np.float64] | None:
        if recorded_tf is None:
            return None
        transform = recorded_tf.get(target, source, ts, warn=False)
        return None if transform is None else transform_to_matrix(transform)

    ingestor = PatchIngestor(memory, model, config, lookup=lookup)
    for name in (color_info_stream, depth_info_stream):
        first = next(iter(recording.streams[name].order_by(TIMELINE)), None)
        if first is None:
            raise typer.BadParameter(f"stream {name!r} is empty")
        ingestor.add_camera_info(first.data)
    colors = recording.streams[color_stream].order_by(TIMELINE)
    depths = recording.streams[depth_stream].order_by(TIMELINE)
    start_ts = float(colors.first().ts)
    # Only the tf the slice can use; a whole recording's tf is hundreds of
    # thousands of messages the query side would otherwise decode.
    transforms = 0
    for observation in recording.streams[tf_stream].order_by(TIMELINE):
        stamp = float(observation.ts)
        if stamp < start_ts - 5.0 or stamp > start_ts + max_seconds + 5.0:
            continue
        ingestor.add_tf(observation.data, ts=stamp)
        transforms += 1
    typer.echo(f"tf: {transforms} messages")

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
            typer.echo(
                f"{stamp - first_ts:.0f}s: {ingestor.stats['embedded']} embedded, "
                f"{ingestor.stats['kept']} kept ({time.monotonic() - started:.0f}s)"
            )
    ingestor.flush()
    return dict(ingestor.stats)


def heat_colors(scores: NDArray[np.float64]) -> NDArray[np.uint8]:
    """Dark red at the cutoff through orange to white at the peak."""
    ramp = np.array(
        [[120, 20, 0], [230, 90, 10], [255, 190, 60], [255, 255, 245]], dtype=np.float64
    )
    position = np.clip(scores, 0.0, 1.0) * (len(ramp) - 1)
    low = np.floor(position).astype(int)
    high = np.minimum(low + 1, len(ramp) - 1)
    blend = (position - low)[:, None]
    return (ramp[low] * (1 - blend) + ramp[high] * blend).astype(np.uint8)


def channel_colors(patch: NDArray[np.float64], segment: NDArray[np.float64]) -> NDArray[np.uint8]:
    """Blue for the patch channel, red for the segment channel, purple where
    both are hot; brightness follows the score."""
    red = 60 + 195 * np.clip(segment, 0.0, 1.0)
    blue = 60 + 195 * np.clip(patch, 0.0, 1.0)
    green = 30 + 40 * np.minimum(patch, segment)
    return np.column_stack([red, green, blue]).astype(np.uint8)


def send_blueprint(scene_centres: NDArray[np.float64], queries: dict[str, str]) -> None:
    """One top-down 3D view per query (its voxels over the scene), in a grid,
    the eye fixed above the scene so every view opens on the whole map."""
    import rerun as rr
    import rerun.blueprint as rrb

    low, high = np.percentile(scene_centres, [2, 98], axis=0)
    centre = (low + high) / 2
    height = float(max(high[0] - low[0], high[1] - low[1], 4.0)) * 1.1
    eye = rrb.archetypes.EyeControls3D(
        kind="FirstPerson",
        position=[centre[0], centre[1], centre[2] + height],
        look_target=centre,
        eye_up=[0.0, 1.0, 0.0],
    )
    views = [
        rrb.Spatial3DView(
            name=text,
            origin="world",
            contents=["+ world/scene/**", "+ world/keyframes/**", f"+ {entity}/**"],
            eye_controls=eye,
        )
        for text, entity in queries.items()
    ]
    rr.send_blueprint(rrb.Blueprint(rrb.Grid(*views), collapse_panels=True))


def write_rrd(
    engine: HyperspaceQuery,
    answers: list[dict[str, Any]],
    out: Path,
    *,
    recording: Path,
    cutoff: float,
    ingest_stats: dict[str, int],
) -> None:
    """One rrd: the scene once, the camera path, then each query on its own entity."""
    import rerun as rr

    from dimos.visualization.rerun.init import rerun_init

    size = engine.voxel_size
    half = size * 0.5
    rerun_init("hyperspace")

    scene = engine.scene_voxels()
    entities = {
        answer["id"]: "world/query/"
        + (
            "".join(c if c.isalnum() else "_" for c in answer["text"]).strip("_")
            or str(answer["id"])
        )
        for answer in answers
    }
    if scene:
        centres = (np.asarray([i for i, _ in scene], dtype=np.float64) + 0.5) * size
        send_blueprint(centres, {answer["text"]: entities[answer["id"]] for answer in answers})
        counts = np.asarray([n for _, n in scene], dtype=np.float64)
        alpha = (SCENE_ALPHA_MIN + 160 * counts / max(counts.max(), 1.0)).astype(np.uint8)
        colors = np.column_stack([np.tile(SCENE_COLOR, (len(scene), 1)).astype(np.uint8), alpha])
        rr.log(
            "world/scene",
            rr.Boxes3D(
                centers=centres, half_sizes=np.full((len(scene), 3), half * 0.9), colors=colors
            ),
            static=True,
        )

    place = engine.placer(engine.world_frame)
    path = []
    for obs in engine.store.stream(KEYFRAME_STREAM, dict).order_by(TIMELINE):
        payload = obs.data
        pose = place(
            hs.Keyframe(
                id=obs.id,
                camera_frame=payload["camera_frame"],
                ts=float(payload["ts"]),
                rows=1,
                cols=1,
                intrinsics=hs.Intrinsics(**payload["intrinsics"]),
                patch_depth=np.zeros(0, np.float32),
            )
        )
        if pose is not None:
            path.append(pose[:3, 3])
    if path:
        rr.log(
            "world/keyframes",
            rr.LineStrips3D([np.asarray(path)], colors=[(94, 160, 255)], radii=0.02),
            static=True,
        )
        rr.log(
            "world/keyframes/positions",
            rr.Points3D(positions=np.asarray(path), colors=[(94, 160, 255)], radii=0.05),
            static=True,
        )

    for answer in answers:
        result: hs.Heatmap = answer["heatmap"]
        centres = result.centres()
        scores = result.scores()
        keep = scores >= cutoff
        entity = entities[answer["id"]]
        if not keep.any():
            typer.echo(f"{answer['text']!r}: nothing above {cutoff}")
            continue
        centres, scores = centres[keep], scores[keep]
        sizes = np.repeat(half * (0.55 + 0.45 * scores)[:, None], 3, axis=1)
        if result.channels:
            kept = [index for (index, _), k in zip(result.voxels, keep, strict=True) if k]
            per_channel = np.asarray([result.channels[index] for index in kept], dtype=np.float64)
            colors = channel_colors(per_channel[:, 0], per_channel[:, 1])
        else:
            colors = heat_colors(scores)
        rr.log(
            entity,
            rr.Boxes3D(
                centers=centres,
                half_sizes=sizes,
                colors=colors,
                fill_mode=rr.components.FillMode.Solid,
            ),
            static=True,
        )
        best = centres[int(np.argmax(scores))]
        rr.log(
            f"{entity}/best",
            rr.Points3D(
                positions=[best], colors=[(255, 255, 255)], radii=0.06, labels=[answer["text"]]
            ),
            static=True,
        )
        typer.echo(
            f"{answer['text']!r}: {int(keep.sum())} voxels above {cutoff}, "
            f"best at ({best[0]:.2f}, {best[1]:.2f}, {best[2]:.2f})"
        )

    rr.log(
        "meta",
        rr.TextDocument(
            f"recording: {recording}\n"
            f"frame: {engine.world_frame}\n"
            f"voxel size: {size} m\n"
            f"keyframes: {len(path)}\n"
            f"ingest: {json.dumps(ingest_stats)}\n"
            + "\n".join(f"query {a['text']!r}: {json.dumps(a['stats'])}" for a in answers)
        ),
        static=True,
    )
    rr.save(str(out))


@app.command()
def main(
    recording: Path = typer.Argument(..., help="A memory2 .db or an .mcap"),
    query: list[str] = typer.Option(..., "--query", "-q", help="Text query; repeat for several"),
    out: Path | None = typer.Option(
        None, "--out", "-o", help="Where to write the rrd; omitted = a temp file, opened in rerun"
    ),
    memory_db: Path | None = typer.Option(
        None, help="Memory db for keyframes + patches (default: <recording>.hyperspace.db)"
    ),
    reuse: bool = typer.Option(True, help="Reuse an existing memory db instead of re-embedding"),
    hz: float = typer.Option(5.0, help="Colour frames per second to consider"),
    max_seconds: float = typer.Option(1e9, help="Stop after this much of the recording"),
    frame: str = typer.Option("odom", help="Frame to answer in"),
    voxel_size: float = typer.Option(0.1, help="Voxel edge length, meters"),
    cutoff: float = typer.Option(0.3, help="Hide answer voxels scoring below this"),
    cap_near: float = typer.Option(0.99, help="Pyramids start at this fraction of the patch depth"),
    cap_far: float = typer.Option(1.01, help="Pyramids end at this fraction of the patch depth"),
    refine: str = typer.Option(
        "default",
        help="Refinement steps, comma separated, from "
        + ", ".join(METHODS)
        + "; 'default' = QueryConfig.refine, 'none' = the raw map",
    ),
    models: str = typer.Option(
        ",".join(DEFAULT_MEMBERS),
        help="Comma separated SigLIP2 checkpoints (ids or dirs; NaFlex ones as id@budget). "
        "One = the classic single model; several = an ensemble pooled per cell. "
        "When reusing a db its own members are used.",
    ),
    pool: str = typer.Option("2nd", help="Ensemble pooling: min, 2nd (second lowest) or mean"),
    min_frames: int = typer.Option(
        1,
        help="Keyframes a voxel must be seen from (refine 'support'); 2 was the single-model setting",
    ),
    pooled_hot_threshold: float = typer.Option(
        0.02, help="Hot threshold on the pooled score (use ~0.005 with --pool min)"
    ),
    device: str = typer.Option("auto", help="cuda, mps, cpu, or auto"),
    max_depth: float = typer.Option(10.0, help="Depth readings beyond this many meters are holes"),
    color_stream: str = typer.Option("", help="Colour image stream (auto-detected by name)"),
    depth_stream: str = typer.Option("", help="Depth image stream (auto-detected by name)"),
    color_info_stream: str = typer.Option("", help="Colour camera_info stream"),
    depth_info_stream: str = typer.Option("", help="Depth camera_info stream"),
    tf_stream: str = typer.Option("tf", help="Transform stream"),
) -> None:
    """Query a recording and write an rrd with the answer highlighted."""
    source = open_store(recording)
    color = pick_stream(source, color_stream or None, "color", "image")
    depth = pick_stream(source, depth_stream or None, "depth", "image")
    color_info = pick_stream(source, color_info_stream or None, "camera_info")
    depth_info = pick_stream(source, depth_info_stream or None, "depth", "camera_info")
    typer.echo(
        f"streams: color={color} depth={depth} info={color_info}/{depth_info} tf={tf_stream}"
    )

    memory_path = memory_db or recording.with_suffix(".hyperspace.db")
    fresh = not (reuse and memory_path.exists())
    memory = open_store(memory_path, must_exist=False)
    device = pick_device(device)
    stats: dict[str, int] = {}
    specs = [spec.strip() for spec in models.split(",") if spec.strip()]
    if fresh:
        model = PatchEnsemble(specs, device=device, towers="vision")
        model.start()
        typer.echo(f"embedding with {model.tags} on {device} -> {memory_path}")
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
            max_seconds=max_seconds,
            config=IngestConfig(
                gate=hs.KeyframeGateConfig(max_angular_velocity=None),
                max_depth_m=max_depth,
            ),
        )
        typer.echo(f"ingest: {stats}")
        model.stop()
    else:
        typer.echo(f"reusing {memory_path} (pass --no-reuse to re-embed)")
        specs = store_members(memory) or specs
    text_model = PatchEnsemble(specs, device=device, towers="text")
    text_model.start()
    query_config = hs.QueryConfig(
        cap_near=cap_near, cap_far=cap_far, pool=pool, pooled_hot_threshold=pooled_hot_threshold
    )
    engine = HyperspaceQuery(
        memory,
        text_model.embed_text,
        query_config,
        world_frame=frame,
        voxel_size=voxel_size,
        refine_config=refine_config_of(refine, query_config.refine, cutoff, min_frames),
    )
    answers = []
    for index, text in enumerate(query, start=1):
        started = time.monotonic()
        answer = engine.answer(text, index)
        typer.echo(
            f"{text!r}: {answer['voxels']} voxels in {(time.monotonic() - started) * 1000:.0f} ms "
            f"{answer['stats']}"
        )
        for cluster in answer["heatmap"].clusters[:10]:
            typer.echo(
                f"  cluster {cluster.rank}: score {cluster.score:.2f}, {cluster.voxels} voxels, "
                f"centre ({cluster.centre[0]:.2f}, {cluster.centre[1]:.2f}, {cluster.centre[2]:.2f}), "
                f"extent {tuple(round(e, 1) for e in cluster.extent)} m"
            )
        answers.append(answer)

    open_after = out is None
    if out is None:
        slug = "_".join("".join(c if c.isalnum() else "_" for c in q).strip("_") for q in query)
        out = Path(tempfile.gettempdir()) / f"hyperspace_{recording.stem}_{slug[:60]}.rrd"
    write_rrd(engine, answers, out, recording=recording, cutoff=cutoff, ingest_stats=stats)
    typer.echo(f"wrote {out}")
    if open_after:
        # Detached: the viewer outlives this command. If a viewer is already
        # up on rerun's default port the file streams into it instead.
        subprocess.Popen(["rerun", str(out)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        typer.echo("opened in rerun")


if __name__ == "__main__":
    app()
