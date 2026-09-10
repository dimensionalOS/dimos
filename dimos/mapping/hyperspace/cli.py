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

Reads a memory2 ``.db`` or an ``.mcap``, runs the same pipeline the module runs
over its colour + depth + tf, then writes an rrd holding the scene voxels and,
per query, the voxels that answer it.

    uv run python -m dimos.mapping.hyperspace.cli RECORDING -q "a chair" -o out.rrd

Without ``--model-dir`` the embedder is a stub: the rrd is structurally real but
the highlighted voxels are meaningless. Point it at a SigLIP2 snapshot (and
build the binary with the `siglip` feature) for answers that mean something.
"""

from __future__ import annotations

import json
from pathlib import Path
import shutil
import subprocess
import tempfile
from typing import TYPE_CHECKING, Any

import numpy as np
import typer

from dimos.constants import DIMOS_PROJECT_ROOT

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from dimos.memory.store.base import Store

app = typer.Typer(add_completion=False, help=__doc__)

# Where `cargo build --release` puts the offline driver.
OFFLINE_BIN = DIMOS_PROJECT_ROOT / "target" / "release" / "hyperspace_offline"

TIMELINE = "ts"
# Scene voxels are context, the answer is the subject: keep the scene quiet.
SCENE_COLOR = (110, 120, 135)
SCENE_ALPHA_MIN = 40


def open_store(path: Path) -> Store:
    """Open a recording, picking the store from the file extension."""
    if path.suffix == ".mcap":
        from dimos.memory.store.mcap import McapStore

        store = McapStore(path=str(path))
    elif path.suffix == ".db":
        from dimos.memory.store.sqlite import SqliteStore

        store = SqliteStore(path=str(path), must_exist=True)
    else:
        raise typer.BadParameter(
            f"expected a .db or .mcap recording, got {path.suffix or path.name!r}"
        )
    store.start()
    return store


def pick_stream(store: Store, wanted: str | None, *keywords: str) -> str:
    """Name the stream to use: the one asked for, else the best keyword match."""
    names = list(store.streams.keys())
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


def export_frames(
    store: Store,
    out_dir: Path,
    *,
    color_stream: str,
    depth_stream: str,
    color_info_stream: str,
    depth_info_stream: str,
    tf_stream: str,
    hz: float,
    max_seconds: float,
) -> int:
    """Write the frame directory the offline driver reads. Returns frame count."""
    import cv2

    (out_dir / "color").mkdir(parents=True, exist_ok=True)
    (out_dir / "depth").mkdir(parents=True, exist_ok=True)

    def camera_info(stream_name: str) -> dict[str, Any]:
        info = next(iter(store.streams[stream_name].order_by(TIMELINE))).data
        matrix = np.asarray(info.K, dtype=float).reshape(3, 3)
        return {
            "frame_id": info.header.frame_id,
            "width": int(info.width),
            "height": int(info.height),
            "fx": float(matrix[0, 0]),
            "fy": float(matrix[1, 1]),
            "cx": float(matrix[0, 2]),
            "cy": float(matrix[1, 2]),
            "distortion_model": info.distortion_model or "plumb_bob",
            "distortion": [float(value) for value in (info.D or [])],
        }

    (out_dir / "intrinsics.json").write_text(
        json.dumps(
            {"color": camera_info(color_info_stream), "depth": camera_info(depth_info_stream)},
            indent=1,
        )
    )

    transforms = 0
    with (out_dir / "tf.jsonl").open("w") as handle:
        for observation in store.streams[tf_stream].order_by(TIMELINE):
            for stamped in observation.data.transforms:
                stamp = stamped.header.stamp
                translation = stamped.transform.translation
                rotation = stamped.transform.rotation
                handle.write(
                    json.dumps(
                        {
                            "parent": stamped.header.frame_id,
                            "child": stamped.child_frame_id,
                            "ts": float(stamp.sec) + float(stamp.nsec) * 1e-9,
                            "t": [float(translation.x), float(translation.y), float(translation.z)],
                            "q": [
                                float(rotation.x),
                                float(rotation.y),
                                float(rotation.z),
                                float(rotation.w),
                            ],
                        }
                    )
                    + "\n"
                )
                transforms += 1
    typer.echo(f"tf: {transforms} transforms")

    index: list[dict[str, Any]] = []
    min_interval = 1.0 / hz if hz > 0 else 0.0
    last_kept = -np.inf
    first_ts: float | None = None
    colors = store.streams[color_stream].order_by(TIMELINE)
    depths = store.streams[depth_stream].order_by(TIMELINE)
    for pair in colors.align(depths, tolerance=0.05):
        color_obs, depth_obs = pair.data[0], pair.data[1]
        stamp = float(color_obs.ts)
        if first_ts is None:
            first_ts = stamp
        if stamp - first_ts > max_seconds:
            break
        if stamp - last_kept < min_interval:
            continue
        last_kept = stamp
        color = color_obs.data
        depth = depth_obs.data
        rgb = color.to_numpy() if hasattr(color, "to_numpy") else np.asarray(color.data)
        rgb = np.asarray(rgb).reshape(color.height, color.width, -1)[:, :, :3]
        name = f"{len(index):05d}"
        cv2.imwrite(
            str(out_dir / "color" / f"{name}.jpg"), rgb[:, :, ::-1], [cv2.IMWRITE_JPEG_QUALITY, 92]
        )
        millimetres = np.asarray(depth.to_numpy() if hasattr(depth, "to_numpy") else depth.data)
        millimetres = millimetres.reshape(depth.height, depth.width).astype("<u2", copy=False)
        (out_dir / "depth" / f"{name}.u16").write_bytes(millimetres.tobytes())
        index.append(
            {
                "name": name,
                "ts": stamp,
                "depth_ts": float(depth_obs.ts),
                "color_frame": color.header.frame_id,
                "depth_frame": depth.header.frame_id,
                "width": int(color.width),
                "height": int(color.height),
            }
        )
        if len(index) % 100 == 0:
            typer.echo(f"exported {len(index)} frames ({stamp - first_ts:.0f}s)")
    (out_dir / "index.json").write_text(json.dumps(index, indent=1))
    return len(index)


def run_offline(
    export_dir: Path,
    queries: list[str],
    *,
    frame: str,
    voxel_size: float,
    model_dir: str,
    depth_weights: str,
    cuda: bool,
) -> dict[str, Any]:
    """Ingest the export and score the queries. Returns the driver's JSON."""
    if not OFFLINE_BIN.exists():
        raise typer.BadParameter(
            f"{OFFLINE_BIN} is missing; build it with\n"
            f"    cd {DIMOS_PROJECT_ROOT / 'dimos/mapping/hyperspace/rust'} && cargo build --release"
        )
    command = [
        str(OFFLINE_BIN),
        "--export",
        str(export_dir),
        "--frame",
        frame,
        "--voxel-size",
        str(voxel_size),
    ]
    for query in queries:
        command += ["--query", query]
    if model_dir:
        command += ["--model-dir", model_dir]
    if depth_weights:
        command += ["--depth-weights", depth_weights]
    if cuda:
        command.append("--cuda")
    typer.echo(" ".join(command))
    finished = subprocess.run(command, capture_output=True, text=True, check=False)
    if finished.returncode != 0:
        typer.echo(finished.stderr, err=True)
        raise typer.Exit(finished.returncode)
    typer.echo(finished.stderr.strip())
    return json.loads(finished.stdout)


def heat_colors(scores: NDArray[np.float64]) -> NDArray[np.uint8]:
    """Dark red at the threshold through orange to white at the peak."""
    ramp = np.array(
        [[120, 20, 0], [230, 90, 10], [255, 190, 60], [255, 255, 245]], dtype=np.float64
    )
    position = np.clip(scores, 0.0, 1.0) * (len(ramp) - 1)
    low = np.floor(position).astype(int)
    high = np.minimum(low + 1, len(ramp) - 1)
    blend = (position - low)[:, None]
    return (ramp[low] * (1 - blend) + ramp[high] * blend).astype(np.uint8)


def write_rrd(result: dict[str, Any], out: Path, *, recording: Path, cutoff: float) -> None:
    """One rrd: the scene once, then a query's voxels on their own entity."""
    import rerun as rr

    from dimos.visualization.rerun.init import rerun_init

    voxel_size = float(result["voxel_size"])
    half = voxel_size * 0.5
    rerun_init("hyperspace")

    scene = np.asarray(result["scene"], dtype=np.float64).reshape(-1, 4)
    if len(scene):
        centers = (scene[:, :3] + 0.5) * voxel_size
        counts = scene[:, 3]
        alpha = (SCENE_ALPHA_MIN + 160 * counts / max(counts.max(), 1.0)).astype(np.uint8)
        colors = np.column_stack([np.tile(SCENE_COLOR, (len(scene), 1)).astype(np.uint8), alpha])
        rr.log(
            "world/scene",
            rr.Boxes3D(
                centers=centers, half_sizes=np.full((len(scene), 3), half * 0.9), colors=colors
            ),
            static=True,
        )

    poses = result.get("keyframes", [])
    if poses:
        path = np.asarray([pose["t"] for pose in poses], dtype=np.float64)
        rr.log(
            "world/keyframes",
            rr.LineStrips3D([path], colors=[(94, 160, 255)], radii=0.02),
            static=True,
        )
        rr.log(
            "world/keyframes/positions",
            rr.Points3D(positions=path, colors=[(94, 160, 255)], radii=0.05),
            static=True,
        )

    for answer in result["queries"]:
        voxels = np.asarray(answer["voxels"], dtype=np.float64).reshape(-1, 4)
        keep = voxels[:, 3] >= cutoff if len(voxels) else np.zeros(0, dtype=bool)
        voxels = voxels[keep]
        entity = f"world/query/{answer['text'].replace('/', ' ')}"
        if not len(voxels):
            typer.echo(f"{answer['text']!r}: nothing above {cutoff}")
            continue
        centers = (voxels[:, :3] + 0.5) * voxel_size
        scores = voxels[:, 3]
        # Size with the score too, so the peak reads as the peak from any angle.
        sizes = np.repeat(half * (0.55 + 0.45 * scores)[:, None], 3, axis=1)
        rr.log(
            entity,
            rr.Boxes3D(
                centers=centers,
                half_sizes=sizes,
                colors=heat_colors(scores),
                fill_mode=rr.components.FillMode.Solid,
            ),
            static=True,
        )
        best = centers[int(np.argmax(scores))]
        rr.log(
            f"{entity}/best",
            rr.Points3D(
                positions=[best], colors=[(255, 255, 255)], radii=0.06, labels=[answer["text"]]
            ),
            static=True,
        )
        typer.echo(
            f"{answer['text']!r}: {len(voxels)} voxels above {cutoff}, "
            f"best at ({best[0]:.2f}, {best[1]:.2f}, {best[2]:.2f})"
        )

    rr.log(
        "meta",
        rr.TextDocument(
            f"recording: {recording}\n"
            f"frame: {result['frame']}\n"
            f"voxel size: {voxel_size} m\n"
            f"keyframes: {len(poses)}\n"
            f"ingest: {json.dumps(result.get('ingest', {}))}\n"
            + "\n".join(
                f"query {answer['text']!r}: {json.dumps(answer['stats'])}"
                for answer in result["queries"]
            )
        ),
        static=True,
    )
    rr.save(str(out))


@app.command()
def main(
    recording: Path = typer.Argument(..., help="A memory2 .db or an .mcap"),
    query: list[str] = typer.Option(..., "--query", "-q", help="Text query; repeat for several"),
    out: Path = typer.Option(Path("hyperspace.rrd"), "--out", "-o", help="Where to write the rrd"),
    hz: float = typer.Option(5.0, help="Colour frames per second to ingest"),
    max_seconds: float = typer.Option(1e9, help="Stop after this much of the recording"),
    frame: str = typer.Option("odom", help="Frame to answer in"),
    voxel_size: float = typer.Option(0.1, help="Voxel edge length, meters"),
    cutoff: float = typer.Option(0.3, help="Hide answer voxels scoring below this"),
    model_dir: str = typer.Option(
        "", help="SigLIP2 snapshot directory; empty uses the stub embedder"
    ),
    depth_weights: str = typer.Option(
        "", help="depth2depth weights directory; empty uses raw depth"
    ),
    cuda: bool = typer.Option(False, help="Run the models on CUDA"),
    color_stream: str = typer.Option("", help="Colour image stream (auto-detected by name)"),
    depth_stream: str = typer.Option("", help="Depth image stream (auto-detected by name)"),
    color_info_stream: str = typer.Option("", help="Colour camera_info stream"),
    depth_info_stream: str = typer.Option("", help="Depth camera_info stream"),
    tf_stream: str = typer.Option("tf", help="Transform stream"),
    keep_export: bool = typer.Option(False, help="Keep the intermediate frame directory"),
) -> None:
    """Query a recording and write an rrd with the answer highlighted."""
    store = open_store(recording)
    color = pick_stream(store, color_stream or None, "color", "image")
    depth = pick_stream(store, depth_stream or None, "depth", "image")
    color_info = pick_stream(store, color_info_stream or None, "camera_info")
    depth_info = pick_stream(store, depth_info_stream or None, "depth", "camera_info")
    typer.echo(
        f"streams: color={color} depth={depth} info={color_info}/{depth_info} tf={tf_stream}"
    )

    export_dir = Path(tempfile.mkdtemp(prefix="hyperspace_export_"))
    try:
        frames = export_frames(
            store,
            export_dir,
            color_stream=color,
            depth_stream=depth,
            color_info_stream=color_info,
            depth_info_stream=depth_info,
            tf_stream=tf_stream,
            hz=hz,
            max_seconds=max_seconds,
        )
        if frames == 0:
            raise typer.BadParameter(
                "no colour frames paired with a depth frame; check --color-stream/--depth-stream"
            )
        typer.echo(f"exported {frames} frames to {export_dir}")
        result = run_offline(
            export_dir,
            list(query),
            frame=frame,
            voxel_size=voxel_size,
            model_dir=model_dir,
            depth_weights=depth_weights,
            cuda=cuda,
        )
        write_rrd(result, out, recording=recording, cutoff=cutoff)
        typer.echo(f"wrote {out}")
    finally:
        if keep_export:
            typer.echo(f"kept {export_dir}")
        else:
            shutil.rmtree(export_dir, ignore_errors=True)


if __name__ == "__main__":
    app()
