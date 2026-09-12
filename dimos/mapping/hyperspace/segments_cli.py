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

"""Segment a recording and write an mcap with the overlays, to judge the segmenter by eye.

    dimos map segments RECORDING [-o out.mcap] [--hz 2] [--max-seconds 60]

Runs the live module's segmentation (SegFormer + flat-plane gate) over the
recording's colour + depth and writes three ROS 2 image topics into an mcap
Foxglove or Lichtblick can open: ``/segments/color`` (the input),
``/segments/overlay`` (class colours blended in, demoted pixels dark grey) and
``/segments/flat`` (white where the depth is flat). Segment records also go
into a memory db next to the recording, like the live module writes.
"""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import tempfile
import time
from typing import TYPE_CHECKING

import numpy as np
import typer

from dimos.mapping.hyperspace import segmenter as seg
from dimos.mapping.hyperspace.cli import TIMELINE, open_store, pick_device, pick_stream
from dimos.mapping.hyperspace.embedder import SIGLIP2_MODEL_NAME, SigLIP2Patches
from dimos.mapping.hyperspace.ingest import transform_to_matrix
from dimos.mapping.hyperspace.segments import SegmentIngestConfig, SegmentIngestor
from dimos.memory.tf import StreamTF
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat

if TYPE_CHECKING:
    from numpy.typing import NDArray

COMPRESSED_IMAGE = "sensor_msgs/msg/CompressedImage"
COMPRESSED_IMAGE_MSGDEF = """\
std_msgs/Header header
string format
uint8[] data
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
"""


class OverlayMcap:
    """Three jpeg image topics in ROS 2 CDR, so any mcap viewer shows them."""

    def __init__(self, path: Path) -> None:
        from mcap_ros2.writer import Writer

        self.file = open(path, "wb")
        self.writer = Writer(self.file)
        self.schema = self.writer.register_msgdef(COMPRESSED_IMAGE, COMPRESSED_IMAGE_MSGDEF)
        self.count = 0

    def write(self, topic: str, rgb: NDArray[np.uint8], ts: float, frame_id: str) -> None:
        image = Image.from_numpy(np.ascontiguousarray(rgb), format=ImageFormat.RGB)
        stamp = int(ts * 1e9)
        self.writer.write_message(
            topic,
            self.schema,
            {
                "header": {
                    "stamp": {"sec": stamp // 10**9, "nanosec": stamp % 10**9},
                    "frame_id": frame_id,
                },
                "format": "jpeg",
                "data": image.to_jpeg_bytes(quality=85),
            },
            log_time=stamp,
            publish_time=stamp,
        )
        self.count += 1

    def close(self) -> None:
        self.writer.finish()
        self.file.close()


def main(
    recording: Path = typer.Argument(..., help="A memory2 .db or an .mcap"),
    out: Path | None = typer.Option(
        None,
        "--out",
        "-o",
        help="Where to write the mcap; omitted = a temp file, opened in Foxglove",
    ),
    memory_db: Path | None = typer.Option(
        None, help="Memory db for the segment records (default: <recording>.hyperspace.db)"
    ),
    hz: float = typer.Option(2.0, help="Frames per second to segment"),
    max_seconds: float = typer.Option(1e9, help="Stop after this much of the recording"),
    seek: float = typer.Option(0.0, help="Skip this many seconds of the recording first"),
    segmenter_name: str = typer.Option(
        seg.SEGFORMER_MODEL_NAME, help="Segmenter: HF id or local directory"
    ),
    model_name: str = typer.Option(
        SIGLIP2_MODEL_NAME, help="SigLIP2 snapshot for the label embeddings; '' = skip"
    ),
    device: str = typer.Option("auto", help="cuda, mps, cpu, or auto"),
    max_depth: float = typer.Option(10.0, help="Depth readings beyond this many meters are holes"),
    flat_rms_max: float = typer.Option(0.015, help="Plane fit RMS (m) a flat surface may have"),
    flat_max_deviation: float = typer.Option(
        0.04, help="Largest plane deviation (m) a flat surface may have"
    ),
    flat_noise_per_m2: float = typer.Option(
        0.003, help="Grow both limits with depth squared at this rate (m per m^2); 0 = fixed"
    ),
    no_gate: bool = typer.Option(False, help="Skip the flat-plane gate (raw segmenter output)"),
    color_stream: str = typer.Option("", help="Colour image stream (auto-detected by name)"),
    depth_stream: str = typer.Option("", help="Depth image stream (auto-detected by name)"),
    color_info_stream: str = typer.Option("", help="Colour camera_info stream"),
    depth_info_stream: str = typer.Option("", help="Depth camera_info stream"),
    tf_stream: str = typer.Option("tf", help="Transform stream"),
) -> None:
    """Segment a recording and write an mcap with the overlays."""
    source = open_store(recording)
    color = pick_stream(source, color_stream or None, "color", "image")
    depth = pick_stream(source, depth_stream or None, "depth", "image")
    color_info = pick_stream(source, color_info_stream or None, "camera_info")
    depth_info = pick_stream(source, depth_info_stream or None, "depth", "camera_info")
    typer.echo(
        f"streams: color={color} depth={depth} info={color_info}/{depth_info} tf={tf_stream}"
    )

    chosen = pick_device(device)
    typer.echo(f"loading {segmenter_name} on {chosen}")
    flatness = seg.FlatnessConfig(
        rms_max=flat_rms_max, max_deviation=flat_max_deviation, noise_per_m2=flat_noise_per_m2
    )
    segmenter = seg.SegFormerSegmenter(
        seg.SegmenterConfig(model_name=segmenter_name, device=chosen, flatness=flatness)
    )
    if no_gate:
        segmenter.structural_ids = set()
    embed_text = None
    text_model = None
    if model_name:
        text_model = SigLIP2Patches(model_name=model_name, device="cpu", towers="text")
        text_model.start()
        embed_text = lambda text: text_model.embed_text_array(text)[0]  # noqa: E731

    memory_path = memory_db or recording.with_suffix(recording.suffix + ".hyperspace.db")
    memory = open_store(memory_path, must_exist=False)
    recorded_tf = StreamTF.from_store(source, tf_stream)

    def lookup(target: str, frame: str, ts: float) -> NDArray[np.float64] | None:
        if recorded_tf is None:
            return None
        transform = recorded_tf.get(target, frame, ts, warn=False)
        return None if transform is None else transform_to_matrix(transform)

    ingestor = SegmentIngestor(
        memory,
        segmenter,
        SegmentIngestConfig(
            min_frame_interval_s=1.0 / hz if hz > 0 else 0.0, max_depth_m=max_depth
        ),
        embed_text=embed_text,
        lookup=lookup,
    )
    for name in (color_info, depth_info):
        first = next(iter(source.streams[name].order_by(TIMELINE)), None)
        if first is None:
            raise typer.BadParameter(f"stream {name!r} is empty")
        ingestor.add_camera_info(first.data)

    out_path = out or Path(tempfile.mkdtemp(prefix="hyperspace_")) / "segments.mcap"
    mcap = OverlayMcap(out_path)
    colors = source.streams[color].order_by(TIMELINE)
    depths = source.streams[depth].order_by(TIMELINE)
    first_ts: float | None = None
    started = time.monotonic()
    label_counts: dict[str, int] = {}
    demoted: list[float] = []
    for pair in colors.align(depths, tolerance=ingestor.config.depth_max_dt):
        color_obs, depth_obs = pair.data[0], pair.data[1]
        stamp = float(color_obs.ts)
        if first_ts is None:
            first_ts = stamp
        if stamp - first_ts < seek:
            continue
        if stamp - first_ts - seek > max_seconds:
            break
        ingestor.add_depth(depth_obs.data)
        frame = ingestor.add_image(color_obs.data)
        if frame is None:
            continue
        mcap.write("/segments/color", frame.rgb, stamp, frame.camera_frame)
        mcap.write("/segments/overlay", frame.overlay, stamp, frame.camera_frame)
        flat = np.repeat((frame.result.flat * 255).astype(np.uint8)[..., None], 3, axis=-1)
        mcap.write("/segments/flat", flat, stamp, frame.camera_frame)
        demoted.append(frame.result.demoted_fraction)
        for segment in frame.result.segments:
            label_counts[segment.name] = label_counts.get(segment.name, 0) + 1
        if ingestor.stats["segmented"] % 10 == 0:
            typer.echo(
                f"{stamp - first_ts:.0f}s: {ingestor.stats['segmented']} frames, "
                f"{ingestor.stats['segments']} segments ({time.monotonic() - started:.0f}s)"
            )
    mcap.close()
    memory.stop()
    if text_model is not None:
        text_model.stop()

    top = sorted(label_counts.items(), key=lambda item: -item[1])[:15]
    typer.echo(
        json.dumps({"frames": ingestor.stats["segmented"], "segments": ingestor.stats["segments"]})
    )
    typer.echo(f"labels seen most: {top}")
    if demoted:
        typer.echo(
            f"pixels demoted to unsure by the flat gate: mean {np.mean(demoted):.1%}, max {max(demoted):.1%}"
        )
    typer.echo(f"wrote {out_path} ({mcap.count} images, {out_path.stat().st_size / 1e6:.1f} MB)")
    typer.echo(f"segment records in {memory_path}")
    if out is None:
        subprocess.Popen(["open", str(out_path)])
