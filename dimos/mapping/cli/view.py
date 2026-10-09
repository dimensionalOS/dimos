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

"""`dimos map view`: open a `.pc2.cdr` cloud - `dimos map global --export` - in rerun."""

from __future__ import annotations

from pathlib import Path

from dimos_message_build.registry import decode as cdr_decode
import typer


def main(
    path: Path = typer.Argument(..., help="Point cloud to view (.pc2.cdr)"),
    voxel: float = typer.Option(0.05, "--voxel", help="Rendered point size (m)"),
    bottom_cutoff: float | None = typer.Option(
        None,
        "--bottom-cutoff",
        help="Drop points below this Z (m) when rendering; e.g. 0 strips the floor",
    ),
    out: Path | None = typer.Option(
        None, "--out", help="Write a .rrd instead of opening the viewer"
    ),
) -> None:
    """Load a CDR-encoded PointCloud2 and display it in rerun."""
    from dimos_generated.sensor_msgs.msg import PointCloud2
    import rerun as rr

    from dimos.visualization.rerun.init import rerun_init
    from dimos.visualization.rerun.message_helpers import cloud_archetype

    cloud = cdr_decode(path.read_bytes(), PointCloud2)
    print(f"{path}: {cloud.width * cloud.height} points in {cloud.header.frame_id!r}")

    rerun_init("dimos map view")
    if out is not None:
        rr.save(str(out))
    else:
        rr.spawn()
    rr.log(
        f"world/{path.name.split('.')[0]}/pointcloud",
        cloud_archetype(cloud, ui_radius=voxel / 2, bottom_cutoff=bottom_cutoff),
        static=True,
    )
    if out is not None:
        print(f"wrote {out}")


if __name__ == "__main__":
    typer.run(main)
