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

"""Build a sliceable 3D model of a scanned space from a Rerun recording.

Fuses the lidar map and trajectory in a .rrd (plus, optionally, the recorded
camera stream for color) into a `SceneModel`, then exports:

  - <out>.rrd            — interactive colored 3D model (`dimos-viewer <out>.rrd`)
  - <out>.plan-z*.png    — horizontal cross-sections (plan cuts)
  - <out>.section-*.png  — vertical cross-sections (elevations) along cut lines
  - <out>.mesh.<fmt>     — optional Poisson surface mesh (--mesh glb,obj,ply,stl,gltf)

Usage:
    python scripts/demo_rrd_3d_model.py --rrd chinaOffice.rrd \\
        --session-dir ~/Desktop/2026-06-12_03-26am-PST__china_office1 \\
        --plan-cuts 1.5,4.8,8.2 --section "0,-30,15,25" --mesh glb,obj
"""

from __future__ import annotations

import argparse
from pathlib import Path

from dimos.mapping.reconstruction.scene_model import SceneModel
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rrd", type=Path, required=True, help="Rerun recording to model")
    parser.add_argument(
        "--session-dir", type=Path, default=None,
        help="recording session dir (mem2.db + gtsam_odom.tum + camera_intrinsics.json) "
             "— colorizes the model from the camera stream",
    )
    parser.add_argument(
        "--plan-cuts", default=None,
        help="comma-separated z heights for horizontal sections (default: 1.2 m above "
             "each detected occupancy band)",
    )
    parser.add_argument(
        "--section", action="append", default=[],
        help='vertical cut "x1,y1,x2,y2" (repeatable)',
    )
    parser.add_argument("--thickness", type=float, default=0.25, help="cut thickness, m")
    parser.add_argument("--resolution", type=float, default=0.05, help="raster cell, m")
    parser.add_argument("--max-frames", type=int, default=120,
                        help="camera frames used for colorization")
    parser.add_argument(
        "--mesh", nargs="?", const="glb", default=None, metavar="FMT[,FMT...]",
        help="also export a Poisson surface mesh in these formats "
             "(glb, gltf, obj, ply, stl; bare --mesh means glb)",
    )
    parser.add_argument("--mesh-voxel", type=float, default=0.08,
                        help="downsample voxel before meshing, m")
    parser.add_argument("--mesh-depth", type=int, default=10, help="Poisson octree depth")
    parser.add_argument("--out", default="scene", help="output path prefix")
    args = parser.parse_args()

    model = SceneModel.from_rrd(args.rrd)
    if len(model.points) == 0:
        raise SystemExit(f"No point cloud found in {args.rrd}")

    if args.session_dir is not None:
        model.colorize_from_session(args.session_dir, max_frames=args.max_frames)

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)

    if args.plan_cuts:
        cuts = [float(v) for v in args.plan_cuts.split(",")]
    else:
        # default: one cut through the middle of the observed z range
        z = model.points[:, 2]
        cuts = [float((z.min() + z.max()) / 2)]
    for z_cut in cuts:
        section = model.horizontal_section(z_cut, args.thickness, args.resolution)
        png = out.parent / f"{out.name}.plan-z{z_cut:+.2f}.png"
        section.save_png(png, title=f"PLAN CUT AT z = {z_cut:+.2f} m")
        logger.info(f"Wrote {png}")

    for i, spec in enumerate(args.section):
        x1, y1, x2, y2 = (float(v) for v in spec.split(","))
        section = model.vertical_section((x1, y1), (x2, y2), max(args.thickness, 0.4),
                                         args.resolution)
        png = out.parent / f"{out.name}.section-{i}.png"
        section.save_png(png, title=f"SECTION ({x1:.1f},{y1:.1f}) → ({x2:.1f},{y2:.1f})")
        logger.info(f"Wrote {png}")

    model.save_rerun(out.with_suffix(".rrd"))

    if args.mesh:
        mesh = model.to_mesh(voxel=args.mesh_voxel, poisson_depth=args.mesh_depth)
        for fmt in args.mesh.split(","):
            model.save_mesh(
                out.parent / f"{out.name}.mesh.{fmt.strip().lower()}", mesh=mesh
            )


if __name__ == "__main__":
    main()
