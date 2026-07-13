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

"""CLI for the architectural floorplan generator (wishlist item 1).

Thin wrapper over `dimos.mapping.floorplan.generator.generate_floorplan` —
all pipeline logic lives there; the agent-callable skill is
`dimos/skills/mapping/floorplan_skill.py`.

Usage:
    python scripts/demo_lidar_floorplan.py --rrd chinaOffice.rrd --project "CHINA OFFICE"
    python scripts/demo_lidar_floorplan.py --duration 120 --explore
    python scripts/demo_lidar_floorplan.py --rrd b.rrd --ai-render --viewer
"""

from __future__ import annotations

import argparse
from pathlib import Path

from dimos.mapping.floorplan.generator import (
    BLUEPRINT_STYLES,
    FloorplanOptions,
    generate_floorplan,
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rrd", type=Path, default=None,
                        help="read a Rerun recording instead of a live instance")
    parser.add_argument(
        "--session-dir", type=Path, default=None,
        help="recording session dir (gtsam_odom.tum + mem2.db) — camera frames are used "
             "to visually confirm ambiguous surfaces like glass walls (needs --ai-review)",
    )
    parser.add_argument("--duration", type=float, default=30.0, help="live collection time, s")
    parser.add_argument("--lidar-topic", default="/lidar")
    parser.add_argument(
        "--levels", default=None,
        help='manual floor elevations, e.g. "-0.2,3.2,6.7" (overrides auto-detection)',
    )
    parser.add_argument(
        "--wall-z", type=float, default=None,
        help="height above the floor where returns count as structural evidence, m "
             "(default: 1.6 for recordings; 0.95 live — the MuJoCo sim caps lidar at 1.2 m)",
    )
    parser.add_argument("--resolution", type=float, default=0.05, help="grid cell, m")
    parser.add_argument("--min-hits", type=int, default=2)
    parser.add_argument("--spin", action="store_true", help="rotate in place while collecting")
    parser.add_argument(
        "--explore", action="store_true",
        help="trigger the frontier explorer for full-coverage mapping",
    )
    parser.add_argument(
        "--ai-review", action=argparse.BooleanOptionalAction, default=None,
        help="AI double-check of wall/furniture classification "
             "(default: on when OPENAI_API_KEY is set)",
    )
    parser.add_argument(
        "--furniture", action=argparse.BooleanOptionalAction, default=True,
        help="include movable objects as thin outlines on A-FURN (drafting convention)",
    )
    parser.add_argument(
        "--ai-render", nargs="?", const="drafted,presentation", default=None,
        metavar="STYLE[,STYLE...]",
        help="stylized versions of the finished sheet via the OpenAI image model "
             f"(named styles: {', '.join(BLUEPRINT_STYLES)}; anything else is a custom "
             "prompt; bare flag = drafted,presentation)",
    )
    parser.add_argument("--voxel", type=float, default=0.1,
                        help="3D voxel edge for the scene voxel model, m")
    parser.add_argument(
        "--close-loops", action=argparse.BooleanOptionalAction, default=True,
        help="bridge lidar gaps so each connected area's boundary closes into loops",
    )
    parser.add_argument(
        "--viewer", action="store_true",
        help="write <out>.model.rrd and open it in dimos-viewer — the time slider "
             "sweeps a horizontal cross-section through the property",
    )
    parser.add_argument(
        "--debug-artifacts", action=argparse.BooleanOptionalAction, default=True,
        help="keep AI-evidence images (.debug/.inspect/.review) next to the output "
             "(the skill/MCP path defaults these OFF)",
    )
    parser.add_argument("--out", default="floorplan", help="output path prefix")
    parser.add_argument("--project", default="DIMOS SITE SURVEY", help="title block project")
    args = parser.parse_args()

    result = generate_floorplan(
        FloorplanOptions(
            rrd=args.rrd,
            session_dir=args.session_dir,
            duration=args.duration,
            lidar_topic=args.lidar_topic,
            levels=args.levels,
            wall_z=args.wall_z,
            resolution=args.resolution,
            min_hits=args.min_hits,
            spin=args.spin,
            explore=args.explore,
            ai_review=args.ai_review,
            furniture=args.furniture,
            ai_render=args.ai_render,
            voxel=args.voxel,
            close_loops=args.close_loops,
            save_3d_model=args.viewer,
            open_viewer=args.viewer,
            debug_artifacts=args.debug_artifacts,
            out=Path(args.out),
            project=args.project,
        )
    )
    print(result.summary())


if __name__ == "__main__":
    main()
