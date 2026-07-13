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

"""Agent-callable floorplan generation and navigated-space analysis (wishlist item 1).

`FloorplanSkillContainer` wraps `dimos.mapping.floorplan.generator` as a
`@skill` so an LLM agent (via McpServer/McpClient or any MCP client) can ask
the robot for an architectural floor plan of the space it has navigated, and
read back measured attributes of that space: number of levels and their
elevations, wall/door/stair counts, glazed (glass) wall fraction, mezzanines
and sunken areas, and per-level extents.

Debug artifacts and 3D-viewer output are OFF by default here — an agent that
only wants the analysis or the drawing should not litter the filesystem or
pop a viewer window; both can be re-enabled per call.
"""

from __future__ import annotations

from pathlib import Path
import time

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.mapping.floorplan.generator import FloorplanOptions, generate_floorplan
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class FloorplanSkillContainer(Module):
    """Skills for turning navigated space into architectural drawings + attributes."""

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()

    @skill
    def generate_floorplan(
        self,
        rrd_path: str = "",
        duration: float = 60.0,
        explore: bool = False,
        session_dir: str = "",
        levels: str = "",
        ai_review: bool = False,
        ai_render_styles: str = "",
        include_furniture: bool = True,
        save_3d_model: bool = False,
        debug_artifacts: bool = False,
        output_prefix: str = "",
        project_name: str = "DIMOS SITE SURVEY",
    ) -> str:
        """Generate an architectural floor plan of the navigated space and report its attributes.

        Builds a multi-level architectural drawing set (DXF with AIA layers +
        JPEG sheet) from lidar, then returns a text summary of the space:
        levels with their floor elevations, wall/door/stair counts, glazed
        (glass) walls, mezzanines/sunken areas, and per-level extents — useful
        both for producing drawings and for answering questions about the
        building the robot has explored.

        Args:
            rrd_path: Path to a Rerun (.rrd) recording of a mapping session to analyze. Empty = collect live lidar from the running robot instead.
            duration: Live-collection time in seconds (only used when rrd_path is empty). Longer = better coverage.
            explore: When collecting live, drive the frontier explorer for full-coverage mapping instead of standing still.
            session_dir: Optional recording-session directory (gtsam_odom.tum + mem2.db + camera_intrinsics.json). Camera frames are then used to visually confirm ambiguous surfaces such as glass walls.
            levels: Manual floor elevations as comma-separated z values in meters, e.g. "-0.2,3.2,6.7". Empty = detect floors automatically from the robot trajectory (including mezzanines and sunken areas).
            ai_review: Run the AI quality passes (classification review, glass/solid/open surface confirmation, and a defect-gated plan critique). Slower and uses the OpenAI API; the geometric pipeline alone is usually sufficient.
            ai_render_styles: Comma-separated stylized rendition styles produced by the OpenAI image model from the finished sheet ("drafted", "cyanotype", "presentation", or a custom prompt). Empty = no renditions.
            include_furniture: Draw movable objects as thin outlines per drafting convention. False = permanent structure only.
            save_3d_model: Also write a sliceable 3D model (<output>.model.rrd) for interactive viewing in dimos-viewer. Off by default — only request it when the 3D file is actually needed; the viewer is never auto-opened from this skill.
            debug_artifacts: Keep intermediate AI-evidence images (classification maps, camera inspection frames, critique review sheets) next to the outputs. Off by default: they are written to a temporary directory and deleted after the run.
            output_prefix: Path prefix for the outputs (e.g. "/tmp/site/plan" gives plan.dxf/plan.jpg). Empty = a timestamped prefix under the current directory.
            project_name: Project title printed in the drawing's title block.
        """
        if not output_prefix:
            output_prefix = f"floorplan-{time.strftime('%Y%m%d-%H%M%S')}"
        try:
            result = generate_floorplan(
                FloorplanOptions(
                    rrd=Path(rrd_path) if rrd_path else None,
                    session_dir=Path(session_dir) if session_dir else None,
                    duration=duration,
                    explore=explore,
                    levels=levels or None,
                    ai_review=ai_review,
                    ai_render=ai_render_styles or None,
                    furniture=include_furniture,
                    save_3d_model=save_3d_model,
                    open_viewer=False,  # a skill must never pop UI on the host
                    debug_artifacts=debug_artifacts,
                    out=Path(output_prefix),
                    project=project_name,
                )
            )
        except (ValueError, RuntimeError, OSError) as e:
            return f"Floorplan generation failed: {e}"
        return result.summary()


floorplan_skill_container = FloorplanSkillContainer.blueprint
