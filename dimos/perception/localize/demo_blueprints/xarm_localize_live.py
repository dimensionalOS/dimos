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

"""Live object memory on a depth camera rig, fed over ports.

:class:`PerceptionLocalize` consumes ``color_image``, ``depth_image``,
``camera_info`` and ``tf`` and answers ``localize`` from its own bounded
memory. Nothing here reads a file. In development the ports are fed by
``xarm-feed`` in another terminal; on a robot they are fed by its connection.

Usage:
    uv run python -m dimos.perception.localize.demo_blueprints.xarm_feed    # terminal 1
    dimos run xarm-localize-live                                     # terminal 2
    uv run dimos shell                                               # terminal 3
    app.PerceptionLocalize.state()
    app.PerceptionLocalize.localize("book,pen", -60.0, 60.0)
    uv run dimos mcp call localize -a 'objects=book,pen'             # or over MCP
"""

from __future__ import annotations

from typing import Any

from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.perception.localize.module import PerceptionLocalize
from dimos.visualization.vis_module import vis_module

OPTICAL_FRAME = "camera_color_optical_frame"


def _rerun_blueprint() -> Any:
    """Camera feed beside the 3D world with the tf tree and the localize answers."""
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(origin="world/color_image", name="Camera"),
            rrb.Spatial3DView(
                origin="world",
                name="3D",
                background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                line_grid=rrb.LineGrid3D(plane=rr.components.Plane3D.XY.with_distance(0.1)),
            ),
            column_shares=[1, 2],
        ),
        rrb.TimePanel(state="expanded"),
        rrb.SelectionPanel(state="hidden"),
    )


def _convert_camera_info(camera_info: Any) -> Any:
    return camera_info.to_rerun(image_topic="/world/color_image", optical_frame=OPTICAL_FRAME)


rerun_config: dict[str, Any] = {
    "blueprint": _rerun_blueprint,
    "visual_override": {
        "world/camera_info": _convert_camera_info,
        "world/depth_image": None,
    },
    "max_hz": {"world/color_image": 10},
    "memory_limit": "5%",
    "tf_axes": 0.1,
}


xarm_localize_live = autoconnect(
    vis_module(viewer_backend=global_config.viewer, rerun_config=rerun_config),
    PerceptionLocalize.blueprint(world_frame="world"),
    McpServer.blueprint(),
).global_config(n_workers=5)
