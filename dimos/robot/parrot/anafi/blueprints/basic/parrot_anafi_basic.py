# Copyright 2025-2026 Dimensional Inc.
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

from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import JpegLcmTransport
from dimos.msgs.sensor_msgs.Image import Image
from dimos.protocol.service.system_configurator.clock_sync import ClockSyncConfigurator
from dimos.robot.parrot.anafi.connection import AnafiConnectionModule
from dimos.visualization.vis_module import vis_module

_transports_base = autoconnect().transports(
    {
        ("color_image", Image): JpegLcmTransport("/color_image", Image),
    }
)


def _convert_camera_info(camera_info: Any) -> Any:
    return camera_info.to_rerun(
        image_topic="/world/color_image",
        optical_frame="camera_optical",
    )


def _static_drone_body(rr: Any) -> list[Any]:
    """Static visualization of the Anafi body."""
    return [
        rr.Boxes3D(
            half_sizes=[0.18, 0.12, 0.05],
            colors=[(80, 160, 255)],
        ),
        rr.Transform3D(parent_frame="tf#/base_link"),
    ]


def _anafi_rerun_blueprint() -> Any:
    """Split layout: camera feed on the left, 3D world view on the right."""
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(origin="world/color_image", name="Camera"),
            rrb.Spatial3DView(
                origin="world",
                name="3D",
                background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                line_grid=rrb.LineGrid3D(
                    plane=rr.components.Plane3D.XY.with_distance(0.5),
                ),
            ),
            column_shares=[1, 2],
        ),
        rrb.TimePanel(state="hidden"),
        rrb.SelectionPanel(state="hidden"),
    )


rerun_config = {
    "blueprint": _anafi_rerun_blueprint,
    # Custom converters for specific rerun entity paths
    # Normally all these would be specified in their respectative modules
    # Until this is implemented we have central overrides here
    #
    # This is unsustainable once we move to multi robot etc
    "visual_override": {
        "world/camera_info": _convert_camera_info,
    },
    "max_hz": {
        "world/color_image": 0,
    },
    "static": {
        "world/tf/base_link": _static_drone_body,
    },
}

_with_vis = autoconnect(
    _transports_base,
    vis_module(
        viewer_backend=global_config.viewer,
        rerun_config=rerun_config,
        foxglove_config={"shm_channels": ["/color_image#sensor_msgs.Image"]},
    ),
)

parrot_anafi_basic = (
    autoconnect(
        _with_vis,
        AnafiConnectionModule.blueprint(),
    )
    .global_config(n_workers=4, robot_model="parrot_anafi")
    .configurators(ClockSyncConfigurator())
)

__all__ = ["parrot_anafi_basic"]
