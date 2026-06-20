"""Basic XLeRobot blueprint — connection + camera + visualization.

Supports full robot (arms + head + base) or base-only mode via config.
"""

import platform
from typing import Any

from dimos.constants import DEFAULT_CAPACITY_COLOR_IMAGE
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import pSHMTransport
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.xlerobot.xlerobot_module import XLeRobotConnection
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule

_mac_transports: dict[tuple[str, type], pSHMTransport[Image]] = {
    ("color_image", Image): pSHMTransport(
        "color_image", default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE
    ),
}

_transports_base = (
    autoconnect() if platform.system() == "Linux" else autoconnect().transports(_mac_transports)
)


def _xlerobot_rerun_blueprint() -> Any:
    """Split layout: camera + 3D view side by side."""
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


def _static_base_link(rr: Any) -> list[Any]:
    """XLeRobot body wireframe — roughly a cylinder shape."""
    return [
        rr.Boxes3D(
            half_sizes=[0.15, 0.15, 0.12],
            colors=[(0, 200, 255)],
        ),
        rr.Transform3D(),
    ]


rerun_config = {
    "blueprint": _xlerobot_rerun_blueprint,
    "max_hz": {
        "world/color_image": 10,
    },
    "static": {
        "world/tf/base_link": _static_base_link,
    },
}


if global_config.viewer.startswith("rerun"):
    from dimos.visualization.rerun.bridge import RerunBridgeModule, _resolve_viewer_mode

    _vis = autoconnect(
        _transports_base,
        RerunBridgeModule.blueprint(viewer_mode=_resolve_viewer_mode(), **rerun_config),
    )
else:
    _vis = _transports_base

xlerobot_basic = autoconnect(
    _vis,
    XLeRobotConnection.blueprint(),
    WebsocketVisModule.blueprint(),
)

__all__ = ["xlerobot_basic"]
