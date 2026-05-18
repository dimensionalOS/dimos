#!/usr/bin/env python3

from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport
from dimos.mapping.costmapper import CostMapper
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.visualization.vis_module import vis_module


def _static_drone_body(rr: Any) -> list[Any]:
    return [
        rr.Boxes3D(
            half_sizes=[0.05, 0.05, 0.02],
            colors=[(0, 200, 255)],
            fill_mode="MajorWireframe",
        ),
        rr.Transform3D(parent_frame="tf#/base_link"),
    ]


def _drone_rerun_blueprint() -> Any:
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Spatial3DView(
            origin="world",
            name="3D",
            background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
            line_grid=rrb.LineGrid3D(
                plane=rr.components.Plane3D.XY.with_distance(0.0),
            ),
        ),
    )


rerun_config = {
    "blueprint": _drone_rerun_blueprint,
    "static": {
        "world/tf/base_link": _static_drone_body,
    },
}

_with_vis = vis_module(viewer_backend=global_config.viewer, rerun_config=rerun_config)

drone_primitive_no_nav = (
    autoconnect(
        _with_vis,
        CostMapper.blueprint(),
    )
    .global_config(n_workers=4, robot_model="drone", mujoco_room="empty")
    .transports(
        {
            ("cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
            ("odom", PoseStamped): LCMTransport("/odom", PoseStamped),
        }
    )
)

__all__ = ["drone_primitive_no_nav"]
