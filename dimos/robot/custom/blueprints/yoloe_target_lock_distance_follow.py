from __future__ import annotations

from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.robot.custom.tasks.bbox_distance_behavior_module import BBoxDistanceBehaviorModule
from dimos.robot.custom.modules.bbox_selection_module import BBoxSelectionModule
from dimos.robot.custom.modules.target_lock_module import TargetLockModule
from dimos.robot.custom.modules.yoloe_tracking_module import (
    YoloeTrackingModule,
    _require_yoloe_lrpc_model,
)
from dimos.robot.custom.visualization.detection2d_overlay import (
    selected_bbox_overlay,
    yoloe_overlay,
)
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import (
    rerun_config as go2_rerun_config,
    unitree_go2_basic,
)
from dimos.visualization.vis_module import vis_module

_YOLOE_DETECTIONS_TOPIC = "/color_image/yoloe_detections"
_USER_SELECTED_BBOX_TOPIC = "/color_image/selected_bbox"
_LOCKED_BBOX_TOPIC = "/color_image/locked_bbox"

_YOLOE_DETECTIONS_ENTITY = "world/color_image/yoloe_detections"
_USER_SELECTED_BBOX_ENTITY = "world/color_image/selected_bbox"
_LOCKED_BBOX_ENTITY = "world/color_image/locked_bbox"


def _target_lock_rerun_blueprint() -> Any:
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(
                origin="world/color_image",
                contents=["world/color_image/**"],
                name="Camera",
            ),
            rrb.Spatial3DView(
                origin="world",
                contents=[
                    "world/**",
                    f"-{_YOLOE_DETECTIONS_ENTITY}",
                    f"-{_USER_SELECTED_BBOX_ENTITY}",
                    f"-{_LOCKED_BBOX_ENTITY}",
                ],
                name="3D",
                background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                line_grid=rrb.LineGrid3D(
                    plane=rr.components.Plane3D.XY.with_distance(0.5),
                ),
                overrides={
                    "world/lidar": rrb.EntityBehavior(visible=False),
                },
            ),
            column_shares=[1, 2],
        ),
        rrb.TimePanel(state="hidden"),
        rrb.SelectionPanel(state="hidden"),
    )


_target_lock_rerun_config = {
    **go2_rerun_config,
    "blueprint": _target_lock_rerun_blueprint,
    "visual_override": {
        **go2_rerun_config["visual_override"],
        _YOLOE_DETECTIONS_ENTITY: yoloe_overlay,
        _USER_SELECTED_BBOX_ENTITY: selected_bbox_overlay,
        _LOCKED_BBOX_ENTITY: selected_bbox_overlay,
    },
}

_target_lock_vis = vis_module(
    viewer_backend=global_config.viewer,
    rerun_config=_target_lock_rerun_config,
)


yoloe_target_lock_distance_follow = (
    autoconnect(
        unitree_go2_basic,
        _target_lock_vis,
        YoloeTrackingModule.blueprint(),
        BBoxSelectionModule.blueprint(),
        TargetLockModule.blueprint(),
        BBoxDistanceBehaviorModule.blueprint(),
    )
    .global_config(
        n_workers=10,
        robot_model="unitree_go2",
    )
    .remappings(
        [
            (BBoxSelectionModule, "selected_bbox", "user_selected_bbox"),
            (TargetLockModule, "selected_bbox", "user_selected_bbox"),
            (TargetLockModule, "locked_bbox", "selected_bbox"),
        ]
    )
    .transports(
        {
            ("detections", Detection2DArray): LCMTransport(
                _YOLOE_DETECTIONS_TOPIC,
                Detection2DArray,
            ),
            ("user_selected_bbox", Detection2DArray): LCMTransport(
                _USER_SELECTED_BBOX_TOPIC,
                Detection2DArray,
            ),
            ("locked_bbox", Detection2DArray): LCMTransport(
                _LOCKED_BBOX_TOPIC,
                Detection2DArray,
            ),
        }
    )
    .requirements(
        _require_yoloe_lrpc_model,
    )
)

__all__ = [
    "yoloe_target_lock_distance_follow",
]


if __name__ == "__main__":
    ModuleCoordinator.build(yoloe_target_lock_distance_follow).loop()