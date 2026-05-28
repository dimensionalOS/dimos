from __future__ import annotations

from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.navigation.movement_manager.movement_manager import MovementManager
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
)
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import unitree_go2
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop
from dimos.visualization.vis_module import vis_module

_YOLOE_DETECTIONS_TOPIC = "/color_image/yoloe_detections"
_USER_SELECTED_BBOX_TOPIC = "/color_image/selected_bbox"
_LOCKED_BBOX_TOPIC = "/color_image/locked_bbox"
_NAV_CMD_VEL_TOPIC = "/nav_cmd_vel"
_TELE_CMD_VEL_TOPIC = "/tele_cmd_vel"

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
        unitree_go2,
        _target_lock_vis,
        YoloeTrackingModule.blueprint(),
        BBoxSelectionModule.blueprint(),
        TargetLockModule.blueprint(),
        BBoxDistanceBehaviorModule.blueprint(),
        # Keyboard teleop: publishes tele_cmd_vel when keys held; silent otherwise.
        # MovementManager (from unitree_go2) muxes tele_cmd_vel (priority) + nav_cmd_vel
        # (task) → cmd_vel.
        # When teleop fires, MovementManager publishes stop_movement → teleop_active →
        # BBoxDistanceBehaviorModule resets to idle so YOLO keeps detecting while
        # the user drives; clicking a new bbox restarts the approach task.
        # When a 3D point is clicked, MovementManager also fires stop_movement → cancels
        # bbox tracking; BBoxDistanceBehaviorModule cancels A* via _planner spec when a
        # new bbox task starts.
        KeyboardTeleop.blueprint(publish_only_when_active=True),
    )
    .global_config(
        n_workers=16,
        robot_model="unitree_go2",
    )
    .remappings(
        [
            (BBoxSelectionModule, "selected_bbox", "user_selected_bbox"),
            (TargetLockModule, "selected_bbox", "user_selected_bbox"),
            (TargetLockModule, "locked_bbox", "selected_bbox"),
            # Task cmd_vel → MovementManager nav_cmd_vel (lower priority than keyboard)
            (BBoxDistanceBehaviorModule, "cmd_vel", "nav_cmd_vel"),
            # Keyboard cmd_vel → MovementManager tele_cmd_vel (higher priority)
            (KeyboardTeleop, "cmd_vel", "tele_cmd_vel"),
            # MovementManager stop_movement → task teleop_active (interrupt on teleop)
            (MovementManager, "stop_movement", "teleop_active"),
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
            ("nav_cmd_vel", Twist): LCMTransport(
                _NAV_CMD_VEL_TOPIC,
                Twist,
            ),
            ("tele_cmd_vel", Twist): LCMTransport(
                _TELE_CMD_VEL_TOPIC,
                Twist,
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