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

"""Marauder's Map blueprint for the Go2.

A *thin* composition layer on top of jamjam's target-lock / distance-follow /
selection stack. This file lives under ``dimos/apps/`` so the upstream jamjam
control modules (``dimos/robot/custom/...``) and the dimos navigation /
movement manager can be fast-forwarded with zero merge conflict here.

How selection is wired without touching jamjam's modules:
    * Rerun camera-image click → BBoxSelectionModule.clicked_point  (existing)
    * Web map click           → ReidMapModule synthesizes a PointStamped at
                                  the chosen person's bbox center with
                                  ``frame_id="color_image/web_click"``, which
                                  flows into the SAME clicked_point topic
                                  → BBoxSelectionModule treats it identically
                                  to a Rerun camera click.

Other web→robot wiring (also non-invasive):
    * ReidMapModule.cmd_vel → ``tele_cmd_vel`` (mux'd with priority by
      MovementManager — replaces pygame KeyboardTeleop, which crashes on
      macOS from a worker thread).
    * ReidMapModule.goal_request → ReplanningAStarPlanner (click free map
      space to publish a navigation goal).

Run:
    uv run dimos --robot-ip 192.168.12.1 --rerun-open native run go2-marauders-map
    dimos --replay run go2-marauders-map     # no hardware, replay data has people

Open: http://localhost:7782/  (the map)  +  Rerun viewer (camera + 3D + bboxes)
"""

from __future__ import annotations

from typing import Any

from dimos.apps.marauders_map.module import ReidMapModule
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.voxels import VoxelGridMapper
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.robot.custom.modules.bbox_selection_module import BBoxSelectionModule
from dimos.robot.custom.modules.target_lock_module import TargetLockModule
from dimos.robot.custom.modules.yoloe_tracking_module import (
    YoloeTrackingModule,
    _require_yoloe_lrpc_model,
)
from dimos.robot.custom.tasks.bbox_distance_behavior_module import (
    BBoxDistanceBehaviorModule,
)
from dimos.robot.custom.visualization.detection2d_overlay import (
    selected_bbox_overlay,
    yoloe_overlay,
)
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import (
    rerun_config as go2_rerun_config,
    unitree_go2_basic,
)
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.visualization.vis_module import vis_module

# LCM topic / Rerun entity wiring (matches yoloe_target_lock_distance_follow).
_YOLOE_DETECTIONS_TOPIC = "/color_image/yoloe_detections"
_USER_SELECTED_BBOX_TOPIC = "/color_image/selected_bbox"
_LOCKED_BBOX_TOPIC = "/color_image/locked_bbox"
_NAV_CMD_VEL_TOPIC = "/nav_cmd_vel"
_TELE_CMD_VEL_TOPIC = "/tele_cmd_vel"

_YOLOE_DETECTIONS_ENTITY = "world/color_image/yoloe_detections"
_USER_SELECTED_BBOX_ENTITY = "world/color_image/selected_bbox"
_LOCKED_BBOX_ENTITY = "world/color_image/locked_bbox"


def _marauders_rerun_blueprint() -> Any:
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
                line_grid=rrb.LineGrid3D(plane=rr.components.Plane3D.XY.with_distance(0.5)),
                overrides={"world/lidar": rrb.EntityBehavior(visible=False)},
            ),
            column_shares=[1, 2],
        ),
        rrb.TimePanel(state="hidden"),
        rrb.SelectionPanel(state="hidden"),
    )


_marauders_rerun_config = {
    **go2_rerun_config,
    "blueprint": _marauders_rerun_blueprint,
    "visual_override": {
        **go2_rerun_config["visual_override"],
        _YOLOE_DETECTIONS_ENTITY: yoloe_overlay,
        _USER_SELECTED_BBOX_ENTITY: selected_bbox_overlay,
        _LOCKED_BBOX_ENTITY: selected_bbox_overlay,
    },
}

_marauders_vis = vis_module(
    viewer_backend=global_config.viewer,
    rerun_config=_marauders_rerun_config,
)


# Composition (jamjam control stack untouched; only routed):
#   * unitree_go2_basic       — connection + camera + lidar + base viewer wiring
#   * VoxelGridMapper         — world-frame voxel map (needed for ReidMapModule's
#                               Detection3DPC.from_2d person localization)
#   * CostMapper              — 2D occupancy/cost grid, drawn as the parchment walls
#   * YoloeTrackingModule     — YOLO-E open-vocab detect + BoT-SORT track
#   * BBoxSelectionModule     — single source of truth for "which person is selected"
#                               (accepts Rerun click + our synthesized web click via
#                               the same clicked_point input)
#   * TargetLockModule        — locked-bbox state machine
#   * BBoxDistanceBehaviorModule — distance-follow control loop
#   * MovementManager         — tele/nav cmd_vel mux
#   * ReplanningAStarPlanner  — point-to-go navigation (consumes goal_request)
#   * ReidMapModule           — Marauder's Map web view on :7782 (UI layer only)
go2_marauders_map = (
    autoconnect(
        unitree_go2_basic,
        VoxelGridMapper.blueprint(emit_every=5),
        CostMapper.blueprint(),
        _marauders_vis,
        YoloeTrackingModule.blueprint(),
        BBoxSelectionModule.blueprint(),
        TargetLockModule.blueprint(),
        BBoxDistanceBehaviorModule.blueprint(),
        MovementManager.blueprint(),
        ReplanningAStarPlanner.blueprint(),
        ReidMapModule.blueprint(
            camera_info=GO2Connection.camera_info_static,
            enable_reid=False,  # rely on YOLO-E/BoT-SORT + spatial re-association
        ),
    )
    .global_config(
        n_workers=13,
        robot_model="unitree_go2",
    )
    .remappings(
        [
            # ─── Selection chain ────────────────────────────────────────────
            # Both Rerun camera click and our web "click" feed BBoxSelectionModule
            # through its existing `clicked_point` input. BBoxSelectionModule is
            # the sole writer of /user_selected_bbox; TargetLockModule consumes
            # it; the locked output is remapped to /selected_bbox for the
            # follower task and the Rerun green-box overlay.
            (BBoxSelectionModule, "selected_bbox", "user_selected_bbox"),
            (TargetLockModule, "selected_bbox", "user_selected_bbox"),
            (TargetLockModule, "locked_bbox", "selected_bbox"),
            # ─── Follow task → cmd_vel mux ──────────────────────────────────
            (BBoxDistanceBehaviorModule, "cmd_vel", "nav_cmd_vel"),
            # ─── Web teleop → mux (priority) ────────────────────────────────
            (ReidMapModule, "cmd_vel", "tele_cmd_vel"),
            # ─── Teleop activity → pause follow + clear selection ───────────
            # jamjam e9a82939: BBoxSelectionModule and TargetLockModule both gained
            # `stop_movement: In[Bool]` so any source of teleop-activity wipes the
            # current target and the old bbox can't auto-restart the follow on the
            # next detection frame.
            (MovementManager, "stop_movement", "teleop_active"),
            (BBoxSelectionModule, "stop_movement", "teleop_active"),
            (TargetLockModule, "stop_movement", "teleop_active"),
            # ─── Marauder's Map world localization ──────────────────────────
            (ReidMapModule, "pointcloud", "global_map"),
        ]
    )
    .transports(
        {
            ("detections", Detection2DArray): LCMTransport(
                _YOLOE_DETECTIONS_TOPIC, Detection2DArray
            ),
            ("user_selected_bbox", Detection2DArray): LCMTransport(
                _USER_SELECTED_BBOX_TOPIC, Detection2DArray
            ),
            ("locked_bbox", Detection2DArray): LCMTransport(_LOCKED_BBOX_TOPIC, Detection2DArray),
            ("nav_cmd_vel", Twist): LCMTransport(_NAV_CMD_VEL_TOPIC, Twist),
            ("tele_cmd_vel", Twist): LCMTransport(_TELE_CMD_VEL_TOPIC, Twist),
        }
    )
    .requirements(_require_yoloe_lrpc_model)
)

__all__ = ["go2_marauders_map"]


if __name__ == "__main__":
    ModuleCoordinator.build(go2_marauders_map).loop()
