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

"""Robot-local M20 integration for the current DimOS 3D navigation stack."""

from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import ZenohTransport
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.msgs.foxglove_msgs.CompressedVideo import CompressedVideo
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.navigation.dannav.holonomic_tc.module import DanHolonomicTC
from dimos.navigation.dannav.local_planner.module import DanLocalPlanner
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_3d.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.navigation.nav_3d.mls_planner.viz import planner_visual_override
from dimos.protocol.pubsub.impl.zenohpubsub import QOS_LATEST_WINS, Topic as ZenohTopic
from dimos.robot.deeprobotics.m20.camera import M20CameraRelay
from dimos.robot.deeprobotics.m20.connection import M20Connection
from dimos.robot.deeprobotics.m20.constants import (
    BASE_LINK_HEIGHT_M,
    BODY_LENGTH_M,
    BODY_WIDTH_M,
    PLANNING_HEIGHT_M,
    ROTATION_DIAMETER_M,
)
from dimos.visualization.vis_module import vis_module

VOXEL_SIZE_M = 0.1
PLANNER_VIZ_HZ = 0.0


def _render_path(msg: Any) -> Any:
    if len(msg.poses) == 0:
        return None
    return msg


def _render_h265(msg: CompressedVideo) -> Any:
    import rerun as rr

    return rr.VideoStream(codec=rr.VideoCodec.H265, sample=msg.data.tobytes())


def _render_front_camera_info(msg: CameraInfo) -> Any:
    return msg.to_rerun(
        image_topic="world/front_camera",
        optical_frame="front_camera_optical",
    )


def _render_rear_camera_info(msg: CameraInfo) -> Any:
    return msg.to_rerun(
        image_topic="world/rear_camera",
        optical_frame="rear_camera_optical",
    )


def _static_robot_body(rr: Any) -> list[Any]:
    return [
        rr.Boxes3D(
            half_sizes=[
                BODY_LENGTH_M * 0.5,
                BODY_WIDTH_M * 0.5,
                PLANNING_HEIGHT_M * 0.5,
            ],
            centers=[0.0, 0.0, PLANNING_HEIGHT_M * 0.5 - BASE_LINK_HEIGHT_M],
            colors=[(0, 255, 127)],
        ),
        rr.Transform3D(parent_frame="tf#/base_link"),
    ]


def _m20_rerun_blueprint() -> Any:
    """Go2-style navigation layout with both M20 camera streams."""
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Vertical(
                rrb.Spatial2DView(origin="world/front_camera", name="Front Camera"),
                rrb.Spatial2DView(origin="world/rear_camera", name="Rear Camera"),
                row_shares=[1, 1],
            ),
            rrb.Spatial3DView(
                origin="world",
                name="M20 KronkNav",
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


_rerun_config = {
    "blueprint": _m20_rerun_blueprint,
    "tf_axes": 0.35,
    "max_hz": {
        "world/local_map": 0.5,
    },
    "visual_override": {
        # The navigation view shows maps rather than the registered lidar.
        "world/lidar": None,
        "world/slam_body_points": None,
        "world/planner_path": _render_path,
        "world/path": None,
        "world/front_camera": _render_h265,
        "world/rear_camera": _render_h265,
        "world/front_camera_info": _render_front_camera_info,
        "world/rear_camera_info": _render_rear_camera_info,
        **planner_visual_override(PLANNER_VIZ_HZ),
    },
    "static": {
        "world/robot_body": _static_robot_body,
    },
}


deeprobotics_m20_kronknav_control = (
    autoconnect(
        vis_module(viewer_backend=global_config.viewer, rerun_config=_rerun_config),
        M20CameraRelay.blueprint(instance_name="M20CameraRelay"),
        M20Connection.blueprint().remappings([(M20Connection, "odometry", "slam_odom")]),
        RayTracingVoxelMap.blueprint(
            voxel_size=VOXEL_SIZE_M,
            max_range=25.0,
            ray_subsample=5,
            emit_every=1,
            global_emit_every=50,
            support_min=4,
            world_frame="map",
            worker_threads=3,
        ).remappings([(RayTracingVoxelMap, "lidar", "slam_body_points")]),
        MLSPlannerNative.blueprint(
            world_frame="map",
            base_frame="base_link",
            voxel_size=VOXEL_SIZE_M,
            robot_height=PLANNING_HEIGHT_M,
            start_z_offset_m=BASE_LINK_HEIGHT_M,
            wall_clearance_m=0.3,
            wall_buffer_m=0.85,
            wall_buffer_weight=100.0,
            step_threshold_m=0.25,
            step_penalty_weight=4.0,
            viz_publish_hz=PLANNER_VIZ_HZ,
            worker_threads=2,
        ).remappings(
            [
                (MLSPlannerNative, "global_map", "global_map_unused"),
                (MLSPlannerNative, "path", "planner_path"),
            ]
        ),
        DanLocalPlanner.blueprint(
            lock_replan=0.4,
            # Preserve MLS's 3D waypoints; the 2D resampler replaces every Z with zero.
            resample_spacing_m=0.0,
        ),
        DanHolonomicTC.blueprint(
            run_profile="walk",
            control_frequency=10.0,
        ),
        MovementManager.blueprint(),
    )
    .global_config(
        n_workers=4,
        obstacle_avoidance=False,
        robot_width=BODY_WIDTH_M,
        robot_rotation_diameter=ROTATION_DIAMETER_M,
        transport="zenoh",
    )
    .transports(
        {
            ("front_camera", CompressedVideo): ZenohTransport.spec(
                ZenohTopic("dimos/front_camera", CompressedVideo, qos=QOS_LATEST_WINS)
            ),
            ("rear_camera", CompressedVideo): ZenohTransport.spec(
                ZenohTopic("dimos/rear_camera", CompressedVideo, qos=QOS_LATEST_WINS)
            ),
        }
    )
)
