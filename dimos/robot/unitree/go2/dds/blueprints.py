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


"""GO2DDS blueprints: the robot-side stack on the Jetson, and the viewer that dials it."""

from functools import partial
import os
from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.hardware.sensors.lidar.pointlio.pointlio_blueprints import mid360_for_pointlio
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.mapping.ray_tracing.viz import MAP_REGIONS_ENTITY, render_map_region
from dimos.navigation.global_planner.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.navigation.global_planner.mls_planner.viz import (
    SURFACE_MAP_ENTITY,
    render_surface_region,
)
from dimos.navigation.global_planner.viz import HEIGHT_RANGE, nav_static, nav_visual_override
from dimos.navigation.local_planner.native import LocalPlannerNative
from dimos.navigation.local_planner.viz import motion_visual_override
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.trajectory_follower.fancy.native import TrajectoryFollowerNative
from dimos.protocol.service.zenohservice import ZenohConfig
from dimos.robot.unitree.go2.constants import ROBOT_HEIGHT, ROBOT_LENGTH, ROBOT_WIDTH
from dimos.robot.unitree.go2.dds.module import GO2DDS
from dimos.robot.unitree.go2.nav_3d_config import (
    ray_tracing_config,
    relocalization,
    voxel_size,
    wall_clearance_m,
)
from dimos.visualization.vis_module import vis_module

# GO2DDS doubles as a zenoh router
go2_dds = GO2DDS.blueprint(
    iface="enP8p1s0", session=ZenohConfig(mode="router", listen=["tcp/0.0.0.0:7447"], connect=[])
).global_config(transport="zenoh", robot_model="unitree_go2")

# Raise above 0 (2.0 works) to draw what the planner searched over: surface, nodes and
# cost-colored edges. Drives both its publishing and the rerun overrides.
planner_viz_hz = 2.0
MOTION_BODY_DILATE_M = -0.03

_mls_planner_motion = MLSPlannerNative.blueprint(
    world_frame="odom",
    voxel_size=voxel_size,
    robot_height=0.4,
    surface_closing_radius=0.4,
    wall_clearance_m=0.05,
    wall_buffer_m=0.2,
    wall_buffer_weight=20.0,
    step_threshold_m=0.16,
    step_penalty_weight=4.0,
    viz_publish_hz=planner_viz_hz,
).remappings(
    [
        (MLSPlannerNative, "global_map", "global_map_unused"),
        (MLSPlannerNative, "path", "planner_path"),
    ]
)

# The head L1 stays off and Point-LIO owns odom, so GO2DDS publishes no lidar, odometry or
# odom tf edge. Its raw L1 cloud and body IMU move aside so only the MID-360 reaches
# Point-LIO's inputs.
go2_dds_mid360 = GO2DDS.blueprint(
    iface="enP8p1s0",
    session=ZenohConfig(mode="router", listen=["tcp/0.0.0.0:7447"], connect=[]),
    lidar_on=False,
    tf_root="mid360_link",
).remappings(
    [
        (GO2DDS, "odometry", "go2_odometry_unused"),
        (GO2DDS, "lidar", "go2_lidar_unused"),
        (GO2DDS, "lidar_raw", "go2_lidar_raw_unused"),
        (GO2DDS, "imu", "body_imu"),
    ]
)

# MLS stays global; its path becomes the carrot source (planner_path) for the local planner
# over the raycaster's local map. GO2DDS's native process is the zenoh router (the Go2
# forwards 7447 to the Jetson, so the viewer dials go22); every other process dials it on
# loopback. Headless: go2-dds-mid360-viewer on another machine is the screen.
# The Mid-360 IP comes from MID360__LIDAR_IP; host_ip is auto-detected.
go2_dds_nav = autoconnect(
    go2_dds_mid360,
    mid360_for_pointlio(),
    RayTracingVoxelMap.blueprint(**ray_tracing_config.model_dump(exclude_unset=True)),
    _mls_planner_motion,
    LocalPlannerNative.blueprint(body_dilate_m=MOTION_BODY_DILATE_M),
    TrajectoryFollowerNative.blueprint(),
    MovementManager.blueprint(),
    relocalization(republish_loaded_map=0.0),
    PointLio.blueprint(),
).global_config(
    transport="zenoh",
    zenoh_connect="tcp/127.0.0.1:7447",
    n_workers=11,
    robot_model="unitree_go2",
)


# h264 lands here off `video`; jpeg is redirected onto it, whichever the robot serves
CAMERA_ENTITY = "world/video"


def _camera_info_to_pinhole(camera_info: Any) -> Any:
    """Log the pinhole onto the camera image's entity instead of camera_info's own.

    Entities are named after topics, so the two land on sibling paths, and a Pinhole only
    projects its own entity and its children, hence a frustum that draws but stays empty.
    No ``optical_frame``: the video's frame_id already anchors it, a second parent is
    rejected.
    """
    return camera_info.to_rerun(image_topic=CAMERA_ENTITY)


def _image_to_camera(image: Any) -> Any:
    """GO2DDS's jpeg `image` onto the h264 `video` entity, so the pane is encoder-blind."""
    return [(CAMERA_ENTITY, image.to_rerun())]


def _rerun_blueprint() -> Any:
    """Split layout: camera feed + 3D world, as the WebRTC go2 blueprint has."""
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(origin=CAMERA_ENTITY, name="Camera"),
            rrb.Spatial3DView(
                origin="world",
                name="3D",
                background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                line_grid=rrb.LineGrid3D(plane=rr.components.Plane3D.XY.with_distance(0.5)),
                # Hidden rather than dropped: still in the entity tree, tickable in the
                # viewer.
                overrides={
                    "world/pointlio_map": rrb.EntityBehavior(visible=False),
                    "world/lidar": rrb.EntityBehavior(visible=False),
                    "world/nodes": rrb.EntityBehavior(visible=False),
                    "world/node_edges": rrb.EntityBehavior(visible=False),
                },
            ),
            column_shares=[1, 2],
        ),
        rrb.TimePanel(state="hidden"),
        rrb.SelectionPanel(state="hidden"),
    )


def _render_map(msg: Any) -> Any:
    return msg.to_rerun(voxel_size=0.01)


def _rerun_config(visual_override: dict[str, Any] | None = None) -> dict[str, Any]:
    """The bridge's own view, plus whatever the layer above it adds."""
    return {
        "blueprint": _rerun_blueprint,
        "tf_axes": 0.5,
        # The robot box hangs off base_link on its own entity: a static transform
        # under world/tf would override the live one.
        "static": nav_static(ROBOT_LENGTH, ROBOT_WIDTH, ROBOT_HEIGHT, wall_clearance_m),
        "visual_override": {
            "world/camera_info": _camera_info_to_pinhole,
            "world/image": _image_to_camera,
            "world/pointlio_map": _render_map,
            "world/lidar": _render_map,
            **nav_visual_override(planner_viz_hz, voxel_size, wall_clearance_m),
            # the local plan plus its body poses on world/path/body, coloured by the
            # stamped precision (green room, amber in the ramp, red at the floor)
            **motion_visual_override(body_dilate_m=MOTION_BODY_DILATE_M),
            **(visual_override or {}),
        },
    }


# The viewer half alone, for the machine with the screen. Zenoh keeps the newest sample
# per topic, so the drop sits in front of the wifi instead of rerun's lossless stream
# replaying history. `topics` is one subscription per name: unlisted never crosses the link.
# The router is named, not scouted: behind wifi multicast scouting finds nothing
# (docs/usage/transports/zenoh.md). --robot-ip still adds its endpoint alongside.
GO2_ROUTER = os.environ.get("DIMOS_GO2_ROUTER", "tcp/go22:7447")
# Ceiling cut for map_regions in odom: the origin is the lidar at start, ~0.5m above the floor.
MAP_CEILING_M = 1.5
# The storey the surface_map shows, in odom: the floor sits ~0.5m below the start pose.
SURFACE_Z_BAND = (-0.5, MAP_CEILING_M)

go2_dds_nav_viewer = autoconnect(
    vis_module(
        viewer_backend=global_config.viewer,
        rerun_config={
            **_rerun_config(
                {
                    MAP_REGIONS_ENTITY: partial(
                        render_map_region,
                        voxel_size=voxel_size,
                        height_range=HEIGHT_RANGE,
                        max_z=MAP_CEILING_M,
                    ),
                    SURFACE_MAP_ENTITY: partial(
                        render_surface_region,
                        voxel_size=voxel_size,
                        wall_clearance_m=wall_clearance_m,
                        clearance_clamp_m=1.0,
                        z_band=SURFACE_Z_BAND,
                    ),
                }
            ),
            "topics": [
                "tf",
                "odometry",
                "path",
                "planner_path",
                "nodes",
                "node_edges",
                "surface_map",
                "map_regions",
                "goal",
                "way_point",
                "goal_reached",
                # h264 or jpeg, whichever GO2DDS's video_encoding serves
                "video",
                "image",
                "camera_info",
            ],
        },
    ),
).global_config(
    transport="zenoh",
    # a client: the router forwards to clients only, never between peers
    zenoh_mode="client",
    zenoh_connect=GO2_ROUTER,
    # the router appears well after the robot's dimos run, keep dialing until it does
    zenoh_connect_timeout=120.0,
    # the robot's stack owns the bus-wide `Coordinator` name; this one only watches
    serve_coordinator_rpc=False,
    n_workers=3,
    robot_model="unitree_go2",
)
