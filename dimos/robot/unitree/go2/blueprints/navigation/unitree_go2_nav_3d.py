#!/usr/bin/env python3
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

"""3d navigation on Go2 with ray tracing and MLS planning"""

from typing import Any

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.hardware.sensors.lidar.pointlio.mount_correction import mount_correction_matrix
from dimos.hardware.sensors.lidar.pointlio.recorder import PointlioRecorder
from dimos.hardware.sensors.lidar.virtual_mid360.recorder import Mid360PcapRecorder
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.navigation.basic_path_follower.module import BasicPathFollower
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_3d.mls_planner.goal_relay import GoalRelay
from dimos.navigation.nav_3d.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import rerun_config
from dimos.robot.unitree.go2.config import mid360_rotated_urdf_path, mid360_urdf_path
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.robot.urdf_loader import UrdfLoader
from dimos.visualization.vis_module import vis_module

# Correction that un-rotates a physically-rotated Mid-360's Point-LIO cloud +
# odometry back into the normal mount frame. Used by the `_rotated` blueprint; the
# plain blueprint passes None (identity — sensor mounted normally).
_rotated_mount_correction = mount_correction_matrix(
    original_static_tf=mid360_rotated_urdf_path, new_static_tf=mid360_urdf_path
)

# GO2Connection is a TfModule; feeding it the normal mount publishes those frames
# on its static interval, matching the corrected cloud both blueprints emit.
go2_mid360_model = UrdfLoader(name="go2_mid360", model_path=mid360_urdf_path)

voxel_size = 0.08
# Height of the head-mounted lidar above the ground while standing.
go2_lidar_height = 0.5


def _render_global_map(msg: Any) -> Any:
    return msg.to_rerun()


def _render_path(msg: Any) -> Any:
    # The planner emits an empty path when it finds no route to the goal.
    # Logging those would blank the line, so drop them and keep the last path.
    if len(msg.poses) == 0:
        return None
    return msg


def _render_surface_map(msg: Any) -> Any:
    # Walkable surface the planner solved on. Cyan voxel boxes so it reads as a
    # distinct carpet over the raw global_map.
    return msg.to_rerun(voxel_size=voxel_size, colors=[80, 200, 255], mode="boxes")


def _render_nodes(msg: Any) -> Any:
    # Planning-graph nodes as fat magenta spheres so they pop above the surface.
    return msg.to_rerun(voxel_size=0.25, colors=[255, 0, 200], mode="spheres")


def _render_node_edges(msg: Any) -> Any:
    # LineSegments3D already colors edges by traversability (green/yellow/red).
    return msg.to_rerun()


def _static_robot_body(rr: Any) -> list[Any]:
    """Go2-shaped box + forward arrow on pointlio's body frame. The hack fakes a
    level, forward-facing mount, so the box needs no counter-rotation."""
    return [
        rr.Boxes3D(half_sizes=[0.35, 0.155, 0.2], colors=[(0, 255, 127)]),
        # Red arrow out the nose marks the robot's forward (+x) direction.
        rr.Arrows3D(
            origins=[(0.35, 0.0, 0.0)],
            vectors=[(0.3, 0.0, 0.0)],
            colors=[(255, 40, 40)],
        ),
        rr.Transform3D(parent_frame="tf#/body"),
    ]


_nav_rerun_config = {
    **rerun_config,
    "max_hz": {
        **rerun_config["max_hz"],
        "world/global_map": 1.0,
        "world/local_map": 1.0,
    },
    "memory_limit": "256MB",
    # base_link tf comes from the go2 internal odometry, which is not the map
    # frame. Anchor the robot box to pointlio's body frame instead and hide the
    # camera frustum that rides base_link. Use a dedicated entity path (not
    # world/tf/body) so the box's Transform3D doesn't collide with the live
    # odom->body TF that PointLio logs onto world/tf/body.
    "static": {"world/robot_body": _static_robot_body},
    "visual_override": {
        **rerun_config["visual_override"],
        "world/global_map": _render_global_map,
        "world/path": _render_path,
        "world/camera_info": None,
        "world/color_image": None,
        "world/lidar": None,
        "world/surface_map": _render_surface_map,
        "world/nodes": _render_nodes,
        "world/node_edges": _render_node_edges,
    },
}


def _nav_3d(mount_correction: list[float] | None) -> Blueprint:
    """Go2 3D nav + recording stack. ``mount_correction`` un-rotates a physically
    rotated Mid-360 back into the normal mount frame; None = normal mount (no
    correction)."""
    return autoconnect(
        vis_module(viewer_backend=global_config.viewer, rerun_config=_nav_rerun_config),
        # "mcf" for stair traversal
        GO2Connection.blueprint(
            static_transforms=dict(go2_mid360_model.static_transforms),
            lidar=False,
            camera=False,
            motion_mode="mcf",
        ).remappings(
            [
                (GO2Connection, "lidar", "lidar_l1"),
                (GO2Connection, "odom", "odom_go2"),
            ]
        ),
        # gravity_align is off so pointlio runs in the raw mount frame. For the
        # rotated blueprint, transform rewrites its cloud + odometry (and odom->body
        # TF) into the normal mount; otherwise transform is None (identity).
        # auto_build recompiles the native binary for the transform support.
        PointLio.blueprint(
            gravity_align=False,
            space_down_sample=False,
            auto_build=True,
            transform=mount_correction,
        ),
        # Record the corrected (normal-mount) cloud + odometry, not PointLio's raw
        # tilted output, so a replay reproduces what the nav stack actually consumed.
        # The remap only renames the wire topics to PointLio's lidar/odometry; the
        # recorder's ports stay pointlio_lidar/pointlio_odometry, so that's what the
        # db streams are named.
        PointlioRecorder.blueprint().remappings(
            [
                (PointlioRecorder, "pointlio_lidar", "lidar"),
                (PointlioRecorder, "pointlio_odometry", "odometry"),
            ]
        ),
        # Raw Livox UDP capture (tcpdump). Pulls the lidar IP + iface from env
        # (DIMOS_MID360_LIDAR_IP / DIMOS_MID360_PCAP_IFACE); pcap lands in recordings/.
        Mid360PcapRecorder.blueprint(),
        RayTracingVoxelMap.blueprint(
            voxel_size=voxel_size,
            emit_every=1,
            global_emit_every=50,
            max_health=10,
            graze_cos=0.85,
        ),
        # global_map is remapped off so the planner runs purely on the
        # incremental local_map + region_bounds pair.
        MLSPlannerNative.blueprint(
            world_frame="odom",
            voxel_size=voxel_size,
            robot_height=go2_lidar_height,
            wall_clearance_m=0.2,
            wall_buffer_m=0.75,
            wall_buffer_weight=100.0,
            step_threshold_m=0.16,
            step_penalty_weight=1.0,
            viz_publish_hz=2.0,
        ).remappings([(MLSPlannerNative, "global_map", "global_map_unused")]),
        GoalRelay.blueprint(),
        BasicPathFollower.blueprint(speed=0.5, heading_gain=0.4, max_angular=0.6),
        MovementManager.blueprint(),
    )


# Standard mount (level, forward-facing): no correction.
unitree_go2_nav_3d = _nav_3d(None).global_config(
    n_workers=10, robot_model="unitree_go2", obstacle_avoidance=False
)

# Rotated mount: un-rotate the cloud + odometry back into the normal mount frame.
unitree_go2_nav_3d_rotated = _nav_3d(_rotated_mount_correction).global_config(
    n_workers=10, robot_model="unitree_go2", obstacle_avoidance=False
)
