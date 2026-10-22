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

"""G1-specific Rerun visual helpers (robot dimensions, TF overrides)."""

from __future__ import annotations

from functools import cache
from importlib import import_module
from typing import Any

import numpy as np
import rerun as rr
import rerun.blueprint as rrb

from dimos.core.coordination.blueprints import Blueprint
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.robot.unitree.g1.model.config import G1_URDF_PATH
from dimos.visualization.rerun.constants import ViewerBackend
from dimos.visualization.rerun.urdf_robot import (
    UrdfRobotJointStateRerunFactory,
    UrdfRobotStaticRerunFactory,
)
from dimos.visualization.vis_module import vis_module

G1_RERUN_ROOT = "world/odom/g1"
G1_RERUN_URDF = "g1_urdf/g1.fixed.urdf"

# Classic costmap palette, indexed by grid value + 1:
# transparent unknown, blue free, orange occupied, red lethal.
_COSTMAP_LOOKUP_TABLE = np.zeros((102, 4), dtype=np.uint8)
_COSTMAP_LOOKUP_TABLE[0] = (0, 0, 0, 0)
_COSTMAP_LOOKUP_TABLE[1] = (72, 73, 129, 255)
_COSTMAP_LOOKUP_TABLE[2:101] = (255, 140, 0, 255)
_COSTMAP_LOOKUP_TABLE[101] = (220, 30, 30, 255)


def g1_costmap(grid: Any, z_offset: float = 0.02) -> Any:
    """Render an OccupancyGrid with the classic costmap palette.

    The default z_offset lifts the mesh 2cm off the floor plane to avoid
    z-fighting with the ground.
    """
    return grid.to_rerun(color_lookup_table=_COSTMAP_LOOKUP_TABLE, z_offset=z_offset)


def g1_urdf_static_robot(root_path: str = G1_RERUN_ROOT) -> UrdfRobotStaticRerunFactory:
    """Create a static Rerun logger for the G1 URDF visual meshes."""
    return UrdfRobotStaticRerunFactory(urdf_path=G1_RERUN_URDF, root_path=root_path)


def g1_urdf_joint_state(root_path: str = G1_RERUN_ROOT) -> UrdfRobotJointStateRerunFactory:
    """Create a Rerun JointState converter for the G1 URDF."""
    return UrdfRobotJointStateRerunFactory(urdf_path=G1_RERUN_URDF, root_path=root_path)


def g1_static_robot(rr: Any) -> list[Any]:
    """Static G1 humanoid wireframe box attached to the sensor TF frame.

    Half-sizes are ~50x40x120 cm (the G1 humanoid), and the box is
    centered 0.6m below the sensor (lidar mounted at head height).
    """
    return [
        rr.Boxes3D(
            half_sizes=[0.25, 0.20, 0.6],
            centers=[[0, 0, -0.6]],
            colors=[(0, 255, 127)],
            fill_mode="MajorWireframe",
        ),
        rr.Transform3D(parent_frame="tf#/sensor"),
    ]


def g1_odometry_tf_override(odom: Any) -> Any:
    """Publish odometry as a TF frame so sensor_scan/path/robot can reference it.

    The z is zeroed because point clouds already have the full init_pose
    transform applied (ground at z≈0). Using the raw odom.z (= mount height)
    would double-count the vertical offset.
    """
    tf = rr.Transform3D(
        translation=[odom.x, odom.y, 0.0],
        rotation=rr.Quaternion(
            xyzw=[
                odom.orientation.x,
                odom.orientation.y,
                odom.orientation.z,
                odom.orientation.w,
            ]
        ),
        parent_frame="tf#/map",
        child_frame="tf#/sensor",
    )
    return [
        ("tf#/sensor", tf),
    ]


def _g1_motor_view() -> Any:
    return rrb.Blueprint(
        rrb.Spatial3DView(
            origin="world",
            name="Unitree G1",
            background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
            line_grid=rrb.LineGrid3D(plane=rr.components.Plane3D.XY.with_distance(0.0)),
        ),
        rrb.TimePanel(state="collapsed"),
    )


def _g1_nav_path(path: NavPath) -> Any:
    return path.to_rerun(z_offset=0.3)


@cache
def _g1_pelvis_to_mid360() -> Any:
    """Rest-pose pelvis-to-lidar transform, loaded only for hardware visualization."""
    urdf_path = G1_URDF_PATH
    urdf = import_module("yourdfpy").URDF.load(str(urdf_path), load_meshes=False)
    urdf.update_cfg(np.zeros(len(urdf.actuated_joint_names)))
    return urdf.get_transform("mid360_link", "pelvis")


def _g1_real_odometry_root(odom: Any) -> Any:
    """Convert the LIO's inverted Mid360 pose to the robot-mesh pelvis pose."""
    t_world_mid360 = np.eye(4)
    # The hardware mounting flip is not represented in this URDF.
    t_world_mid360[:3, :3] = odom.orientation.to_rotation_matrix() @ np.diag([1.0, -1.0, -1.0])
    t_world_mid360[:3, 3] = (odom.x, odom.y, odom.z)
    t_world_pelvis = t_world_mid360 @ np.linalg.inv(_g1_pelvis_to_mid360())
    q = Quaternion.from_rotation_matrix(t_world_pelvis[:3, :3])
    return rr.Transform3D(
        translation=t_world_pelvis[:3, 3].tolist(),
        rotation=rr.Quaternion(xyzw=[q.x, q.y, q.z, q.w]),
    )


def _g1_real_costmap(grid: Any) -> Any:
    # Nominal standing pelvis height in the LIO boot frame, as before extraction.
    ground_z = -(float(_g1_pelvis_to_mid360()[2, 3]) + 0.74)
    return g1_costmap(grid, z_offset=ground_z + 0.02)


def g1_viewer(viewer_backend: ViewerBackend, *, simulated: bool, joint_topic: str) -> Blueprint:
    """Display a G1 using the selected controller's actual measured-joint stream."""
    root = G1_RERUN_ROOT if simulated else "world/odometry/g1"
    joints_entity = f"world/{joint_topic.lstrip('/')}"
    config: dict[str, Any] = {
        "blueprint": _g1_motor_view,
        "visual_override": {
            "world/color_image": None,
            "world/camera_info": None,
            "world/depth_image": None,
            "world/depth_camera_info": None,
            joints_entity: g1_urdf_joint_state(root_path=root),
            "world/global_costmap": g1_costmap,
            "world/navigation_costmap": g1_costmap,
            "world/path": _g1_nav_path,
        },
        "max_hz": {
            joints_entity: 20.0,
            "world/g1_imu": 10.0,
            "world/low_level_imu": 10.0,
            "world/imu": 10.0,
            "world/g1_motor_states": 10.0,
            "world/motor_states": 10.0,
            "world/g1_motor_command": 10.0,
            "world/motor_command": 10.0,
            "world/odometry": 15.0,
            "world/global_map": 1.0,
            "world/global_costmap": 2.0,
            "world/navigation_costmap": 2.0,
            # Do not let the planner's preceding empty path throttle the new path.
            "world/path": 0,
        },
        "static": {root: g1_urdf_static_robot(root_path=root)},
    }
    if not simulated:
        config["visual_override"].update(
            {
                "world/odometry": _g1_real_odometry_root,
                "world/global_costmap": _g1_real_costmap,
                "world/navigation_costmap": _g1_real_costmap,
                "world/lidar": None,
                "world/lidar_raw": None,
            }
        )
    return vis_module(viewer_backend=viewer_backend, rerun_config=config)
