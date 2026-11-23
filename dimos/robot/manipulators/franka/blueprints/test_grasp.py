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

import math
import platform
import sys

import numpy as np
import pytest

# The tests extra excludes yourdfpy on Linux ARM because embreex has no wheel.
if sys.platform == "linux" and platform.machine() == "aarch64":
    pytest.importorskip(
        "yourdfpy", reason="yourdfpy is unavailable in the Linux ARM test environment"
    )

from scipy.spatial.transform import Rotation

from dimos.core.coordination.blueprints import Blueprint
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.mapping.pointclouds.box_filter import PointCloudBoxFilter
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.robot.manipulators.franka.blueprints.grasp import fr3_grasp
from dimos.robot.manipulators.franka.config import fr3_wrist_camera_transform
from dimos.robot.manipulators.franka.connection import FrankaConnection
from dimos.robot.urdf_manager_point_clearing import UrdfManagerPointClearing


def _topics(blueprint: Blueprint) -> dict[tuple[type, str], str]:
    """The name each module's stream connects on, keyed by (module, stream)."""
    return {
        (atom.module, stream.name): blueprint.remapping_map.get(
            (atom.name, stream.name), stream.name
        )
        for atom in blueprint.active_blueprints
        for stream in atom.streams
    }


def test_the_planner_map_passes_through_the_box_filter() -> None:
    topics = _topics(fr3_grasp.solidify())
    planner_map = topics[ManipulationModule, "voxel_map"]
    assert (
        topics[UrdfManagerPointClearing, "filtered_pointcloud"]
        == topics[RayTracingVoxelMap, "lidar"]
    )
    assert topics[RayTracingVoxelMap, "global_map"] == topics[PointCloudBoxFilter, "cloud"]
    assert topics[PointCloudBoxFilter, "filtered_cloud"] == planner_map


def test_the_stack_drives_the_real_arm_and_the_wrist_camera() -> None:
    modules = [atom.module for atom in fr3_grasp.solidify().active_blueprints]
    assert FrankaConnection in modules
    assert RealSenseCamera in modules


def test_the_wrist_camera_looks_down_the_hand_beside_the_fingers() -> None:
    transform = fr3_wrist_camera_transform()
    assert (transform.frame_id, transform.child_frame_id) == ("fr3_hand", "camera_link")
    q = transform.rotation
    rotation = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    # camera_link's x is the lens axis: within a few degrees of the hand's +z.
    assert math.degrees(math.acos(rotation[:, 0] @ [0.0, 0.0, 1.0])) < 5.0
    # A few centimetres off the hand's axis, behind the fingertips (0.1034 m).
    t = np.array([transform.translation.x, transform.translation.y, transform.translation.z])
    assert 0.03 < np.hypot(t[0], t[1]) < 0.09
    assert 0.0 < t[2] < 0.1034


def test_the_self_filter_can_place_the_cloud_in_the_models_root() -> None:
    # UrdfManagerPointClearing looks up root link -> camera on tf; the planner
    # publishes world -> each of tf_extra_links, so the root must be one of them.
    from dimos.manipulation.manipulation_module import ManipulationModule
    from dimos.robot.manipulators.franka.config import fr3_model

    atom = next(a for a in fr3_grasp.solidify().active_blueprints if a.module is ManipulationModule)
    root = fr3_model(robot="arm").load().root_link
    assert root in atom.kwargs["model"].tf_extra_links
