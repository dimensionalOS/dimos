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

"""Every Metal-in-MuJoCo stack maps the bench from the sim camera's cloud and plans around it."""

import pytest

from dimos.core.coordination.blueprints import Blueprint
from dimos.manipulation.manipulation_module import ManipulationModule, ManipulationModuleConfig
from dimos.mapping.pointclouds.box_filter import PointCloudBoxFilter
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap, RayTracingVoxelMapConfig
from dimos.robot.manipulators.makermods_metal.blueprints.autoresearch import autoresearch_metal_sim
from dimos.robot.manipulators.makermods_metal.blueprints.learning_collection_sim import (
    metal_quest_collection_sim,
)
from dimos.robot.manipulators.makermods_metal.blueprints.sim import metal_sim
from dimos.robot.urdf_manager import UrdfManager
from dimos.robot.urdf_manager_point_clearing import UrdfManagerPointClearing
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule, MujocoSimModuleConfig


def _topics(blueprint: Blueprint) -> dict[tuple[type, str], str]:
    """The name each module's stream connects on, keyed by (module, stream)."""
    return {
        (atom.module, stream.name): blueprint.remapping_map.get(
            (atom.name, stream.name), stream.name
        )
        for atom in blueprint.active_blueprints
        for stream in atom.streams
    }


@pytest.mark.parametrize(
    "blueprint", [metal_sim, metal_quest_collection_sim, autoresearch_metal_sim]
)
def test_the_sim_cloud_becomes_the_planner_map_through_the_self_and_box_filters(
    blueprint: Blueprint,
) -> None:
    atoms = blueprint.active_blueprints
    topics = _topics(blueprint)
    [sim] = [atom for atom in atoms if atom.module is MujocoSimModule]
    [planner] = [atom for atom in atoms if atom.module is ManipulationModule]
    [mapper] = [atom for atom in atoms if atom.module is RayTracingVoxelMap]
    # One UrdfManager serves the URDF, and it is the self filter.
    [urdf] = [atom for atom in atoms if issubclass(atom.module, UrdfManager)]
    assert urdf.module is UrdfManagerPointClearing

    camera = MujocoSimModuleConfig.model_validate(sim.kwargs)
    assert camera.enable_pointcloud and camera.enable_mujoco_lidar
    assert camera.mujoco_lidar_in_camera_frame
    assert topics[MujocoSimModule, "pointcloud"] == topics[UrdfManagerPointClearing, "pointcloud"]
    assert (
        topics[UrdfManagerPointClearing, "filtered_pointcloud"]
        == topics[RayTracingVoxelMap, "lidar"]
    )
    assert topics[RayTracingVoxelMap, "global_map"] == topics[PointCloudBoxFilter, "cloud"]
    assert topics[PointCloudBoxFilter, "filtered_cloud"] == topics[ManipulationModule, "voxel_map"]
    # A viewer takes a cloud named like a map for the map.
    assert "voxel" in topics[ManipulationModule, "voxel_map"]

    mapping = RayTracingVoxelMapConfig.model_validate(mapper.kwargs)
    planning = ManipulationModuleConfig.model_validate(planner.kwargs)
    assert mapping.world_frame == planning.world_frame
    assert mapping.voxel_size == planning.voxel_map_resolution
    # About one cell, so a voxel the camera now sees through just above the bench clears.
    assert mapping.grace_depth < 2 * mapping.voxel_size
