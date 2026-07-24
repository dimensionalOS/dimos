# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.perception.point_cloud_self_filter import PointCloudSelfFilter
from dimos.protocol.tf.tf_pose_source import TfPoseSource
from dimos.robot.manipulators.xarm.blueprints.simulation import (
    XARM_VOXEL_PLANNING_RESOLUTION,
    xarm_voxel_planning_viser_demo,
)
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule


def _atom(module: type):
    return next(atom for atom in xarm_voxel_planning_viser_demo.blueprints if atom.module is module)


def test_demo_composes_simulation_voxel_planning_modules() -> None:
    modules = {atom.module for atom in xarm_voxel_planning_viser_demo.blueprints}
    assert {
        ManipulationModule,
        MujocoSimModule,
        PointCloudSelfFilter,
        TfPoseSource,
        RayTracingVoxelMap,
    } <= modules

    manip_kwargs = _atom(ManipulationModule).kwargs
    assert manip_kwargs["world_backend"] == "roboplan"
    assert manip_kwargs["planner_name"] == "roboplan"
    assert manip_kwargs["kinematics"] == {"backend": "pink"}
    assert manip_kwargs["visualization"] == {"backend": "viser"}
    assert manip_kwargs["planning_world_frame"] == "world"
    assert manip_kwargs["planning_voxel_resolution"] == 0.05
    assert "PickAndPlaceModule" not in {module.__name__ for module in modules}


def test_demo_wires_camera_filter_voxel_map_and_snapshot() -> None:
    mujoco_kwargs = _atom(MujocoSimModule).kwargs
    assert mujoco_kwargs["enable_pointcloud"] is True
    assert mujoco_kwargs["camera_name"] == "wrist_camera"
    assert mujoco_kwargs["base_frame_id"] == ""

    assert {stream.name for stream in _atom(MujocoSimModule).streams} >= {"pointcloud"}
    assert {stream.name for stream in _atom(PointCloudSelfFilter).streams} >= {
        "pointcloud",
        "filtered_pointcloud",
    }
    assert {stream.name for stream in _atom(RayTracingVoxelMap).streams} >= {
        "lidar",
        "odometry",
        "global_map",
    }

    filter_kwargs = _atom(PointCloudSelfFilter).kwargs
    assert {region.frame_id for region in filter_kwargs["regions"]} == {"link7", "link_tcp"}
    assert filter_kwargs["drop_cloud_on_missing_tf"] is True
    assert xarm_voxel_planning_viser_demo.remapping_map == {
        (RayTracingVoxelMap.name, "lidar"): "filtered_pointcloud",
        (ManipulationModule.name, "planning_voxel_map"): "global_map",
    }

    assert _atom(RayTracingVoxelMap).kwargs["voxel_size"] == 0.05
    assert _atom(RayTracingVoxelMap).kwargs["pose_match_tolerance_s"] == 0.1
    assert XARM_VOXEL_PLANNING_RESOLUTION == 0.05
    assert hasattr(ManipulationModule, "committed_planning_collision_snapshot")


def test_demo_is_simulation_only_and_pose_feeds_voxel_odometry() -> None:
    assert xarm_voxel_planning_viser_demo.global_config_overrides == {"simulation": "mujoco"}
    pose_kwargs = _atom(TfPoseSource).kwargs
    assert pose_kwargs["target_frame"] == "world"
    assert pose_kwargs["source_frame"] == "wrist_camera_color_optical_frame"

    pose_out = next(stream for stream in _atom(TfPoseSource).streams if stream.name == "odometry")
    voxel_in = next(
        stream for stream in _atom(RayTracingVoxelMap).streams if stream.name == "odometry"
    )
    assert pose_out.direction == "out"
    assert voxel_in.direction == "in"
