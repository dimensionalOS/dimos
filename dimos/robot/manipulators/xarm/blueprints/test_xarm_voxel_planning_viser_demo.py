# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

import xml.etree.ElementTree as ET

import numpy as np
import pytest

from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.groups.registry import PlanningGroupRegistry
from dimos.manipulation.planning.kinematics.jacobian_ik import JacobianIK
from dimos.manipulation.planning.world.roboplan_world import RoboPlanWorld
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.perception.point_cloud_self_filter import PointCloudSelfFilter, SelfFilterBox
from dimos.protocol.tf.tf_pose_source import TfPoseSource
from dimos.robot.manipulators.xarm.blueprints.simulation import (
    XARM_VOXEL_PLANNING_RESOLUTION,
    xarm_voxel_planning_viser_demo,
)
from dimos.robot.manipulators.xarm.config import XARM7_SIM_PATH, make_xarm7_sim_robot_config
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
    assert manip_kwargs["planner"] == {"backend": "roboplan"}
    assert manip_kwargs["kinematics"] == {"backend": "jacobian"}
    assert manip_kwargs["visualization"] == {"backend": "viser"}
    assert manip_kwargs["planning_world_frame"] == "world"
    assert manip_kwargs["planning_voxel_resolution"] == 0.05
    assert manip_kwargs["planning_collision_max_age_s"] == 1.0
    assert "PickAndPlaceModule" not in {module.__name__ for module in modules}


def test_sim_planning_world_matches_mujoco_robot_mount() -> None:
    robot = make_xarm7_sim_robot_config()
    model = ET.parse(XARM7_SIM_PATH.with_name("xarm7.xml")).getroot()
    link_base = model.find("./worldbody/body[@name='link_base']")

    assert link_base is not None
    mujoco_base_position = [float(value) for value in link_base.attrib["pos"].split()]
    assert robot.base_pose.x == 0.0
    assert robot.base_pose.y == 0.0
    assert robot.base_pose.z == mujoco_base_position[2]
    assert [float(value) for value in robot.xacro_args["attach_xyz"].split()] == [0.0, 0.0, 0.0]


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
        "robot_clear_mask",
        "global_map",
    }

    filter_kwargs = _atom(PointCloudSelfFilter).kwargs
    assert filter_kwargs["robot_model"].base_link == "link_base"
    assert filter_kwargs["padding_m"] == 0.025
    assert filter_kwargs["additional_boxes"] == [
        SelfFilterBox(
            link="link7",
            center_xyz=(0.0, 0.0, 0.08),
            size_xyz=(0.08, 0.18, 0.18),
        )
    ]
    assert filter_kwargs["tf_tolerance_s"] == 0.02
    assert filter_kwargs["tf_forward_tolerance_s"] == 0.05
    assert xarm_voxel_planning_viser_demo.remapping_map == {
        (RayTracingVoxelMap.name, "lidar"): "filtered_pointcloud",
        (ManipulationModule.name, "planning_voxel_map"): "global_map",
    }

    assert _atom(RayTracingVoxelMap).kwargs["voxel_size"] == 0.05
    assert _atom(RayTracingVoxelMap).kwargs["pose_match_tolerance_s"] == 0.02
    assert XARM_VOXEL_PLANNING_RESOLUTION == 0.05
    assert hasattr(ManipulationModule, "committed_planning_collision_snapshot")


def test_demo_is_simulation_only_and_pose_feeds_voxel_odometry() -> None:
    assert xarm_voxel_planning_viser_demo.global_config_overrides == {"simulation": "mujoco"}
    pose_kwargs = _atom(TfPoseSource).kwargs
    assert pose_kwargs["target_frame"] == "world"
    assert pose_kwargs["source_frame"] == "wrist_camera_color_optical_frame"
    assert pose_kwargs["publish_rate_hz"] == 50.0

    pose_out = next(stream for stream in _atom(TfPoseSource).streams if stream.name == "odometry")
    voxel_in = next(
        stream for stream in _atom(RayTracingVoxelMap).streams if stream.name == "odometry"
    )
    assert pose_out.direction == "out"
    assert voxel_in.direction == "in"


@pytest.mark.self_hosted
def test_roboplan_jacobian_reaches_world_frame_xarm_gizmo_target() -> None:
    robot = make_xarm7_sim_robot_config()
    assert robot.home_joints is not None
    seed = JointState(name=robot.joint_names, position=robot.home_joints)
    world = RoboPlanWorld()
    robot_id = world.add_robot(robot)
    world.finalize()
    group = PlanningGroupRegistry([robot]).get("arm/manipulator")
    with world.scratch_context() as context:
        world.set_joint_state(context, robot_id, seed)
        current = world.get_group_ee_pose(context, group.id)
    target = PoseStamped(
        frame_id="world",
        position=Vector3(current.position.x, current.position.y + 0.05, current.position.z),
        orientation=current.orientation,
    )

    result = JacobianIK().solve_pose_targets(
        world,
        {group: target},
        seed=seed,
        check_collision=False,
        max_attempts=1,
    )

    assert result.is_success()
    assert result.joint_state is not None
    assert result.position_error < 0.001
    assert np.max(np.abs(np.asarray(result.joint_state.position) - seed.position)) > 0.05
