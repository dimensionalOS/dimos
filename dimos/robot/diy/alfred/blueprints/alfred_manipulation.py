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

"""alfred-perception plus mobile manipulation: the whole-body planner, its obstacle map and picks.

    dimos run alfred-manipulation              # GraspGenX grasps (its own env, synced on first start)
    dimos run alfred-manipulation-heuristic    # one top-down grasp per object, no second environment

The pillar, the OpenArms and the base are planned by the ManipulationModule on the whole-body model
in the robot's own frame (``base_link``): arm targets ride with the robot and never depend on
odometry, and a plan that drives the base is a motion from where the robot stands, which the
coordinator's base task lays down from its odometry pose only while it runs. The Controller's arm
view drives that planner, so there is no viser.

The D455's cloud is self-filtered and voxelised into a short-range map in ``odom`` (so a drive leaves
the bench where it is) and handed to the planner as ``voxel_map``, re-expressed in ``base_link``,
with the robot as it stands carved back out of it. ``UrdfManagerPointClearing`` does the filtering
and takes over from alfred-base's ``UrdfManager``: one module poses the robot and cuts it out.

A pick scans through localize, so the objects are in ``odom`` like the map, and PickAndPlace
re-expresses an object's cloud in ``base_link`` through tf before it asks for grasps.

An operator can hold the robot: alfred-base's coordinator runs the operator hold; ``OperatorHandoff``
gives an agent ``request_operator`` and keeps the operator event log. The stack's own agent
(McpClient, Alfred's prompt) joins alfred-perception's McpServer when ``OPENAI_API_KEY`` is set.
"""

from __future__ import annotations

import math
import os
from typing import Any, Literal

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.skills.observe_skill import ObserveSkill
from dimos.control.operator_handoff import OperatorHandoff
from dimos.control.tasks.trajectory_task.trajectory_task import JOINT_TRAJECTORY_TASK_NAME
from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.grasping.grasp_gen_x.module import GraspGenXModule
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.manipulation.pick_and_place_module import ArmReach, PickAndPlaceModule
from dimos.mapping.pointclouds.box_filter import PointCloudBoxFilter
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.diy.alfred.alfred_model import (
    ALFRED_PLANAR_BASE,
    ALFRED_SELF_FILTER_MODEL,
    alfred_arm_joints,
    alfred_planar_model_config,
)
from dimos.robot.diy.alfred.blueprints.alfred_base import (
    ALFRED_GRIPPER_TASKS,
    BASE_FRAME,
    BASE_TRAJECTORY_TASK_NAME,
    JOINT_TF_HZ,
    ODOM_FRAME,
)
from dimos.robot.diy.alfred.blueprints.alfred_perception import alfred_perception
from dimos.robot.diy.alfred.gripper import (
    OPENARM_V10_GRASP_FRAME_TO_TCP,
    OPENARM_V10_SWEEP_VOLUME,
)
from dimos.robot.diy.alfred.pillar_connection import PILLAR_LIFT_JOINT
from dimos.robot.diy.alfred.system_prompt import ALFRED_SYSTEM_PROMPT
from dimos.robot.manipulators.common.blueprints import planner
from dimos.robot.urdf_manager import UrdfManager
from dimos.robot.urdf_manager_point_clearing import UrdfManagerPointClearing

# Edge of the arm map's voxels and of the octree the planner builds from them: the two must be
# equal. At 2 cm the bench's layer reaches 1.5 cm above the real surface (3 cm at 5 cm voxels), a
# 5 cm can spans 3 cells instead of 1 or 2, and a collision check costs 2.5 ms against 2.1 ms with
# no map (2026-10-25, see the README).
MANIP_VOXEL_M = 0.02

MANIP_VOXEL_MAP = "ManipVoxelMap"
"""The arm map's instance name; its streams are renamed so a nav map's would stay put."""
# Measured from the camera on the mast (1.55 m up): to the far edge of the bench, not the
# floor beyond it. At 2.5 m the map held 7000 voxels, a third of them floor and far walls,
# and the planner spent 160 ms a collision check on them (2026-10-25).
MANIP_MAP_RANGE_M = 1.5

# base_link -> camera runs through UrdfManager's edges, stamped with the joint state that made
# them, and a CAN read on the real arms stalls that state for up to ~0.3 s now and then.
TF_TOLERANCE_S = 0.3
JOINT_STATE_TOLERANCE_S = 0.3
MANIP_GRACE_DEPTH_M = 0.05
MANIP_GRAZE_COS = 0.5
MANIP_SHADOW_DEPTH_M = 0.0
SELF_FILTER_PADDING_M = 0.05
MANIP_MAP_SELF_FILTER = "ManipMapSelfFilter"
"""The second self filter's instance name: it carves the robot out of the planner's map."""
# A map cell the robot touches has its centre within half the cell's diagonal of the robot (17.3 mm
# at 2 cm). The 3 mm over that is 20 times a resting arm's jitter between a map's stamp and a plan's
# start (one encoder count, 0.15 mm at the fingertips); more would open the obstacle around the
# hand for its next motion.
MANIP_MAP_CARVE_PADDING_M = MANIP_VOXEL_M * math.sqrt(3) / 2 + 0.003
MANIP_MAP_EMIT_EVERY = 5  # of the camera's 5 Hz: 1 Hz

# How far an OpenArm gets on its own: shoulder (link1, on tf from UrdfManager) to wrist with
# the arm straight, 0.066 + 0.154 + 0.096 + 0.121 m of URDF offsets, and the v1.0 hand's
# 0.186 m from wrist to grasp_frame. A bench object 0.70 m out needs the wrist 0.58 m from
# the shoulder for a top-down grasp: a pick says so at once instead of 12 s of IK.
OPENARM_REACH_M = 0.44
OPENARM_WRIST_TO_TIP_M = 0.186
ALFRED_ARM_REACH = {
    f"{side}_manipulator": ArmReach(
        radius=OPENARM_REACH_M,
        shoulder_frame=f"openarm_{side}_link1",
        wrist_to_tip=OPENARM_WRIST_TO_TIP_M,
    )
    for side in ("left", "right")
}

# The arm planner's map, cropped in base_link to where the arms reach with the base helping: kept
# in odom, the map remembers everything the camera saw as the base drove round the room, and at
# 200148 points the planner refused every map and planned on one hours stale (2026-10-26).
# From alfred_v1.urdf, the shoulders (openarm_*_link1) are at x 0.14, y +-0.155, and 0.95-1.45 m
# up over the lift's range; each arm reaches OPENARM_REACH_M + OPENARM_WRIST_TO_TIP_M = 0.63 m
# to the fingertips, so on their own x -0.49..0.77, y -0.78..0.78, z 0.32..2.07. A whole-body
# plan drives the base too: 0.8 m more ahead, where the camera stops seeing the bench anyway (1.5 m
# from it, 1.53 m ahead at bench height); 0.5 m more to either side. Down to the floor the base
# drives over; behind, the arms' own reach, past the base's 0.37 m footprint. Only the voxels
# whose centres fall inside are re-rasterized: at 2 cm a crop with floor and a bench across it
# holds ~22k cells, and walled in on all four sides as well ~110k, still under the planner's
# 200k MAX_OCTREE_POINTS (test_alfred_stacks).
MANIP_MAP_CROP_MIN = (-0.5, -1.3, 0.0)
MANIP_MAP_CROP_MAX = (1.6, 1.3, 2.1)

GraspGenerator = Literal["heuristic", "graspgenx"]
"""Who answers the pick pipeline's ``propose_grasps``: the top-down heuristic, or GraspGenX, the
learned generator, in its own Python environment (``native/python/graspgenx``) with the v1.0
gripper's geometry (``alfred.gripper``)."""
# IsolatedPythonModule's 30 s default bounds only GraspGenX's READY handshake: `uv run --frozen
# --with-editable` and importing graspgenx_runtime.runtime with its dimos imports. On the Orin
# that runs while ten workers start and OWLv2 and EdgeTAM load, so 30 s is tight; the model
# itself loads afterwards, in start(), which this does not bound.
GRASPGENX_STARTUP_TIMEOUT_S = 90.0

# The planner's world is the robot itself. Anything that calls the planning frame "world"
# (the Controller's arm targets, the stock manipulation tooling) resolves through this
# identity edge under base_link, so a target is a pose on the robot: it rides along when the
# base drives and survives odometry jumps, and the planner's model root is the same base_link
# tf draws the robot at. The whole-body model's planar base joints are the base's motion
# *from here*: zero whenever a plan starts, and executed as a relative base trajectory.
_WORLD_IS_BASE = Transform(
    translation=Vector3(0.0, 0.0, 0.0),
    rotation=Quaternion.from_euler(Vector3(0.0, 0.0, 0.0)),
    frame_id=BASE_FRAME,
    child_frame_id="world",
)


def alfred_planner() -> Any:
    """The whole-body planner in base_link: the planar base joints are not fed from odometry, they
    are held at zero and a plan moves them from there. Its obstacle map arrives in base_link too."""
    return planner(
        model=alfred_planar_model_config(),
        visualization={"backend": "none"},
        world_frame=BASE_FRAME,
        voxel_map_resolution=MANIP_VOXEL_M,
        static_transforms=[_WORLD_IS_BASE],
        joint_state_defaults=dict.fromkeys(ALFRED_PLANAR_BASE.joint_names, 0.0),
        gripper_tasks=ALFRED_GRIPPER_TASKS,
        # Splits a whole-body plan; either side aborting cancels the other.
        trajectory_tasks={
            JOINT_TRAJECTORY_TASK_NAME: [*alfred_arm_joints(), PILLAR_LIFT_JOINT],
            BASE_TRAJECTORY_TASK_NAME: list(ALFRED_PLANAR_BASE.joint_names),
        },
    )


def alfred_obstacle_map_modules() -> tuple[Any, ...]:
    """The camera's cloud to the planner's ``voxel_map``: self filter, voxel map, crop, carve."""
    return (
        # The planner's own model: whatever it drives is dropped from the cloud, so the lift
        # and arms never become obstacles to themselves. It is the stack's UrdfManager as well
        # (the blueprints below disable alfred-base's).
        UrdfManagerPointClearing.blueprint(
            model=ALFRED_SELF_FILTER_MODEL,
            joint_tf_hz=JOINT_TF_HZ,
            unchanged_tf_hz=JOINT_TF_HZ,
            state_tolerance_s=JOINT_STATE_TOLERANCE_S,
            tf_tolerance_s=TF_TOLERANCE_S,
            tf_forward_tolerance_s=TF_TOLERANCE_S,
            # The filter's fingers are fixed closed; the real ones open 4.4 cm a side, and at
            # the default 1 cm an open finger in view became a voxel right where the grasp goes.
            padding_m=SELF_FILTER_PADDING_M,
        ),
        # Built in odom, on wheel odometry (good to a cm over a few metres): the bench stays
        # where it is when the base drives. Pinned to base_link, a whole-body pick that moved
        # the base 0.5 m and 0.9 rad left every voxel in the wrong place relative to the robot;
        # the planner refused all plans (COLLISION_AT_START) until the camera re-saw the bench
        # (2026-10-25). The box filter below hands it to the planner in base_link, cropped: the
        # mapper forgets nothing by distance (region_m and region_percentile only shape what it
        # sends on, not what it keeps), so the map grows with the room the base drives through.
        RayTracingVoxelMap.blueprint(
            instance_name=MANIP_VOXEL_MAP,
            voxel_size=MANIP_VOXEL_M,
            max_range=MANIP_MAP_RANGE_M,
            world_frame=ODOM_FRAME,
            # odom -> camera is odom -> base_link (the odometry relay, ~47 Hz) and UrdfManager's
            # 10 Hz chain; at the default 0.1 s a stalled joint state leaves clouds with no edge
            # close enough to their stamp, and they are dropped.
            tf_match_tolerance_s=TF_TOLERANCE_S,
            # Clearing, tuned on 60 s of this camera on the bench replayed through the mapper
            # (2026-10-25). A picked object has to leave the map, and a voxel within grace_depth
            # of a return is never cleared: at 20 cm (the default) or 10 cm a removed can kept
            # 67% or 34% of its voxels for good, at 5 cm it is gone in 3 s. graze_cos 0.5 spares
            # a surface from rays within 30 degrees of it (the far bench); at 0.7 a removed
            # can's top, seen as shallow as the bench, never quite cleared. No shadow: rays
            # carried past a return carved the layer under the bench top, which flickered
            # wherever the top straddles two voxel layers (3.4% of the workspace map changed
            # from one map to the next; 0.65% without).
            grace_depth=MANIP_GRACE_DEPTH_M,
            graze_cos=MANIP_GRAZE_COS,
            shadow_depth=MANIP_SHADOW_DEPTH_M,
            # ManipulationModule rebuilds its octree under the planner's lock for every map it
            # gets (60-70 ms each on the Orin): once a second is plenty for a bench.
            global_emit_every=MANIP_MAP_EMIT_EVERY,
        ).remappings(
            [
                (MANIP_VOXEL_MAP, "lidar", "filtered_pointcloud"),
                (MANIP_VOXEL_MAP, "loaded_map", "manip_loaded_map"),
                (MANIP_VOXEL_MAP, "global_map", "manip_global_map"),
                (MANIP_VOXEL_MAP, "local_map", "manip_local_map"),
                (MANIP_VOXEL_MAP, "local_map_fine", "manip_local_map_fine"),
                # Beside a nav map, its region_bounds pair with the nav local_map by stamp in
                # MLSPlannerNative, which keeps only the latest: this map's 5 Hz bounds would
                # displace the nav map's and drop its updates.
                (MANIP_VOXEL_MAP, "region_bounds", "manip_region_bounds"),
                (MANIP_VOXEL_MAP, "seed_map", "manip_seed_map"),
                (MANIP_VOXEL_MAP, "seed_bounds", "manip_seed_bounds"),
                (MANIP_VOXEL_MAP, "map_regions", "manip_map_regions"),
            ]
        ),
        # Re-expresses the odom map in the planner's frame at each map's stamp (the planner
        # drops a map in any other frame), crops it to the arms' reach around the robot, and
        # lets a pick leave the object it is picking out of it. Its output goes on to the map
        # self filter below.
        PointCloudBoxFilter.blueprint(
            target_frame=BASE_FRAME,
            tf_tolerance_s=TF_TOLERANCE_S,
            tf_forward_tolerance_s=TF_TOLERANCE_S,
            # Each voxel is re-rasterized onto the octree's own base_link cells. Moved one centre
            # per voxel, 17% of the bench top fell in empty octree cells with the base turned 42
            # degrees since the run started (6% at 15, 1% at 0 or 90); re-rasterized, 0.1%, under
            # the map's own 0.4% (60 s of the bench replayed, 2026-10-25). The planner gets ~4000
            # cells for ~3500 voxels, and the filter spends 4 ms a map on it.
            target_voxel_m=MANIP_VOXEL_M,
            crop_min=MANIP_MAP_CROP_MIN,
            crop_max=MANIP_MAP_CROP_MAX,
        ).remappings(
            [
                (PointCloudBoxFilter, "cloud", "manip_global_map"),
                (PointCloudBoxFilter, "filtered_cloud", "manip_uncarved_map"),
            ]
        ),
        # Carves the robot, as it stood at the map's stamp, out of the map: a cell the robot fills
        # is no obstacle, so the start of a plan never collides with the map. The map remembers
        # the bench in odom, and its 2 cm layer reaches up to 1.7 cm above the real top: a hand
        # hovering over the bench edge overlapped two cells, and every plan from there failed
        # COLLISION_AT_START (2026-10-26). The camera filter's model and tolerances; the map is
        # in base_link, the model's own root, at the camera cloud's stamp. Its output is the
        # ManipulationModule's `voxel_map` input. Tf and the URDF's files stay the other's.
        UrdfManagerPointClearing.blueprint(
            instance_name=MANIP_MAP_SELF_FILTER,
            model=ALFRED_SELF_FILTER_MODEL,
            publish_joint_tf=False,
            state_tolerance_s=JOINT_STATE_TOLERANCE_S,
            tf_tolerance_s=TF_TOLERANCE_S,
            tf_forward_tolerance_s=TF_TOLERANCE_S,
            padding_m=MANIP_MAP_CARVE_PADDING_M,
        ).remappings(
            [
                (MANIP_MAP_SELF_FILTER, "pointcloud", "manip_uncarved_map"),
                (MANIP_MAP_SELF_FILTER, "filtered_pointcloud", "voxel_map"),
                (MANIP_MAP_SELF_FILTER, "render_file_request", "manip_carve_file_request_unused"),
                (MANIP_MAP_SELF_FILTER, "file_info", "manip_carve_file_info_unused"),
            ]
        ),
    )


def _grasp_generator(grasp: GraspGenerator) -> Any:
    if grasp == "graspgenx":
        return GraspGenXModule.blueprint(
            gripper=OPENARM_V10_SWEEP_VOLUME,
            grasp_frame_to_tcp=OPENARM_V10_GRASP_FRAME_TO_TCP,
            startup_timeout=GRASPGENX_STARTUP_TIMEOUT_S,
        )
    if grasp == "heuristic":
        return HeuristicGraspModule.blueprint()
    raise ValueError(f"unknown grasp generator {grasp!r}")


def alfred_operator_modules() -> tuple[Any, ...]:
    """The handoff around the coordinator's operator hold, and, when there is a key for it, an
    LLM agent with Alfred's prompt and the skills that prompt names."""
    agent = (
        (
            McpClient.blueprint(system_prompt=ALFRED_SYSTEM_PROMPT),
            ManipulationSkills.blueprint(),
            ObserveSkill.blueprint(),
        )
        if os.environ.get("OPENAI_API_KEY")
        else ()
    )
    return (OperatorHandoff.blueprint(), *agent)


def alfred_manipulation_modules(*, grasp: GraspGenerator = "graspgenx") -> tuple[Any, ...]:
    """What manipulation adds to a stack with localize in it; the blueprint disables its
    ``UrdfManager``. ``grasp`` picks who answers the pick pipeline's ``propose_grasps``."""
    return (
        *alfred_obstacle_map_modules(),
        alfred_planner(),
        PickAndPlaceModule.blueprint(planning_frame=BASE_FRAME, reach=ALFRED_ARM_REACH),
        _grasp_generator(grasp),
        *alfred_operator_modules(),
    )


# The registry scans for module-level autoconnect(...) chains: each stack is spelled out.
# GraspGenX proposes the grasps (ranked, angled ones an OpenArm can reach; it runs in its own
# environment, synced on first start, weights downloaded then too); the heuristic's one
# top-down grasp per object is the fallback stack.
alfred_manipulation = (
    autoconnect(alfred_perception, *alfred_manipulation_modules())
    .disabled_modules(UrdfManager)
    .global_config(n_workers=10, robot_model="alfred")
)
alfred_manipulation_heuristic = (
    autoconnect(alfred_perception, *alfred_manipulation_modules(grasp="heuristic"))
    .disabled_modules(UrdfManager)
    .global_config(n_workers=10, robot_model="alfred")
)
