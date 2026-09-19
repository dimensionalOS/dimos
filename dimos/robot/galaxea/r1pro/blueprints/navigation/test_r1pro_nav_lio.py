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

"""The nav-on-Point-LIO blueprints: who places the robot, and what feeds the map."""

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.hardware.sensors.camera.depth_cloud.module import StereoCloud
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.navigation.dannav.holonomic_tc.module import DanHolonomicTC
from dimos.navigation.dannav.local_planner.module import DanLocalPlanner
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_3d.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.robot.galaxea.r1pro.blueprints.navigation.r1pro_nav_lio import (
    HEAD_CLOUD_MAX_HEIGHT_M,
    HEAD_CLOUD_MIN_HEIGHT_M,
    r1pro_nav_lio,
    r1pro_nav_lio_replay,
)
from dimos.robot.galaxea.r1pro.connection import R1ProConnection
from dimos.robot.galaxea.r1pro.lio import (
    LIDAR_FRAME,
    ODOM_FRAME,
    R1ProLioMountTf,
    R1ProLioOdomPose,
    R1ProPointLio,
)
from dimos.robot.galaxea.r1pro.replay import R1ProReplay


def _atoms(blueprint):
    return {atom.module: atom for atom in blueprint.active_blueprints}


def test_base_link_has_one_parent_and_it_is_pointlio() -> None:
    atoms = _atoms(r1pro_nav_lio)
    # The connection's wheel odometry, and its odom -> base_link edge, are off.
    assert atoms[R1ProConnection].kwargs["publish_odom"] is False
    # Point-LIO publishes odom -> lidar_pointlio_link, and the mount tf hangs
    # base_link under it: exactly one path from odom to base_link.
    assert atoms[R1ProPointLio].kwargs == {"frame_id": ODOM_FRAME, "sensor_frame_id": LIDAR_FRAME}
    assert R1ProLioMountTf in atoms
    # And the planners read Point-LIO's pose under the name they always did.
    remaps = r1pro_nav_lio.remapping_map
    key = r1pro_nav_lio._instance_key
    assert remaps[(key(R1ProLioOdomPose), "pose")] == "chassis_odom"
    assert remaps[(key(DanLocalPlanner), "odom")] == "chassis_odom"
    assert remaps[(key(DanHolonomicTC), "odom")] == "chassis_odom"


def test_both_clouds_land_on_the_lidar_bus_and_the_head_is_cut_to_the_lidars_blind_band() -> None:
    atoms = _atoms(r1pro_nav_lio)
    remaps = r1pro_nav_lio.remapping_map
    key = r1pro_nav_lio._instance_key
    assert remaps[(key(R1ProPointLio), "lidar")] == "lidar"
    assert remaps[(key(StereoCloud), "cloud")] == "lidar"
    stereo = atoms[StereoCloud].kwargs
    assert stereo["min_height_m"] == HEAD_CLOUD_MIN_HEIGHT_M
    assert stereo["max_height_m"] == HEAD_CLOUD_MAX_HEIGHT_M
    # The lidar sits 0.29 m up; the band stops just above it and no higher.
    assert 0.29 < HEAD_CLOUD_MAX_HEIGHT_M < 0.5
    assert HEAD_CLOUD_MIN_HEIGHT_M < 0.0
    voxel_map = atoms[RayTracingVoxelMap].kwargs
    assert voxel_map["world_frame"] == ODOM_FRAME
    assert voxel_map["max_cloud_rate_hz"] == 10.0


def test_the_stack_is_map_planner_local_planner_controller() -> None:
    atoms = _atoms(r1pro_nav_lio)
    for module in (
        RayTracingVoxelMap,
        MLSPlannerNative,
        DanLocalPlanner,
        DanHolonomicTC,
        MovementManager,
    ):
        assert module in atoms, module.__name__
    remaps = r1pro_nav_lio.remapping_map
    key = r1pro_nav_lio._instance_key
    assert remaps[(key(MLSPlannerNative), "path")] == "planner_path"
    assert atoms[MLSPlannerNative].kwargs["base_frame"] == "base_link"


def test_every_remapping_names_a_real_port_and_the_config_parses() -> None:
    for blueprint in (r1pro_nav_lio, r1pro_nav_lio_replay):
        atoms = _atoms(blueprint)
        ports_by_key = {
            blueprint._instance_key(module): {stream.name for stream in atom.streams}
            for module, atom in atoms.items()
        }
        for module_key, port in blueprint.remapping_map:
            assert port in ports_by_key[module_key], f"{module_key} has no port {port}"
        BlueprintConfigParser(blueprint).parse(environ={})


def test_replay_stands_in_for_the_robot_with_the_same_downstream() -> None:
    live, replay = _atoms(r1pro_nav_lio), _atoms(r1pro_nav_lio_replay)
    assert R1ProReplay in replay and R1ProConnection not in replay
    assert R1ProPointLio not in replay, "the recording carries Point-LIO's pose and tf"
    for module in (
        StereoCloud,
        RayTracingVoxelMap,
        MLSPlannerNative,
        DanLocalPlanner,
        MovementManager,
    ):
        assert module in live and module in replay
    assert r1pro_nav_lio_replay.global_config_overrides["replay"] is True
