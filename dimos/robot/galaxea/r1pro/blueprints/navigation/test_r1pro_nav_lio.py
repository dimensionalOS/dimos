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
from dimos.hardware.sensors.lidar.livox.module import Mid360
from dimos.hardware.sensors.lidar.pointlio.module import PointLioRust
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.navigation.experimental.dannav.holonomic_tc.module import DanHolonomicTC
from dimos.navigation.experimental.dannav.local_planner.module import DanLocalPlanner
from dimos.navigation.global_planner.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.perception.depth2depth_cloud.module import Depth2DepthCloud
from dimos.robot.galaxea.r1pro.blueprints.navigation.r1pro_nav_lio import (
    HEAD_CLOUD_MAX_HEIGHT_M,
    HEAD_CLOUD_MIN_HEIGHT_M,
    r1pro_nav_lio,
)
from dimos.robot.galaxea.r1pro.config import (
    R1PRO_CHASSIS_LIDAR_HOST_IP,
    R1PRO_CHASSIS_LIDAR_IP,
)
from dimos.robot.galaxea.r1pro.connection import R1ProConnection
from dimos.robot.galaxea.r1pro.lio import (
    LIDAR_FRAME,
    ODOM_FRAME,
    R1ProLioMountTf,
    R1ProLioOdomPose,
)


def _atoms(blueprint):
    """By instance name: the blueprint runs two ray-traced maps."""
    return {atom.name: atom for atom in blueprint.active_blueprints}


key = r1pro_nav_lio._instance_key


def test_base_link_has_one_parent_and_it_is_pointlio() -> None:
    atoms = _atoms(r1pro_nav_lio)
    # The connection's wheel odometry, and its odom -> base_link edge, are off.
    assert atoms[key(R1ProConnection)].kwargs["publish_odom"] is False
    # Point-LIO publishes odom -> lidar_pointlio_link and the mount tf hangs base_link under it.
    assert atoms[key(PointLioRust)].kwargs == {
        "frame_id": ODOM_FRAME,
        "sensor_frame_id": LIDAR_FRAME,
    }
    assert atoms[key(Mid360)].kwargs == {
        "frame_id": LIDAR_FRAME,
        "lidar_ip": R1PRO_CHASSIS_LIDAR_IP,
        "host_ip": R1PRO_CHASSIS_LIDAR_HOST_IP,
    }
    assert key(R1ProLioMountTf) in atoms
    # And the planners read Point-LIO's pose under the name they always did.
    remaps = r1pro_nav_lio.remapping_map
    assert remaps[(key(R1ProLioOdomPose), "pose")] == "chassis_odom"
    assert remaps[(key(DanLocalPlanner), "odom")] == "chassis_odom"
    assert remaps[(key(DanHolonomicTC), "odom")] == "chassis_odom"


def test_both_clouds_land_on_the_lidar_bus_and_the_head_is_cut_to_the_lidars_blind_band() -> None:
    atoms = _atoms(r1pro_nav_lio)
    remaps = r1pro_nav_lio.remapping_map
    assert remaps[(key(Mid360), "lidar")] == "lidar_raw"
    assert remaps[(key(PointLioRust), "lidar")] == "lidar"
    assert remaps[(key(Depth2DepthCloud), "depth_cloud")] == "lidar"
    # It anchors on the same bus, and skips its own clouds there by frame.
    assert remaps[(key(Depth2DepthCloud), "lidar")] == "lidar"
    head = atoms[key(Depth2DepthCloud)].kwargs
    assert head["min_height_m"] == HEAD_CLOUD_MIN_HEIGHT_M
    assert head["max_height_m"] == HEAD_CLOUD_MAX_HEIGHT_M
    # The lidar sits 0.29 m up; the band stops just above it and no higher.
    assert 0.29 < HEAD_CLOUD_MAX_HEIGHT_M < 0.5
    assert HEAD_CLOUD_MIN_HEIGHT_M < 0.0
    assert atoms[RayTracingVoxelMap.name].kwargs["world_frame"] == ODOM_FRAME


def test_the_stack_is_map_planner_local_planner_controller() -> None:
    atoms = _atoms(r1pro_nav_lio)
    for name in (
        RayTracingVoxelMap.name,
        key(MLSPlannerNative),
        key(DanLocalPlanner),
        key(DanHolonomicTC),
        key(MovementManager),
    ):
        assert name in atoms, name
    remaps = r1pro_nav_lio.remapping_map
    assert remaps[(key(MLSPlannerNative), "path")] == "planner_path"
    assert atoms[key(MLSPlannerNative)].kwargs["base_frame"] == "base_link"


def test_every_remapping_names_a_real_port_and_the_config_parses() -> None:
    atoms = _atoms(r1pro_nav_lio)
    ports_by_key = {name: {stream.name for stream in atom.streams} for name, atom in atoms.items()}
    for module_key, port in r1pro_nav_lio.remapping_map:
        assert port in ports_by_key[module_key], f"{module_key} has no port {port}"
    BlueprintConfigParser(r1pro_nav_lio).parse(environ={})
