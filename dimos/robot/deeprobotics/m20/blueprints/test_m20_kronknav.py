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

"""Safety contract tests for the exported M20 blueprints."""

from dimos.core.coordination.blueprints import Blueprint
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.navigation.dannav.holonomic_tc.module import DanHolonomicTC
from dimos.navigation.dannav.local_planner.module import DanLocalPlanner
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_3d.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.robot.deeprobotics.m20.blueprints.m20_kronknav import (
    deeprobotics_m20_kronknav,
    deeprobotics_m20_kronknav_control,
)
from dimos.robot.deeprobotics.m20.bridge.module import M20ROSBridge
from dimos.robot.deeprobotics.m20.connection import M20Connection


def _bridge_kwargs(blueprint: Blueprint) -> dict[str, object]:
    atoms = [atom for atom in blueprint.blueprints if atom.module is M20ROSBridge]
    assert len(atoms) == 1
    return atoms[0].kwargs


def _endpoint_modules(
    blueprint: Blueprint,
    name: str,
    stream_type: type,
    direction: str,
) -> set[type]:
    result: set[type] = set()
    for atom in blueprint.active_blueprints:
        for stream in atom.streams:
            effective_name = blueprint.remapping_map.get((atom.name, stream.name), stream.name)
            if (
                effective_name == name
                and stream.type is stream_type
                and stream.direction == direction
            ):
                result.add(atom.module)
    return result


def test_default_kronknav_blueprint_cannot_publish_robot_commands() -> None:
    assert _bridge_kwargs(deeprobotics_m20_kronknav)["enable_command_output"] is False


def test_control_blueprint_explicitly_enables_robot_command_publisher() -> None:
    assert _bridge_kwargs(deeprobotics_m20_kronknav_control)["enable_command_output"] is True


def test_m20_blueprints_pin_native_sdk_supported_local_transport() -> None:
    assert deeprobotics_m20_kronknav.global_config_overrides["transport"] == "lcm"
    assert deeprobotics_m20_kronknav_control.global_config_overrides["transport"] == "lcm"


def test_sensor_and_pose_streams_reach_mapping_and_navigation() -> None:
    blueprint = deeprobotics_m20_kronknav

    assert _endpoint_modules(blueprint, "lidar", PointCloud2, "out") == {M20ROSBridge}
    assert RayTracingVoxelMap in _endpoint_modules(blueprint, "lidar", PointCloud2, "in")
    assert _endpoint_modules(blueprint, "tf", TFMessage, "out") == {M20ROSBridge}
    assert _endpoint_modules(blueprint, "tf", TFMessage, "in") == {
        RayTracingVoxelMap,
        MLSPlannerNative,
    }
    assert _endpoint_modules(blueprint, "odom", PoseStamped, "out") == {M20ROSBridge}
    assert {
        DanLocalPlanner,
        DanHolonomicTC,
    } <= _endpoint_modules(blueprint, "odom", PoseStamped, "in")


def test_kronknav_path_and_guarded_command_chain_is_complete() -> None:
    blueprint = deeprobotics_m20_kronknav_control

    assert _endpoint_modules(blueprint, "planner_path", Path, "out") == {MLSPlannerNative}
    assert _endpoint_modules(blueprint, "planner_path", Path, "in") == {DanLocalPlanner}
    assert _endpoint_modules(blueprint, "path", Path, "out") == {DanLocalPlanner}
    assert DanHolonomicTC in _endpoint_modules(blueprint, "path", Path, "in")
    assert _endpoint_modules(blueprint, "cmd_vel", Twist, "out") == {MovementManager}
    assert _endpoint_modules(blueprint, "cmd_vel", Twist, "in") == {M20Connection}
    assert _endpoint_modules(blueprint, "safe_cmd_vel", Twist, "out") == {M20Connection}
    assert _endpoint_modules(blueprint, "safe_cmd_vel", Twist, "in") == {M20ROSBridge}
