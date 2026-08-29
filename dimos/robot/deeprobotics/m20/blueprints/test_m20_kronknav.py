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
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Bool import Bool
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.navigation.dannav.holonomic_tc.module import DanHolonomicTC
from dimos.navigation.dannav.local_planner.module import DanLocalPlanner
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_3d.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.robot.deeprobotics.m20.blueprints.m20_kronknav import (
    deeprobotics_m20_kronknav,
    deeprobotics_m20_kronknav_control,
    deeprobotics_m20_pointlio,
)
from dimos.robot.deeprobotics.m20.bridge.module import M20ROSBridge
from dimos.robot.deeprobotics.m20.connection import M20Connection
from dimos.robot.deeprobotics.m20.pointlio.module import M20PointLio
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule


def _bridge_kwargs(blueprint: Blueprint) -> dict[str, object]:
    atoms = [atom for atom in blueprint.blueprints if atom.module is M20ROSBridge]
    assert len(atoms) == 1
    return atoms[0].kwargs


def _module_kwargs(blueprint: Blueprint, module: type) -> dict[str, object]:
    atoms = [atom for atom in blueprint.active_blueprints if atom.module is module]
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


def test_pointlio_bringup_blueprint_has_no_command_publisher() -> None:
    assert _bridge_kwargs(deeprobotics_m20_pointlio)["enable_command_output"] is False
    assert not any(atom.module is M20Connection for atom in deeprobotics_m20_pointlio.blueprints)


def test_control_blueprint_explicitly_enables_robot_command_publisher() -> None:
    assert _bridge_kwargs(deeprobotics_m20_kronknav_control)["enable_command_output"] is True


def test_m20_blueprints_pin_native_sdk_supported_local_transport() -> None:
    assert deeprobotics_m20_kronknav.global_config_overrides["transport"] == "lcm"
    assert deeprobotics_m20_kronknav_control.global_config_overrides["transport"] == "lcm"


def test_sensor_and_pose_streams_reach_mapping_and_navigation() -> None:
    blueprint = deeprobotics_m20_kronknav

    assert _endpoint_modules(blueprint, "raw_lidar", PointCloud2, "out") == {M20ROSBridge}
    assert _endpoint_modules(blueprint, "raw_lidar", PointCloud2, "in") == {M20PointLio}
    assert _endpoint_modules(blueprint, "imu", Imu, "out") == {M20ROSBridge}
    assert _endpoint_modules(blueprint, "imu", Imu, "in") == {M20PointLio}
    assert _endpoint_modules(blueprint, "lidar", PointCloud2, "out") == {M20PointLio}
    assert RayTracingVoxelMap in _endpoint_modules(blueprint, "lidar", PointCloud2, "in")
    assert _endpoint_modules(blueprint, "lidar_ready", Bool, "out") == {M20ROSBridge}
    assert _endpoint_modules(blueprint, "lidar_ready", Bool, "in") == {M20Connection}
    assert _endpoint_modules(blueprint, "localization_ready", Bool, "out") == {M20PointLio}
    assert _endpoint_modules(blueprint, "localization_ready", Bool, "in") == {
        M20ROSBridge,
        M20Connection,
    }
    assert _endpoint_modules(blueprint, "tf", TFMessage, "out") == {M20PointLio}
    assert _endpoint_modules(blueprint, "tf", TFMessage, "in") == {
        RayTracingVoxelMap,
        MLSPlannerNative,
    }
    assert _endpoint_modules(blueprint, "odom", PoseStamped, "out") == {M20PointLio}
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


def test_rerun_click_and_teleop_inputs_reach_navigation_and_control() -> None:
    blueprint = deeprobotics_m20_kronknav_control

    assert _endpoint_modules(blueprint, "clicked_point", PointStamped, "out") == {
        RerunWebSocketServer
    }
    assert _endpoint_modules(blueprint, "clicked_point", PointStamped, "in") == {MovementManager}
    assert _endpoint_modules(blueprint, "tele_cmd_vel", Twist, "out") == {
        RerunWebSocketServer,
        WebsocketVisModule,
    }
    assert _endpoint_modules(blueprint, "tele_cmd_vel", Twist, "in") == {MovementManager}
    assert _endpoint_modules(blueprint, "goal", PointStamped, "out") == {MovementManager}
    assert _endpoint_modules(blueprint, "goal", PointStamped, "in") == {
        MLSPlannerNative,
        DanLocalPlanner,
    }
    assert _endpoint_modules(blueprint, "nav_cmd_vel", Twist, "out") == {DanHolonomicTC}
    assert _endpoint_modules(blueprint, "nav_cmd_vel", Twist, "in") == {MovementManager}


def test_rerun_uses_go2_navigation_data_budget() -> None:
    config = _module_kwargs(deeprobotics_m20_kronknav_control, RerunBridgeModule)
    visual_override = config["visual_override"]
    max_hz = config["max_hz"]

    assert isinstance(visual_override, dict)
    assert visual_override["world/raw_lidar"] is None
    assert visual_override["world/imu"] is None
    assert visual_override["world/lidar"] is None
    assert isinstance(max_hz, dict)
    assert max_hz["world/local_map"] == 0.5
    assert config["memory_limit"] == "64MB"


def test_global_map_is_rate_limited_at_the_source() -> None:
    config = _module_kwargs(deeprobotics_m20_kronknav_control, RayTracingVoxelMap)

    assert config["global_emit_every"] == 50
